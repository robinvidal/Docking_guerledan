"""
Nœud ROS2 mock simulant le sonar Oculus M750d.
Publie des frames synthétiques avec cage simulée.
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame
from geometry_msgs.msg import Twist
import numpy as np
from docking_utils.filters import (
    median_filter, gaussian_filter, contrast_enhancement,
    range_compensation
)
import cv2
import time


class SonarMockNode(Node):
    """Mock du sonar pour développement sans matériel."""
    
    def __init__(self):
        super().__init__('sonar_mock')
        
        # Paramètres sonar
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('range_count', 512)
        self.declare_parameter('bearing_count', 256)
        self.declare_parameter('bearing_angle', 140.0)  # Ouverture totale en degrés
        self.declare_parameter('min_range', 1.0)  # m
        self.declare_parameter('max_range', 40.0)  # m
        self.declare_parameter('cage_distance', 8.0)  # m (distance initiale)
        self.declare_parameter('cage_width', 2.0)  # m
        self.declare_parameter('noise_level', 20.0)  # 0-100
        self.declare_parameter('cmd_vel_topic', '/bluerov/cmd_vel')  # Topic commandes
        
        # Paramètres de filtrage
        self.declare_parameter('enable_median', True)
        self.declare_parameter('median_kernel', 3)
        self.declare_parameter('enable_gaussian', True)
        self.declare_parameter('gaussian_sigma', 1.5)
        self.declare_parameter('enable_contrast', True)
        self.declare_parameter('contrast_clip', 2.0)
        self.declare_parameter('enable_range_comp', True)
        self.declare_parameter('range_comp_alpha', 0.0001)
        
        rate = self.get_parameter('publish_rate').value
        self.range_count = self.get_parameter('range_count').value
        self.bearing_count = self.get_parameter('bearing_count').value
        self.bearing_angle = self.get_parameter('bearing_angle').value  # degrés
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        cage_distance_init = self.get_parameter('cage_distance').value
        self.cage_width = self.get_parameter('cage_width').value
        self.noise_level = self.get_parameter('noise_level').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        # === Pré-calcul des grilles polaires (OPTIMISATION) ===
        # Calcul unique au démarrage, réutilisé à chaque frame
        self.ranges = np.linspace(self.min_range, self.max_range, self.range_count)
        half_angle = self.bearing_angle / 2.0
        self.bearings = np.linspace(-half_angle * np.pi/180, half_angle * np.pi/180, self.bearing_count)
        self.half_angle_rad = half_angle * np.pi / 180.0
        
        # === État de la simulation (position relative cage/ROV) ===
        # Position de la cage dans le repère du ROV (fixe au départ)
        # La cage est fixe dans le monde, le ROV bouge
        # Donc on stocke la position relative cage vue depuis le ROV
        self.cage_x = 0.0  # Position latérale (m) - 0 = centré
        self.cage_y = cage_distance_init  # Distance frontale (m)
        self.cage_theta = 0.0  # Orientation relative cage (rad)
        
        # Vitesses actuelles du ROV (reçues de /cmd_vel)
        self.cmd_vx = 0.0  # Vitesse latérale (m/s)
        self.cmd_vy = 0.0  # Vitesse frontale (m/s)
        self.cmd_omega = 0.0  # Vitesse angulaire (rad/s)
        
        # Temps de la dernière mise à jour physique
        self.last_update_time = time.time()
        
        # Subscriber pour commandes de vitesse
        self.cmd_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_callback,
            10
        )
        
        # Publisher (publie données filtrées sur /docking/sonar/raw)
        self.publisher_ = self.create_publisher(Frame, '/docking/sonar/raw', 10)
        
        # Timer pour mise à jour physique (50 Hz pour simulation fluide)
        self.physics_timer = self.create_timer(1.0 / 50.0, self.update_physics)
        
        # Timer pour publication sonar (rate configuré)
        self.publish_timer = self.create_timer(1.0 / rate, self.publish_frame)
        
        self.get_logger().info(
            f'Sonar mock démarré: {rate} Hz, cage initiale @ ({self.cage_x:.2f}, {self.cage_y:.2f})m'
        )
        self.get_logger().info(f'Écoute des commandes sur: {cmd_vel_topic}')

        # === Optimisations: buffers réutilisables et RNG ===
        # Générateur de nombres rapide et avec état
        self.rng = np.random.default_rng()

        # Buffers réutilisés pour éviter allocations répétées
        self._float_buf = np.empty((self.bearing_count, self.range_count), dtype=np.float32)
        self._int_buf = np.empty((self.bearing_count, self.range_count), dtype=np.uint8)

        # Paramètre de sous-échantillonnage pour filtrage rapide (2 => 4x moins d'opérations)
        self.declare_parameter('downsample_factor', 2)
        self.ds_factor = max(1, int(self.get_parameter('downsample_factor').value))
    
    def cmd_callback(self, msg: Twist):
        """
        Callback pour réception des commandes de vitesse du ROV.
        
        Convention Twist:
        - linear.x : vitesse avant/arrière (surge)
        - linear.y : vitesse latérale gauche/droite (sway)
        - angular.z : vitesse rotation autour axe vertical (yaw)
        """
        self.cmd_vx = msg.linear.y  # Sway (latéral)
        self.cmd_vy = msg.linear.x  # Surge (frontal)
        self.cmd_omega = msg.angular.z  # Yaw rate
        
        # Log pour debug (première fois seulement pour ne pas spammer)
        if not hasattr(self, '_cmd_received'):
            self._cmd_received = True
            self.get_logger().info(
                f'Première commande reçue: vx={self.cmd_vx:.2f}, vy={self.cmd_vy:.2f}, ω={self.cmd_omega:.2f}'
            )
    
    def update_physics(self):
        """
        Mise à jour de la position relative de la cage (physique inverse).
        
        Principe : La cage est fixe dans le monde, le ROV bouge.
        Donc si le ROV avance de +v_y, la cage recule de -v_y dans le repère du ROV.
        
        Étapes :
        1. Calculer Δt depuis dernière mise à jour
        2. Calculer déplacement inverse : Δx = -vx·Δt, Δy = -vy·Δt, Δθ = -ω·Δt
        3. Appliquer translation à la position cage
        4. Appliquer rotation inverse autour de l'origine (position ROV)
        """
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Déplacements inverses (cage vue depuis ROV qui bouge)
        delta_x = -self.cmd_vx * dt
        delta_y = -self.cmd_vy * dt
        delta_theta = -self.cmd_omega * dt
        
        # Translation de la cage
        cage_x_translated = self.cage_x + delta_x
        cage_y_translated = self.cage_y + delta_y
        
        # Rotation inverse autour de l'origine (position du ROV)
        # Matrice de rotation 2D : [cos(-θ) -sin(-θ); sin(-θ) cos(-θ)]
        cos_theta = np.cos(-delta_theta)
        sin_theta = np.sin(-delta_theta)
        
        # Appliquer rotation
        self.cage_x = cos_theta * cage_x_translated - sin_theta * cage_y_translated
        self.cage_y = sin_theta * cage_x_translated + cos_theta * cage_y_translated
        
        # Mise à jour orientation relative de la cage
        self.cage_theta += delta_theta
        
        # Log périodique (toutes les 2 secondes environ)
        if np.random.rand() < 0.02:  # ~1% de chance à 50 Hz
            self.get_logger().info(
                f'Cage relative: x={self.cage_x:.2f}m, y={self.cage_y:.2f}m, '
                f'θ={np.rad2deg(self.cage_theta):.1f}°'
            )
    
    def generate_synthetic_frame(self) -> np.ndarray:
        """Génère une frame sonar synthétique avec cage à la position courante.

        Optimisations appliquées:
        - Réutilisation de buffers préalloués
        - RNG basé sur numpy.default_rng()
        - Speckle vectorisé
        """
        std = max(1.0, float(self.noise_level))
        mean_bg = std * 0.2

        # Remplir buffer flottant via RNG (évite nouvelles allocations)
        self._float_buf[:] = self.rng.normal(loc=mean_bg, scale=std, size=self._float_buf.shape)

        # Speckle (bruit impulsionnel) vectorisé
        sp_prob = min(0.12, self.noise_level / 400.0)
        if sp_prob > 0.0:
            rand_vals = self.rng.random(self._float_buf.shape)
            speckle_mask = rand_vals < sp_prob
            if speckle_mask.any():
                high_mask = rand_vals[speckle_mask] < (sp_prob * 0.7)
                # Génération des valeurs hauteurs speckle en une fois
                speckle_vals = np.empty(high_mask.shape, dtype=np.uint8)
                # haut -> fortes intensités, bas -> 0
                speckle_vals[high_mask] = self.rng.integers(200, 255, size=high_mask.sum(), dtype=np.uint8)
                speckle_vals[~high_mask] = 0
                self._float_buf[speckle_mask] = speckle_vals

        # Conversion clamp et écriture dans int buffer
        np.clip(self._float_buf, 0, 255, out=self._float_buf)
        self._int_buf[:] = self._float_buf.astype(np.uint8, copy=False)
        
        # === OPTIMISATION 3 : Dessin des montants optimisé ===
        cage_half_width = self.cage_width / 2.0
        
        # Pré-calcul trigonométrie (une seule fois)
        cos_theta = np.cos(self.cage_theta)
        sin_theta = np.sin(self.cage_theta)
        
        # Positions des 4 montants (numpy array pour vectorisation partielle)
        montant_offsets = np.array([
            [-cage_half_width, 0.0],
            [-cage_half_width * 0.7, 0.0],
            [cage_half_width * 0.7, 0.0],
            [cage_half_width, 0.0]
        ])
        
        # Boucle sur les montants (peu nombreux, pas critique)
        for offset in montant_offsets:
            offset_x, offset_y = offset
            
            # Transformation en une ligne (rotation + translation)
            montant_x = self.cage_x + (offset_x * cos_theta - offset_y * sin_theta)
            montant_y = self.cage_y + (offset_x * sin_theta + offset_y * cos_theta)
            
            # Conversion polaire
            montant_range = np.sqrt(montant_x**2 + montant_y**2)
            montant_bearing = np.arctan2(montant_x, montant_y)
            
            # Vérifications rapides (early exit)
            if not (self.min_range <= montant_range <= self.max_range):
                continue
            if abs(montant_bearing) > self.half_angle_rad:
                continue
            
            # OPTIMISATION 4 : Recherche d'index optimisée (searchsorted plus rapide que argmin)
            bearing_idx = np.searchsorted(self.bearings, montant_bearing)
            range_idx = np.searchsorted(self.ranges, montant_range)
            
            # Borner les index
            bearing_idx = np.clip(bearing_idx, 0, self.bearing_count - 1)
            range_idx = np.clip(range_idx, 0, self.range_count - 1)
            
            # OPTIMISATION 5 : Dessin par slicing NumPy (plus rapide que boucles imbriquées)
            range_width = 2
            bearing_width = 3
            
            # Calcul des bornes du patch
            b_min = max(0, bearing_idx - bearing_width)
            b_max = min(self.bearing_count, bearing_idx + bearing_width + 1)
            r_min = max(0, range_idx - range_width)
            r_max = min(self.range_count, range_idx + range_width + 1)
            
            # Remplissage par slicing (beaucoup plus rapide)
            # Utiliser RNG pour remplir patch rapidement
            self._int_buf[b_min:b_max, r_min:r_max] = self.rng.integers(200, 255, size=(b_max-b_min, r_max-r_min), dtype=np.uint8)
        
        return self._int_buf
    
    def publish_frame(self):
        """Publie une frame sonar filtrée sur /docking/sonar/raw."""
        timestamp = self.get_clock().now().to_msg()
        
        # Génération image synthétique brute
        raw = self.generate_synthetic_frame()

        # Travail sur buffer int (éviter copies si possible)
        filtered = raw  # vue sur _int_buf

        # Filtrage rapide: appliquer filtres coûteux sur version sous-échantillonnée
        if (self.ds_factor > 1) and (self.get_parameter('enable_median').value or self.get_parameter('enable_gaussian').value):
            ds = self.ds_factor
            # Sous-échantillonnage par slicing (vue)
            small = filtered[::ds, ::ds]

            # Appliquer les filtres sur la petite image (OpenCV interne est optimisé)
            small_filtered = small
            if self.get_parameter('enable_median').value:
                kernel = self.get_parameter('median_kernel').value
                small_filtered = median_filter(small_filtered, kernel)
            if self.get_parameter('enable_gaussian').value:
                sigma = self.get_parameter('gaussian_sigma').value
                small_filtered = gaussian_filter(small_filtered, sigma)

            # Upsample via OpenCV (nearest) — rapide et sans interpolation coûteuse
            up = cv2.resize(small_filtered, (self.range_count, self.bearing_count), interpolation=cv2.INTER_NEAREST)

            # Mélange en uint8 via OpenCV (évite conversions float coûteuses)
            alpha = 0.6
            filtered = cv2.addWeighted(filtered, float(alpha), up, float(1.0 - alpha), 0)
        else:
            # Application simple si pas de sous-échantillonnage
            if self.get_parameter('enable_median').value:
                kernel = self.get_parameter('median_kernel').value
                filtered = median_filter(filtered, kernel)
            if self.get_parameter('enable_gaussian').value:
                sigma = self.get_parameter('gaussian_sigma').value
                filtered = gaussian_filter(filtered, sigma)

        # Compensation en portée et amélioration contraste (coûts O(B*R) mais légers)
        if self.get_parameter('enable_range_comp').value:
            alpha = self.get_parameter('range_comp_alpha').value
            # Range compensation requires float to avoid uint8 wrap-around
            filtered = range_compensation(filtered.astype(np.float32), self.ranges, alpha)
            # Revenir en uint8 pour les étapes suivantes (CLAHE, sérialisation)
            filtered = np.clip(filtered, 0, 255).astype(np.uint8)

        if self.get_parameter('enable_contrast').value:
            clip = self.get_parameter('contrast_clip').value
            filtered = contrast_enhancement(filtered, clip)

        # Sérialisation: ravel -> liste (nota: assigner bytes/array possible mais dépend du binding rclpy)
        intensities_list = filtered.ravel().tolist()
        
        # Publication sur /docking/sonar/raw (contient données filtrées)
        msg = Frame()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'sonar_link'
        msg.range_count = self.range_count
        msg.bearing_count = self.bearing_count
        msg.range_resolution = (self.max_range - self.min_range) / self.range_count
        msg.bearing_resolution = (self.bearing_angle * np.pi / 180.0) / self.bearing_count
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.intensities = intensities_list
        msg.sound_speed = 1500.0
        msg.gain = 50
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SonarMockNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
