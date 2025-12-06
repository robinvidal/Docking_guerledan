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
import time
from rcl_interfaces.msg import SetParametersResult


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
        self.declare_parameter('cage_width', 1.0)  # m
        self.declare_parameter('noise_level', 20.0)  # 0-100
        self.declare_parameter('cmd_vel_topic', '/bluerov/cmd_vel')  # Topic commandes
        # Noise as point cloud parameters
        self.declare_parameter('noise_point_count', 200)
        self.declare_parameter('noise_i_min', 5.0)
        self.declare_parameter('noise_i_max', 120.0)
        self.declare_parameter('noise_blur_sigma', 1.2)
        self.declare_parameter('noise_blur_iters', 1)
        # Post intensities (left / right)
        self.declare_parameter('post_intensity_left', 230.0)
        self.declare_parameter('post_intensity_right', 230.0)
        
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
        self.noise_point_count = int(self.get_parameter('noise_point_count').value)
        self.noise_i_min = float(self.get_parameter('noise_i_min').value)
        self.noise_i_max = float(self.get_parameter('noise_i_max').value)
        self.noise_blur_sigma = float(self.get_parameter('noise_blur_sigma').value)
        self.noise_blur_iters = int(self.get_parameter('noise_blur_iters').value)
        self.post_intensity_left = float(self.get_parameter('post_intensity_left').value)
        self.post_intensity_right = float(self.get_parameter('post_intensity_right').value)
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
        
        # Parameters update callback (live reconfigure via `ros2 param set`)
        self.add_on_set_parameters_callback(self._on_set_parameters)
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
        delta_x = self.cmd_vx * dt
        delta_y = -self.cmd_vy * dt
        delta_theta = self.cmd_omega * dt  # inverted rotation direction
        
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

    def _on_set_parameters(self, params):
        """Callback invoked when parameters are changed at runtime.

        Update internal attributes for immediate effect and refresh derived grids
        (ranges, bearings) when angle/ranges are changed.
        """
        successful = True

        new_min_range = self.min_range
        new_max_range = self.max_range
        new_bearing_angle = self.bearing_angle

        for p in params:
            name = p.name
            try:
                if name == 'noise_point_count':
                    self.noise_point_count = int(p.value)
                elif name == 'noise_i_min':
                    self.noise_i_min = float(p.value)
                elif name == 'noise_i_max':
                    self.noise_i_max = float(p.value)
                elif name == 'noise_blur_sigma':
                    self.noise_blur_sigma = float(p.value)
                elif name == 'noise_blur_iters':
                    self.noise_blur_iters = int(p.value)
                elif name == 'post_intensity_left':
                    self.post_intensity_left = float(p.value)
                elif name == 'post_intensity_right':
                    self.post_intensity_right = float(p.value)
                elif name == 'noise_level':
                    self.noise_level = float(p.value)
                elif name == 'min_range':
                    new_min_range = float(p.value)
                elif name == 'max_range':
                    new_max_range = float(p.value)
                elif name == 'bearing_angle':
                    new_bearing_angle = float(p.value)
            except Exception:
                successful = False

        # Validate and apply range changes
        if new_min_range <= 0 or new_min_range >= new_max_range:
            successful = False
        else:
            self.min_range = new_min_range
            self.max_range = new_max_range
            self.ranges = np.linspace(self.min_range, self.max_range, self.range_count)

        # Validate and apply bearing angle change
        if new_bearing_angle <= 0:
            successful = False
        else:
            self.bearing_angle = new_bearing_angle
            half_angle = self.bearing_angle / 2.0
            self.bearings = np.linspace(-half_angle * np.pi / 180, half_angle * np.pi / 180, self.bearing_count)
            self.half_angle_rad = half_angle * np.pi / 180.0

        return SetParametersResult(successful=successful)
    
    def generate_synthetic_frame(self) -> np.ndarray:
        """Génère une frame sonar synthétique avec cage à la position courante."""
        # OPTIMISATION 1 : Réutiliser les grilles pré-calculées
        # (plus besoin de recalculer ranges/bearings à chaque frame)
        
        # OPTIMISATION 2 : Génération bruit vectorisée
        std = max(1.0, float(self.noise_level))
        mean_bg = std * 0.2
        
        # Option: point-based noise (scatter n points then blur)
        intensities = np.full((self.bearing_count, self.range_count), mean_bg, dtype=np.float32)

        # Scatter random points
        if self.noise_point_count > 0 and (self.noise_i_max > self.noise_i_min):
            n = max(0, int(self.noise_point_count))
            bear_idx = np.random.randint(0, self.bearing_count, size=n)
            range_idx = np.random.randint(0, self.range_count, size=n)
            vals = np.random.uniform(self.noise_i_min, self.noise_i_max, size=n)
            intensities[bear_idx, range_idx] = vals

        # Add light Gaussian speckle background if desired
        if self.noise_level > 0:
            intensities += np.random.normal(loc=0.0, scale=self.noise_level * 0.2,
                                            size=intensities.shape).astype(np.float32)

        # Apply blur / spreading to make points look natural
        for _ in range(max(1, int(self.noise_blur_iters))):
            if float(self.noise_blur_sigma) > 0.0:
                try:
                    intensities = gaussian_filter(intensities, float(self.noise_blur_sigma)).astype(np.float32)
                except Exception:
                    # If gaussian_filter fails, ignore and leave intensities as-is
                    pass

        # Clip and convert
        intensities = np.clip(intensities, 0, 255).astype(np.uint8)

        # Return intensities (posts will be added after filtering)
        return intensities
    
    def publish_frame(self):
        """Publie une frame sonar filtrée sur /docking/sonar/raw."""
        timestamp = self.get_clock().now().to_msg()
        
        # Génération image synthétique brute
        intensities = self.generate_synthetic_frame()
        
        # OPTIMISATION 6 : Éviter les copies inutiles
        # On travaille directement sur l'array (in-place quand possible)
        filtered = intensities  # Pas de copy() initial
        
        # Application des filtres (certains retournent une nouvelle array)
        if self.get_parameter('enable_median').value:
            kernel = self.get_parameter('median_kernel').value
            filtered = median_filter(filtered, kernel)
        
        if self.get_parameter('enable_gaussian').value:
            sigma = self.get_parameter('gaussian_sigma').value
            filtered = gaussian_filter(filtered, sigma)
        
        if self.get_parameter('enable_range_comp').value:
            alpha = self.get_parameter('range_comp_alpha').value
            # OPTIMISATION 7 : Réutiliser self.ranges (déjà calculé)
            filtered = range_compensation(filtered, self.ranges, alpha)
        
        if self.get_parameter('enable_contrast').value:
            clip = self.get_parameter('contrast_clip').value
            filtered = contrast_enhancement(filtered, clip)

        # Add cage posts after filters so posts are not affected by filters
        try:
            self.add_posts(filtered)
        except Exception:
            # If adding posts fails, continue without stopping the node
            pass

        # Ensure integers 0..255 before publishing
        filtered = np.clip(filtered, 0, 255).astype(np.uint8)

        # OPTIMISATION 8 : Conversion liste optimisée
        # flatten() + tolist() est plus rapide que flatten().tolist()
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


    def add_posts(self, arr: np.ndarray):
        """Ajoute les deux montants de la cage à l'image `arr` en-place.

        `arr` peut être float32 ou uint8; la méthode écrit des valeurs d'intensité
        directement dans le tableau (en convertissant si nécessaire).
        """
        # Work on float copy to avoid issues with uint8 wrapping
        if arr.dtype != np.float32 and arr.dtype != np.float64:
            work = arr.astype(np.float32)
        else:
            work = arr

        cage_half_width = self.cage_width / 2.0
        cos_theta = np.cos(self.cage_theta)
        sin_theta = np.sin(self.cage_theta)
        post_offsets = [(-cage_half_width, 0.0), (cage_half_width, 0.0)]

        for i, offset in enumerate(post_offsets):
            offset_x, offset_y = offset
            post_x = self.cage_x + (offset_x * cos_theta - offset_y * sin_theta)
            post_y = self.cage_y + (offset_x * sin_theta + offset_y * cos_theta)

            post_range = np.sqrt(post_x**2 + post_y**2)
            post_bearing = np.arctan2(post_x, post_y)

            if not (self.min_range <= post_range <= self.max_range):
                continue
            if abs(post_bearing) > self.half_angle_rad:
                continue

            bearing_idx = np.searchsorted(self.bearings, post_bearing)
            range_idx = int((post_range - self.min_range) / (self.max_range - self.min_range) * (self.range_count - 1))
            bearing_idx = np.clip(bearing_idx, 0, self.bearing_count - 1)
            range_idx = np.clip(range_idx, 0, self.range_count - 1)

            intensity_val = self.post_intensity_left if i == 0 else self.post_intensity_right

            # Patch size based on physical diameter (30 cm)
            diameter_m = 0.30
            radius_m = diameter_m / 2.0
            range_res = (self.max_range - self.min_range) / max(self.range_count, 1)
            bearing_res = (self.bearing_angle * np.pi / 180.0) / max(self.bearing_count, 1)

            range_width = max(1, int(np.ceil(radius_m / max(range_res, 1e-6))))
            bearing_width = max(
                1,
                int(
                    np.ceil(
                        np.arctan2(radius_m, max(post_range, 1e-6)) / max(bearing_res, 1e-6)
                    )
                ),
            )

            b_min = max(0, bearing_idx - bearing_width)
            b_max = min(self.bearing_count, bearing_idx + bearing_width + 1)
            r_min = max(0, range_idx - range_width)
            r_max = min(self.range_count, range_idx + range_width + 1)

            work[b_min:b_max, r_min:r_max] = float(np.clip(intensity_val, 0, 255))

        # Write back into original array in-place if possible
        if work is not arr:
            # Attempt to cast back to original dtype in-place
            try:
                arr[:] = work.astype(arr.dtype)
            except Exception:
                # If that fails, return the work array (caller may ignore)
                return work
        return arr


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
