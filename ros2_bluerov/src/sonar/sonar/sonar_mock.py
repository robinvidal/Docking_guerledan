"""
Nœud ROS2 Mock - Simulation du sonar Oculus M750d.

On conseille fortement d'utiliser des Rosbag pour les tests de la pipeline de traitement, mais ce module peut être utile si aucun rosbag n'a été enregistré.

Ce module fournit un simulateur de sonar pour le développement et les tests
sans nécessiter le matériel physique et sans avoir besoin de rosbag. Il génère des frames sonar synthétiques
avec une cage simulée dont la position évolue en réponse aux commandes de vitesse.

Fonctionnalités principales:
    - Génération d'images sonar synthétiques (bruit + poteaux de cage)
    - Simulation physique de la position relative cage/ROV
    - Application de filtres de traitement d'image configurables
    - Reconfiguration dynamique des paramètres via ROS2

Auteur: Équipe Docking Guerlédan
Date: 2026
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame
from geometry_msgs.msg import Twist
import numpy as np
from scipy import ndimage
import time
from rcl_interfaces.msg import SetParametersResult


# =============================================================================
# FILTRES DE TRAITEMENT D'IMAGE (intégrés depuis docking_utils)
# =============================================================================

def median_filter(image: np.ndarray, kernel_size: int = 3) -> np.ndarray:
    """
    Applique un filtre médian pour réduire le bruit impulsionnel.
    
    Args:
        image: Image sonar 2D [bearing, range]
        kernel_size: Taille du noyau (impair)
    
    Returns:
        Image filtrée
    """
    return ndimage.median_filter(image, size=kernel_size)


def gaussian_filter(image: np.ndarray, sigma: float = 1.0) -> np.ndarray:
    """
    Applique un filtre gaussien pour lissage.
    
    Args:
        image: Image sonar 2D
        sigma: Écart-type du noyau gaussien
    
    Returns:
        Image lissée
    """
    return ndimage.gaussian_filter(image, sigma=sigma)


def contrast_enhancement(image: np.ndarray, clip_limit: float = 2.0) -> np.ndarray:
    """
    Amélioration du contraste par étirement d'histogramme.
    
    Args:
        image: Image niveaux de gris [0-255]
        clip_limit: Limite d'écrêtage pour éviter sur-amplification
    
    Returns:
        Image avec contraste amélioré
    """
    p_low, p_high = np.percentile(image, [clip_limit, 100 - clip_limit])
    clipped = np.clip(image, p_low, p_high)
    normalized = (clipped - p_low) / (p_high - p_low) * 255
    return normalized.astype(np.uint8)


def range_compensation(image: np.ndarray, ranges: np.ndarray, 
                       alpha: float = 0.0001) -> np.ndarray:
    """
    Compense l'atténuation en fonction de la distance (spreading loss).
    
    Args:
        image: Image sonar 2D [bearing, range]
        ranges: Array des distances correspondant aux bins (m)
        alpha: Coefficient d'atténuation volumique (m^-1)
    
    Returns:
        Image compensée
    """
    gain = ranges[:, np.newaxis].T * np.exp(alpha * ranges[:, np.newaxis].T)
    compensated = image * gain
    compensated = np.clip(compensated, 0, 255)
    return compensated.astype(image.dtype)


# =============================================================================
# NODE ROS2 MOCK SONAR
# =============================================================================


class SonarMockNode(Node):
    """
    Nœud ROS2 simulant un sonar Oculus M750d pour le développement offline.
    
    Ce mock génère des images sonar synthétiques représentant une cage
    à poteaux dont la position relative au ROV évolue selon les commandes
    de vitesse reçues. Permet de tester la pipeline de traitement sans
    matériel réel, et sans rosbag.
    
    Subscriptions:
        - /bluerov/cmd_vel (geometry_msgs/Twist): Commandes de vitesse du ROV
        
    Publications:
        - /docking/sonar/raw (docking_msgs/Frame): Frames sonar simulées
    """
    
    def __init__(self):
        super().__init__('sonar_mock')
        
        # ===================================================================
        # DÉCLARATION DES PARAMÈTRES ROS2
        # Tous les paramètres sont reconfigurables dynamiquement via:
        #   ros2 param set /sonar_mock <param_name> <value>
        # ===================================================================
        
        # --- Paramètres géométriques du sonar simulé ---
        self.declare_parameter('publish_rate', 10.0)      # Fréquence de publication (Hz)
        self.declare_parameter('range_count', 512)        # Nombre de bins en distance
        self.declare_parameter('bearing_count', 256)      # Nombre de bins en azimut
        self.declare_parameter('bearing_angle', 140.0)    # Ouverture totale du sonar (degrés)
        self.declare_parameter('min_range', 1.0)          # Distance minimale détectable (m)
        self.declare_parameter('max_range', 40.0)         # Portée maximale (m)
        
        # --- Paramètres de la cage simulée ---
        self.declare_parameter('cage_distance', 8.0)      # Distance initiale cage-ROV (m)
        self.declare_parameter('cage_width', 1.0)         # Écartement entre les poteaux (m)
        self.declare_parameter('cmd_vel_topic', '/bluerov/cmd_vel')  # Topic des commandes
        
        # --- Paramètres de génération du bruit ---
        # Le bruit est généré par dispersion de points aléatoires puis floutage
        self.declare_parameter('noise_level', 20.0)       # Niveau de bruit de fond (0-100)
        self.declare_parameter('noise_point_count', 200)  # Nombre de points de bruit dispersés
        self.declare_parameter('noise_i_min', 5.0)        # Intensité min des points de bruit
        self.declare_parameter('noise_i_max', 120.0)      # Intensité max des points de bruit
        self.declare_parameter('noise_blur_sigma', 1.2)   # Écart-type du flou gaussien
        self.declare_parameter('noise_blur_iters', 1)     # Nombre d'itérations de flou
        
        # --- Intensités des poteaux de cage (0-255) ---
        self.declare_parameter('post_intensity_left', 230.0)
        self.declare_parameter('post_intensity_right', 230.0)
        
        # --- Paramètres de filtrage d'image ---
        # Ces filtres sont appliqués séquentiellement avant publication
        self.declare_parameter('enable_median', True)      # Filtre médian anti-bruit sel/poivre
        self.declare_parameter('median_kernel', 3)         # Taille du noyau médian (impair)
        self.declare_parameter('enable_gaussian', True)    # Lissage gaussien
        self.declare_parameter('gaussian_sigma', 1.5)      # Écart-type du gaussien
        self.declare_parameter('enable_contrast', True)    # Amélioration du contraste (CLAHE)
        self.declare_parameter('contrast_clip', 2.0)       # Limite de clip CLAHE
        self.declare_parameter('enable_range_comp', True)  # Compensation de l'atténuation
        self.declare_parameter('range_comp_alpha', 0.0001) # Coefficient d'atténuation
        
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
        
        # ===================================================================
        # PRÉ-CALCUL DES GRILLES POLAIRES (OPTIMISATION)
        # Ces tableaux sont calculés une seule fois au démarrage pour éviter
        # des recalculs coûteux à chaque frame.
        # ===================================================================
        self.ranges = np.linspace(self.min_range, self.max_range, self.range_count)
        half_angle = self.bearing_angle / 2.0
        # Conversion en radians: angle va de -half_angle à +half_angle
        self.bearings = np.linspace(-half_angle * np.pi/180, half_angle * np.pi/180, self.bearing_count)
        self.half_angle_rad = half_angle * np.pi / 180.0
        
        # ===================================================================
        # ÉTAT DE LA SIMULATION (Position relative cage/ROV)
        # ===================================================================
        # CONVENTION: La cage est FIXE dans le repère monde.
        # Le ROV se déplace, donc on stocke la position de la cage
        # vue depuis le repère du ROV (qui change à chaque instant).
        #
        # Repère ROV:
        #   - X: axe latéral (positif = droite)
        #   - Y: axe frontal (positif = devant)
        #   - θ: orientation de la cage par rapport au ROV
        # ===================================================================
        self.cage_x = 0.0                  # Position latérale de la cage (m)
        self.cage_y = cage_distance_init   # Distance frontale cage-ROV (m)
        self.cage_theta = 0.0              # Orientation relative (rad)
        
        # Vitesses du ROV reçues via /cmd_vel
        self.cmd_vx = 0.0      # Sway - vitesse latérale (m/s)
        self.cmd_vy = 0.0      # Surge - vitesse frontale (m/s)  
        self.cmd_omega = 0.0   # Yaw rate - vitesse angulaire (rad/s)
        
        # Horodatage pour calcul du dt dans la simulation physique
        self.last_update_time = time.time()
        
        # ===================================================================
        # CONFIGURATION ROS2: CALLBACKS, PUBLISHERS, TIMERS
        # ===================================================================
        
        # Callback pour reconfiguration dynamique des paramètres
        self.add_on_set_parameters_callback(self._on_set_parameters)
        
        # Subscription aux commandes de vitesse du ROV
        self.cmd_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_callback,
            10
        )
        
        # Publisher vers le topic de données sonar brutes (filtrées)
        self.publisher_ = self.create_publisher(Frame, '/docking/sonar/raw', 10)
        
        # Timer physique à 50 Hz pour une simulation fluide
        # NOTE: Fréquence plus élevée que la publication pour précision physique
        self.physics_timer = self.create_timer(1.0 / 50.0, self.update_physics)
        
        # Timer de publication sonar (fréquence configurable)
        self.publish_timer = self.create_timer(1.0 / rate, self.publish_frame)
        
        self.get_logger().info(
            f'Sonar mock démarré: {rate} Hz, cage initiale @ ({self.cage_x:.2f}, {self.cage_y:.2f})m'
        )
        self.get_logger().info(f'Écoute des commandes sur: {cmd_vel_topic}')
    
    def cmd_callback(self, msg: Twist):
        """
        Callback de réception des commandes de vitesse du ROV.
        
        CONVENTION TWIST (Standard ROS2 pour véhicules marins):
            - linear.x  : Surge  - Vitesse avant/arrière (m/s)
            - linear.y  : Sway   - Vitesse latérale gauche/droite (m/s)
            - angular.z : Yaw    - Vitesse de rotation (rad/s)
        
        NOTE IMPORTANTE: Le mapping ici inverse X et Y car:
            - Dans notre repère interne, X = latéral, Y = frontal
            - Dans la convention Twist, X = frontal, Y = latéral
        
        Args:
            msg: Message Twist contenant les consignes de vitesse
        """
        self.cmd_vx = msg.linear.y   # Sway → déplacement latéral interne
        self.cmd_vy = msg.linear.x   # Surge → déplacement frontal interne
        self.cmd_omega = msg.angular.z
        
        # Log pour debug (première fois seulement pour ne pas spammer)
        if not hasattr(self, '_cmd_received'):
            self._cmd_received = True
            self.get_logger().info(
                f'Première commande reçue: vx={self.cmd_vx:.2f}, vy={self.cmd_vy:.2f}, ω={self.cmd_omega:.2f}'
            )
    
    def update_physics(self):
        """
        Mise à jour de la position relative de la cage (cinématique inverse).
        
        PRINCIPE PHYSIQUE:
        ===================
        La cage est FIXE dans le repère monde. Le ROV se déplace.
        Pour simuler cette situation, on applique le mouvement INVERSE
        du ROV à la position de la cage dans le repère ROV.
        
        Si le ROV avance de +Δy (surge positif), la cage RECULE de -Δy
        dans le repère du ROV.
        
        FORMULES APPLIQUÉES:
        =====================
        1. Calcul du pas de temps: Δt = t_courant - t_précédent
        
        2. Translation (mouvement inverse):
           Δx_cage = +vx · Δt  (latéral: même sens car sway positif = droite)
           Δy_cage = -vy · Δt  (frontal: inversé car surge positif = avancer)
        
        3. Rotation autour de l'origine (position ROV):
           [θ inversé car si le ROV tourne à droite, la cage semble tourner à gauche]
           
           Matrice de rotation 2D pour angle -Δθ:
           | cos(-Δθ)  -sin(-Δθ) |   | x' |
           | sin(-Δθ)   cos(-Δθ) | × | y' |
        
        NOTE CRITIQUE: L'ordre des opérations (translation puis rotation)
        est important pour la cohérence physique.
        """
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Étape 1: Déplacements inverses (mouvement relatif cage/ROV)
        delta_x = self.cmd_vx * dt       # Latéral: même signe
        delta_y = -self.cmd_vy * dt      # Frontal: signe inversé
        delta_theta = self.cmd_omega * dt
        
        # Étape 2: Translation de la cage
        cage_x_translated = self.cage_x + delta_x
        cage_y_translated = self.cage_y + delta_y
        
        # Étape 3: Rotation inverse autour de l'origine
        # NOTE: On applique -delta_theta car la rotation du ROV 
        # fait tourner la cage dans le sens opposé vu depuis le ROV
        cos_theta = np.cos(-delta_theta)
        sin_theta = np.sin(-delta_theta)
        
        # Application de la matrice de rotation 2D
        self.cage_x = cos_theta * cage_x_translated - sin_theta * cage_y_translated
        self.cage_y = sin_theta * cage_x_translated + cos_theta * cage_y_translated
        
        # Mise à jour de l'orientation relative de la cage
        self.cage_theta += delta_theta
        
        # Log périodique (toutes les 2 secondes environ)
        if np.random.rand() < 0.02:  # ~1% de chance à 50 Hz
            self.get_logger().info(
                f'Cage relative: x={self.cage_x:.2f}m, y={self.cage_y:.2f}m, '
                f'θ={np.rad2deg(self.cage_theta):.1f}°'
            )

    def _on_set_parameters(self, params):
        """
        Callback de reconfiguration dynamique des paramètres.
        
        Permet de modifier les paramètres à chaud via:
            ros2 param set /sonar_mock <param_name> <value>
        
        PARAMÈTRES GÉRANT DES GRILLES PRÉ-CALCULÉES:
        Les paramètres min_range, max_range et bearing_angle nécessitent
        une mise à jour des grilles pré-calculées (self.ranges, self.bearings).
        
        Args:
            params: Liste des paramètres modifiés
            
        Returns:
            SetParametersResult indiquant le succès/échec de la mise à jour
        """
        successful = True

        # Stockage temporaire pour validation avant application
        new_min_range = self.min_range
        new_max_range = self.max_range
        new_bearing_angle = self.bearing_angle

        for p in params:
            name = p.name
            try:
                # --- Paramètres de bruit ---
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
                # --- Paramètres des poteaux ---
                elif name == 'post_intensity_left':
                    self.post_intensity_left = float(p.value)
                elif name == 'post_intensity_right':
                    self.post_intensity_right = float(p.value)
                elif name == 'noise_level':
                    self.noise_level = float(p.value)
                # --- Paramètres géométriques (nécessitent recalcul des grilles) ---
                elif name == 'min_range':
                    new_min_range = float(p.value)
                elif name == 'max_range':
                    new_max_range = float(p.value)
                elif name == 'bearing_angle':
                    new_bearing_angle = float(p.value)
            except Exception:
                successful = False

        # Validation et application des changements de portée
        if new_min_range <= 0 or new_min_range >= new_max_range:
            successful = False
        else:
            self.min_range = new_min_range
            self.max_range = new_max_range
            # Recalcul de la grille des distances
            self.ranges = np.linspace(self.min_range, self.max_range, self.range_count)

        # Validation et application des changements d'angle
        if new_bearing_angle <= 0:
            successful = False
        else:
            self.bearing_angle = new_bearing_angle
            half_angle = self.bearing_angle / 2.0
            # Recalcul de la grille des azimuts
            self.bearings = np.linspace(-half_angle * np.pi / 180, half_angle * np.pi / 180, self.bearing_count)
            self.half_angle_rad = half_angle * np.pi / 180.0

        return SetParametersResult(successful=successful)
    
    def generate_synthetic_frame(self) -> np.ndarray:
        """
        Génère une frame sonar synthétique avec bruit réaliste.
        
        PROCESSUS DE GÉNÉRATION:
        ==========================
        1. Création d'une image de fond à faible intensité
        2. Dispersion de points aléatoires (simule les particules en suspension)
        3. Ajout de bruit gaussien léger pour le speckle
        4. Application d'un flou gaussien pour rendre le bruit naturel
        
        NOTE: Les poteaux de cage sont ajoutés APRÈS le filtrage (dans publish_frame)
        pour éviter qu'ils ne soient atténués par les filtres.
        
        Returns:
            np.ndarray: Image sonar (bearing_count x range_count), valeurs 0-255
        """
        # Calcul du niveau de fond basé sur le paramètre de bruit
        std = max(1.0, float(self.noise_level))
        mean_bg = std * 0.2
        
        # Initialisation de l'image avec intensité de fond uniforme
        intensities = np.full((self.bearing_count, self.range_count), mean_bg, dtype=np.float32)

        # --- Dispersion de points de bruit aléatoires ---
        # Simule les réflexions sur particules en suspension dans l'eau
        if self.noise_point_count > 0 and (self.noise_i_max > self.noise_i_min):
            n = max(0, int(self.noise_point_count))
            # Positions aléatoires dans l'image
            bear_idx = np.random.randint(0, self.bearing_count, size=n)
            range_idx = np.random.randint(0, self.range_count, size=n)
            # Intensités aléatoires dans la plage configurée
            vals = np.random.uniform(self.noise_i_min, self.noise_i_max, size=n)
            intensities[bear_idx, range_idx] = vals

        # --- Ajout de bruit gaussien de fond (speckle) ---
        if self.noise_level > 0:
            intensities += np.random.normal(
                loc=0.0, 
                scale=self.noise_level * 0.2,
                size=intensities.shape
            ).astype(np.float32)

        # --- Application du flou gaussien ---
        # Lisse les points pour un aspect plus naturel/réaliste
        for _ in range(max(1, int(self.noise_blur_iters))):
            if float(self.noise_blur_sigma) > 0.0:
                try:
                    intensities = gaussian_filter(
                        intensities, 
                        float(self.noise_blur_sigma)
                    ).astype(np.float32)
                except Exception:
                    # Si le filtre échoue, on conserve l'image telle quelle
                    pass

        # Clippage et conversion en uint8 (0-255)
        intensities = np.clip(intensities, 0, 255).astype(np.uint8)

        return intensities
    
    def publish_frame(self):
        """
        Génère, filtre et publie une frame sonar sur /docking/sonar/raw.
        
        PIPELINE DE TRAITEMENT:
        ========================
        1. Génération de l'image synthétique brute (bruit uniquement)
        2. Application séquentielle des filtres activés:
           a. Filtre médian (suppression bruit impulsionnel)
           b. Filtre gaussien (lissage)
           c. Compensation de portée (atténuation avec distance)
           d. Amélioration de contraste (CLAHE)
        3. Ajout des poteaux de cage (APRÈS filtrage pour préserver leur intensité)
        4. Publication du message Frame
        """
        timestamp = self.get_clock().now().to_msg()
        
        # Étape 1: Génération image synthétique brute
        intensities = self.generate_synthetic_frame()
        
        # Étape 2: Application des filtres (travail in-place quand possible)
        filtered = intensities
        
        # --- 2a. Filtre médian: supprime le bruit "sel et poivre" ---
        if self.get_parameter('enable_median').value:
            kernel = self.get_parameter('median_kernel').value
            filtered = median_filter(filtered, kernel)
        
        # --- 2b. Filtre gaussien: lissage général ---
        if self.get_parameter('enable_gaussian').value:
            sigma = self.get_parameter('gaussian_sigma').value
            filtered = gaussian_filter(filtered, sigma)
        
        # --- 2c. Compensation de portée: corrige l'atténuation acoustique ---
        # FORMULE: I_corrigé = I_brut * exp(α * range)
        # L'atténuation augmente avec la distance, on amplifie donc les échos lointains
        if self.get_parameter('enable_range_comp').value:
            alpha = self.get_parameter('range_comp_alpha').value
            filtered = range_compensation(filtered, self.ranges, alpha)
        
        # --- 2d. Amélioration de contraste (CLAHE) ---
        # Contrast Limited Adaptive Histogram Equalization
        if self.get_parameter('enable_contrast').value:
            clip = self.get_parameter('contrast_clip').value
            filtered = contrast_enhancement(filtered, clip)

        # Étape 3: Ajout des poteaux de cage APRÈS les filtres
        # NOTE CRITIQUE: On ajoute les poteaux après le filtrage pour qu'ils
        # conservent leur intensité maximale et ne soient pas atténués
        try:
            self.add_posts(filtered)
        except Exception:
            # En cas d'échec, on continue sans bloquer le noeud
            pass

        # Clippage final et conversion en uint8
        filtered = np.clip(filtered, 0, 255).astype(np.uint8)

        # Étape 4: Construction et publication du message Frame
        # NOTE: ravel() + tolist() est plus performant que flatten().tolist()
        intensities_list = filtered.ravel().tolist()
        
        # Construction du message Frame (format standard docking_msgs)
        msg = Frame()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'sonar_link'  # Repère TF du sonar
        
        # Métadonnées géométriques de l'image polaire
        msg.range_count = self.range_count
        msg.bearing_count = self.bearing_count
        msg.range_resolution = (self.max_range - self.min_range) / self.range_count
        msg.bearing_resolution = (self.bearing_angle * np.pi / 180.0) / self.bearing_count
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        
        # Données d'intensité aplaties (format: bearing-major)
        msg.intensities = intensities_list
        
        # Paramètres acoustiques (valeurs par défaut pour simulation)
        msg.sound_speed = 1500.0  # Vitesse du son dans l'eau (m/s)
        msg.gain = 50             # Gain simulé (%)
        
        self.publisher_.publish(msg)


    def add_posts(self, arr: np.ndarray):
        """
        Dessine les deux poteaux de cage dans l'image sonar.
        
        GÉOMÉTRIE DES POTEAUX:
        =======================
        Les poteaux sont positionnés symétriquement par rapport au centre
        de la cage, à une distance ±(cage_width/2) du centre.
        
        TRANSFORMATION COORDONNÉES:
        ============================
        1. Position locale du poteau dans le repère cage:
           offset_local = (±cage_width/2, 0)
        
        2. Rotation par l'orientation de la cage (cage_theta):
           | cos(θ)  -sin(θ) |   | offset_x |
           | sin(θ)   cos(θ) | × | offset_y |
        
        3. Translation par la position du centre de cage:
           post_world = cage_center + offset_rotated
        
        4. Conversion en coordonnées polaires (range, bearing):
           range = √(x² + y²)
           bearing = atan2(x, y)  # NOTE: atan2(x,y) car Y est l'axe frontal
        
        5. Conversion en indices de pixels dans l'image sonar
        
        DIAMÈTRE PHYSIQUE:
        ==================
        Les poteaux ont un diamètre de 30 cm, ce qui détermine la taille
        du patch dessiné dans l'image.
        
        Args:
            arr: Image sonar à modifier (in-place)
        
        Returns:
            L'image modifiée (même référence si modification in-place réussie)
        """
        # Conversion en float32 pour éviter les problèmes de dépassement uint8
        if arr.dtype != np.float32 and arr.dtype != np.float64:
            work = arr.astype(np.float32)
        else:
            work = arr

        # Géométrie de la cage
        cage_half_width = self.cage_width / 2.0
        cos_theta = np.cos(self.cage_theta)
        sin_theta = np.sin(self.cage_theta)
        
        # Offsets des deux poteaux (gauche et droite)
        post_offsets = [(-cage_half_width, 0.0), (cage_half_width, 0.0)]

        for i, offset in enumerate(post_offsets):
            offset_x, offset_y = offset
            
            # --- Transformation repère cage → repère ROV ---
            # Application de la rotation puis translation
            post_x = self.cage_x + (offset_x * cos_theta - offset_y * sin_theta)
            post_y = self.cage_y + (offset_x * sin_theta + offset_y * cos_theta)

            # --- Conversion en coordonnées polaires ---
            # NOTE CRITIQUE: atan2(x, y) et non atan2(y, x) car Y est l'axe frontal
            post_range = np.sqrt(post_x**2 + post_y**2)
            post_bearing = np.arctan2(post_x, post_y)

            # Vérification que le poteau est dans le champ de vue
            if not (self.min_range <= post_range <= self.max_range):
                continue
            if abs(post_bearing) > self.half_angle_rad:
                continue

            # --- Conversion en indices de pixels ---
            bearing_idx = np.searchsorted(self.bearings, post_bearing)
            range_idx = int((post_range - self.min_range) / (self.max_range - self.min_range) * (self.range_count - 1))
            bearing_idx = np.clip(bearing_idx, 0, self.bearing_count - 1)
            range_idx = np.clip(range_idx, 0, self.range_count - 1)

            # Intensité du poteau (configurable gauche/droite)
            intensity_val = self.post_intensity_left if i == 0 else self.post_intensity_right

            # --- Calcul de la taille du patch (diamètre physique = 30 cm) ---
            diameter_m = 0.30
            radius_m = diameter_m / 2.0
            range_res = (self.max_range - self.min_range) / max(self.range_count, 1)
            bearing_res = (self.bearing_angle * np.pi / 180.0) / max(self.bearing_count, 1)

            # Taille en pixels (dépend de la résolution et de la distance)
            range_width = max(1, int(np.ceil(radius_m / max(range_res, 1e-6))))
            bearing_width = max(
                1,
                int(np.ceil(np.arctan2(radius_m, max(post_range, 1e-6)) / max(bearing_res, 1e-6))),
            )

            # Limites du patch dans l'image
            b_min = max(0, bearing_idx - bearing_width)
            b_max = min(self.bearing_count, bearing_idx + bearing_width + 1)
            r_min = max(0, range_idx - range_width)
            r_max = min(self.range_count, range_idx + range_width + 1)

            # Dessin du patch rectangulaire
            work[b_min:b_max, r_min:r_max] = float(np.clip(intensity_val, 0, 255))

        # Réécriture dans le tableau original si possible
        if work is not arr:
            try:
                arr[:] = work.astype(arr.dtype)
            except Exception:
                return work
        return arr


def main(args=None):
    """
    Point d'entrée du nœud sonar_mock.
    
    Usage:
        ros2 run sonar sonar_mock
        ros2 run sonar sonar_mock --ros-args -p cage_distance:=10.0
    """
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
