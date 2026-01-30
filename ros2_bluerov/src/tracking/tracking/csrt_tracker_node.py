"""
Nœud de tracking CSRT pour sonar.
Deux modes disponibles:
1. Rectangle CSRT classique: dessiner un rectangle pour tracker avec CSRT
2. 4 corners avec features: pointer 4 coins et tracker les features au sein du rectangle
"""

import numpy as np
import rclpy
from rclpy.node import Node
from docking_msgs.msg import FrameCartesian, BBoxSelection, TrackedObject, RotatedBBoxInit

try:
    import cv2
except ImportError:
    cv2 = None


class CSRTTrackerNode(Node):
    """Tracker CSRT pour suivre la cage dans les images sonar cartésiennes."""
    
    def __init__(self):
        super().__init__('csrt_tracker_node')
        
        # ========== PARAMÈTRES ==========
        # Activation
        self.declare_parameter('enable_tracking', True)
        
        # Paramètres optical flow pour tracking rotatif
        self.declare_parameter('lk_win_size', 31)  # Taille fenêtre Lucas-Kanade (augmentée pour sonar)
        self.declare_parameter('lk_max_level', 3)  # Niveaux pyramide (plus de niveaux pour robustesse)
        self.declare_parameter('lk_max_iterations', 30)  # Itérations max
        self.declare_parameter('lk_epsilon', 0.01)  # Critère de convergence
        
        # Paramètres pour la détection de features
        self.declare_parameter('max_corners', 50)  # Nombre max de corners à détecter
        self.declare_parameter('quality_level', 0.01)  # Qualité minimale des corners
        self.declare_parameter('min_distance', 10)  # Distance min entre corners
        
        # Paramètre de continuité de l'angle
        self.declare_parameter('max_angle_change_per_frame', 0.2)  # radians (~11°) max entre 2 frames
        
        # Contrainte de mouvement pour éviter la dérive
        self.declare_parameter('max_displacement_per_frame', 20)  # pixels max entre 2 frames (très contraint)
        
        # Paramètres du tracker CSRT optimisés pour sonar (learning rates très faibles)
        self.declare_parameter('use_hog', True)
        self.declare_parameter('use_color_names', False)  # False pour grayscale
        self.declare_parameter('use_gray', True)
        self.declare_parameter('use_rgb', False)
        self.declare_parameter('use_channel_weights', True)
        self.declare_parameter('use_segmentation', False)  # Désactivé pour images bruitées
        self.declare_parameter('window_function', 'hann')  # hann, cheb, kaiser
        self.declare_parameter('kaiser_alpha', 3.75)
        self.declare_parameter('cheb_attenuation', 45.0)
        self.declare_parameter('template_size', 200.0)
        self.declare_parameter('gsl_sigma', 1.0)
        self.declare_parameter('hog_orientations', 9)
        self.declare_parameter('hog_clip', 0.2)
        self.declare_parameter('padding', 3.0)
        self.declare_parameter('filter_lr', 0.005)  # Très réduit pour stabilité maximale
        self.declare_parameter('weights_lr', 0.005)  # Très réduit pour stabilité maximale
        self.declare_parameter('num_hog_channels_used', 18)
        self.declare_parameter('admm_iterations', 4)
        self.declare_parameter('histogram_bins', 16)
        self.declare_parameter('histogram_lr', 0.01)  # Très réduit pour stabilité
        self.declare_parameter('background_ratio', 2)
        self.declare_parameter('number_of_scales', 33)
        self.declare_parameter('scale_sigma_factor', 0.25)
        self.declare_parameter('scale_model_max_area', 512.0)
        self.declare_parameter('scale_lr', 0.01)  # Très réduit pour garder l'échelle fixe
        self.declare_parameter('scale_step', 1.01)  # Pas d'échelle plus petit
        self.declare_parameter('psr_threshold', 0.02)  # Très permissif pour sonar bruité
        
        # État du tracker
        self.tracker = None
        self.is_initialized = False
        self.is_tracking = False
        self.last_bbox = None  # (x, y, w, h) en pixels
        self.current_resolution = None  # mètres par pixel
        self.current_frame_info = None  # Info du dernier frame
        
        # État du tracking rotatif (corners)
        self.tracking_mode = 'bbox'  # 'bbox' ou 'rotated'
        self.corners = None  # 4 corners définissant la bbox (shape: 4x2)
        self.tracked_features = None  # Features réellement trackées (Nx1x2)
        self.entry_angle_offset = None  # Angle du côté d'entrée par rapport à l'axe de la bbox (fixe)
        self.last_gray = None  # Dernière frame grayscale pour optical flow
        self.bbox_angle = 0.0  # Angle de rotation actuel
        self.previous_angle = 0.0  # Angle précédent pour filtre de continuité
        self.initial_bbox_angle = 0.0  # Angle initial de la bbox (au moment de l'init)
        self.initial_width = 0.0  # Largeur initiale de la bbox (fixée)
        self.initial_height = 0.0  # Hauteur initiale de la bbox (fixée)
        self.initial_corners_m = None  # Coins initiaux en mètres (P1, P2, P3, P4) - JAMAIS modifiés!
        
        # Subscriptions
        self.frame_sub = self.create_subscription(
            FrameCartesian,
            '/docking/sonar/cartesian_filtered',
            self.frame_callback,
            10
        )
        
        self.bbox_sub = self.create_subscription(
            BBoxSelection,
            '/docking/sonar/bbox_selection',
            self.bbox_callback,
            10
        )
        
        self.rotated_bbox_sub = self.create_subscription(
            RotatedBBoxInit,
            '/docking/sonar/rotated_bbox_init',
            self.rotated_bbox_callback,
            10
        )
        
        # Publisher
        self.tracked_pub = self.create_publisher(
            TrackedObject,
            '/docking/tracking/tracked_object',
            10
        )
        
        self.get_logger().info('CSRT Tracker node démarré')
        
        if cv2 is None:
            self.get_logger().error('OpenCV non disponible! Le tracking ne fonctionnera pas.')
    
    def _create_tracker(self):
        """Crée un nouveau tracker CSRT avec les paramètres configurés."""
        if cv2 is None:
            return None
        
        # Tracker KCF - bon compromis vitesse/précision, moins sensible au bruit
        #self.get_logger().info('Création tracker KCF')
        #return cv2.legacy.TrackerKCF_create()

        
        # Version CSRT (à réactiver si besoin)
        # Créer les paramètres CSRT
        self.get_logger().info('Création tracker CSRT avec paramètres personnalisés')
        params = cv2.TrackerCSRT_Params()
        
        params.use_hog = self.get_parameter('use_hog').value
        params.use_color_names = self.get_parameter('use_color_names').value
        params.use_gray = self.get_parameter('use_gray').value
        params.use_rgb = self.get_parameter('use_rgb').value
        params.use_channel_weights = self.get_parameter('use_channel_weights').value
        params.use_segmentation = self.get_parameter('use_segmentation').value
        params.window_function = self.get_parameter('window_function').value
        params.kaiser_alpha = float(self.get_parameter('kaiser_alpha').value)
        params.cheb_attenuation = float(self.get_parameter('cheb_attenuation').value)
        params.template_size = float(self.get_parameter('template_size').value)
        params.gsl_sigma = float(self.get_parameter('gsl_sigma').value)
        params.hog_orientations = int(self.get_parameter('hog_orientations').value)
        params.hog_clip = float(self.get_parameter('hog_clip').value)
        params.padding = float(self.get_parameter('padding').value)
        params.filter_lr = float(self.get_parameter('filter_lr').value)
        params.weights_lr = float(self.get_parameter('weights_lr').value)
        params.num_hog_channels_used = int(self.get_parameter('num_hog_channels_used').value)
        params.admm_iterations = int(self.get_parameter('admm_iterations').value)
        params.histogram_bins = int(self.get_parameter('histogram_bins').value)
        params.histogram_lr = float(self.get_parameter('histogram_lr').value)
        params.background_ratio = int(self.get_parameter('background_ratio').value)
        params.number_of_scales = int(self.get_parameter('number_of_scales').value)
        params.scale_sigma_factor = float(self.get_parameter('scale_sigma_factor').value)
        params.scale_model_max_area = float(self.get_parameter('scale_model_max_area').value)
        params.scale_lr = float(self.get_parameter('scale_lr').value)
        params.scale_step = float(self.get_parameter('scale_step').value)
        params.psr_threshold = float(self.get_parameter('psr_threshold').value)
        
        return cv2.TrackerCSRT_create(params)
        
    
    def _meters_to_pixels(self, x_m, y_m, frame_msg):
        """Convertit des coordonnées en mètres vers des pixels."""
        resolution = frame_msg.resolution
        max_range = frame_msg.max_range
        
        # L'image a pour origine (0,0) en haut à gauche
        # x_m = 0 correspond au centre de l'image (origin_x)
        # y_m = 0 correspond au ROV (origin_y = 0, bas de l'image)
        
        # Conversion
        # x_px = (x_m / resolution) + origin_x
        # y_px = height - (y_m / resolution)  # Inverser car y augmente vers le haut
        
        x_px = int((x_m / resolution) + frame_msg.origin_x)
        y_px = int(frame_msg.height - (y_m / resolution))
        
        return x_px, y_px
    
    def _pixels_to_meters(self, x_px, y_px, frame_msg):
        """Convertit des coordonnées en pixels vers des mètres."""
        resolution = frame_msg.resolution
        
        x_m = (x_px - frame_msg.origin_x) * resolution
        y_m = (frame_msg.height - y_px) * resolution
        
        return x_m, y_m
    
    def bbox_callback(self, msg: BBoxSelection):
        """Initialise le tracker avec une bbox manuelle."""
        if not self.get_parameter('enable_tracking').value:
            return
        
        if cv2 is None:
            self.get_logger().error('OpenCV non disponible')
            return
        
        if not msg.is_valid:
            return
        
        # Stocker la bbox pour initialisation lors du prochain frame
        self._pending_init = ('bbox', msg.x, msg.y, msg.width, msg.height)
        self.get_logger().info(f'BBox reçue: ({msg.x}, {msg.y}, {msg.width}, {msg.height}) - initialisation en attente')
    
    def rotated_bbox_callback(self, msg: RotatedBBoxInit):
        """Initialise le tracker rotatif avec 4 coins (ou 3 si p4 non fourni)."""
        if not self.get_parameter('enable_tracking').value:
            return
        
        if cv2 is None:
            self.get_logger().error('OpenCV non disponible')
            return
        
        if not msg.is_valid:
            return
        
        # Vérifier si P4 est fourni ou doit être calculé
        if abs(msg.p4_x) < 0.001 and abs(msg.p4_y) < 0.001:
            # P4 non fourni - calculer automatiquement (ancien mode 3 points)
            self._pending_init = ('rotated', msg.p1_x, msg.p1_y, msg.p2_x, msg.p2_y, msg.p3_x, msg.p3_y)
            self.get_logger().info(
                f'Rotated BBox reçue (3 pts): P1({msg.p1_x:.2f}, {msg.p1_y:.2f}), '
                f'P2({msg.p2_x:.2f}, {msg.p2_y:.2f}), P3({msg.p3_x:.2f}, {msg.p3_y:.2f}) - '
                f'initialisation en attente'
            )
        else:
            # P4 fourni - utiliser les 4 coins directement
            self._pending_init = ('rotated4', msg.p1_x, msg.p1_y, msg.p2_x, msg.p2_y, 
                                 msg.p3_x, msg.p3_y, msg.p4_x, msg.p4_y)
            self.get_logger().info(
                f'Rotated BBox reçue (4 coins): '
                f'P1({msg.p1_x:.2f}, {msg.p1_y:.2f}), P2({msg.p2_x:.2f}, {msg.p2_y:.2f}), '
                f'P3({msg.p3_x:.2f}, {msg.p3_y:.2f}), P4({msg.p4_x:.2f}, {msg.p4_y:.2f}) - '
                f'initialisation en attente'
            )
    
    
    def _init_corners_from_4points(self, p1_m, p2_m, p3_m, p4_m, frame_msg):
        """
        Initialise les 4 corners en pixels à partir de 4 points en mètres.
        Garde l'ordre des clics pour préserver le côté d'entrée entre P1 et P2.
        Retourne: corners array (4x1x2) en pixels
        """
        # IMPORTANT: Stocker les coordonnées métriques des coins AVANT toute transformation
        # Cela nous permettra de toujours retrouver le côté d'entrée P1→P2
        self.initial_corners_m = np.array([
            [p1_m[0], p1_m[1]],  # P1
            [p2_m[0], p2_m[1]],  # P2
            [p3_m[0], p3_m[1]],  # P3
            [p4_m[0], p4_m[1]]   # P4
        ])
        
        # Convertir les points en pixels
        p1_px = np.array(self._meters_to_pixels(p1_m[0], p1_m[1], frame_msg))
        p2_px = np.array(self._meters_to_pixels(p2_m[0], p2_m[1], frame_msg))
        p3_px = np.array(self._meters_to_pixels(p3_m[0], p3_m[1], frame_msg))
        p4_px = np.array(self._meters_to_pixels(p4_m[0], p4_m[1], frame_msg))
        
        # IMPORTANT: Garder l'ordre des clics directement
        # P1→P2→P3→P4 définit le quadrilatère avec P1→P2 comme côté d'entrée
        points = np.array([p1_px, p2_px, p3_px, p4_px])
        
        # Calculer l'angle du côté d'entrée (p1 → p2)
        entry_vector = p2_px - p1_px
        entry_angle = np.arctan2(entry_vector[1], entry_vector[0])
        
        self.get_logger().info(
            f'Côté d\'entrée: angle = {np.degrees(entry_angle):.1f}° '
            f'(entre 1er et 2ème clic: p1-p2)'
        )
        
        # Calculer le centroid et l'angle moyen de la bbox
        centroid = np.mean(points, axis=0)
        
        # Calculer l'angle moyen de la bbox (moyenne des vecteurs vers les coins)
        angles_to_corners = np.arctan2(points[:, 1] - centroid[1], 
                                       points[:, 0] - centroid[0])
        bbox_angle_rad = np.mean(angles_to_corners)
        
        # Mémoriser l'offset: l'angle du côté P1→P2 est le côté d'entrée (index 0→1)
        # Puisqu'on garde l'ordre des clics, le côté 0→1 sera toujours l'entrée
        # L'offset est simplement 0 car le côté 0→1 correspond au 1er côté
        self.entry_angle_offset = 0.0  # Le côté d'entrée est toujours entre corner[0] et corner[1]
        
        self.get_logger().info(
            f'Bbox: angle moyen = {np.degrees(bbox_angle_rad):.1f}°, '
            f'offset entrée = {np.degrees(self.entry_angle_offset):.1f}° (côté 0→1 fixe)'
        )
        
        # Créer l'array de corners (4x1x2 pour OpenCV) DANS L'ORDRE DES CLICS
        corners = points.reshape(4, 1, 2).astype(np.float32)
        
        # Flipper horizontalement car l'image est flippée
        corners[:, 0, 0] = frame_msg.width - corners[:, 0, 0]
        
        return corners
    
    def _detect_features_in_bbox(self, img_gray, corners):
        """
        Détecte automatiquement des features robustes à l'intérieur de la bbox.
        Retourne: features array (Nx1x2) de points à tracker
        """
        # Créer un masque de la région d'intérêt
        mask = np.zeros_like(img_gray, dtype=np.uint8)
        corners_int = corners.reshape(4, 2).astype(np.int32)
        cv2.fillPoly(mask, [corners_int], 255)
        
        # Détecter des good features dans cette région
        feature_params = dict(
            maxCorners=self.get_parameter('max_corners').value,
            qualityLevel=self.get_parameter('quality_level').value,
            minDistance=self.get_parameter('min_distance').value,
            blockSize=7,
            mask=mask
        )
        
        features = cv2.goodFeaturesToTrack(img_gray, **feature_params)
        
        if features is None or len(features) < 4:
            self.get_logger().warn(f'Seulement {len(features) if features is not None else 0} features détectées - utilisation des corners')
            return corners
        
        self.get_logger().info(f'{len(features)} features détectées pour tracking')
        return features
    
    def _update_features_optical_flow(self, old_gray, new_gray, old_features):
        """
        Met à jour les features trackées avec Lucas-Kanade optical flow.
        Calcule la rotation par transformation rigide au lieu de minAreaRect (plus stable).
        Retourne: (success, new_features, new_corners, rotation_angle)
        """
        if old_features is None or len(old_features) < 4:
            return False, None, None, 0.0
        
        # Paramètres Lucas-Kanade optimisés pour sonar
        lk_params = dict(
            winSize=(self.get_parameter('lk_win_size').value, 
                     self.get_parameter('lk_win_size').value),
            maxLevel=self.get_parameter('lk_max_level').value,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 
                     self.get_parameter('lk_max_iterations').value,
                     self.get_parameter('lk_epsilon').value)
        )
        
        # Calculer optical flow sur toutes les features
        new_features, status, err = cv2.calcOpticalFlowPyrLK(
            old_gray, new_gray, old_features, None, **lk_params
        )
        
        if new_features is None or status is None:
            return False, None, None, 0.0
        
        # Filtrer les features valides (celles qui ont été bien trackées)
        good_old = old_features[status.flatten() == 1]
        good_new = new_features[status.flatten() == 1]
        num_valid = len(good_new)
        
        if num_valid < 4:
            self.get_logger().warn(f'Trop peu de features trackées: {num_valid}/{len(old_features)}')
            return False, None, None, 0.0
        
        # Calculer la rotation moyenne par transformation rigide
        # Ceci est beaucoup plus stable que minAreaRect !
        old_features_2d = good_old.reshape(-1, 2)
        new_features_2d = good_new.reshape(-1, 2)
        
        # Centre de masse des features (avant et après)
        old_centroid = np.mean(old_features_2d, axis=0)
        new_centroid = np.mean(new_features_2d, axis=0)
        
        # Features centrées
        old_centered = old_features_2d - old_centroid
        new_centered = new_features_2d - new_centroid
        
        # Calculer la rotation par SVD (méthode de Kabsch simplifiée)
        # H = sum(old_i * new_i^T)
        H = old_centered.T @ new_centered
        
        # SVD pour trouver la rotation optimale
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Vérifier que c'est bien une rotation (det = 1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # Extraire l'angle de rotation
        rotation_angle = np.arctan2(R[1, 0], R[0, 0])
        
        # IMPORTANT: Ne PAS utiliser minAreaRect car il réordonne les corners!
        # On tracke les corners directement avec optical flow pour préserver l'ordre P1→P2→P3→P4
        # Les corners sont trackés séparément dans frame_callback
        new_corners = None  # Sera mis à jour par _update_corners_optical_flow
        
        self.get_logger().info(f'{num_valid} features trackées, rotation={np.degrees(rotation_angle):.1f}°')
        
        return True, good_new, new_corners, rotation_angle
    
    def _update_corners_optical_flow(self, old_gray, new_gray, old_corners):
        """
        Met à jour les corners avec Lucas-Kanade optical flow.
        Retourne: (success, new_corners)
        """
        if old_corners is None or len(old_corners) != 4:
            return False, None
        
        # Paramètres Lucas-Kanade optimisés pour sonar
        lk_params = dict(
            winSize=(self.get_parameter('lk_win_size').value, 
                     self.get_parameter('lk_win_size').value),
            maxLevel=self.get_parameter('lk_max_level').value,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 
                     self.get_parameter('lk_max_iterations').value,
                     self.get_parameter('lk_epsilon').value)
        )
        
        # Calculer optical flow
        new_corners, status, err = cv2.calcOpticalFlowPyrLK(
            old_gray, new_gray, old_corners, None, **lk_params
        )
        
        # Vérifier que tous les points ont été trackés
        if new_corners is None or status is None:
            return False, None
        
        # Nombre de corners valides
        num_valid = np.sum(status)
        
        if num_valid == 4:
            # Tous les corners trackés - parfait
            return True, new_corners
        
        elif num_valid == 3:
            # 3 corners trackés - on peut reconstruire le 4ème
            self.get_logger().info('Reconstruction du corner perdu (3/4 trackés)')
            
            # Identifier quel corner est perdu
            lost_idx = np.where(status.flatten() == 0)[0][0]
            
            # Les 3 corners valides permettent de recalculer le 4ème
            # P4 = P2 + P3 - P1 (ou équivalent selon le corner perdu)
            valid_corners = new_corners[status.flatten() == 1]
            
            if lost_idx == 0:  # P1 perdu: P1 = P2 + P3 - P4
                p2, p3, p4 = valid_corners
                new_corners[0] = p2 + p3 - p4
            elif lost_idx == 1:  # P2 perdu: P2 = P1 + P4 - P3
                p1, p3, p4 = valid_corners
                new_corners[1] = p1 + p4 - p3
            elif lost_idx == 2:  # P3 perdu: P3 = P1 + P4 - P2
                p1, p2, p4 = valid_corners
                new_corners[2] = p1 + p4 - p2
            else:  # P4 perdu: P4 = P2 + P3 - P1
                p1, p2, p3 = valid_corners
                new_corners[3] = p2 + p3 - p1
            
            return True, new_corners
        
        else:
            # Moins de 3 corners - impossible de reconstruire
            self.get_logger().warn(f'Seulement {num_valid}/4 corners trackés - tracking perdu')
            return False, None
    
    def _init_bbox_tracker(self, bbox, img_bgr, msg):
        """Initialise le tracker CSRT classique avec une bbox."""
        bbox_x, bbox_y, bbox_w, bbox_h = bbox
        
        if bbox_x < 0 or bbox_y < 0 or bbox_x + bbox_w > msg.width or bbox_y + bbox_h > msg.height:
            self.get_logger().error(
                f'BBox hors limites ! Image: {msg.width}x{msg.height}, '
                f'BBox: ({bbox_x}, {bbox_y}, {bbox_w}, {bbox_h})'
            )
            return
        
        if bbox_w < 5 or bbox_h < 5:
            self.get_logger().error(f'BBox trop petite: {bbox_w}x{bbox_h}')
            return
        
        self.get_logger().info(
            f'Initialisation tracker: bbox=({bbox_x}, {bbox_y}, {bbox_w}, {bbox_h}), '
            f'image={msg.width}x{msg.height}'
        )
        
        self.tracker = self._create_tracker()
        if self.tracker is None:
            self.get_logger().error('Impossible de créer le tracker CSRT')
            return
        
        if img_bgr is None or img_bgr.size == 0:
            self.get_logger().error('Image vide reçue !')
            return
        
        bbox_for_opencv = (int(bbox_x), int(bbox_y), int(bbox_w), int(bbox_h))
        
        try:
            success = self.tracker.init(img_bgr, bbox_for_opencv)
            if success is None:
                success = True
        except Exception as e:
            self.get_logger().error(f'Exception lors de l\'initialisation: {type(e).__name__}: {e}')
            success = False
        
        if success:
            self.is_initialized = True
            self.is_tracking = True
            self.last_bbox = bbox
            self.bbox_angle = 0.0
            self.get_logger().info(f'✓ Tracker CSRT initialisé avec succès !')
        else:
            self.get_logger().error('Échec de l\'initialisation du tracker')
            self.is_initialized = False
            self.is_tracking = False
    
    def frame_callback(self, msg: FrameCartesian):
        """Traite une frame cartésienne et met à jour le tracking."""
        if not self.get_parameter('enable_tracking').value:
            return
        
        if cv2 is None:
            return
        
        # Reconstruire l'image
        img = np.array(msg.intensities, dtype=np.uint8).reshape(
            (msg.height, msg.width)
        )
        
        # Améliorer le contraste avec égalisation d'histogramme (aide le tracking)
        img = cv2.equalizeHist(img)
        
        # CORRECTION: Flipper l'image horizontalement pour correspondre à l'affichage
        # Le sonar affiche l'image flippée, donc on doit aussi flipper pour le tracker
        img = cv2.flip(img, 1)  # 1 = flip horizontal
        
        # Convertir en BGR pour OpenCV (CSRT fonctionne mieux avec 3 canaux)
        img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        # S'assurer que l'image est contiguë en mémoire (requis par certains trackers)
        if not img_bgr.flags['C_CONTIGUOUS']:
            img_bgr = np.ascontiguousarray(img_bgr)
        
        self.current_frame_info = msg
        self.current_resolution = msg.resolution
        
        # Vérifier s'il y a une initialisation en attente
        if hasattr(self, '_pending_init') and self._pending_init is not None:
            init_data = self._pending_init
            self._pending_init = None
            
            # Calculer la bounding box selon le mode
            if init_data[0] == 'rotated4':
                # Mode rotatif (4 coins): initialiser corners directement
                p1_m = (init_data[1], init_data[2])
                p2_m = (init_data[3], init_data[4])
                p3_m = (init_data[5], init_data[6])
                p4_m = (init_data[7], init_data[8])
                
                self.corners = self._init_corners_from_4points(p1_m, p2_m, p3_m, p4_m, msg)
                
                # Détecter automatiquement des features robustes dans la bbox
                self.tracked_features = self._detect_features_in_bbox(img, self.corners)
                
                self.last_gray = img.copy()
                self.tracking_mode = 'rotated'
                self.is_initialized = True
                self.is_tracking = True
                
                # Calculer et FIXER les dimensions initiales
                pts = self.corners.reshape(4, 2)
                rect = cv2.minAreaRect(pts)
                (cx, cy), (w, h), angle = rect
                self.initial_width = w * msg.resolution  # En mètres
                self.initial_height = h * msg.resolution  # En mètres
                
                # Bbox pour publication (dimensions en pixels)
                self.last_bbox = (int(cx - w/2), int(cy - h/2), int(w), int(h))
                self.bbox_angle = np.deg2rad(angle)
                self.previous_angle = self.bbox_angle  # Initialiser pour le filtre
                self.initial_bbox_angle = self.bbox_angle  # Stocker l'angle initial!
                
                self.get_logger().info(
                    f'✓ Tracker rotatif initialisé - {len(self.tracked_features)} features - '
                    f'dims: {self.initial_width:.2f}x{self.initial_height:.2f}m, angle={angle:.1f}°'
                )
                
            elif init_data[0] == 'bbox':
                # Mode manuel: bbox fournie directement en pixels
                # CORRECTION: Flipper la bbox horizontalement car l'image est flippée
                bbox_x_original = init_data[1]
                bbox_x_flipped = msg.width - bbox_x_original - init_data[3]  # width - x - w
                bbox = (bbox_x_flipped, init_data[2], init_data[3], init_data[4])
                self.get_logger().info(f'BBox flippée: x {bbox_x_original} -> {bbox_x_flipped}')
                self.tracking_mode = 'bbox'
                self._init_bbox_tracker(bbox, img_bgr, msg)
                
            else:
                self.get_logger().error(f'Mode d\'initialisation inconnu: {init_data[0]}')
                return
        
        # Mettre à jour le tracking selon le mode
        if self.is_initialized:
            if self.tracking_mode == 'rotated':
                # Tracking rotatif par optical flow sur les features UNIQUEMENT
                # Ne plus tracker les corners séparément - trop fragile!
                success, new_features, new_corners, rotation_angle = self._update_features_optical_flow(
                    self.last_gray, img, self.tracked_features
                )
                
                if success:
                    self.tracked_features = new_features
                    self.last_gray = img.copy()
                    self.is_tracking = True
                    
                    # Calculer le centre à partir des features trackées (beaucoup plus robuste)
                    features_2d = new_features.reshape(-1, 2)
                    center_x_px = np.mean(features_2d[:, 0])
                    center_y_px = np.mean(features_2d[:, 1])
                    
                    # Utiliser l'angle de rotation calculé par transformation rigide
                    new_angle = self.previous_angle + rotation_angle
                    
                    # Reconstruire les corners géométriquement (plus robuste que de les tracker)
                    # Dimensions fixes, on reconstruit le rectangle autour du centre
                    w_px = int(self.initial_width / msg.resolution)
                    h_px = int(self.initial_height / msg.resolution)
                    
                    # Corners du rectangle centré sur (0,0)
                    half_w = w_px / 2.0
                    half_h = h_px / 2.0
                    corners_local = np.array([
                        [-half_w, -half_h],  # P1
                        [+half_w, -half_h],  # P2
                        [+half_w, +half_h],  # P3
                        [-half_w, +half_h]   # P4
                    ], dtype=np.float32)
                    
                    # Rotation
                    cos_a = np.cos(new_angle)
                    sin_a = np.sin(new_angle)
                    rot_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
                    corners_rotated = corners_local @ rot_matrix.T
                    
                    # Translation au centre actuel
                    corners_translated = corners_rotated + np.array([center_x_px, center_y_px])
                    
                    # Stocker les nouveaux corners reconstruits
                    self.corners = corners_translated.reshape(4, 1, 2).astype(np.float32)
                    
                    
                    # Filtrer l'angle pour éviter les sauts
                    max_angle_change = self.get_parameter('max_angle_change_per_frame').value
                    
                    if self.previous_angle is not None:
                        angle_diff = new_angle - self.previous_angle
                        
                        # Normaliser la différence dans [-pi, pi]
                        while angle_diff > np.pi:
                            angle_diff -= 2 * np.pi
                        while angle_diff < -np.pi:
                            angle_diff += 2 * np.pi
                        
                        # Limiter la variation
                        if abs(angle_diff) > max_angle_change:
                            self.get_logger().warn(
                                f'Angle change trop brusque: {np.degrees(angle_diff):.1f}° > '
                                f'{np.degrees(max_angle_change):.1f}° - filtré'
                            )
                            # Limiter le changement
                            angle_diff = np.clip(angle_diff, -max_angle_change, max_angle_change)
                            new_angle = self.previous_angle + angle_diff
                    
                    self.bbox_angle = new_angle
                    self.previous_angle = new_angle
                    
                    # Bbox pour publication (coin supérieur gauche approximatif)
                    self.last_bbox = (int(center_x_px - w_px/2), int(center_y_px - h_px/2), w_px, h_px)
                else:
                    self.is_tracking = False
                    self.get_logger().warn('Tracking rotatif perdu!')
                    
            elif self.tracking_mode == 'bbox' and self.tracker is not None:
                # Tracking classique CSRT
                success, bbox = self.tracker.update(img_bgr)
                
                if success:
                    new_bbox = tuple(map(int, bbox))
                    
                    # Contrainte de déplacement maximal pour éviter la dérive
                    if self.last_bbox is not None:
                        max_disp = self.get_parameter('max_displacement_per_frame').value
                        old_x, old_y = self.last_bbox[0], self.last_bbox[1]
                        new_x, new_y = new_bbox[0], new_bbox[1]
                        
                        dx = new_x - old_x
                        dy = new_y - old_y
                        displacement = np.sqrt(dx**2 + dy**2)
                        
                        # Si le déplacement est trop grand, le limiter
                        if displacement > max_disp:
                            self.get_logger().warn(
                                f'Déplacement trop grand ({displacement:.1f}px > {max_disp}px), limité'
                            )
                            scale = max_disp / displacement
                            new_x = int(old_x + dx * scale)
                            new_y = int(old_y + dy * scale)
                            new_bbox = (new_x, new_y, new_bbox[2], new_bbox[3])
                    
                    self.is_tracking = True
                    self.last_bbox = new_bbox
                else:
                    self.is_tracking = False
                    self.get_logger().warn('Tracking perdu!')
            else:
                self.is_tracking = False
                self.get_logger().warn('Tracking perdu!')
        
        # Publier le résultat
        self._publish_tracked_object(msg)
    
    def _publish_tracked_object(self, frame_msg):
        """Publie l'état actuel du tracking."""
        out_msg = TrackedObject()
        out_msg.header = frame_msg.header
        out_msg.is_initialized = self.is_initialized
        out_msg.is_tracking = self.is_tracking
        
        if self.is_tracking and self.last_bbox is not None:
            bbox_x, bbox_y, bbox_w, bbox_h = self.last_bbox
            
            # CORRECTION: Flipper la bbox en retour pour l'affichage
            # La bbox est dans l'espace de l'image flippée, on doit la re-flipper
            bbox_x_display = frame_msg.width - bbox_x - bbox_w
            
            # Bounding box en pixels (coordonnées pour l'affichage)
            out_msg.bbox_x = int(bbox_x_display)
            out_msg.bbox_y = int(bbox_y)
            out_msg.bbox_width = int(bbox_w)
            out_msg.bbox_height = int(bbox_h)
            
            # Centre en pixels
            center_x_px = bbox_x + bbox_w // 2
            center_y_px = bbox_y + bbox_h // 2
            
            # Convertir en mètres
            center_x_m, center_y_m = self._pixels_to_meters(center_x_px, center_y_px, frame_msg)
            
            out_msg.center_x = float(center_x_m)
            out_msg.center_y = float(center_y_m)
            
            # Position polaire (range et bearing)
            out_msg.range = float(np.sqrt(center_x_m**2 + center_y_m**2))
            out_msg.bearing = float(np.arctan2(center_x_m, center_y_m))
            
            # Angle de rotation (en radians)
            out_msg.angle = float(self.bbox_angle)
            
            # Dimensions en mètres
            out_msg.width = float(bbox_w * frame_msg.resolution)
            out_msg.height = float(bbox_h * frame_msg.resolution)
            
            # Points d'entrée (si tracking rotatif avec entry_angle_offset)
            if self.tracking_mode == 'rotated' and self.initial_corners_m is not None:
                # Utiliser les coordonnées métriques initiales qui ne sont JAMAIS modifiées
                # Ces coordonnées préservent l'ordre exact des clics P1→P2→P3→P4
                # Le côté d'entrée est TOUJOURS entre P1 (index 0) et P2 (index 1)
                
                # Calculer le centre actuel de la bbox en mètres (où elle est maintenant)
                center_x_m = out_msg.center_x
                center_y_m = out_msg.center_y
                
                # Calculer le centroid initial (où était la bbox au départ)
                initial_center_x = np.mean(self.initial_corners_m[:, 0])
                initial_center_y = np.mean(self.initial_corners_m[:, 1])
                
                # Rotation entre angle initial et actuel
                rotation_delta = self.bbox_angle - self.initial_bbox_angle
                
                # Appliquer rotation et translation aux coins initiaux
                cos_r = np.cos(rotation_delta)
                sin_r = np.sin(rotation_delta)
                rot_matrix = np.array([[cos_r, -sin_r], [sin_r, cos_r]])
                
                # Centrer les coins initiaux
                centered_corners = self.initial_corners_m - np.array([initial_center_x, initial_center_y])
                
                # Appliquer la rotation
                rotated_corners = centered_corners @ rot_matrix.T
                
                # Translater au centre actuel
                current_corners = rotated_corners + np.array([center_x_m, center_y_m])
                
                # Le côté d'entrée est entre le coin 0 (P1) et le coin 1 (P2)
                c1 = current_corners[0]
                c2 = current_corners[1]
                
                # Coordonnées cartésiennes
                out_msg.entry_p1_x = float(c1[0])
                out_msg.entry_p1_y = float(c1[1])
                out_msg.entry_p2_x = float(c2[0])
                out_msg.entry_p2_y = float(c2[1])
                
                # Coordonnées polaires
                out_msg.entry_p1_range = float(np.sqrt(c1[0]**2 + c1[1]**2))
                out_msg.entry_p1_bearing = float(np.arctan2(c1[0], c1[1]))
                out_msg.entry_p2_range = float(np.sqrt(c2[0]**2 + c2[1]**2))
                out_msg.entry_p2_bearing = float(np.arctan2(c2[0], c2[1]))
            else:
                # Pas de tracking rotatif ou pas d'entrée définie
                out_msg.entry_p1_x = 0.0
                out_msg.entry_p1_y = 0.0
                out_msg.entry_p1_range = 0.0
                out_msg.entry_p1_bearing = 0.0
                out_msg.entry_p2_x = 0.0
                out_msg.entry_p2_y = 0.0
                out_msg.entry_p2_range = 0.0
                out_msg.entry_p2_bearing = 0.0
            
            # Confiance (les trackers OpenCV ne fournissent pas de score de confiance)
            out_msg.confidence = 0.0
        else:
            out_msg.center_x = 0.0
            out_msg.center_y = 0.0
            out_msg.range = 0.0
            out_msg.bearing = 0.0
            out_msg.angle = 0.0
            out_msg.width = 0.0
            out_msg.height = 0.0
            out_msg.bbox_x = 0
            out_msg.bbox_y = 0
            out_msg.bbox_width = 0
            out_msg.bbox_height = 0
            out_msg.confidence = 0.0
            out_msg.entry_p1_x = 0.0
            out_msg.entry_p1_y = 0.0
            out_msg.entry_p1_range = 0.0
            out_msg.entry_p1_bearing = 0.0
            out_msg.entry_p2_x = 0.0
            out_msg.entry_p2_y = 0.0
            out_msg.entry_p2_range = 0.0
            out_msg.entry_p2_bearing = 0.0
        
        self.tracked_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CSRTTrackerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()