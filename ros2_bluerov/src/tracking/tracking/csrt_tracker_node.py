"""
Nœud de tracking CSRT simplifié pour sonar.
Tracker basique avec bbox rectangle (sans gestion d'orientation).
Supporte maintenant un mode AUTO-DETECT qui utilise la détection Hough de cage en U.
"""

import numpy as np
import math
import rclpy
from rclpy.node import Node
from docking_msgs.msg import FrameCartesian, BBoxSelection, TrackedObject
from std_msgs.msg import Bool

import cv2


# =============================================================================
# FONCTIONS DE DÉTECTION HOUGH (extraites de hough_lines_node.py)
# =============================================================================

def merge_similar_lines(candidates, rho_tol=0.3, theta_tol=0.2):
    """
    Fusionne les lignes similaires en clusters basés sur rho et theta.
    
    Args:
        candidates: Liste de dicts {'p1', 'p2', 'rho', 'theta', 'length'}
        rho_tol: Tolérance sur rho (mètres)
        theta_tol: Tolérance sur theta (radians)
    
    Returns:
        Liste de lignes fusionnées
    """
    if not candidates:
        return []
    
    candidates = list(candidates)  # Copie pour ne pas modifier l'original
    candidates.sort(key=lambda x: x['length'], reverse=True)
    merged = []
    
    while len(candidates) > 0:
        base = candidates.pop(0)
        cluster = [base]
        remaining = []
        
        for other in candidates:
            d_theta = abs(base['theta'] - other['theta'])
            if d_theta > np.pi / 2:
                d_theta = abs(np.pi - d_theta)
            d_rho = abs(base['rho'] - other['rho'])
            
            if d_rho < rho_tol and d_theta < theta_tol:
                cluster.append(other)
            else:
                remaining.append(other)
        
        candidates = remaining
        
        # Calculer la ligne fusionnée (moyenne des extrémités)
        p1 = np.mean([c['p1'] for c in cluster], axis=0)
        p2 = np.mean([c['p2'] for c in cluster], axis=0)
        length = np.linalg.norm(p2 - p1)
        angle = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
        theta = (angle + np.pi / 2) % np.pi
        rho = p1[0] * np.cos(theta) + p1[1] * np.sin(theta)
        
        merged.append({
            'p1': p1, 'p2': p2,
            'rho': rho, 'theta': theta,
            'length': length, 'is_u': 0.0
        })
    
    return merged


def detect_u_shape(lines, cage_width=0.82, dist_tol=0.15, perp_tol=0.25, conn_tol=0.40):
    """
    Détecte une forme en U (cage) parmi les lignes.
    
    Args:
        lines: Liste de lignes fusionnées
        cage_width: Largeur attendue de la cage (m)
        dist_tol: Tolérance sur la distance entre bras (m)
        perp_tol: Tolérance sur la perpendicularité (rad)
        conn_tol: Tolérance sur la connexion base-bras (m)
    
    Returns:
        (bary, yaw, lines, u_lines) où:
        - bary: barycentre de la cage (np.array ou None)
        - yaw: angle d'orientation (float ou None)
        - lines: lignes avec marquage is_u
        - u_lines: liste des 3 lignes formant le U (ou None)
    """
    if len(lines) < 3:
        return None, None, lines, None
    
    best_score = -1
    best_data = (None, None)
    best_u_lines = None
    
    def check_perp(l1, l2):
        diff = abs(l1['theta'] - l2['theta'])
        return abs(diff - np.pi / 2) < perp_tol or abs(diff - 3 * np.pi / 2) < perp_tol
    
    def get_min_dist(l1, l2):
        pts = [l1['p1'], l1['p2'], l2['p1'], l2['p2']]
        return min(
            np.linalg.norm(pts[0] - pts[2]),
            np.linalg.norm(pts[0] - pts[3]),
            np.linalg.norm(pts[1] - pts[2]),
            np.linalg.norm(pts[1] - pts[3])
        )
    
    for i, base in enumerate(lines):
        for j, arm1 in enumerate(lines):
            if i == j:
                continue
            for k, arm2 in enumerate(lines):
                if k == i or k == j:
                    continue
                
                # Vérifier perpendicularité
                if not check_perp(base, arm1) or not check_perp(base, arm2):
                    continue
                
                # Vérifier distance entre milieux des bras ≈ cage_width
                m1 = (arm1['p1'] + arm1['p2']) / 2
                m2 = (arm2['p1'] + arm2['p2']) / 2
                dist = np.linalg.norm(m1 - m2)
                if abs(dist - cage_width) > dist_tol:
                    continue
                
                # Vérifier connexion base-bras
                if get_min_dist(base, arm1) > conn_tol or get_min_dist(base, arm2) > conn_tol:
                    continue
                
                # Score = somme des longueurs
                score = base['length'] + arm1['length'] + arm2['length']
                if score > best_score:
                    best_score = score
                    bary = (base['p1'] + base['p2'] + arm1['p1'] + arm1['p2'] + arm2['p1'] + arm2['p2']) / 6
                    base_mid = (base['p1'] + base['p2']) / 2
                    angle = math.atan2(bary[1] - base_mid[1], bary[0] - base_mid[0])
                    best_data = (bary, angle)
                    best_u_lines = [base, arm1, arm2]
                    
                    # Marquer les lignes du U
                    for idx in [i, j, k]:
                        lines[idx]['is_u'] = 1.0
    
    return best_data[0], best_data[1], lines, best_u_lines


def compute_bbox_from_u_shape(u_lines, margin_ratio=0.3):
    """
    Calcule une bounding box englobant les 3 lignes du U avec une marge.
    
    Args:
        u_lines: Liste de 3 lignes formant le U
        margin_ratio: Ratio de marge à ajouter (0.3 = 30% de plus de chaque côté)
    
    Returns:
        (min_x, min_y, max_x, max_y) en mètres, ou None si invalide
    """
    if u_lines is None or len(u_lines) < 3:
        return None
    
    # Collecter tous les points
    all_points = []
    for line in u_lines:
        all_points.append(line['p1'])
        all_points.append(line['p2'])
    
    all_points = np.array(all_points)
    
    min_x, min_y = np.min(all_points, axis=0)
    max_x, max_y = np.max(all_points, axis=0)
    
    # Ajouter une marge
    width = max_x - min_x
    height = max_y - min_y
    margin_x = width * margin_ratio
    margin_y = height * margin_ratio
    
    return (
        min_x - margin_x,
        min_y - margin_y,
        max_x + margin_x,
        max_y + margin_y
    )


# =============================================================================
# CLASSE PRINCIPALE
# =============================================================================

class CSRTTrackerNode(Node):
    """
    Tracker CSRT simplifié pour suivre la cage dans les images sonar cartésiennes.
    
    Modes de fonctionnement:
    - MANUEL: Initialisation via bbox sélectionnée manuellement (comportement original)
    - AUTO: Détection automatique de la cage en U via Hough, puis tracking CSRT
    """
    
    def __init__(self):
        super().__init__('csrt_tracker_node')
        
        # ========== PARAMÈTRES ==========
        self.declare_parameter('enable_tracking', True)
        
        # Paramètres CSRT (learning rates pour adaptation progressive)
        self.declare_parameter('filter_lr', 0.02)  # Apprentissage du filtre (plus haut = s'adapte plus vite)
        self.declare_parameter('scale_lr', 0.02)  # Apprentissage de l'échelle
        self.declare_parameter('psr_threshold', 0.02)  # Seuil de confiance (bas = plus permissif)
        
        # ========== PARAMÈTRES AUTO-DETECT ==========
        self.declare_parameter('auto_detect_enabled', False)  # Active le mode auto-detect
        # Note: La relance automatique si tracking perdu est TOUJOURS active
        
        # Paramètres Hough (pour la détection de lignes)
        self.declare_parameter('hough_rho_resolution', 1.0)
        self.declare_parameter('hough_theta_resolution', 1.0)  # en degrés
        self.declare_parameter('hough_threshold', 40)
        self.declare_parameter('hough_min_line_length', 15)
        self.declare_parameter('hough_max_line_gap', 10)
        
        # Filtres physiques (mètres)
        self.declare_parameter('filter_min_length_m', 0.2)
        self.declare_parameter('filter_max_length_m', 2.5)
        self.declare_parameter('merge_rho_tolerance', 0.3)
        self.declare_parameter('merge_theta_tolerance', 0.2)
        
        # Géométrie Cage (U)
        self.declare_parameter('cage_width', 0.82)
        self.declare_parameter('dist_tol', 0.15)
        self.declare_parameter('perp_tol', 0.25)
        self.declare_parameter('conn_tol', 0.40)
        
        # Marge autour de la détection pour le tracker (ratio)
        self.declare_parameter('bbox_margin_ratio', 0.3)
        
        # --- PARAMÈTRES DE FILTRAGE (depuis hough_lines_node.py) ---
        self.declare_parameter('window_size', 7)  # Taille de la fenêtre glissante
        self.declare_parameter('outlier_threshold_deg', 60.0)  # Seuil de rejet anti-saut (degrés)
        self.declare_parameter('max_consecutive_outliers', 5)  # Nb max de rejets consécutifs avant force-accept
        
        # État du tracker
        self.tracker = None
        self.is_initialized = False
        self.is_tracking = False
        self.last_bbox = None  # (x, y, w, h) en pixels
        self.current_resolution = None
        
        # État auto-detect
        self.auto_detect_searching = False  # True quand on cherche la cage
        self.auto_detect_found_once = False  # True si on a déjà trouvé la cage au moins une fois
        
        # --- ÉTAT DU FILTRAGE ANTI-SAUT ---
        from collections import deque
        win_size = self.get_parameter('window_size').value
        self.detection_history = deque(maxlen=win_size)  # Historique (x, y, yaw)
        self.last_valid_yaw = None
        self.consecutive_outliers = 0
        
        # Subscription pour les frames
        self.frame_sub = self.create_subscription(
            FrameCartesian,
            '/docking/sonar/cartesian_filtered',
            self.frame_callback,
            10
        )
        
        # Subscription pour la sélection de bbox (mode manuel)
        self.bbox_sub = self.create_subscription(
            BBoxSelection,
            '/docking/sonar/bbox_selection',
            self.bbox_callback,
            10
        )
        
        # Subscription pour déclencher la recherche auto (optionnel)
        self.auto_trigger_sub = self.create_subscription(
            Bool,
            '/docking/tracking/trigger_auto_detect',
            self.trigger_auto_detect_callback,
            10
        )
        
        # Publisher
        self.tracked_pub = self.create_publisher(
            TrackedObject,
            '/docking/tracking/tracked_object',
            10
        )
        
        # Publisher pour signaler si auto-detect est actif
        self.auto_detect_status_pub = self.create_publisher(
            Bool,
            '/docking/tracking/auto_detect_status',
            10
        )
        
        self._pending_init = None
        
        # Démarrer en mode recherche si auto_detect activé
        if self.get_parameter('auto_detect_enabled').value:
            self.auto_detect_searching = True
            self.get_logger().info('CSRT Tracker node démarré (mode AUTO-DETECT activé)')
        else:
            self.get_logger().info('CSRT Tracker node démarré (mode manuel)')
        
        if cv2 is None:
            self.get_logger().error('OpenCV non disponible!')
    
    def trigger_auto_detect_callback(self, msg: Bool):
        """Permet de déclencher/arrêter la recherche automatique via un topic."""
        if msg.data:
            # Activer la recherche
            self.auto_detect_searching = True
            self.is_initialized = False
            self.is_tracking = False
            self.tracker = None
            # Réinitialiser le filtrage
            self.detection_history.clear()
            self.last_valid_yaw = None
            self.consecutive_outliers = 0
            self.get_logger().info('Recherche automatique de cage ACTIVÉE')
        else:
            # Désactiver la recherche
            self.auto_detect_searching = False
            self.get_logger().info('Recherche automatique de cage DÉSACTIVÉE')
    
    # =========================================================================
    # MÉTHODES DE FILTRAGE ANTI-SAUT (depuis hough_lines_node.py)
    # =========================================================================
    
    def _angle_diff(self, a, b):
        """Différence angulaire normalisée entre -PI et PI."""
        d = a - b
        while d > np.pi:
            d -= 2 * np.pi
        while d < -np.pi:
            d += 2 * np.pi
        return d
    
    def _compute_average_pose(self, history_buffer):
        """
        Calcule la moyenne de X, Y et l'angle moyen correct (moyenne vectorielle).
        Évite le problème du saut +/- PI.
        """
        if not history_buffer:
            return None, None, None
        
        xs = [h[0] for h in history_buffer]
        ys = [h[1] for h in history_buffer]
        avg_x = np.mean(xs)
        avg_y = np.mean(ys)
        
        # Moyenne vectorielle pour l'angle
        angles = [h[2] for h in history_buffer]
        sin_sum = np.sum(np.sin(angles))
        cos_sum = np.sum(np.cos(angles))
        avg_yaw = np.arctan2(sin_sum, cos_sum)
        
        return avg_x, avg_y, avg_yaw
    
    def _filter_detection(self, raw_x, raw_y, raw_yaw):
        """
        Applique le filtrage anti-saut et la moyenne glissante.
        
        Returns:
            (filtered_x, filtered_y, filtered_yaw, accepted) où accepted=False si rejeté
        """
        threshold_deg = self.get_parameter('outlier_threshold_deg').value
        threshold_rad = np.radians(threshold_deg)
        max_outliers = self.get_parameter('max_consecutive_outliers').value
        
        accept_measurement = True
        
        # 1. Vérification Anti-Saut
        if self.last_valid_yaw is not None:
            diff = abs(self._angle_diff(raw_yaw, self.last_valid_yaw))
            
            if diff > threshold_rad:
                self.consecutive_outliers += 1
                if self.consecutive_outliers <= max_outliers:
                    accept_measurement = False
                    self.get_logger().debug(
                        f'Outlier rejeté ({self.consecutive_outliers}/{max_outliers}): '
                        f'diff={math.degrees(diff):.1f}° > seuil={threshold_deg}°'
                    )
                else:
                    # Sécurité : On accepte si le saut persiste
                    self.get_logger().warn(
                        f'Force update: Trop de rejets consécutifs ({max_outliers}). '
                        f'Nouveau yaw accepté: {math.degrees(raw_yaw):.1f}°'
                    )
                    self.consecutive_outliers = 0
            else:
                self.consecutive_outliers = 0
        
        # 2. Mise à jour de l'historique si accepté
        if accept_measurement:
            self.last_valid_yaw = raw_yaw
            self.detection_history.append((raw_x, raw_y, raw_yaw))
        
        # 3. Calcul de la Pose Moyenne (Filtrée)
        filtered_x, filtered_y, filtered_yaw = self._compute_average_pose(self.detection_history)
        
        return filtered_x, filtered_y, filtered_yaw, accept_measurement
    
    # =========================================================================
    
    def _create_tracker(self):
        """Crée un nouveau tracker CSRT."""
        if cv2 is None:
            return None
        
        params = cv2.TrackerCSRT_Params()
        params.use_hog = True
        params.use_color_names = False
        params.use_gray = True
        params.use_segmentation = False
        params.filter_lr = float(self.get_parameter('filter_lr').value)
        params.scale_lr = float(self.get_parameter('scale_lr').value)
        params.psr_threshold = float(self.get_parameter('psr_threshold').value)
        
        return cv2.TrackerCSRT_create(params)
    
    def _pixels_to_meters(self, x_px, y_px, frame_msg):
        """Convertit des coordonnées en pixels vers des mètres."""
        x_m = (x_px - frame_msg.origin_x) * frame_msg.resolution
        y_m = (frame_msg.height - y_px) * frame_msg.resolution
        return x_m, y_m
    
    def bbox_callback(self, msg: BBoxSelection):
        """Initialise le tracker avec une bbox manuelle."""
        if not self.get_parameter('enable_tracking').value:
            return
        
        if cv2 is None or not msg.is_valid:
            return
        
        # Désactiver la recherche auto si on reçoit une bbox manuelle
        self.auto_detect_searching = False
        
        self._pending_init = (msg.x, msg.y, msg.width, msg.height)
        self.get_logger().info(f'BBox manuelle reçue: ({msg.x}, {msg.y}, {msg.width}, {msg.height})')
    
    def _detect_cage_in_frame(self, img, frame_msg):
        """
        Détecte la cage en U dans une frame sonar via Hough Lines.
        
        Args:
            img: Image en niveaux de gris (uint8)
            frame_msg: Message FrameCartesian pour la conversion pixels/mètres
        
        Returns:
            (bbox_pixels, bary_m, yaw) où bbox_pixels est (x, y, w, h) en pixels
            ou (None, None, None) si pas de détection
        """
        # Paramètres Hough
        rho_res = self.get_parameter('hough_rho_resolution').value
        theta_res = self.get_parameter('hough_theta_resolution').value * np.pi / 180.0
        threshold = int(self.get_parameter('hough_threshold').value)
        min_line_length = int(self.get_parameter('hough_min_line_length').value)
        max_line_gap = int(self.get_parameter('hough_max_line_gap').value)
        
        # Filtres
        min_len_m = self.get_parameter('filter_min_length_m').value
        max_len_m = self.get_parameter('filter_max_length_m').value
        rho_tol = self.get_parameter('merge_rho_tolerance').value
        theta_tol = self.get_parameter('merge_theta_tolerance').value
        
        # Géométrie cage
        cage_width = self.get_parameter('cage_width').value
        dist_tol = self.get_parameter('dist_tol').value
        perp_tol = self.get_parameter('perp_tol').value
        conn_tol = self.get_parameter('conn_tol').value
        margin_ratio = self.get_parameter('bbox_margin_ratio').value
        
        # Détection Hough
        cv_lines = cv2.HoughLinesP(
            img, rho=rho_res, theta=theta_res,
            threshold=threshold,
            minLineLength=min_line_length,
            maxLineGap=max_line_gap
        )
        
        if cv_lines is None:
            return None, None, None
        
        # Convertir les lignes pixels → mètres
        candidates = []
        for l in cv_lines:
            x1_px, y1_px, x2_px, y2_px = l[0]
            
            # Conversion pixels → mètres (même convention que hough_lines_node.py)
            p1 = np.array([
                -(x1_px - frame_msg.origin_x) * frame_msg.resolution,
                (y1_px - frame_msg.origin_y) * frame_msg.resolution + frame_msg.min_range
            ])
            p2 = np.array([
                -(x2_px - frame_msg.origin_x) * frame_msg.resolution,
                (y2_px - frame_msg.origin_y) * frame_msg.resolution + frame_msg.min_range
            ])
            
            length = np.linalg.norm(p2 - p1)
            if min_len_m < length < max_len_m:
                angle = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
                theta = (angle + np.pi / 2) % np.pi
                rho = p1[0] * np.cos(theta) + p1[1] * np.sin(theta)
                candidates.append({
                    'p1': p1, 'p2': p2,
                    'rho': rho, 'theta': theta,
                    'length': length,
                    # Garder aussi les pixels pour le calcul de bbox
                    'p1_px': np.array([x1_px, y1_px]),
                    'p2_px': np.array([x2_px, y2_px])
                })
        
        if not candidates:
            return None, None, None
        
        # Fusionner les lignes similaires
        merged = merge_similar_lines(candidates, rho_tol, theta_tol)
        
        # Détecter la forme en U
        bary, yaw, _, u_lines = detect_u_shape(
            merged, cage_width, dist_tol, perp_tol, conn_tol
        )
        
        if bary is None or u_lines is None:
            return None, None, None
        
        # Calculer la bbox en mètres puis convertir en pixels
        bbox_m = compute_bbox_from_u_shape(u_lines, margin_ratio)
        if bbox_m is None:
            return None, None, None
        
        min_x_m, min_y_m, max_x_m, max_y_m = bbox_m
        
        # Conversion mètres → pixels (inverse de la conversion ci-dessus)
        # p_m = [-(x_px - origin_x) * res, (y_px - origin_y) * res + min_range]
        # => x_px = origin_x - p_m[0] / res
        # => y_px = origin_y + (p_m[1] - min_range) / res
        
        x1_px = int(frame_msg.origin_x - max_x_m / frame_msg.resolution)  # max_x_m car signe négatif
        x2_px = int(frame_msg.origin_x - min_x_m / frame_msg.resolution)
        y1_px = int(frame_msg.origin_y + (min_y_m - frame_msg.min_range) / frame_msg.resolution)
        y2_px = int(frame_msg.origin_y + (max_y_m - frame_msg.min_range) / frame_msg.resolution)
        
        # Clamp aux dimensions de l'image
        x1_px = max(0, min(x1_px, frame_msg.width - 1))
        x2_px = max(0, min(x2_px, frame_msg.width - 1))
        y1_px = max(0, min(y1_px, frame_msg.height - 1))
        y2_px = max(0, min(y2_px, frame_msg.height - 1))
        
        bbox_w = x2_px - x1_px
        bbox_h = y2_px - y1_px
        
        if bbox_w < 10 or bbox_h < 10:
            return None, None, None
        
        return (x1_px, y1_px, bbox_w, bbox_h), bary, yaw
    
    def frame_callback(self, msg: FrameCartesian):
        """Traite une frame et met à jour le tracking."""
        if not self.get_parameter('enable_tracking').value or cv2 is None:
            return
        
        # Reconstruire l'image
        img = np.array(msg.intensities, dtype=np.uint8).reshape((msg.height, msg.width))
        img_processed = cv2.equalizeHist(img)
        img_flipped = cv2.flip(img_processed, 0)  # Flip vertical uniquement
        img_bgr = cv2.cvtColor(img_flipped, cv2.COLOR_GRAY2BGR)
        
        if not img_bgr.flags['C_CONTIGUOUS']:
            img_bgr = np.ascontiguousarray(img_bgr)
        
        self.current_resolution = msg.resolution
        
        # ========== MODE AUTO-DETECT ==========
        if self.auto_detect_searching and not self.is_initialized:
            # Chercher la cage dans l'image NON flippée (convention hough_lines_node)
            bbox_result = self._detect_cage_in_frame(img, msg)
            
            if bbox_result[0] is not None:
                bbox_px, bary_m, raw_yaw = bbox_result
                bbox_x, bbox_y, bbox_w, bbox_h = bbox_px
                
                # Appliquer le filtrage anti-saut
                raw_x, raw_y = float(bary_m[0]), float(bary_m[1])
                filtered_x, filtered_y, filtered_yaw, accepted = self._filter_detection(
                    raw_x, raw_y, float(raw_yaw)
                )
                
                if not accepted:
                    # Détection rejetée (outlier), on attend la prochaine frame
                    # Publier quand même le statut
                    status_msg = Bool()
                    status_msg.data = self.auto_detect_searching
                    self.auto_detect_status_pub.publish(status_msg)
                    self._publish_tracked_object(msg)
                    return
                
                # Flipper Y pour correspondre à l'image traitée
                bbox_y_flipped = msg.height - bbox_y - bbox_h
                
                # Initialiser le tracker CSRT
                self.tracker = self._create_tracker()
                if self.tracker is not None:
                    try:
                        self.tracker.init(img_bgr, (bbox_x, bbox_y_flipped, bbox_w, bbox_h))
                        self.is_initialized = True
                        self.is_tracking = True
                        self.auto_detect_found_once = True
                        self.auto_detect_searching = False  # Arrêter la recherche
                        self.last_bbox = (bbox_x, bbox_y_flipped, bbox_w, bbox_h)
                        
                        self.get_logger().info(
                            f'AUTO-DETECT: Cage trouvée! '
                            f'BBox=({bbox_x}, {bbox_y}, {bbox_w}x{bbox_h}) '
                            f'Centre≈({filtered_x:.2f}m, {filtered_y:.2f}m) '
                            f'Yaw≈{math.degrees(filtered_yaw):.1f}° (filtré)'
                        )
                    except Exception as e:
                        self.get_logger().error(f'Erreur init auto-detect: {e}')
            
            # Publier le statut de recherche
            status_msg = Bool()
            status_msg.data = self.auto_detect_searching
            self.auto_detect_status_pub.publish(status_msg)
        
        # ========== INITIALISATION MANUELLE EN ATTENTE ==========
        if self._pending_init is not None:
            bbox_x, bbox_y, bbox_w, bbox_h = self._pending_init
            self._pending_init = None
            
            # Flipper uniquement Y car l'image est flippée verticalement
            bbox_y_flipped = msg.height - bbox_y - bbox_h
            
            self.tracker = self._create_tracker()
            if self.tracker is not None:
                try:
                    self.tracker.init(img_bgr, (bbox_x, bbox_y_flipped, bbox_w, bbox_h))
                    self.is_initialized = True
                    self.is_tracking = True
                    self.last_bbox = (bbox_x, bbox_y_flipped, bbox_w, bbox_h)
                    self.get_logger().info('Tracker initialisé (manuel)')
                except Exception as e:
                    self.get_logger().error(f'Erreur init: {e}')
        
        # ========== MISE À JOUR DU TRACKING ==========
        if self.is_initialized and self.tracker is not None:
            success, bbox = self.tracker.update(img_bgr)
            
            if success:
                self.is_tracking = True
                self.last_bbox = tuple(map(int, bbox))
            else:
                self.is_tracking = False
                
                # Tracking perdu -> TOUJOURS relancer la recherche automatique
                if self.auto_detect_found_once:
                    self.auto_detect_searching = True
                    self.is_initialized = False
                    self.tracker = None
                    # Réinitialiser le filtrage pour repartir proprement
                    self.detection_history.clear()
                    self.last_valid_yaw = None
                    self.consecutive_outliers = 0
                    self.get_logger().warn('Tracking perdu - Relance de la recherche automatique')
                    
                    # Publier le statut de recherche pour informer l'UI
                    status_msg = Bool()
                    status_msg.data = True  # On recherche à nouveau
                    self.auto_detect_status_pub.publish(status_msg)
        
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
            
            # Re-flipper uniquement Y pour l'affichage
            bbox_y_display = frame_msg.height - bbox_y - bbox_h
            
            out_msg.bbox_x = int(bbox_x)
            out_msg.bbox_y = int(bbox_y_display)
            out_msg.bbox_width = int(bbox_w)
            out_msg.bbox_height = int(bbox_h)
            
            # Centre en pixels puis en mètres (utiliser les coordonnées flippées)
            center_x_px = bbox_x + bbox_w // 2
            center_y_px = frame_msg.height - bbox_y_display - bbox_h // 2
            center_x_m, center_y_m = self._pixels_to_meters(center_x_px, center_y_px, frame_msg)
            
            # CORRECTION: Inversion du signe de X
            # ====================================
            # La conversion polaire→cartésien utilise arctan2(-xv, yv) (T2),
            # ce qui inverse l'axe X de l'image cartésienne par rapport à
            # la convention sonar (X+ = droite).
            # On compense ici en inversant le signe de center_x.
            center_x_m = -center_x_m

            out_msg.center_x = float(center_x_m)
            out_msg.center_y = float(center_y_m)
            out_msg.range = float(np.sqrt(center_x_m**2 + center_y_m**2))
            out_msg.bearing = float(np.arctan2(center_x_m, center_y_m))
            out_msg.angle = 0.0
            out_msg.width = float(bbox_w * frame_msg.resolution)
            out_msg.height = float(bbox_h * frame_msg.resolution)
            out_msg.confidence = 0.0
            
            # Pas de points d'entrée en mode simplifié
            out_msg.entry_p1_x = 0.0
            out_msg.entry_p1_y = 0.0
            out_msg.entry_p1_range = 0.0
            out_msg.entry_p1_bearing = 0.0
            out_msg.entry_p2_x = 0.0
            out_msg.entry_p2_y = 0.0
            out_msg.entry_p2_range = 0.0
            out_msg.entry_p2_bearing = 0.0
        else:
            # Valeurs par défaut
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