"""
Nœud de filtrage cartésien.
Applique des filtres sur données cartésiennes (x × y).
Conversion polaire → cartésien identique à sonar_display.py.
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame, FrameCartesian, TrackedObject
import numpy as np
from scipy.ndimage import map_coordinates
try:
    import cv2
except ImportError:
    cv2 = None


class TraitementCartesianNode(Node):
    """Applique filtres sur frames sonar cartésiennes."""
    
    def __init__(self):
        super().__init__('traitement_cartesian_node')
        
        # ========== PARAMÈTRES DE CONVERSION ==========
        self.declare_parameter('cartesian_scale_factor', 2.0)  # Facteur d'échelle pour la largeur
        
        # ========== FILTRES CARTÉSIENS ==========
        # Canny Edge Detection
        self.declare_parameter('cart_enable_canny', False)
        self.declare_parameter('cart_canny_threshold1', 50.0)
        self.declare_parameter('cart_canny_threshold2', 150.0)
        self.declare_parameter('cart_canny_aperture', 3)
        
        # Binarisation par centiles
        self.declare_parameter('cart_enable_percentile_binarization', False)
        self.declare_parameter('cart_percentile_threshold', 90.0)
        
        # ========== FILTRE SPATIAL GAUSSIEN (ÉTAPE 1) ==========
        self.declare_parameter('enable_spatial_filter', False)  # Désactivé par défaut
        self.declare_parameter('spatial_filter_radius', 2.0)    # Rayon en mètres
        self.declare_parameter('spatial_filter_sigma', 0.8)     # Écart-type gaussien (m)
        
        # ========== OPÉRATIONS MORPHOLOGIQUES (ÉTAPE 2) ==========
        self.declare_parameter('cart_enable_morphology', False)
        self.declare_parameter('cart_morphology_kernel_size', 3)
        self.declare_parameter('cart_morphology_iterations', 2)
        
        # Subscription - peut s'abonner aux données brutes ou filtrées polaires
        self.declare_parameter('subscribe_to_filtered', True)
        subscribe_filtered = self.get_parameter('subscribe_to_filtered').value
        
        topic = '/docking/sonar/polar_filtered' if subscribe_filtered else '/docking/sonar/raw'
        
        self.subscription = self.create_subscription(
            Frame,
            topic,
            self.frame_callback,
            10
        )
        
        # Subscription au tracker pour le filtre spatial (ÉTAPE 1)
        self.tracked_object_sub = self.create_subscription(
            TrackedObject,
            '/docking/tracking/tracked_object',
            self.tracked_object_callback,
            10
        )
        
        # État du filtre spatial
        self.last_tracked_position = None  # (center_x, center_y) en mètres
        self.last_tracked_is_tracking = False
        
        # Publisher - données cartésiennes filtrées
        self.publisher_ = self.create_publisher(FrameCartesian, '/docking/sonar/cartesian_filtered', 10)
        
        # Cache pour la conversion polaire → cartésien
        self._mapping_cache = {
            'bearing_count': None,
            'range_count': None,
            'coords': None,
            'out_shape': None,
        }
        
        # Cache pour le filtre spatial gaussien (éviter recalcul si position similaire)
        self._spatial_filter_cache = {
            'center': None,
            'shape': None,
            'resolution': None,
            'origin_x': None,
            'mask': None,
        }
        
        self.get_logger().info(f'Traitement Cartesian node démarré (subscribe to: {topic})')
    
    # ========== CALLBACK TRACKER (ÉTAPE 1) ==========
    
    def tracked_object_callback(self, msg: TrackedObject):
        """Mémorise la dernière position trackée pour le filtre spatial."""
        self.last_tracked_is_tracking = msg.is_tracking
        if msg.is_tracking:
            self.last_tracked_position = (msg.center_x, msg.center_y)
            # Log de debug (première fois seulement)
            if not hasattr(self, '_tracker_logged'):
                self._tracker_logged = True
                self.get_logger().info(
                    f'Filtre spatial: position trackée reçue ({msg.center_x:.2f}m, {msg.center_y:.2f}m)'
                )
        else:
            # On garde la dernière position connue si le tracker perd la cible
            pass
    
    # ========== CONVERSION POLAIRE → CARTÉSIEN ==========
    
    def polar_to_cartesian(self, polar_img: np.ndarray, frame_msg: Frame) -> tuple:
        """
        Convertit une image polaire en cartésienne.
        Utilise la même méthode que sonar_display.py avec interpolation.
        Retourne (image_cartésienne, metadata).
        """
        bc = frame_msg.bearing_count
        rc = frame_msg.range_count
        max_r = frame_msg.max_range
        min_r = frame_msg.min_range
        total_angle = frame_msg.bearing_resolution * bc
        
        scale_factor = float(self.get_parameter('cartesian_scale_factor').value)
        
        cache = self._mapping_cache
        
        # Vérifier si on doit recalculer les coordonnées
        if cache['bearing_count'] != bc or cache['range_count'] != rc:
            # Dimensions de sortie (hauteur = range_count, largeur = 2 * range_count)
            out_h = rc
            out_w = int(scale_factor * rc)
            
            # Grille cartésienne
            xs = np.linspace(-max_r, max_r, out_w)
            ys = np.linspace(0, max_r, out_h)
            xv, yv = np.meshgrid(xs, ys)
            
            # Conversion en coordonnées polaires
            rr = np.sqrt(xv**2 + yv**2)
            # Inversion de xv pour corriger le mirroring horizontal (comme dans sonar_display.py)
            th = np.arctan2(-xv, yv)
            
            # Indices flottants dans l'image polaire
            i_float = (th + total_angle / 2.0) / total_angle * (bc - 1)
            j_float = (rr - min_r) / (max_r - min_r) * (rc - 1)
            
            coords = np.vstack((i_float.ravel(), j_float.ravel()))
            
            # Mise en cache
            cache['bearing_count'] = bc
            cache['range_count'] = rc
            cache['coords'] = coords
            cache['out_shape'] = (out_h, out_w)
        else:
            coords = cache['coords']
            out_h, out_w = cache['out_shape']
        
        # Interpolation
        sampled = map_coordinates(
            polar_img.astype(np.float32),
            coords,
            order=1,
            mode='constant',
            cval=0.0
        )
        cart_img = sampled.reshape((out_h, out_w)).astype(np.uint8)
        
        # Métadonnées
        resolution = (2.0 * max_r) / out_w  # mètres par pixel
        origin_x = out_w // 2  # Centre en X
        origin_y = 0  # Bas de l'image en Y
        
        return cart_img, resolution, origin_x, origin_y, total_angle
    
    # ========== FILTRES CARTÉSIENS ==========
    
    def cart_canny_edge(self, img: np.ndarray) -> np.ndarray:
        """Détection de contours Canny sur image cartésienne."""
        if cv2 is None:
            self.get_logger().warn('OpenCV non disponible, Canny ignoré')
            return img
        
        threshold1 = float(self.get_parameter('cart_canny_threshold1').value)
        threshold2 = float(self.get_parameter('cart_canny_threshold2').value)
        aperture = int(self.get_parameter('cart_canny_aperture').value)
        
        # Assurer que aperture est impair et dans [3, 7]
        aperture = max(3, min(7, aperture))
        if aperture % 2 == 0:
            aperture += 1
        
        return cv2.Canny(img, threshold1, threshold2, apertureSize=aperture)
    
    def cart_percentile_binarization(self, img: np.ndarray) -> np.ndarray:
        """
        Binarisation par centile.
        Garde les N% pixels les plus lumineux.
        """
        percentile = float(self.get_parameter('cart_percentile_threshold').value)
        
        # Calcul du seuil au centile donné
        threshold = np.percentile(img, percentile)
        
        # Binarisation
        binary = np.where(img >= threshold, 255, 0).astype(np.uint8)
        
        return binary
    
    # ========== FILTRE SPATIAL GAUSSIEN (ÉTAPE 1) ==========
    
    def apply_spatial_gaussian_filter(self, img: np.ndarray, resolution: float, 
                                       origin_x: int, max_range: float) -> np.ndarray:
        """
        Applique un filtre gaussien spatial centré sur la position trackée.
        Atténue progressivement les intensités au-delà du rayon spécifié.
        
        Args:
            img: Image cartésienne (height x width)
            resolution: Résolution en m/pixel
            origin_x: Position X de l'origine (centre) en pixels
            max_range: Portée maximale du sonar en mètres
            
        Returns:
            Image filtrée avec atténuation gaussienne
        """
        if not self.get_parameter('enable_spatial_filter').value:
            return img
        
        if self.last_tracked_position is None:
            return img  # Pas de position trackée, pas de filtrage
        
        if not self.last_tracked_is_tracking:
            return img  # Tracker non actif, pas de filtrage
        
        center_x_m_tracker, center_y_m = self.last_tracked_position
        sigma_m = float(self.get_parameter('spatial_filter_sigma').value)
        
        height, width = img.shape
        
        # CORRECTION: Le tracker publie center_x_m basé sur l'image flippée
        # Donc center_x_m_tracker correspond en fait à -center_x_m_real
        # On inverse la coordonnée X pour avoir la vraie position dans l'image cartésienne
        center_x_m_real = -center_x_m_tracker
        
        # Vérifier si on peut utiliser le cache
        cache = self._spatial_filter_cache
        cache_valid = (
            cache['center'] is not None and
            cache['shape'] == (height, width) and
            abs(cache['resolution'] - resolution) < 1e-6 and
            cache['origin_x'] == origin_x and
            abs(cache['center'][0] - center_x_m_real) < 0.05 and  # Tolérance 5cm
            abs(cache['center'][1] - center_y_m) < 0.05
        )
        
        if cache_valid:
            mask = cache['mask']
        else:
            # Créer une grille de coordonnées en mètres
            # L'axe X (horizontal) : x_m = (j - origin_x) * resolution
            # L'axe Y (vertical) : y_m = (height - i) * resolution (Y croît vers le haut)
            j_indices = np.arange(width)
            i_indices = np.arange(height)
            j_grid, i_grid = np.meshgrid(j_indices, i_indices)
            
            x_grid_m = (j_grid - origin_x) * resolution
            y_grid_m = (height - i_grid) * resolution
            
            # Distance au centre tracké (avec coordonnée X corrigée)
            dist_m = np.sqrt((x_grid_m - center_x_m_real)**2 + (y_grid_m - center_y_m)**2)
            
            # Masque gaussien (1.0 au centre, décroît progressivement)
            # Plus sigma est petit, plus la transition est abrupte
            mask = np.exp(-(dist_m**2) / (2 * sigma_m**2))
            
            # Mise en cache (avec la coordonnée corrigée)
            cache['center'] = (center_x_m_real, center_y_m)
            cache['shape'] = (height, width)
            cache['resolution'] = resolution
            cache['origin_x'] = origin_x
            cache['mask'] = mask
            
            # Log de debug pour vérifier les coordonnées
            if not hasattr(self, '_debug_logged'):
                self._debug_logged = True
                self.get_logger().info(
                    f'Filtre spatial: tracker=({center_x_m_tracker:.2f}, {center_y_m:.2f}m) '
                    f'→ corrigé=({center_x_m_real:.2f}, {center_y_m:.2f}m)'
                )
        
        # Appliquer le masque (multiplication élément par élément)
        filtered_img = (img.astype(np.float32) * mask).astype(np.uint8)
        
        return filtered_img
    
    # ========== OPÉRATIONS MORPHOLOGIQUES (ÉTAPE 2) ==========
    
    def apply_morphology_operations(self, img: np.ndarray) -> np.ndarray:
        """
        Applique des opérations morphologiques pour nettoyer l'image.
        - Closing : ferme les petits trous dans les objets
        - Opening : supprime les petits bruits isolés
        
        Args:
            img: Image en niveaux de gris ou binaire
            
        Returns:
            Image nettoyée par opérations morphologiques
        """
        if not self.get_parameter('cart_enable_morphology').value:
            return img
        
        if cv2 is None:
            self.get_logger().warn('OpenCV non disponible, morphologie ignorée', 
                                   throttle_duration_sec=5.0)
            return img
        
        kernel_size = int(self.get_parameter('cart_morphology_kernel_size').value)
        iterations = int(self.get_parameter('cart_morphology_iterations').value)
        
        # Assurer kernel_size impair et >= 3
        kernel_size = max(3, kernel_size)
        if kernel_size % 2 == 0:
            kernel_size += 1
        
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        
        # Closing : fermer les petits trous (dilatation puis érosion)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel, iterations=iterations)
        
        # Opening : supprimer les petits bruits (érosion puis dilatation)
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel, iterations=1)
        
        return img
    
    def frame_callback(self, msg: Frame):
        """Traite une frame sonar polaire et publie en cartésien."""
        # Reconstruction image 2D polaire (bearing × range)
        polar_img = np.array(msg.intensities, dtype=np.uint8).reshape(
            (msg.bearing_count, msg.range_count)
        )
        
        # ========== CONVERSION POLAIRE → CARTÉSIEN ==========
        cart_img, resolution, origin_x, origin_y, total_angle = self.polar_to_cartesian(
            polar_img, msg
        )
        
        # ========== APPLICATION DES FILTRES CARTÉSIENS ==========
        filtered_cart = cart_img.copy()
        
        # 1. Filtre spatial gaussien (ÉTAPE 1) - Appliqué en premier pour isoler la zone d'intérêt
        filtered_cart = self.apply_spatial_gaussian_filter(
            filtered_cart, resolution, origin_x, msg.max_range
        )
        
        # 2. Opérations morphologiques (ÉTAPE 2) - Nettoie l'image
        filtered_cart = self.apply_morphology_operations(filtered_cart)
        
        # 3. Détection de contours Canny
        if self.get_parameter('cart_enable_canny').value:
            filtered_cart = self.cart_canny_edge(filtered_cart)
        
        # 4. Binarisation par centiles
        if self.get_parameter('cart_enable_percentile_binarization').value:
            filtered_cart = self.cart_percentile_binarization(filtered_cart)
        
        # ========== PUBLICATION (FORMAT CARTÉSIEN) ==========
        out_msg = FrameCartesian()
        out_msg.header = msg.header
        out_msg.width = filtered_cart.shape[1]
        out_msg.height = filtered_cart.shape[0]
        out_msg.resolution = resolution
        out_msg.min_range = msg.min_range
        out_msg.max_range = msg.max_range
        out_msg.origin_x = origin_x
        out_msg.origin_y = origin_y
        out_msg.intensities = filtered_cart.flatten().tolist()
        out_msg.sound_speed = msg.sound_speed
        out_msg.gain = msg.gain
        out_msg.total_angle = total_angle
        
        self.publisher_.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TraitementCartesianNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
