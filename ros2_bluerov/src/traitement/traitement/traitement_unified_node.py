"""
Nœud de traitement unifié pour données sonar.
Pipeline simplifié: Polaire (Frost + Médian) → Cartésien (Spatial + Percentile)
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame, FrameCartesian, TrackedObject
import numpy as np
from scipy import ndimage
from scipy.ndimage import map_coordinates
import cv2


class TraitementUnifiedNode(Node):
    """
    Traitement unifié des données sonar.
    
    Pipeline:
    1. Filtre Frost (polaire) - Réduction du speckle adaptatif
    2. Filtre Médian (polaire) - Lissage
    3. Conversion polaire → cartésien
    4. Filtre spatial gaussien (cartésien) - Focus sur zone trackée
    5. Binarisation par centile (cartésien) - Extraction des échos forts
    """
    
    def __init__(self):
        super().__init__('traitement_unified_node')
        
        # ========== PARAMÈTRES POLAIRES ==========
        # Filtre Médian (Denoising)
        self.declare_parameter('polar_enable_median', True)
        self.declare_parameter('polar_median_kernel', 3)
        
        # Filtre Frost
        self.declare_parameter('polar_enable_frost', True)
        self.declare_parameter('polar_frost_window_size', 7)
        self.declare_parameter('polar_frost_damping', 3.4)
        
        # ========== PARAMÈTRES CARTÉSIENS ==========
        self.declare_parameter('cartesian_scale_factor', 2.0)
        
        # Filtre spatial gaussien (actif quand tracker disponible)
        self.declare_parameter('enable_spatial_filter', False)
        self.declare_parameter('spatial_filter_radius', 2.0)
        self.declare_parameter('spatial_filter_sigma', 0.8)
        
        # ========== NOUVEAUX FILTRES CARTÉSIENS ==========
        
        # Filtre Médian Cartésien (Denoising supplémentaire)
        self.declare_parameter('cart_enable_median', True)
        self.declare_parameter('cart_median_kernel_size', 3)
        
        # Filtre CLAHE (Amélioration de contraste)
        self.declare_parameter('cart_enable_clahe', True)
        self.declare_parameter('cart_clahe_clip_limit', 2.0)
        self.declare_parameter('cart_clahe_tile_grid_size', 8)
        
        # Filtre Seuil Bas (Threshold to Zero) - SEUL FILTRE QUI BINARISE
        self.declare_parameter('cart_enable_threshold', True)
        self.declare_parameter('cart_min_intensity_threshold', 40)
        
        # Filtre Morphologique (Closing)
        self.declare_parameter('cart_enable_morphology', True)
        self.declare_parameter('cart_morph_kernel_size', 3)
        self.declare_parameter('cart_morph_iterations', 1)
        
        # Filtre Géométrique (Flip)
        self.declare_parameter('cart_flip_horizontal', False)
        self.declare_parameter('cart_flip_vertical', False)
        
        # Filtre Binarisation Percentile (garde les X% pixels les plus intenses)
        self.declare_parameter('cart_enable_percentile_binarize', False)
        self.declare_parameter('cart_percentile_keep_percent', 1.0)
        
        # Filtre Ouverture-Fermeture (Opening-Closing) - Nettoyage morphologique
        self.declare_parameter('cart_enable_opening_closing', False)
        self.declare_parameter('cart_opening_kernel_size', 3)
        self.declare_parameter('cart_closing_kernel_size', 3)
        self.declare_parameter('cart_opening_iterations', 1)
        self.declare_parameter('cart_closing_iterations', 1)
        
        # ========== SUBSCRIPTIONS ==========
        self.sonar_sub = self.create_subscription(
            Frame,
            '/docking/sonar/raw',
            self.frame_callback,
            10
        )
        
        self.tracker_sub = self.create_subscription(
            TrackedObject,
            '/docking/tracking/tracked_object',
            self.tracked_object_callback,
            10
        )
        
        # ========== PUBLISHERS ==========
        self.polar_pub = self.create_publisher(Frame, '/docking/sonar/polar_filtered', 10)
        self.cartesian_pub = self.create_publisher(FrameCartesian, '/docking/sonar/cartesian_filtered', 10)
        
        # ========== ÉTAT INTERNE ==========
        self.last_tracked_position = None
        self.last_tracked_is_tracking = False
        
        # Cache conversion polaire → cartésien
        self._mapping_cache = {
            'bearing_count': None,
            'range_count': None,
            'coords': None,
            'out_shape': None,
        }
        
        # Cache filtre spatial
        self._spatial_cache = {
            'center': None,
            'shape': None,
            'resolution': None,
            'origin_x': None,
            'mask': None,
        }
        
        self.get_logger().info('Traitement Unified node démarré')
    
    # ========== CALLBACK TRACKER ==========
    
    def tracked_object_callback(self, msg: TrackedObject):
        """Mémorise la position trackée pour le filtre spatial."""
        self.last_tracked_is_tracking = msg.is_tracking
        if msg.is_tracking:
            self.last_tracked_position = (msg.center_x, msg.center_y)
    
    # ========== FILTRES POLAIRES ==========
    
    def apply_frost_filter(self, img: np.ndarray) -> np.ndarray:
        """Filtre Frost - Réduction adaptative du speckle."""
        window_size = int(self.get_parameter('polar_frost_window_size').value)
        damping = float(self.get_parameter('polar_frost_damping').value)
        img_float = img.astype(np.float32)
        
        mean = ndimage.uniform_filter(img_float, size=window_size)
        sqr_mean = ndimage.uniform_filter(img_float**2, size=window_size)
        variance = np.maximum(sqr_mean - mean**2, 0)
        
        cv = np.sqrt(variance) / (mean + 1e-10)
        weights = np.exp(-damping * cv)
        
        filtered = weights * img_float + (1 - weights) * mean
        return np.clip(filtered, 0, 255).astype(np.uint8)
    
    def apply_median_filter(self, img: np.ndarray) -> np.ndarray:
        """Filtre Médian sur données polaires."""
        kernel = int(self.get_parameter('polar_median_kernel').value)
        return ndimage.median_filter(img, size=kernel)
    
    # ========== CONVERSION POLAIRE → CARTÉSIEN ==========
    
    def polar_to_cartesian(self, polar_img: np.ndarray, frame_msg: Frame) -> tuple:
        """Convertit image polaire en cartésienne avec interpolation bilinéaire.
        
        TRANSFORMATION T2 - Voir COORDINATE_TRANSFORMS.md
        
        Convention d'entrée (image polaire):
            - Axe 0 (lignes) = bearing, de -FOV/2 (gauche) à +FOV/2 (droite)
            - Axe 1 (colonnes) = range, de min_range à max_range
        
        Convention de sortie (image cartésienne):
            - Axe 0 (lignes) = Y, de 0 (ROV) à max_range (avant)
            - Axe 1 (colonnes) = X, de -max_range (gauche) à +max_range (droite)
            - origin_x = centre horizontal (colonne du ROV)
            - origin_y = 0 (ligne du ROV)
        
        NOTE IMPORTANTE sur arctan2(-xv, yv):
            On utilise -xv car les bearings dans l'image polaire sont ordonnés
            de gauche à droite (bearing croissant), ce qui correspond à X 
            décroissant dans notre convention cartésienne. L'inversion de X
            dans arctan2 compense cet ordre.
        
        Returns:
            tuple: (cart_img, resolution_m_per_px, origin_x, origin_y, total_angle_rad)
        """
        bc = frame_msg.bearing_count
        rc = frame_msg.range_count
        max_r = frame_msg.max_range
        min_r = frame_msg.min_range
        total_angle = frame_msg.bearing_resolution * bc  # Angle total du FOV en radians
        
        scale_factor = float(self.get_parameter('cartesian_scale_factor').value)
        cache = self._mapping_cache
        
        if cache['bearing_count'] != bc or cache['range_count'] != rc:
            out_h = rc  # Hauteur = nombre de bins de range
            out_w = int(scale_factor * rc)  # Largeur = scale * range (image plus large)
            
            # Grille cartésienne de destination
            xs = np.linspace(-max_r, max_r, out_w)  # X: gauche(-) → droite(+)
            ys = np.linspace(0, max_r, out_h)       # Y: ROV(0) → avant(max)
            xv, yv = np.meshgrid(xs, ys)
            
            # Calcul des coordonnées polaires correspondantes
            rr = np.sqrt(xv**2 + yv**2)  # Range = distance au ROV
            # ATTENTION: -xv pour corriger l'ordre des bearings (voir docstring)
            th = np.arctan2(-xv, yv)     # Bearing = arctan2(x, y) en convention sonar
            
            # Mapping vers les indices de l'image polaire source
            i_float = (th + total_angle / 2.0) / total_angle * (bc - 1)  # Index bearing
            j_float = (rr - min_r) / (max_r - min_r) * (rc - 1)          # Index range
            
            coords = np.vstack((i_float.ravel(), j_float.ravel()))
            
            cache['bearing_count'] = bc
            cache['range_count'] = rc
            cache['coords'] = coords
            cache['out_shape'] = (out_h, out_w)
        else:
            coords = cache['coords']
            out_h, out_w = cache['out_shape']
        
        sampled = map_coordinates(
            polar_img.astype(np.float32),
            coords,
            order=1,
            mode='constant',
            cval=0.0
        )
        cart_img = sampled.reshape((out_h, out_w)).astype(np.uint8)
        
        resolution = (2.0 * max_r) / out_w
        origin_x = out_w // 2
        origin_y = 0
        
        return cart_img, resolution, origin_x, origin_y, total_angle
    
    # ========== FILTRES CARTÉSIENS ==========
    
    def apply_spatial_filter(self, img: np.ndarray, resolution: float, 
                             origin_x: int) -> np.ndarray:
        """Filtre gaussien spatial centré sur la position trackée."""
        if not self.get_parameter('enable_spatial_filter').value:
            return img
        
        if self.last_tracked_position is None or not self.last_tracked_is_tracking:
            return img
        
        center_x_m_tracker, center_y_m = self.last_tracked_position
        sigma_m = float(self.get_parameter('spatial_filter_sigma').value)
        
        height, width = img.shape
        center_x_m_real = center_x_m_tracker  # Correction flip
        
        cache = self._spatial_cache
        cache_valid = (
            cache['center'] is not None and
            cache['shape'] == (height, width) and
            abs(cache['resolution'] - resolution) < 1e-6 and
            cache['origin_x'] == origin_x and
            abs(cache['center'][0] - center_x_m_real) < 0.05 and
            abs(cache['center'][1] - center_y_m) < 0.05
        )
        
        if cache_valid:
            mask = cache['mask']
        else:
            j_indices = np.arange(width)
            i_indices = np.arange(height)
            j_grid, i_grid = np.meshgrid(j_indices, i_indices)
            
            x_grid_m = (j_grid - origin_x) * resolution
            y_grid_m = (height - i_grid) * resolution
            
            dist_m = np.sqrt((x_grid_m - center_x_m_real)**2 + (y_grid_m - center_y_m)**2)
            mask = np.exp(-(dist_m**2) / (2 * sigma_m**2))
            
            cache['center'] = (center_x_m_real, center_y_m)
            cache['shape'] = (height, width)
            cache['resolution'] = resolution
            cache['origin_x'] = origin_x
            cache['mask'] = mask
        
        return (img.astype(np.float32) * mask).astype(np.uint8)
    
    def apply_cart_median_filter(self, img: np.ndarray) -> np.ndarray:
        """Filtre Médian cartésien - Supprime le bruit poivre et sel."""
        if not self.get_parameter('cart_enable_median').value:
            return img
        
        kernel_size = int(self.get_parameter('cart_median_kernel_size').value)
        # Assure que kernel_size est impair
        if kernel_size % 2 == 0:
            kernel_size += 1
        return cv2.medianBlur(img, kernel_size)
    
    def apply_clahe_filter(self, img: np.ndarray) -> np.ndarray:
        """Filtre CLAHE - Amélioration adaptative du contraste local."""
        if not self.get_parameter('cart_enable_clahe').value:
            return img
        
        clip_limit = float(self.get_parameter('cart_clahe_clip_limit').value)
        tile_size = int(self.get_parameter('cart_clahe_tile_grid_size').value)
        
        clahe = cv2.createCLAHE(
            clipLimit=clip_limit,
            tileGridSize=(tile_size, tile_size)
        )
        return clahe.apply(img)
    
    def apply_threshold_filter(self, img: np.ndarray) -> np.ndarray:
        """Filtre Seuil - Met à zéro les pixels sous le seuil (SEUL FILTRE BINARISANT)."""
        if not self.get_parameter('cart_enable_threshold').value:
            return img
        
        min_threshold = int(self.get_parameter('cart_min_intensity_threshold').value)
        # Threshold to zero: pixels < threshold deviennent 0, les autres gardent leur valeur
        _, thresholded = cv2.threshold(img, min_threshold, 255, cv2.THRESH_TOZERO)
        return thresholded
    
    def apply_morphology_filter(self, img: np.ndarray) -> np.ndarray:
        """Filtre Morphologique (Closing) - Comble les discontinuités des barres."""
        if not self.get_parameter('cart_enable_morphology').value:
            return img
        
        kernel_size = int(self.get_parameter('cart_morph_kernel_size').value)
        iterations = int(self.get_parameter('cart_morph_iterations').value)
        
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (kernel_size, kernel_size)
        )
        # Closing = Dilatation + Érosion (solidifie les objets)
        return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel, iterations=iterations)
    
    def apply_flip_filter(self, img: np.ndarray) -> np.ndarray:
        """Filtre Géométrique - Retourne l'image horizontalement et/ou verticalement.
        
        ATTENTION: Ces flips sont optionnels et désactivés par défaut.
        Voir COORDINATE_TRANSFORMS.md pour la documentation complète.
        
        Convention de l'image cartésienne SANS flip:
            - Ligne 0 = Y proche du ROV (y ≈ 0)
            - Ligne max = Y loin du ROV (y = max_range)  
            - Colonne 0 = X gauche (x = -max_range)
            - Colonne max = X droite (x = +max_range)
        
        cv2.flip codes:
            0 = flip vertical (miroir horizontal, échange haut/bas)
            1 = flip horizontal (miroir vertical, échange gauche/droite)
           -1 = flip les deux (rotation 180°)
        """
        flip_h = self.get_parameter('cart_flip_horizontal').value
        flip_v = self.get_parameter('cart_flip_vertical').value
        
        if not flip_h and not flip_v:
            return img
        
        if flip_h and flip_v:
            return cv2.flip(img, -1)  # Flip horizontal + vertical (rotation 180°)
        elif flip_h:
            return cv2.flip(img, 1)   # Flip horizontal: gauche ↔ droite
        elif flip_v:
            return cv2.flip(img, 0)   # Flip vertical: haut ↔ bas
        
        return img
    
    def apply_percentile_binarize_filter(self, img: np.ndarray) -> np.ndarray:
        """Filtre Binarisation Percentile - Garde uniquement les X% pixels les plus intenses.
        
        Ce filtre calcule le seuil correspondant au percentile (100 - keep_percent)
        et met à 255 tous les pixels au-dessus de ce seuil, 0 sinon.
        
        Args:
            img: Image en niveaux de gris (uint8)
            
        Returns:
            Image binarisée avec seulement les pixels les plus intenses à 255
        """
        if not self.get_parameter('cart_enable_percentile_binarize').value:
            return img
        
        keep_percent = float(self.get_parameter('cart_percentile_keep_percent').value)
        
        # Clamp entre 0.1% et 100%
        keep_percent = max(0.1, min(100.0, keep_percent))
        
        # Calcul du percentile (ex: garder 1% = percentile 99)
        percentile_value = 100.0 - keep_percent
        
        # Ne considérer que les pixels non-nuls pour le calcul du seuil
        non_zero_pixels = img[img > 0]
        
        if len(non_zero_pixels) == 0:
            return np.zeros_like(img)
        
        threshold = np.percentile(non_zero_pixels, percentile_value)
        
        # Binarisation: pixels >= seuil deviennent 255, sinon 0
        result = np.where(img >= threshold, 255, 0).astype(np.uint8)
        
        return result
    
    def apply_opening_closing_filter(self, img: np.ndarray) -> np.ndarray:
        """Filtre Ouverture-Fermeture - Nettoyage morphologique après binarisation.
        
        Opening (Ouverture) = Erosion + Dilatation:
            - Supprime les petits objets isolés (bruit)
            - Sépare les objets faiblement connectés
        
        Closing (Fermeture) = Dilatation + Erosion:
            - Comble les petits trous dans les objets
            - Connecte les objets proches
        
        L'ordre Opening puis Closing permet de:
            1. D'abord nettoyer le bruit (opening)
            2. Puis solidifier les structures restantes (closing)
        
        Args:
            img: Image binaire (uint8)
            
        Returns:
            Image nettoyée morphologiquement
        """
        if not self.get_parameter('cart_enable_opening_closing').value:
            return img
        
        opening_kernel_size = int(self.get_parameter('cart_opening_kernel_size').value)
        closing_kernel_size = int(self.get_parameter('cart_closing_kernel_size').value)
        opening_iterations = int(self.get_parameter('cart_opening_iterations').value)
        closing_iterations = int(self.get_parameter('cart_closing_iterations').value)
        
        result = img.copy()

        # Closing (Fermeture) - Solidifie les structures
        if closing_iterations > 0:
            closing_kernel = cv2.getStructuringElement(
                cv2.MORPH_RECT,
                (closing_kernel_size, closing_kernel_size)
            )
            result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, closing_kernel,
                                       iterations=closing_iterations)
        
        # Opening (Ouverture) - Supprime le bruit
        if opening_iterations > 0:
            opening_kernel = cv2.getStructuringElement(
                cv2.MORPH_RECT,
                (opening_kernel_size, opening_kernel_size)
            )
            result = cv2.morphologyEx(result, cv2.MORPH_OPEN, opening_kernel, 
                                       iterations=opening_iterations)
        

        
        return result
    
    # ========== CALLBACK PRINCIPAL ==========
    
    def frame_callback(self, msg: Frame):
        """Pipeline de traitement complet."""
        # Reconstruction image polaire
        polar_img = np.array(msg.intensities, dtype=np.uint8).reshape(
            (msg.bearing_count, msg.range_count)
        )
        
        # ========== FILTRES POLAIRES ==========
        filtered_polar = polar_img.copy()
        
        if self.get_parameter('polar_enable_frost').value:
            filtered_polar = self.apply_frost_filter(filtered_polar)
        
        if self.get_parameter('polar_enable_median').value:
            filtered_polar = self.apply_median_filter(filtered_polar)
        
        # Publication polaire filtrée
        polar_msg = Frame()
        polar_msg.header = msg.header
        polar_msg.range_count = msg.range_count
        polar_msg.bearing_count = msg.bearing_count
        polar_msg.range_resolution = msg.range_resolution
        polar_msg.bearing_resolution = msg.bearing_resolution
        polar_msg.min_range = msg.min_range
        polar_msg.max_range = msg.max_range
        polar_msg.intensities = filtered_polar.flatten().tolist()
        polar_msg.sound_speed = msg.sound_speed
        polar_msg.gain = msg.gain
        self.polar_pub.publish(polar_msg)
        
        # ========== CONVERSION + FILTRES CARTÉSIENS ==========
        cart_img, resolution, origin_x, origin_y, total_angle = self.polar_to_cartesian(
            filtered_polar, msg
        )
        
        filtered_cart = cart_img.copy()
        
        # 1. Flip géométrique (correction d'orientation)
        filtered_cart = self.apply_flip_filter(filtered_cart)
        
        # 2. Filtre Médian (denoising)
        filtered_cart = self.apply_cart_median_filter(filtered_cart)
        
        # 3. CLAHE (amélioration contraste)
        filtered_cart = self.apply_clahe_filter(filtered_cart)
        
        # 4. Filtre spatial gaussien (focus sur zone trackée)
        if self.get_parameter('enable_spatial_filter').value:
            filtered_cart = self.apply_spatial_filter(filtered_cart, resolution, origin_x)
        
        # 5. Seuil bas (SEUL FILTRE BINARISANT - nettoie les faibles intensités)
        filtered_cart = self.apply_threshold_filter(filtered_cart)
        
        # 6. Morphologie (closing - solidifie les structures)
        filtered_cart = self.apply_morphology_filter(filtered_cart)
        
        # 7. Binarisation percentile (garde les X% pixels les plus intenses)
        filtered_cart = self.apply_percentile_binarize_filter(filtered_cart)
        
        # 8. Ouverture-Fermeture (nettoyage morphologique après binarisation)
        filtered_cart = self.apply_opening_closing_filter(filtered_cart)
        
        # Publication cartésienne filtrée
        cart_msg = FrameCartesian()
        cart_msg.header = msg.header
        cart_msg.width = filtered_cart.shape[1]
        cart_msg.height = filtered_cart.shape[0]
        cart_msg.resolution = resolution
        cart_msg.min_range = msg.min_range
        cart_msg.max_range = msg.max_range
        cart_msg.origin_x = origin_x
        cart_msg.origin_y = origin_y
        cart_msg.intensities = filtered_cart.flatten().tolist()
        cart_msg.sound_speed = msg.sound_speed
        cart_msg.gain = msg.gain
        cart_msg.total_angle = total_angle
        self.cartesian_pub.publish(cart_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TraitementUnifiedNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()