"""
Nœud de filtrage cartésien.
Applique des filtres sur données cartésiennes (x × y).
Conversion polaire → cartésien identique à sonar_display.py.
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame, FrameCartesian
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
        
        # Publisher - données cartésiennes filtrées
        self.publisher_ = self.create_publisher(FrameCartesian, '/docking/sonar/cartesian_filtered', 10)
        
        # Cache pour la conversion polaire → cartésien
        self._mapping_cache = {
            'bearing_count': None,
            'range_count': None,
            'coords': None,
            'out_shape': None,
        }
        
        self.get_logger().info(f'Traitement Cartesian node démarré (subscribe to: {topic})')
    
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
        
        # Détection de contours Canny
        if self.get_parameter('cart_enable_canny').value:
            filtered_cart = self.cart_canny_edge(filtered_cart)
        
        # Binarisation par centiles
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
