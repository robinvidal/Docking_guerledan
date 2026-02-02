"""
Nœud de filtrage et prétraitement des données sonar.
Applique des filtres sur données polaires et cartésiennes.
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame
import numpy as np
from scipy import ndimage
try:
    import cv2
except ImportError:
    cv2 = None


class TraitementNode(Node):
    """Applique filtres sur frames sonar (polaires et cartésiennes)."""
    
    def __init__(self):
        super().__init__('traitement_node')
        
        # ========== FILTRES POLAIRES ==========
        # Filtre Gaussien
        self.declare_parameter('polar_enable_gaussian', True)
        self.declare_parameter('polar_gaussian_sigma', 1.0)
        
        # Filtre Médian
        self.declare_parameter('polar_enable_median', True)
        self.declare_parameter('polar_median_kernel', 3)
        
        # Filtre Loss (Lee speckle reduction)
        self.declare_parameter('polar_enable_loss', False)
        self.declare_parameter('polar_loss_window_size', 5)
        
        # Filtre Frost (adaptive speckle filter)
        self.declare_parameter('polar_enable_frost', False)
        self.declare_parameter('polar_frost_window_size', 5)
        self.declare_parameter('polar_frost_damping', 1.0)
        
        # Log Compression
        self.declare_parameter('polar_enable_log_compression', False)
        self.declare_parameter('polar_log_scale', 30.0)  # Facteur d'échelle pour log
        
        # ========== FILTRES CARTÉSIENS ==========
        # Canny Edge Detection
        self.declare_parameter('cart_enable_canny', False)
        self.declare_parameter('cart_canny_threshold1', 50.0)
        self.declare_parameter('cart_canny_threshold2', 150.0)
        self.declare_parameter('cart_canny_aperture', 3)
        
        # Binarisation par centiles
        self.declare_parameter('cart_enable_percentile_binarization', False)
        self.declare_parameter('cart_percentile_threshold', 90.0)  # Garder les 10% les plus lumineux
        
        # Conversion polaire vers cartésien
        self.declare_parameter('enable_cartesian_conversion', True)
        self.declare_parameter('cartesian_output_size', 512)  # Taille de l'image cartésienne (NxN)
        
        # Subscription
        self.subscription = self.create_subscription(
            Frame,
            '/docking/sonar/raw',
            self.frame_callback,
            10
        )
        
        # Publisher
        self.publisher_ = self.create_publisher(Frame, '/docking/sonar/filtered', 10)
        
        self.get_logger().info('Traitement node démarré avec filtres polaires et cartésiens')
    
    # ========== FILTRES POLAIRES ==========
    
    def polar_gaussian_filter(self, img: np.ndarray) -> np.ndarray:
        """Filtre Gaussien sur données polaires."""
        sigma = float(self.get_parameter('polar_gaussian_sigma').value)
        return ndimage.gaussian_filter(img, sigma=sigma)
    
    def polar_median_filter(self, img: np.ndarray) -> np.ndarray:
        """Filtre Médian sur données polaires."""
        kernel = int(self.get_parameter('polar_median_kernel').value)
        return ndimage.median_filter(img, size=kernel)
    
    def polar_loss_filter(self, img: np.ndarray) -> np.ndarray:
        """
        Filtre Loss (Lee) - Réduction du speckle.
        Filtre adaptatif basé sur la variance locale.
        """
        window_size = int(self.get_parameter('polar_loss_window_size').value)
        img_float = img.astype(np.float32)
        
        # Moyenne et variance locales
        mean = ndimage.uniform_filter(img_float, size=window_size)
        sqr_mean = ndimage.uniform_filter(img_float**2, size=window_size)
        variance = sqr_mean - mean**2
        variance = np.maximum(variance, 0)  # Éviter variance négative par arrondi
        
        # Variance du bruit (estimée sur toute l'image)
        noise_variance = np.var(img_float)
        
        # Coefficient de pondération adaptatif
        # Si variance locale >> variance bruit => garder le signal
        # Si variance locale ≈ variance bruit => lisser
        weights = np.where(
            variance > noise_variance,
            (variance - noise_variance) / (variance + 1e-10),
            0
        )
        
        # Filtrage adaptatif
        filtered = mean + weights * (img_float - mean)
        
        return np.clip(filtered, 0, 255).astype(np.uint8)
    
    def polar_frost_filter(self, img: np.ndarray) -> np.ndarray:
        """
        Filtre Frost - Réduction adaptative du speckle.
        Utilise un filtre exponentiel pondéré par la variance locale.
        """
        window_size = int(self.get_parameter('polar_frost_window_size').value)
        damping = float(self.get_parameter('polar_frost_damping').value)
        img_float = img.astype(np.float32)
        
        # Moyenne locale
        mean = ndimage.uniform_filter(img_float, size=window_size)
        
        # Variance locale
        sqr_mean = ndimage.uniform_filter(img_float**2, size=window_size)
        variance = sqr_mean - mean**2
        variance = np.maximum(variance, 0)
        
        # Coefficient de variation local
        cv = np.sqrt(variance) / (mean + 1e-10)
        
        # Paramètre du filtre Frost
        k = damping * cv
        
        # Poids exponentiels (simplifié)
        weights = np.exp(-k)
        
        # Filtrage
        filtered = weights * img_float + (1 - weights) * mean
        
        return np.clip(filtered, 0, 255).astype(np.uint8)
    
    def polar_log_compression(self, img: np.ndarray) -> np.ndarray:
        """
        Compression logarithmique des intensités.
        Améliore la visualisation des faibles échos.
        """
        scale = float(self.get_parameter('polar_log_scale').value)
        img_float = img.astype(np.float32)
        
        # Compression log: I_out = scale * log(1 + I_in)
        compressed = scale * np.log1p(img_float)
        
        # Normalisation vers [0, 255]
        compressed = (compressed / compressed.max()) * 255.0
        
        return compressed.astype(np.uint8)
    
    # ========== CONVERSION POLAIRE → CARTÉSIEN ==========
    
    def polar_to_cartesian(self, polar_img: np.ndarray, msg: Frame) -> np.ndarray:
        """
        Convertit une image polaire (bearing × range) en cartésienne (x × y).
        """
        output_size = int(self.get_parameter('cartesian_output_size').value)
        bearing_count, range_count = polar_img.shape
        
        # Paramètres du sonar
        min_range = msg.min_range
        max_range = msg.max_range
        bearing_step = msg.bearing_resolution  # en radians
        
        # Création de la grille cartésienne
        cart_img = np.zeros((output_size, output_size), dtype=np.uint8)
        
        # Centre de l'image
        center = output_size // 2
        scale = (output_size / 2) / max_range  # pixels par mètre
        
        # Conversion
        for b in range(bearing_count):
            angle = b * bearing_step - np.pi / 2  # Angle en radians (0 = devant)
            
            for r in range(range_count):
                # Distance réelle
                distance = min_range + r * msg.range_resolution
                
                # Coordonnées cartésiennes
                x = center + int(distance * np.cos(angle) * scale)
                y = center + int(distance * np.sin(angle) * scale)
                
                # Vérifier les limites
                if 0 <= x < output_size and 0 <= y < output_size:
                    cart_img[y, x] = polar_img[b, r]
        
        # Interpolation pour combler les trous
        if cv2 is not None:
            # Dilatation légère pour combler les pixels vides
            kernel = np.ones((3, 3), np.uint8)
            cart_img = cv2.morphologyEx(cart_img, cv2.MORPH_CLOSE, kernel)
        
        return cart_img
    
    def cartesian_to_polar(self, cart_img: np.ndarray, msg: Frame) -> np.ndarray:
        """
        Convertit une image cartésienne (x × y) en polaire (bearing × range).
        Reconversion après traitement cartésien pour publier au format polaire.
        """
        bearing_count = msg.bearing_count
        range_count = msg.range_count
        min_range = msg.min_range
        max_range = msg.max_range
        bearing_step = msg.bearing_resolution
        range_step = msg.range_resolution
        
        output_size = cart_img.shape[0]
        center = output_size // 2
        scale = (output_size / 2) / max_range
        
        # Création de l'image polaire
        polar_img = np.zeros((bearing_count, range_count), dtype=np.uint8)
        
        # Conversion inverse: pour chaque pixel polaire, trouver le pixel cartésien correspondant
        for b in range(bearing_count):
            angle = b * bearing_step - np.pi / 2
            
            for r in range(range_count):
                distance = min_range + r * range_step
                
                # Coordonnées cartésiennes correspondantes
                x = center + int(distance * np.cos(angle) * scale)
                y = center + int(distance * np.sin(angle) * scale)
                
                # Lecture de la valeur dans l'image cartésienne
                if 0 <= x < output_size and 0 <= y < output_size:
                    polar_img[b, r] = cart_img[y, x]
        
        return polar_img
    
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
        """Traite une frame sonar avec filtres polaires et cartésiens."""
        # Reconstruction image 2D polaire (bearing × range)
        polar_img = np.array(msg.intensities, dtype=np.uint8).reshape(
            (msg.bearing_count, msg.range_count)
        )
        
        # ========== APPLICATION DES FILTRES POLAIRES ==========
        filtered_polar = polar_img.copy()
        
        # Filtre Gaussien
        if self.get_parameter('polar_enable_gaussian').value:
            filtered_polar = self.polar_gaussian_filter(filtered_polar)
        
        # Filtre Médian
        if self.get_parameter('polar_enable_median').value:
            filtered_polar = self.polar_median_filter(filtered_polar)
        
        # Filtre Loss
        if self.get_parameter('polar_enable_loss').value:
            filtered_polar = self.polar_loss_filter(filtered_polar)
        
        # Filtre Frost
        if self.get_parameter('polar_enable_frost').value:
            filtered_polar = self.polar_frost_filter(filtered_polar)
        
        # Compression logarithmique
        if self.get_parameter('polar_enable_log_compression').value:
            filtered_polar = self.polar_log_compression(filtered_polar)
        
        # ========== CONVERSION CARTÉSIENNE (si activée) ==========
        if self.get_parameter('enable_cartesian_conversion').value:
            # Conversion polaire → cartésien
            cart_img = self.polar_to_cartesian(filtered_polar, msg)
            
            # ========== APPLICATION DES FILTRES CARTÉSIENS ==========
            filtered_cart = cart_img.copy()
            
            # Détection de contours Canny
            if self.get_parameter('cart_enable_canny').value:
                filtered_cart = self.cart_canny_edge(filtered_cart)
            
            # Binarisation par centiles
            if self.get_parameter('cart_enable_percentile_binarization').value:
                filtered_cart = self.cart_percentile_binarization(filtered_cart)
            
            # Reconversion cartésien → polaire pour publier au format polaire
            final_img = self.cartesian_to_polar(filtered_cart, msg)
        else:
            # Pas de conversion cartésienne, on garde le format polaire
            final_img = filtered_polar
        
        # Publication (toujours au format polaire)
        out_msg = Frame()
        out_msg.header = msg.header
        out_msg.range_count = msg.range_count
        out_msg.bearing_count = msg.bearing_count
        out_msg.range_resolution = msg.range_resolution
        out_msg.bearing_resolution = msg.bearing_resolution
        out_msg.min_range = msg.min_range
        out_msg.max_range = msg.max_range
        out_msg.intensities = final_img.flatten().tolist()
        out_msg.sound_speed = msg.sound_speed
        out_msg.gain = msg.gain
        
        self.publisher_.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TraitementNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
