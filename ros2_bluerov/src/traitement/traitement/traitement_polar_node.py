"""
Nœud de filtrage polaire.
Applique des filtres sur données polaires (bearing × range).
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame
import numpy as np
from scipy import ndimage


class TraitementPolarNode(Node):
    """Applique filtres sur frames sonar polaires."""
    
    def __init__(self):
        super().__init__('traitement_polar_node')
        
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
        self.declare_parameter('polar_log_scale', 30.0)
        
        # Subscription
        self.subscription = self.create_subscription(
            Frame,
            '/docking/sonar/raw',
            self.frame_callback,
            10
        )
        
        # Publisher - données polaires filtrées
        self.publisher_ = self.create_publisher(Frame, '/docking/sonar/polar_filtered', 10)
        
        self.get_logger().info('Traitement Polar node démarré')
    
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
        variance = np.maximum(variance, 0)
        
        # Variance du bruit (estimée sur toute l'image)
        noise_variance = np.var(img_float)
        
        # Coefficient de pondération adaptatif
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
        if compressed.max() > 0:
            compressed = (compressed / compressed.max()) * 255.0
        
        return compressed.astype(np.uint8)
    
    def frame_callback(self, msg: Frame):
        """Traite une frame sonar polaire."""
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
        
        # Publication (format polaire)
        out_msg = Frame()
        out_msg.header = msg.header
        out_msg.range_count = msg.range_count
        out_msg.bearing_count = msg.bearing_count
        out_msg.range_resolution = msg.range_resolution
        out_msg.bearing_resolution = msg.bearing_resolution
        out_msg.min_range = msg.min_range
        out_msg.max_range = msg.max_range
        out_msg.intensities = filtered_polar.flatten().tolist()
        out_msg.sound_speed = msg.sound_speed
        out_msg.gain = msg.gain
        
        self.publisher_.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TraitementPolarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
