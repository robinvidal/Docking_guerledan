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
        # Filtre Médian
        self.declare_parameter('polar_enable_median', True)
        self.declare_parameter('polar_median_kernel', 3)
        
        # Filtre Frost
        self.declare_parameter('polar_enable_frost', True)
        self.declare_parameter('polar_frost_window_size', 7)
        self.declare_parameter('polar_frost_damping', 3.4)
        
        # ========== PARAMÈTRES CARTÉSIENS ==========
        self.declare_parameter('cartesian_scale_factor', 2.0)
        
        # Filtre spatial gaussien
        self.declare_parameter('enable_spatial_filter', True)
        self.declare_parameter('spatial_filter_radius', 2.0)
        self.declare_parameter('spatial_filter_sigma', 0.8)
        
        # Binarisation par centile
        self.declare_parameter('cart_enable_percentile_binarization', True)
        self.declare_parameter('cart_percentile_threshold', 95.0)
        
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
        """Convertit image polaire en cartésienne avec interpolation."""
        bc = frame_msg.bearing_count
        rc = frame_msg.range_count
        max_r = frame_msg.max_range
        min_r = frame_msg.min_range
        total_angle = frame_msg.bearing_resolution * bc
        
        scale_factor = float(self.get_parameter('cartesian_scale_factor').value)
        cache = self._mapping_cache
        
        if cache['bearing_count'] != bc or cache['range_count'] != rc:
            out_h = rc
            out_w = int(scale_factor * rc)
            
            xs = np.linspace(-max_r, max_r, out_w)
            ys = np.linspace(0, max_r, out_h)
            xv, yv = np.meshgrid(xs, ys)
            
            rr = np.sqrt(xv**2 + yv**2)
            th = np.arctan2(-xv, yv)
            
            i_float = (th + total_angle / 2.0) / total_angle * (bc - 1)
            j_float = (rr - min_r) / (max_r - min_r) * (rc - 1)
            
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
        center_x_m_real = -center_x_m_tracker  # Correction flip
        
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
    
    def apply_percentile_binarization(self, img: np.ndarray) -> np.ndarray:
        """Binarisation par centile - garde les N% pixels les plus lumineux."""
        percentile = float(self.get_parameter('cart_percentile_threshold').value)
        threshold = np.percentile(img, percentile)
        return np.where(img >= threshold, 255, 0).astype(np.uint8)
    
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
        
        if self.get_parameter('enable_spatial_filter').value:
            filtered_cart = self.apply_spatial_filter(filtered_cart, resolution, origin_x)
        
        if self.get_parameter('cart_enable_percentile_binarization').value:
            filtered_cart = self.apply_percentile_binarization(filtered_cart)
        
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
