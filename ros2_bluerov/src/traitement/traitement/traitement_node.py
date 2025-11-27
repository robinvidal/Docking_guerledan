"""
Nœud de filtrage et prétraitement des données sonar.
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame
import numpy as np
from docking_utils.filters import (
    median_filter, gaussian_filter, contrast_enhancement,
    range_compensation, morphological_opening
)


class TraitementNode(Node):
    """Applique filtres sur frames sonar brutes."""
    
    def __init__(self):
        super().__init__('traitement_node')
        
        # Paramètres
        self.declare_parameter('enable_median', True)
        self.declare_parameter('median_kernel', 3)
        self.declare_parameter('enable_gaussian', True)
        self.declare_parameter('gaussian_sigma', 1.5)
        self.declare_parameter('enable_contrast', True)
        self.declare_parameter('contrast_clip', 2.0)
        self.declare_parameter('enable_range_comp', True)
        self.declare_parameter('range_comp_alpha', 0.0001)
        
        # Subscription
        self.subscription = self.create_subscription(
            Frame,
            '/docking/sonar/raw',
            self.frame_callback,
            10
        )
        
        # Publisher
        self.publisher_ = self.create_publisher(Frame, '/docking/sonar/filtered', 10)
        
        self.get_logger().info('Traitement node démarré')
    
    def frame_callback(self, msg: Frame):
        """Traite une frame sonar."""
        # Reconstruction image 2D
        img = np.array(msg.intensities, dtype=np.uint8).reshape(
            (msg.bearing_count, msg.range_count)
        )
        
        # Pipeline de filtrage
        filtered = img.copy()
        
        if self.get_parameter('enable_median').value:
            kernel = self.get_parameter('median_kernel').value
            filtered = median_filter(filtered, kernel)
        
        if self.get_parameter('enable_gaussian').value:
            sigma = self.get_parameter('gaussian_sigma').value
            filtered = gaussian_filter(filtered, sigma)
        
        if self.get_parameter('enable_range_comp').value:
            alpha = self.get_parameter('range_comp_alpha').value
            ranges = np.linspace(msg.min_range, msg.max_range, msg.range_count)
            filtered = range_compensation(filtered, ranges, alpha)
        
        if self.get_parameter('enable_contrast').value:
            clip = self.get_parameter('contrast_clip').value
            filtered = contrast_enhancement(filtered, clip)
        
        # Publication
        out_msg = Frame()
        out_msg.header = msg.header
        out_msg.range_count = msg.range_count
        out_msg.bearing_count = msg.bearing_count
        out_msg.range_resolution = msg.range_resolution
        out_msg.bearing_resolution = msg.bearing_resolution
        out_msg.min_range = msg.min_range
        out_msg.max_range = msg.max_range
        out_msg.intensities = filtered.flatten().tolist()
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
