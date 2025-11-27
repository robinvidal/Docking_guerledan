"""
Nœud de calcul de la pose relative du ROV par rapport à la cage.
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Borders, PoseRelative
import numpy as np
from docking_utils.geometry import compute_cage_center, compute_cage_orientation, validate_cage_geometry
from docking_utils.conversions import polar_to_cartesian


class LocalisationNode(Node):
    """Calcule la pose 6DOF du ROV dans le repère de la cage."""
    
    def __init__(self):
        super().__init__('localisation_node')
        
        # Paramètres
        self.declare_parameter('cage_width', 2.0)
        self.declare_parameter('cage_depth', 2.0)
        self.declare_parameter('min_borders_confidence', 0.7)
        
        # Subscription
        self.subscription = self.create_subscription(
            Borders,
            '/docking/tracking/borders',
            self.borders_callback,
            10
        )
        
        # Publisher
        self.publisher_ = self.create_publisher(PoseRelative, '/docking/localisation/pose', 10)
        
        self.get_logger().info('Localisation node démarré')
    
    def borders_callback(self, msg: Borders):
        """Calcule la pose à partir des bords détectés."""
        pose_msg = PoseRelative()
        pose_msg.header = msg.header
        
        if not msg.is_valid or len(msg.ranges) != 4:
            pose_msg.is_valid = False
            pose_msg.confidence = 0.0
            self.publisher_.publish(pose_msg)
            return
        
        # Conversion en arrays
        ranges = np.array(msg.ranges)
        bearings = np.array(msg.bearings)
        confidences = np.array(msg.confidences)
        
        # Validation géométrie
        expected_width = self.get_parameter('cage_width').value
        expected_depth = self.get_parameter('cage_depth').value
        
        is_valid, validation_msg = validate_cage_geometry(
            ranges, bearings, expected_width, expected_depth
        )
        
        if not is_valid:
            self.get_logger().warn(f'Géométrie invalide: {validation_msg}')
            pose_msg.is_valid = False
            pose_msg.confidence = 0.0
            self.publisher_.publish(pose_msg)
            return
        
        # Calcul centre cage (repère sonar)
        x_center, y_center = compute_cage_center(ranges, bearings)
        
        # Calcul orientation cage
        yaw = compute_cage_orientation(ranges, bearings)
        
        # Dans le repère cage, le ROV est à la position inverse
        # (si cage est à (x, y) du ROV, le ROV est à (-x, -y) de la cage)
        pose_msg.x = -x_center
        pose_msg.y = y_center  # Distance positive = ROV devant cage
        pose_msg.z = 0.0  # Sonar 2D, pas d'info verticale
        
        pose_msg.yaw = -yaw  # Angle relatif
        pose_msg.pitch = 0.0
        pose_msg.roll = 0.0
        
        # Confiance = moyenne des confidences de détection
        pose_msg.confidence = float(np.mean(confidences))
        pose_msg.is_valid = pose_msg.confidence >= self.get_parameter('min_borders_confidence').value
        
        # Covariance (estimation simple)
        # Variance augmente avec la distance
        var_x = 0.1 + 0.01 * y_center  # ±10cm + 1% distance
        var_y = 0.1 + 0.01 * y_center
        var_z = 0.5  # Pas d'info z, variance élevée
        var_yaw = 0.05  # ±3°
        
        cov = np.zeros(36)
        cov[0] = var_x ** 2  # x
        cov[7] = var_y ** 2  # y
        cov[14] = var_z ** 2  # z
        cov[21] = 0.1  # roll
        cov[28] = 0.1  # pitch
        cov[35] = var_yaw ** 2  # yaw
        
        pose_msg.covariance = cov.tolist()
        
        self.publisher_.publish(pose_msg)
        
        if pose_msg.is_valid:
            self.get_logger().info(
                f'Pose: x={pose_msg.x:.2f}m, y={pose_msg.y:.2f}m, '
                f'yaw={np.rad2deg(pose_msg.yaw):.1f}°, conf={pose_msg.confidence:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LocalisationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
