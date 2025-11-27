"""
Nœud ROS2 driver réel pour sonar Oculus M750d.
À implémenter avec le SDK Oculus.
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame


class SonarNode(Node):
    """Driver sonar Oculus M750d (stub - à implémenter)."""
    
    def __init__(self):
        super().__init__('sonar_node')
        
        # Paramètres
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('ip_address', '192.168.1.10')
        self.declare_parameter('port', 52102)
        self.declare_parameter('range', 40.0)
        self.declare_parameter('gain', 50.0)
        
        # Publisher
        self.publisher_ = self.create_publisher(Frame, '/docking/sonar/raw', 10)
        
        # TODO: Initialiser connexion Oculus SDK
        # self.sonar_device = OculusM750d(ip=..., port=...)
        
        self.get_logger().warn('Driver Oculus M750d non implémenté - utilisez sonar_mock')
        self.get_logger().info('Sonar node prêt (mode stub)')
    
    def connect_sonar(self):
        """Établit connexion avec le sonar."""
        # TODO: Implémenter avec SDK Oculus
        pass
    
    def process_ping(self, ping_data):
        """
        Traite un ping sonar reçu et publie Frame.
        
        Args:
            ping_data: Données brutes du ping Oculus
        """
        # TODO: Convertir données Oculus -> docking_msgs/Frame
        msg = Frame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sonar_link'
        
        # Extraction des données Oculus
        # msg.range_count = ping_data.num_ranges
        # msg.bearing_count = ping_data.num_beams
        # ...
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SonarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
