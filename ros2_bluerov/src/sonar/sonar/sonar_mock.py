"""
Nœud ROS2 mock simulant le sonar Oculus M750d.
Publie des frames synthétiques avec cage simulée.
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame
import numpy as np


class SonarMockNode(Node):
    """Mock du sonar pour développement sans matériel."""
    
    def __init__(self):
        super().__init__('sonar_mock')
        
        # Paramètres
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('range_count', 512)
        self.declare_parameter('bearing_count', 256)
        self.declare_parameter('min_range', 1.0)  # m
        self.declare_parameter('max_range', 40.0)  # m
        self.declare_parameter('cage_distance', 8.0)  # m
        self.declare_parameter('cage_width', 2.0)  # m
        self.declare_parameter('noise_level', 20.0)  # 0-100
        
        rate = self.get_parameter('publish_rate').value
        self.range_count = self.get_parameter('range_count').value
        self.bearing_count = self.get_parameter('bearing_count').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.cage_distance = self.get_parameter('cage_distance').value
        self.cage_width = self.get_parameter('cage_width').value
        self.noise_level = self.get_parameter('noise_level').value
        
        # Publisher
        self.publisher_ = self.create_publisher(Frame, '/docking/sonar/raw', 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / rate, self.publish_frame)
        
        self.get_logger().info(f'Sonar mock démarré: {rate} Hz, cage @ {self.cage_distance}m')
    
    def generate_synthetic_frame(self) -> np.ndarray:
        """Génère une frame sonar synthétique avec cage."""
        # Grille polaire
        ranges = np.linspace(self.min_range, self.max_range, self.range_count)
        bearings = np.linspace(-np.pi/2, np.pi/2, self.bearing_count)
        
        # Image de base (bruit)
        intensities = np.random.randint(0, int(self.noise_level), 
                                       (self.bearing_count, self.range_count),
                                       dtype=np.uint8)
        
        # Simuler 4 montants de cage (forte intensité)
        cage_half_width = self.cage_width / 2.0
        
        # Positions angulaires des montants à distance donnée
        # tan(bearing) = x / y => bearing = atan(x / distance)
        bearing_left1 = np.arctan(-cage_half_width / self.cage_distance)
        bearing_left2 = np.arctan(-(cage_half_width * 0.7) / self.cage_distance)
        bearing_right1 = np.arctan((cage_half_width * 0.7) / self.cage_distance)
        bearing_right2 = np.arctan(cage_half_width / self.cage_distance)
        
        montant_bearings = [bearing_left1, bearing_left2, bearing_right1, bearing_right2]
        
        # Dessiner montants
        for mb in montant_bearings:
            # Trouver index de bearing le plus proche
            bearing_idx = np.argmin(np.abs(bearings - mb))
            
            # Trouver index de range correspondant à distance cage
            range_idx = np.argmin(np.abs(ranges - self.cage_distance))
            
            # Ajouter intensité (montant vertical = plusieurs ranges)
            range_width = 5  # Épaisseur du montant en bins
            bearing_width = 3  # Largeur angulaire
            
            for db in range(-bearing_width, bearing_width + 1):
                for dr in range(-range_width, range_width + 1):
                    b_idx = np.clip(bearing_idx + db, 0, self.bearing_count - 1)
                    r_idx = np.clip(range_idx + dr, 0, self.range_count - 1)
                    intensities[b_idx, r_idx] = 200 + np.random.randint(0, 55)
        
        return intensities
    
    def publish_frame(self):
        """Publie une frame sonar."""
        msg = Frame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sonar_link'
        
        msg.range_count = self.range_count
        msg.bearing_count = self.bearing_count
        msg.range_resolution = (self.max_range - self.min_range) / self.range_count
        msg.bearing_resolution = np.pi / self.bearing_count
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        
        # Génération image synthétique
        intensities = self.generate_synthetic_frame()
        msg.intensities = intensities.flatten().tolist()
        
        msg.sound_speed = 1500.0  # m/s (eau douce)
        msg.gain = 50
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SonarMockNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
