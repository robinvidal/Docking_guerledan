"""
Nœud ROS2 mock simulant le sonar Oculus M750d.
Publie des frames synthétiques avec cage simulée.
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame
import numpy as np
from docking_utils.filters import (
    median_filter, gaussian_filter, contrast_enhancement,
    range_compensation
)


class SonarMockNode(Node):
    """Mock du sonar pour développement sans matériel."""
    
    def __init__(self):
        super().__init__('sonar_mock')
        
        # Paramètres sonar
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('range_count', 512)
        self.declare_parameter('bearing_count', 256)
        self.declare_parameter('bearing_angle', 140.0)  # Ouverture totale en degrés
        self.declare_parameter('min_range', 1.0)  # m
        self.declare_parameter('max_range', 40.0)  # m
        self.declare_parameter('cage_distance', 8.0)  # m
        self.declare_parameter('cage_width', 2.0)  # m
        self.declare_parameter('noise_level', 20.0)  # 0-100
        
        # Paramètres de filtrage
        self.declare_parameter('enable_median', True)
        self.declare_parameter('median_kernel', 3)
        self.declare_parameter('enable_gaussian', True)
        self.declare_parameter('gaussian_sigma', 1.5)
        self.declare_parameter('enable_contrast', True)
        self.declare_parameter('contrast_clip', 2.0)
        self.declare_parameter('enable_range_comp', True)
        self.declare_parameter('range_comp_alpha', 0.0001)
        
        rate = self.get_parameter('publish_rate').value
        self.range_count = self.get_parameter('range_count').value
        self.bearing_count = self.get_parameter('bearing_count').value
        self.bearing_angle = self.get_parameter('bearing_angle').value  # degrés
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.cage_distance = self.get_parameter('cage_distance').value
        self.cage_width = self.get_parameter('cage_width').value
        self.noise_level = self.get_parameter('noise_level').value
        
        # Publisher (publie données filtrées sur /docking/sonar/raw)
        self.publisher_ = self.create_publisher(Frame, '/docking/sonar/raw', 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / rate, self.publish_frame)
        
        self.get_logger().info(f'Sonar mock démarré: {rate} Hz, cage @ {self.cage_distance}m')
    
    def generate_synthetic_frame(self) -> np.ndarray:
        """Génère une frame sonar synthétique avec cage."""
        # Grille polaire (utilise bearing_angle du config)
        ranges = np.linspace(self.min_range, self.max_range, self.range_count)
        half_angle = self.bearing_angle / 2.0  # moitié de l'ouverture
        bearings = np.linspace(-half_angle * np.pi/180, half_angle * np.pi/180, self.bearing_count)
        
        # Image de base: bruit plus réaliste
        # - bruit gaussien de fond (std = noise_level)
        # - speckle / salt-and-pepper aléatoire proportionnel à noise_level
        # noise_level est interprété comme un écart-type raisonnable (0-100)
        std = max(1.0, float(self.noise_level))
        mean_bg = max(0.0, std * 0.2)
        intensities = np.random.normal(loc=mean_bg, scale=std,
                                       size=(self.bearing_count, self.range_count))

        # Ajouter speckle / impulsions (salt-and-pepper) : probabilité dépend de noise_level
        sp_prob = min(0.12, float(self.noise_level) / 400.0)  # borné pour éviter trop d'impulsions
        if sp_prob > 0.0:
            mask = np.random.rand(self.bearing_count, self.range_count) < sp_prob
            # choisir principalement impulsions hautes (retours forts) et quelques creux
            high_or_low = np.random.rand(mask.sum()) < 0.7
            intensities[mask] = np.where(high_or_low, 200 + np.random.randint(0, 55, size=mask.sum()), 0)

        # Clip et conversion en uint8
        intensities = np.clip(intensities, 0, 255).astype(np.uint8)
        
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
            range_width = 2  # Épaisseur du montant en bins
            bearing_width = 3  # Largeur angulaire
            
            for db in range(-bearing_width, bearing_width + 1):
                for dr in range(-range_width, range_width + 1):
                    b_idx = np.clip(bearing_idx + db, 0, self.bearing_count - 1)
                    r_idx = np.clip(range_idx + dr, 0, self.range_count - 1)
                    intensities[b_idx, r_idx] = 200 + np.random.randint(0, 55)
        
        return intensities
    
    def publish_frame(self):
        """Publie une frame sonar filtrée sur /docking/sonar/raw."""
        timestamp = self.get_clock().now().to_msg()
        
        # Génération image synthétique brute
        intensities = self.generate_synthetic_frame()
        
        # Application des filtres
        filtered = intensities.copy()
        
        if self.get_parameter('enable_median').value:
            kernel = self.get_parameter('median_kernel').value
            filtered = median_filter(filtered, kernel)
        
        if self.get_parameter('enable_gaussian').value:
            sigma = self.get_parameter('gaussian_sigma').value
            filtered = gaussian_filter(filtered, sigma)
        
        if self.get_parameter('enable_range_comp').value:
            alpha = self.get_parameter('range_comp_alpha').value
            ranges = np.linspace(self.min_range, self.max_range, self.range_count)
            filtered = range_compensation(filtered, ranges, alpha)
        
        if self.get_parameter('enable_contrast').value:
            clip = self.get_parameter('contrast_clip').value
            filtered = contrast_enhancement(filtered, clip)
        
        # Publication sur /docking/sonar/raw (contient données filtrées)
        msg = Frame()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'sonar_link'
        msg.range_count = self.range_count
        msg.bearing_count = self.bearing_count
        msg.range_resolution = (self.max_range - self.min_range) / self.range_count
        msg.bearing_resolution = (self.bearing_angle * np.pi / 180.0) / self.bearing_count  # angle total en rad / nb faisceaux
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.intensities = filtered.flatten().tolist()
        msg.sound_speed = 1500.0
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
