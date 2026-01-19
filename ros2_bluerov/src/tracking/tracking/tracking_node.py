"""
Nœud de détection des bords de cage dans les données sonar.
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame, Borders
import numpy as np
from docking_utils.filters import adaptive_threshold
from scipy import ndimage


class TrackingNode(Node):
    """Détecte les 4 montants verticaux de la cage."""
    
    def __init__(self):
        super().__init__('tracking_node')
        
        # Paramètres
        self.declare_parameter('intensity_threshold', 150)
        self.declare_parameter('min_confidence', 0.7)
        self.declare_parameter('expected_cage_width', 2.0)
        self.declare_parameter('search_range_min', 0.0)  # m
        self.declare_parameter('search_range_max', 20.0)  # m
        
        # Subscription
        self.subscription = self.create_subscription(
            Frame,
            '/docking/sonar/filtered',
            self.frame_callback,
            10
        )
        
        # Publisher
        self.publisher_ = self.create_publisher(Borders, '/docking/tracking/borders', 10)
        
        self.get_logger().info('Tracking node démarré')
    
    def detect_peaks(self, signal: np.ndarray, threshold: float) -> np.ndarray:
        """Détecte les pics dans un signal 1D."""
        peaks = []
        for i in range(1, len(signal) - 1):
            if signal[i] > threshold and signal[i] > signal[i-1] and signal[i] > signal[i+1]:
                peaks.append(i)
        return np.array(peaks)
    
    def frame_callback(self, msg: Frame):
        """Détecte les bords dans une frame sonar."""
        # Reconstruction image
        img = np.array(msg.intensities, dtype=np.uint8).reshape(
            (msg.bearing_count, msg.range_count)
        )
        
        # Grille polaire
        ranges = np.linspace(msg.min_range, msg.max_range, msg.range_count)
        bearings = np.linspace(-np.pi/2, np.pi/2, msg.bearing_count)
        
        # Limitation zone de recherche
        range_min = self.get_parameter('search_range_min').value
        range_max = self.get_parameter('search_range_max').value
        range_mask = (ranges >= range_min) & (ranges <= range_max)
        
        # Projection angulaire (somme sur les ranges valides)
        angular_projection = np.sum(img[:, range_mask], axis=1)
        
        # Détection de pics (montants verticaux)
        threshold = self.get_parameter('intensity_threshold').value * np.sum(range_mask)
        peak_indices = self.detect_peaks(angular_projection, threshold)
        
        # Création message Borders
        borders_msg = Borders()
        borders_msg.header = msg.header
        
        if len(peak_indices) >= 4:
            # Tri et sélection des 4 pics les plus intenses
            peak_intensities = angular_projection[peak_indices]
            sorted_idx = np.argsort(peak_intensities)[-4:]  # 4 plus forts
            selected_peaks = peak_indices[sorted_idx]
            selected_peaks = np.sort(selected_peaks)  # Gauche->droite
            
            # Conversion indices -> angles
            detected_bearings = bearings[selected_peaks]
            
            # Pour chaque bearing, trouver la distance du montant
            detected_ranges = []
            confidences = []
            
            for bearing_idx in selected_peaks:
                # Profil radial à ce bearing
                radial_profile = img[bearing_idx, range_mask]
                
                # Pic de distance = montant
                if len(radial_profile) > 0:
                    peak_range_idx = np.argmax(radial_profile)
                    peak_range = ranges[range_mask][peak_range_idx]
                    confidence = radial_profile[peak_range_idx] / 255.0
                else:
                    peak_range = msg.max_range / 2.0
                    confidence = 0.0
                
                detected_ranges.append(peak_range)
                confidences.append(confidence)
            
            borders_msg.ranges = detected_ranges
            borders_msg.bearings = detected_bearings.tolist()
            borders_msg.confidences = confidences
            
            # Validation
            min_conf = self.get_parameter('min_confidence').value
            borders_msg.is_valid = all(c >= min_conf for c in confidences)
            
            # Géométrie cage estimée
            from docking_utils.conversions import polar_to_cartesian
            points = [polar_to_cartesian(r, b) for r, b in zip(detected_ranges, detected_bearings)]
            if len(points) >= 4:
                xs = [p[0] for p in points]
                borders_msg.cage_width = max(xs) - min(xs)
                borders_msg.cage_depth = np.mean([p[1] for p in points])
            
        else:
            # Pas assez de bords détectés
            borders_msg.is_valid = False
            borders_msg.ranges = []
            borders_msg.bearings = []
            borders_msg.confidences = []
            borders_msg.cage_width = 0.0
            borders_msg.cage_depth = 0.0
        
        self.publisher_.publish(borders_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
