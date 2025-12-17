"""
Nœud de détection de lignes par transformée de Hough.
S'abonne aux données cartésiennes filtrées et détecte les n meilleures lignes.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from docking_msgs.msg import FrameCartesian, DetectedLines

try:
    import cv2
except ImportError:
    cv2 = None


class HoughLinesNode(Node):
    """Détecte des lignes avec transformée de Hough sur images cartésiennes."""
    
    def __init__(self):
        super().__init__('hough_lines_node')
        
        # Paramètres de détection
        self.declare_parameter('enable_detection', True)
        self.declare_parameter('num_lines', 5)  # Nombre de lignes à détecter
        self.declare_parameter('rho_resolution', 1.0)  # Résolution en distance (pixels)
        self.declare_parameter('theta_resolution', 1.0)  # Résolution angulaire (degrés)
        self.declare_parameter('threshold', 50)  # Seuil de votes minimum
        self.declare_parameter('min_line_length', 20)  # Longueur minimale (pixels)
        self.declare_parameter('max_line_gap', 10)  # Gap maximal (pixels)
        self.declare_parameter('use_probabilistic', True)  # Utiliser HoughLinesP
        
        # Subscription
        self.subscription = self.create_subscription(
            FrameCartesian,
            '/docking/sonar/cartesian_filtered',
            self.frame_callback,
            10
        )
        
        # Publisher
        self.publisher_ = self.create_publisher(
            DetectedLines,
            '/docking/tracking/detected_lines',
            10
        )
        
        self.get_logger().info('Hough Lines node démarré')
    
    def frame_callback(self, msg: FrameCartesian):
        """Traite une frame cartésienne et détecte les lignes."""
        if not self.get_parameter('enable_detection').value:
            return
        
        if cv2 is None:
            self.get_logger().warn('OpenCV non disponible, détection impossible', throttle_duration_sec=5.0)
            return
        
        # Reconstruction de l'image
        img = np.array(msg.intensities, dtype=np.uint8).reshape(
            (msg.height, msg.width)
        )
        
        # Paramètres
        num_lines = int(self.get_parameter('num_lines').value)
        rho_res = float(self.get_parameter('rho_resolution').value)
        theta_res = float(self.get_parameter('theta_resolution').value) * np.pi / 180.0
        threshold = int(self.get_parameter('threshold').value)
        use_prob = self.get_parameter('use_probabilistic').value
        
        # Détection des lignes
        if use_prob:
            min_length = int(self.get_parameter('min_line_length').value)
            max_gap = int(self.get_parameter('max_line_gap').value)
            lines = cv2.HoughLinesP(
                img,
                rho=rho_res,
                theta=theta_res,
                threshold=threshold,
                minLineLength=min_length,
                maxLineGap=max_gap
            )
        else:
            lines = cv2.HoughLines(
                img,
                rho=rho_res,
                theta=theta_res,
                threshold=threshold
            )
        
        # Construction du message de sortie
        out_msg = DetectedLines()
        out_msg.header = msg.header
        
        if lines is None or len(lines) == 0:
            out_msg.is_valid = False
            out_msg.num_lines = 0
            out_msg.rhos = []
            out_msg.thetas = []
            out_msg.x1_points = []
            out_msg.y1_points = []
            out_msg.x2_points = []
            out_msg.y2_points = []
            out_msg.confidences = []
            self.publisher_.publish(out_msg)
            return
        
        # Conversion pixels -> mètres
        resolution = msg.resolution
        max_range = msg.max_range
        min_range = msg.min_range
        origin_x = msg.origin_x
        origin_y = msg.origin_y
        
        # Limiter au nombre demandé
        num_detected = min(num_lines, len(lines))
        
        rhos = []
        thetas = []
        x1_points = []
        y1_points = []
        x2_points = []
        y2_points = []
        confidences = []
        
        if use_prob:
            # HoughLinesP retourne des segments [x1, y1, x2, y2]
            for i in range(num_detected):
                x1_px, y1_px, x2_px, y2_px = lines[i][0]
                
                # Conversion pixels -> mètres (coordonnées cartésiennes)
                # L'image Hough travaille sur l'image AVANT rotation de 90°
                # Dans cette image: axe 0 (y_px) = Y frontal, axe 1 (x_px) = X latéral
                # Inversion de x car l'axe x_px croît vers la droite mais x_m doit croître vers la gauche
                # x_m = -(x_px - origin_x) * resolution  (latéral, inversé)
                # y_m = (y_px - origin_y) * resolution + min_range  (frontal)
                x1_m = -(float(x1_px) - origin_x) * resolution
                y1_m = (float(y1_px) - origin_y) * resolution + min_range
                x2_m = -(float(x2_px) - origin_x) * resolution
                y2_m = (float(y2_px) - origin_y) * resolution + min_range
                
                # Calcul des paramètres polaires (rho, theta) de la ligne
                # Direction de la ligne
                dx = x2_m - x1_m
                dy = y2_m - y1_m
                
                if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                    continue
                
                # Angle de la ligne (direction)
                line_angle = np.arctan2(dy, dx)
                # Normale à la ligne
                theta = line_angle + np.pi / 2.0
                # Normaliser theta dans [0, pi]
                while theta < 0:
                    theta += np.pi
                while theta >= np.pi:
                    theta -= np.pi
                
                # Distance perpendiculaire à l'origine
                # rho = x * cos(theta) + y * sin(theta)
                rho = x1_m * np.cos(theta) + y1_m * np.sin(theta)
                
                # Si rho est négatif, on inverse
                if rho < 0:
                    rho = -rho
                    theta = theta + np.pi
                    if theta >= np.pi:
                        theta -= np.pi
                
                rhos.append(float(rho))
                thetas.append(float(theta))
                x1_points.append(float(x1_m))
                y1_points.append(float(y1_m))
                x2_points.append(float(x2_m))
                y2_points.append(float(y2_m))
                
                # Confiance basée sur la longueur du segment
                length = np.sqrt(dx**2 + dy**2)
                conf = min(1.0, length / (max_range * 0.5))
                confidences.append(float(conf))
        
        else:
            # HoughLines retourne [rho, theta]
            for i in range(num_detected):
                rho_px, theta = lines[i][0]
                
                # Conversion rho de pixels vers mètres
                rho_m = float(rho_px) * resolution
                
                # Calcul des points d'intersection avec les bordures
                # On trouve les intersections avec les 4 bordures de l'image
                cos_t = np.cos(theta)
                sin_t = np.sin(theta)
                
                # Bordures en mètres
                x_min = -max_range
                x_max = max_range
                y_min = min_range
                y_max = max_range
                
                intersections = []
                
                # Intersection avec x = x_min
                if abs(sin_t) > 1e-6:
                    y = (rho_m - x_min * cos_t) / sin_t
                    if y_min <= y <= y_max:
                        intersections.append((x_min, y))
                
                # Intersection avec x = x_max
                if abs(sin_t) > 1e-6:
                    y = (rho_m - x_max * cos_t) / sin_t
                    if y_min <= y <= y_max:
                        intersections.append((x_max, y))
                
                # Intersection avec y = y_min
                if abs(cos_t) > 1e-6:
                    x = (rho_m - y_min * sin_t) / cos_t
                    if x_min <= x <= x_max:
                        intersections.append((x, y_min))
                
                # Intersection avec y = y_max
                if abs(cos_t) > 1e-6:
                    x = (rho_m - y_max * sin_t) / cos_t
                    if x_min <= x <= x_max:
                        intersections.append((x, y_max))
                
                if len(intersections) >= 2:
                    x1_m, y1_m = intersections[0]
                    x2_m, y2_m = intersections[1]
                    
                    rhos.append(float(rho_m))
                    thetas.append(float(theta))
                    x1_points.append(float(x1_m))
                    y1_points.append(float(y1_m))
                    x2_points.append(float(x2_m))
                    y2_points.append(float(y2_m))
                    
                    # Confiance proportionnelle à rho (lignes proches = plus confiantes)
                    conf = max(0.0, 1.0 - rho_m / max_range)
                    confidences.append(float(conf))
        
        # Publication
        out_msg.is_valid = len(rhos) > 0
        out_msg.num_lines = len(rhos)
        out_msg.rhos = rhos
        out_msg.thetas = thetas
        out_msg.x1_points = x1_points
        out_msg.y1_points = y1_points
        out_msg.x2_points = x2_points
        out_msg.y2_points = y2_points
        out_msg.confidences = confidences
        
        self.publisher_.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HoughLinesNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
