"""
Nœud de sélection manuelle de bounding box pour le tracking.
Affiche l'image sonar filtrée et permet à l'utilisateur de dessiner une bbox
avec la souris pour initialiser le tracker CSRT.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from docking_msgs.msg import FrameCartesian, BBoxSelection

try:
    import cv2
except ImportError:
    cv2 = None


class BBoxSelectorNode(Node):
    """Permet la sélection manuelle de bounding box via une fenêtre OpenCV."""
    
    def __init__(self):
        super().__init__('bbox_selector_node')
        
        # Paramètres
        self.declare_parameter('window_name', 'Sélection BBox - Appuyez sur ESPACE pour sélectionner')
        self.declare_parameter('display_scale', 1.0)  # Facteur d'échelle d'affichage
        
        # État
        self.current_frame = None
        self.current_frame_msg = None
        self.selecting = False
        self.bbox = None
        self.start_point = None
        self.end_point = None
        
        # Subscription
        self.frame_sub = self.create_subscription(
            FrameCartesian,
            '/docking/sonar/cartesian_filtered',
            self.frame_callback,
            10
        )
        
        # Publisher
        self.bbox_pub = self.create_publisher(
            BBoxSelection,
            '/docking/sonar/bbox_selection',
            10
        )
        
        # Timer pour affichage et gestion des événements
        self.timer = self.create_timer(0.033, self.display_callback)  # ~30 FPS
        
        self.get_logger().info('BBox Selector node démarré')
        self.get_logger().info('Instructions:')
        self.get_logger().info('  - Dessinez un rectangle avec la souris')
        self.get_logger().info('  - Appuyez sur ESPACE pour valider la sélection')
        self.get_logger().info('  - Appuyez sur ESC ou Q pour quitter')
        
        if cv2 is None:
            self.get_logger().error('OpenCV non disponible!')
    
    def mouse_callback(self, event, x, y, flags, param):
        """Gère les événements souris pour dessiner la bbox."""
        if self.current_frame is None:
            return
        
        # Récupérer le facteur d'échelle pour convertir les coordonnées
        scale = self.get_parameter('display_scale').value
        
        # Convertir les coordonnées affichées vers les coordonnées de l'image originale
        orig_x = int(x / scale)
        orig_y = int(y / scale)
        
        if event == cv2.EVENT_LBUTTONDOWN:
            # Début de la sélection (coordonnées affichées)
            self.selecting = True
            self.start_point = (x, y)
            self.end_point = (x, y)
            self.get_logger().info(f'Début sélection à ({x}, {y}) -> image originale ({orig_x}, {orig_y})')
        
        elif event == cv2.EVENT_MOUSEMOVE:
            # Mise à jour pendant le drag
            if self.selecting:
                self.end_point = (x, y)
        
        elif event == cv2.EVENT_LBUTTONUP:
            # Fin de la sélection
            self.selecting = False
            self.end_point = (x, y)
            
            # Calculer la bbox (coordonnées affichées)
            x1, y1 = self.start_point
            x2, y2 = self.end_point
            
            # Normaliser (x1,y1) = coin supérieur gauche (affichage)
            bbox_x_display = min(x1, x2)
            bbox_y_display = min(y1, y2)
            bbox_w_display = abs(x2 - x1)
            bbox_h_display = abs(y2 - y1)
            
            # Convertir vers coordonnées originales
            bbox_x = int(bbox_x_display / scale)
            bbox_y = int(bbox_y_display / scale)
            bbox_w = int(bbox_w_display / scale)
            bbox_h = int(bbox_h_display / scale)
            
            self.get_logger().info(
                f'Fin sélection: affichage=({bbox_x_display}, {bbox_y_display}, {bbox_w_display}, {bbox_h_display}) '
                f'-> original=({bbox_x}, {bbox_y}, {bbox_w}, {bbox_h})'
            )
            
            if bbox_w > 5 and bbox_h > 5:  # Taille minimale sur image originale
                self.bbox = (bbox_x, bbox_y, bbox_w, bbox_h)
                self.get_logger().info(f'✓ BBox valide enregistrée - Appuyez sur ESPACE pour confirmer')
            else:
                self.get_logger().warn(f'✗ BBox trop petite ({bbox_w}x{bbox_h}), ignorée - Minimum 5x5 pixels')
                self.bbox = None
    
    def frame_callback(self, msg: FrameCartesian):
        """Reçoit une frame cartésienne."""
        # Reconstruire l'image
        img = np.array(msg.intensities, dtype=np.uint8).reshape(
            (msg.height, msg.width)
        )
        
        # Convertir en BGR pour l'affichage
        img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        self.current_frame = img_bgr
        self.current_frame_msg = msg
    
    def display_callback(self):
        """Affiche l'image et gère les interactions."""
        if cv2 is None or self.current_frame is None:
            return
        
        # Copie pour dessiner sans modifier l'original
        display_img = self.current_frame.copy()
        scale = self.get_parameter('display_scale').value
        
        # Dessiner le rectangle en cours de sélection
        if self.selecting and self.start_point and self.end_point:
            cv2.rectangle(display_img, self.start_point, self.end_point, (0, 255, 0), 2)
        
        # Dessiner la bbox validée
        if self.bbox is not None:
            x, y, w, h = self.bbox
            cv2.rectangle(display_img, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(
                display_img, 
                'SPACE pour confirmer', 
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )
        
        # Redimensionner si nécessaire
        if scale != 1.0:
            new_width = int(display_img.shape[1] * scale)
            new_height = int(display_img.shape[0] * scale)
            display_img = cv2.resize(display_img, (new_width, new_height))
        
        # Afficher
        window_name = self.get_parameter('window_name').value
        cv2.imshow(window_name, display_img)
        cv2.setMouseCallback(window_name, self.mouse_callback)
        
        # Gestion clavier
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord(' '):  # ESPACE = valider
            self.get_logger().info('ESPACE détecté !')
            if self.bbox is not None:
                self._publish_bbox()
                self.get_logger().info('✓ BBox publiée et réinitialisée')
                self.bbox = None
                self.start_point = None
                self.end_point = None
            else:
                self.get_logger().warn('✗ Aucune bbox à publier - Dessinez d\'abord un rectangle avec la souris')
        
        elif key == ord('q') or key == 27:  # Q ou ESC = quitter
            self.get_logger().info('Fermeture du sélecteur de bbox')
            cv2.destroyAllWindows()
            rclpy.shutdown()
    
    def _publish_bbox(self):
        """Publie la bbox sélectionnée."""
        if self.bbox is None:
            return
        
        x, y, w, h = self.bbox
        
        msg = BBoxSelection()
        msg.header = self.current_frame_msg.header
        msg.x = int(x)
        msg.y = int(y)
        msg.width = int(w)
        msg.height = int(h)
        msg.is_valid = True
        
        self.bbox_pub.publish(msg)
        self.get_logger().info(f'BBox publiée: ({x}, {y}, {w}, {h})')
    
    def __del__(self):
        """Ferme les fenêtres OpenCV à la destruction."""
        if cv2 is not None:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = BBoxSelectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if cv2 is not None:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
