"""
Nœud de tracking CSRT simplifié pour sonar.
Tracker basique avec bbox rectangle (sans gestion d'orientation).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from docking_msgs.msg import FrameCartesian, BBoxSelection, TrackedObject

import cv2



class CSRTTrackerNode(Node):
    """Tracker CSRT simplifié pour suivre la cage dans les images sonar cartésiennes."""
    
    def __init__(self):
        super().__init__('csrt_tracker_node')
        
        # ========== PARAMÈTRES ==========
        self.declare_parameter('enable_tracking', True)
        
        # Paramètres CSRT (learning rates pour adaptation progressive)
        self.declare_parameter('filter_lr', 0.02)  # Apprentissage du filtre (plus haut = s'adapte plus vite)
        self.declare_parameter('scale_lr', 0.02)  # Apprentissage de l'échelle
        self.declare_parameter('psr_threshold', 0.02)  # Seuil de confiance (bas = plus permissif)
        
        # État du tracker
        self.tracker = None
        self.is_initialized = False
        self.is_tracking = False
        self.last_bbox = None  # (x, y, w, h) en pixels
        self.current_resolution = None
        
        # Subscription pour les frames
        self.frame_sub = self.create_subscription(
            FrameCartesian,
            '/docking/sonar/cartesian_filtered',
            self.frame_callback,
            10
        )
        
        # Subscription pour la sélection de bbox
        self.bbox_sub = self.create_subscription(
            BBoxSelection,
            '/docking/sonar/bbox_selection',
            self.bbox_callback,
            10
        )
        
        # Publisher
        self.tracked_pub = self.create_publisher(
            TrackedObject,
            '/docking/tracking/tracked_object',
            10
        )
        
        self._pending_init = None
        
        self.get_logger().info('CSRT Tracker node démarré (mode simplifié)')
        
        if cv2 is None:
            self.get_logger().error('OpenCV non disponible!')
    
    def _create_tracker(self):
        """Crée un nouveau tracker CSRT."""
        if cv2 is None:
            return None
        
        params = cv2.TrackerCSRT_Params()
        params.use_hog = True
        params.use_color_names = False
        params.use_gray = True
        params.use_segmentation = False
        params.filter_lr = float(self.get_parameter('filter_lr').value)
        params.scale_lr = float(self.get_parameter('scale_lr').value)
        params.psr_threshold = float(self.get_parameter('psr_threshold').value)
        
        return cv2.TrackerCSRT_create(params)
    
    def _pixels_to_meters(self, x_px, y_px, frame_msg):
        """Convertit des coordonnées en pixels vers des mètres."""
        x_m = (x_px - frame_msg.origin_x) * frame_msg.resolution
        y_m = (frame_msg.height - y_px) * frame_msg.resolution
        return x_m, y_m
    
    def bbox_callback(self, msg: BBoxSelection):
        """Initialise le tracker avec une bbox manuelle."""
        if not self.get_parameter('enable_tracking').value:
            return
        
        if cv2 is None or not msg.is_valid:
            return
        
        self._pending_init = (msg.x, msg.y, msg.width, msg.height)
        self.get_logger().info(f'BBox reçue: ({msg.x}, {msg.y}, {msg.width}, {msg.height})')
    
    def frame_callback(self, msg: FrameCartesian):
        """Traite une frame et met à jour le tracking."""
        if not self.get_parameter('enable_tracking').value or cv2 is None:
            return
        
        # Reconstruire l'image
        img = np.array(msg.intensities, dtype=np.uint8).reshape((msg.height, msg.width))
        img = cv2.equalizeHist(img)
        img = cv2.flip(img, 0)  # Flip vertical uniquement (symétrie axe horizontal)
        img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        if not img_bgr.flags['C_CONTIGUOUS']:
            img_bgr = np.ascontiguousarray(img_bgr)
        
        self.current_resolution = msg.resolution
        
        # Initialisation en attente?
        if self._pending_init is not None:
            bbox_x, bbox_y, bbox_w, bbox_h = self._pending_init
            self._pending_init = None
            
            # Flipper uniquement Y car l'image est flippée verticalement
            bbox_y_flipped = msg.height - bbox_y - bbox_h
            
            self.tracker = self._create_tracker()
            if self.tracker is not None:
                try:
                    self.tracker.init(img_bgr, (bbox_x, bbox_y_flipped, bbox_w, bbox_h))
                    self.is_initialized = True
                    self.is_tracking = True
                    self.last_bbox = (bbox_x, bbox_y_flipped, bbox_w, bbox_h)
                    self.get_logger().info('✓ Tracker initialisé')
                except Exception as e:
                    self.get_logger().error(f'Erreur init: {e}')
        
        # Mise à jour du tracking
        if self.is_initialized and self.tracker is not None:
            success, bbox = self.tracker.update(img_bgr)
            
            if success:
                self.is_tracking = True
                self.last_bbox = tuple(map(int, bbox))
            else:
                self.is_tracking = False
        
        # Publier le résultat
        self._publish_tracked_object(msg)
    
    def _publish_tracked_object(self, frame_msg):
        """Publie l'état actuel du tracking."""
        out_msg = TrackedObject()
        out_msg.header = frame_msg.header
        out_msg.is_initialized = self.is_initialized
        out_msg.is_tracking = self.is_tracking
        
        if self.is_tracking and self.last_bbox is not None:
            bbox_x, bbox_y, bbox_w, bbox_h = self.last_bbox
            
            # Re-flipper uniquement Y pour l'affichage
            bbox_y_display = frame_msg.height - bbox_y - bbox_h
            
            out_msg.bbox_x = int(bbox_x)
            out_msg.bbox_y = int(bbox_y_display)
            out_msg.bbox_width = int(bbox_w)
            out_msg.bbox_height = int(bbox_h)
            
            # Centre en pixels puis en mètres (utiliser les coordonnées flippées)
            center_x_px = bbox_x + bbox_w // 2
            center_y_px = frame_msg.height - bbox_y_display - bbox_h // 2
            center_x_m, center_y_m = self._pixels_to_meters(center_x_px, center_y_px, frame_msg)
            
            # CORRECTION: Inversion du signe de X
            # ====================================
            # La conversion polaire→cartésien utilise arctan2(-xv, yv) (T2),
            # ce qui inverse l'axe X de l'image cartésienne par rapport à
            # la convention sonar (X+ = droite).
            # On compense ici en inversant le signe de center_x.
            center_x_m = -center_x_m

            out_msg.center_x = float(center_x_m)
            out_msg.center_y = float(center_y_m)
            out_msg.range = float(np.sqrt(center_x_m**2 + center_y_m**2))
            out_msg.bearing = float(np.arctan2(center_x_m, center_y_m))
            out_msg.angle = 0.0
            out_msg.width = float(bbox_w * frame_msg.resolution)
            out_msg.height = float(bbox_h * frame_msg.resolution)
            out_msg.confidence = 0.0
            
            # Pas de points d'entrée en mode simplifié
            out_msg.entry_p1_x = 0.0
            out_msg.entry_p1_y = 0.0
            out_msg.entry_p1_range = 0.0
            out_msg.entry_p1_bearing = 0.0
            out_msg.entry_p2_x = 0.0
            out_msg.entry_p2_y = 0.0
            out_msg.entry_p2_range = 0.0
            out_msg.entry_p2_bearing = 0.0
        else:
            # Valeurs par défaut
            out_msg.center_x = 0.0
            out_msg.center_y = 0.0
            out_msg.range = 0.0
            out_msg.bearing = 0.0
            out_msg.angle = 0.0
            out_msg.width = 0.0
            out_msg.height = 0.0
            out_msg.bbox_x = 0
            out_msg.bbox_y = 0
            out_msg.bbox_width = 0
            out_msg.bbox_height = 0
            out_msg.confidence = 0.0
            out_msg.entry_p1_x = 0.0
            out_msg.entry_p1_y = 0.0
            out_msg.entry_p1_range = 0.0
            out_msg.entry_p1_bearing = 0.0
            out_msg.entry_p2_x = 0.0
            out_msg.entry_p2_y = 0.0
            out_msg.entry_p2_range = 0.0
            out_msg.entry_p2_bearing = 0.0
        
        self.tracked_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CSRTTrackerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()