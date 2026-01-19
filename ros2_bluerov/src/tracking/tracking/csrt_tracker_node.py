"""
Nœud de tracking CSRT pour sonar.
Utilise le tracker CSRT d'OpenCV pour suivre la cage dans les images cartésiennes filtrées.
Initialisé par un clic utilisateur, la bounding box est dimensionnée selon les dimensions
réelles de la cage définies dans le fichier de configuration.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from docking_msgs.msg import FrameCartesian, ClickPosition, TrackedObject

try:
    import cv2
except ImportError:
    cv2 = None


class CSRTTrackerNode(Node):
    """Tracker CSRT pour suivre la cage dans les images sonar cartésiennes."""
    
    def __init__(self):
        super().__init__('csrt_tracker_node')
        
        # ========== PARAMÈTRES ==========
        # Activation
        self.declare_parameter('enable_tracking', True)
        
        # Dimensions réelles de la cage (mètres)
        self.declare_parameter('cage_width', 0.9)   # Largeur de la cage (m)
        self.declare_parameter('cage_height', 0.5)  # Hauteur de la cage (m)
        
        # Paramètres du tracker CSRT
        self.declare_parameter('use_hog', True)
        self.declare_parameter('use_color_names', False)  # False pour grayscale
        self.declare_parameter('use_gray', True)
        self.declare_parameter('use_rgb', False)
        self.declare_parameter('use_channel_weights', True)
        self.declare_parameter('use_segmentation', True)
        self.declare_parameter('window_function', 'hann')  # hann, cheb, kaiser
        self.declare_parameter('kaiser_alpha', 3.75)
        self.declare_parameter('cheb_attenuation', 45.0)
        self.declare_parameter('template_size', 200.0)
        self.declare_parameter('gsl_sigma', 1.0)
        self.declare_parameter('hog_orientations', 9)
        self.declare_parameter('hog_clip', 0.2)
        self.declare_parameter('padding', 3.0)
        self.declare_parameter('filter_lr', 0.02)
        self.declare_parameter('weights_lr', 0.02)
        self.declare_parameter('num_hog_channels_used', 18)
        self.declare_parameter('admm_iterations', 4)
        self.declare_parameter('histogram_bins', 16)
        self.declare_parameter('histogram_lr', 0.04)
        self.declare_parameter('background_ratio', 2)
        self.declare_parameter('number_of_scales', 33)
        self.declare_parameter('scale_sigma_factor', 0.25)
        self.declare_parameter('scale_model_max_area', 512.0)
        self.declare_parameter('scale_lr', 0.025)
        self.declare_parameter('scale_step', 1.02)
        self.declare_parameter('psr_threshold', 0.035)  # Seuil de confiance
        
        # État du tracker
        self.tracker = None
        self.is_initialized = False
        self.is_tracking = False
        self.last_bbox = None  # (x, y, w, h) en pixels
        self.current_resolution = None  # mètres par pixel
        self.current_frame_info = None  # Info du dernier frame
        
        # Subscriptions
        self.frame_sub = self.create_subscription(
            FrameCartesian,
            '/docking/sonar/cartesian_filtered',
            self.frame_callback,
            10
        )
        
        self.click_sub = self.create_subscription(
            ClickPosition,
            '/docking/sonar/click_position',
            self.click_callback,
            10
        )
        
        # Publisher
        self.tracked_pub = self.create_publisher(
            TrackedObject,
            '/docking/tracking/tracked_object',
            10
        )
        
        self.get_logger().info('CSRT Tracker node démarré')
        
        if cv2 is None:
            self.get_logger().error('OpenCV non disponible! Le tracking ne fonctionnera pas.')
    
    def _create_tracker(self):
        """Crée un nouveau tracker CSRT avec les paramètres configurés."""
        if cv2 is None:
            return None
        
        # Créer les paramètres CSRT
        params = cv2.TrackerCSRT_Params()
        
        params.use_hog = self.get_parameter('use_hog').value
        params.use_color_names = self.get_parameter('use_color_names').value
        params.use_gray = self.get_parameter('use_gray').value
        params.use_rgb = self.get_parameter('use_rgb').value
        params.use_channel_weights = self.get_parameter('use_channel_weights').value
        params.use_segmentation = self.get_parameter('use_segmentation').value
        params.window_function = self.get_parameter('window_function').value
        params.kaiser_alpha = float(self.get_parameter('kaiser_alpha').value)
        params.cheb_attenuation = float(self.get_parameter('cheb_attenuation').value)
        params.template_size = float(self.get_parameter('template_size').value)
        params.gsl_sigma = float(self.get_parameter('gsl_sigma').value)
        params.hog_orientations = int(self.get_parameter('hog_orientations').value)
        params.hog_clip = float(self.get_parameter('hog_clip').value)
        params.padding = float(self.get_parameter('padding').value)
        params.filter_lr = float(self.get_parameter('filter_lr').value)
        params.weights_lr = float(self.get_parameter('weights_lr').value)
        params.num_hog_channels_used = int(self.get_parameter('num_hog_channels_used').value)
        params.admm_iterations = int(self.get_parameter('admm_iterations').value)
        params.histogram_bins = int(self.get_parameter('histogram_bins').value)
        params.histogram_lr = float(self.get_parameter('histogram_lr').value)
        params.background_ratio = int(self.get_parameter('background_ratio').value)
        params.number_of_scales = int(self.get_parameter('number_of_scales').value)
        params.scale_sigma_factor = float(self.get_parameter('scale_sigma_factor').value)
        params.scale_model_max_area = float(self.get_parameter('scale_model_max_area').value)
        params.scale_lr = float(self.get_parameter('scale_lr').value)
        params.scale_step = float(self.get_parameter('scale_step').value)
        params.psr_threshold = float(self.get_parameter('psr_threshold').value)
        
        return cv2.TrackerCSRT_create(params)
    
    def _meters_to_pixels(self, x_m, y_m, frame_msg):
        """Convertit des coordonnées en mètres vers des pixels."""
        resolution = frame_msg.resolution
        max_range = frame_msg.max_range
        
        # L'image a pour origine (0,0) en haut à gauche
        # x_m = 0 correspond au centre de l'image (origin_x)
        # y_m = 0 correspond au ROV (origin_y = 0, bas de l'image)
        
        # Conversion
        # x_px = (x_m / resolution) + origin_x
        # y_px = height - (y_m / resolution)  # Inverser car y augmente vers le haut
        
        x_px = int((x_m / resolution) + frame_msg.origin_x)
        y_px = int(frame_msg.height - (y_m / resolution))
        
        return x_px, y_px
    
    def _pixels_to_meters(self, x_px, y_px, frame_msg):
        """Convertit des coordonnées en pixels vers des mètres."""
        resolution = frame_msg.resolution
        
        x_m = (x_px - frame_msg.origin_x) * resolution
        y_m = (frame_msg.height - y_px) * resolution
        
        return x_m, y_m
    
    def _get_bbox_from_click(self, x_m, y_m, frame_msg):
        """
        Calcule la bounding box en pixels à partir d'un clic en mètres.
        La bbox est centrée sur le clic et dimensionnée selon les dimensions de la cage.
        """
        cage_width_m = float(self.get_parameter('cage_width').value)
        cage_height_m = float(self.get_parameter('cage_height').value)
        resolution = frame_msg.resolution
        
        # Dimensions en pixels
        bbox_w = int(cage_width_m / resolution)
        bbox_h = int(cage_height_m / resolution)
        
        # Position du centre en pixels
        center_x_px, center_y_px = self._meters_to_pixels(x_m, y_m, frame_msg)
        
        # Coin supérieur gauche
        bbox_x = center_x_px - bbox_w // 2
        bbox_y = center_y_px - bbox_h // 2
        
        # Clamp aux limites de l'image
        bbox_x = max(0, min(bbox_x, frame_msg.width - bbox_w))
        bbox_y = max(0, min(bbox_y, frame_msg.height - bbox_h))
        
        return (bbox_x, bbox_y, bbox_w, bbox_h)
    
    def click_callback(self, msg: ClickPosition):
        """Initialise le tracker lors d'un clic."""
        if not self.get_parameter('enable_tracking').value:
            return
        
        if cv2 is None:
            self.get_logger().error('OpenCV non disponible')
            return
        
        if not msg.is_valid:
            return
        
        # Stocker la position du clic pour initialisation lors du prochain frame
        self._pending_init = (msg.x, msg.y)
        self.get_logger().info(f'Clic reçu à ({msg.x:.2f}m, {msg.y:.2f}m) - initialisation en attente')
    
    def frame_callback(self, msg: FrameCartesian):
        """Traite une frame cartésienne et met à jour le tracking."""
        if not self.get_parameter('enable_tracking').value:
            return
        
        if cv2 is None:
            return
        
        # Reconstruire l'image
        img = np.array(msg.intensities, dtype=np.uint8).reshape(
            (msg.height, msg.width)
        )
        
        # Convertir en BGR pour OpenCV (CSRT fonctionne mieux avec 3 canaux)
        img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        self.current_frame_info = msg
        self.current_resolution = msg.resolution
        
        # Vérifier s'il y a une initialisation en attente
        if hasattr(self, '_pending_init') and self._pending_init is not None:
            x_m, y_m = self._pending_init
            self._pending_init = None
            
            # Calculer la bounding box
            bbox = self._get_bbox_from_click(x_m, y_m, msg)
            
            # Créer un nouveau tracker
            self.tracker = self._create_tracker()
            if self.tracker is None:
                self.get_logger().error('Impossible de créer le tracker CSRT')
                return
            
            # Initialiser le tracker
            success = self.tracker.init(img_bgr, bbox)
            
            if success:
                self.is_initialized = True
                self.is_tracking = True
                self.last_bbox = bbox
                self.get_logger().info(f'Tracker initialisé avec bbox: {bbox}')
            else:
                self.get_logger().error('Échec de l\'initialisation du tracker')
                self.is_initialized = False
                self.is_tracking = False
        
        # Mettre à jour le tracking si initialisé
        if self.is_initialized and self.tracker is not None:
            success, bbox = self.tracker.update(img_bgr)
            
            if success:
                self.is_tracking = True
                self.last_bbox = tuple(map(int, bbox))
            else:
                self.is_tracking = False
                self.get_logger().warn('Tracking perdu!')
        
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
            
            # Bounding box en pixels
            out_msg.bbox_x = int(bbox_x)
            out_msg.bbox_y = int(bbox_y)
            out_msg.bbox_width = int(bbox_w)
            out_msg.bbox_height = int(bbox_h)
            
            # Centre en pixels
            center_x_px = bbox_x + bbox_w // 2
            center_y_px = bbox_y + bbox_h // 2
            
            # Convertir en mètres
            center_x_m, center_y_m = self._pixels_to_meters(center_x_px, center_y_px, frame_msg)
            
            out_msg.center_x = float(center_x_m)
            out_msg.center_y = float(center_y_m)
            
            # Dimensions en mètres
            out_msg.width = float(bbox_w * frame_msg.resolution)
            out_msg.height = float(bbox_h * frame_msg.resolution)
            
            # Confiance (CSRT ne fournit pas directement, on met 1.0 si tracking actif)
            out_msg.confidence = 1.0 if self.is_tracking else 0.0
        else:
            out_msg.center_x = 0.0
            out_msg.center_y = 0.0
            out_msg.width = 0.0
            out_msg.height = 0.0
            out_msg.bbox_x = 0
            out_msg.bbox_y = 0
            out_msg.bbox_width = 0
            out_msg.bbox_height = 0
            out_msg.confidence = 0.0
        
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
