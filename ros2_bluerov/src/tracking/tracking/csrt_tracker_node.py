"""
Nœud de tracking CSRT pour sonar.
Utilise le tracker CSRT d'OpenCV pour suivre la cage dans les images cartésiennes filtrées.
Initialisé par un clic utilisateur, la bounding box est dimensionnée selon les dimensions
réelles de la cage définies dans le fichier de configuration.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from docking_msgs.msg import FrameCartesian, ClickPosition, BBoxSelection, TrackedObject
import os
from datetime import datetime

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
        
        # Mode de sélection: 'auto' (clic + dimensions) ou 'manual' (bbox dessinée)
        self.declare_parameter('selection_mode', 'manual')
        
        # Dimensions réelles de la cage (mètres) - utilisées en mode 'auto'
        self.declare_parameter('cage_width', 1)   # Largeur de la cage (m)
        self.declare_parameter('cage_height', 1)  # Hauteur de la cage (m)
        
        # Contrainte de mouvement pour éviter la dérive
        self.declare_parameter('max_displacement_per_frame', 50)  # pixels max entre 2 frames
        
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
        
        # Debug - créer un dossier pour sauvegarder les images
        # self.debug_dir = '/tmp/tracker_debug'
        # os.makedirs(self.debug_dir, exist_ok=True)
        # self.frame_count = 0
        # self.get_logger().info(f'Dossier de debug créé: {self.debug_dir}')
        
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
        
        self.get_logger().info('CSRT Tracker node démarré')
        
        if cv2 is None:
            self.get_logger().error('OpenCV non disponible! Le tracking ne fonctionnera pas.')
    
    def _create_tracker(self):
        """Crée un nouveau tracker CSRT avec les paramètres configurés."""
        if cv2 is None:
            return None
        
        # Tracker KCF - bon compromis vitesse/précision, moins sensible au bruit
        self.get_logger().info('Création tracker KCF')
        return cv2.legacy.TrackerKCF_create()
        
        # Version CSRT (à réactiver si besoin)
        # Créer les paramètres CSRT
        """
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
        """
    
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
        """Initialise le tracker lors d'un clic (mode auto uniquement)."""
        if not self.get_parameter('enable_tracking').value:
            return
        
        mode = self.get_parameter('selection_mode').value
        if mode != 'auto':
            self.get_logger().warn('Mode manuel activé - utilisez bbox_selection au lieu de click')
            return
        
        if cv2 is None:
            self.get_logger().error('OpenCV non disponible')
            return
        
        if not msg.is_valid:
            return
        
        # Stocker la position du clic pour initialisation lors du prochain frame
        self._pending_init = ('click', msg.x, msg.y)
        self.get_logger().info(f'Clic reçu à ({msg.x:.2f}m, {msg.y:.2f}m) - initialisation en attente')
    
    def bbox_callback(self, msg: BBoxSelection):
        """Initialise le tracker avec une bbox manuelle."""
        if not self.get_parameter('enable_tracking').value:
            return
        
        if cv2 is None:
            self.get_logger().error('OpenCV non disponible')
            return
        
        if not msg.is_valid:
            return
        
        # Stocker la bbox pour initialisation lors du prochain frame
        self._pending_init = ('bbox', msg.x, msg.y, msg.width, msg.height)
        self.get_logger().info(f'BBox reçue: ({msg.x}, {msg.y}, {msg.width}, {msg.height}) - initialisation en attente')
    
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
        
        # Améliorer le contraste avec égalisation d'histogramme (aide le tracking)
        img = cv2.equalizeHist(img)
        
        # CORRECTION: Flipper l'image horizontalement pour correspondre à l'affichage
        # Le sonar affiche l'image flippée, donc on doit aussi flipper pour le tracker
        img = cv2.flip(img, 1)  # 1 = flip horizontal
        
        # Convertir en BGR pour OpenCV (CSRT fonctionne mieux avec 3 canaux)
        img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        # S'assurer que l'image est contiguë en mémoire (requis par certains trackers)
        if not img_bgr.flags['C_CONTIGUOUS']:
            img_bgr = np.ascontiguousarray(img_bgr)
        
        self.current_frame_info = msg
        self.current_resolution = msg.resolution
        
        # Vérifier s'il y a une initialisation en attente
        if hasattr(self, '_pending_init') and self._pending_init is not None:
            init_data = self._pending_init
            self._pending_init = None
            
            # Calculer la bounding box selon le mode
            if init_data[0] == 'click':
                # Mode auto: clic + dimensions prédéfinies
                x_m, y_m = init_data[1], init_data[2]
                bbox = self._get_bbox_from_click(x_m, y_m, msg)
            elif init_data[0] == 'bbox':
                # Mode manuel: bbox fournie directement en pixels
                # CORRECTION: Flipper la bbox horizontalement car l'image est flippée
                bbox_x_original = init_data[1]
                bbox_x_flipped = msg.width - bbox_x_original - init_data[3]  # width - x - w
                bbox = (bbox_x_flipped, init_data[2], init_data[3], init_data[4])
                self.get_logger().info(f'BBox flippée: x {bbox_x_original} -> {bbox_x_flipped}')
            else:
                self.get_logger().error(f'Mode d\'initialisation inconnu: {init_data[0]}')
                return
            
            # Vérifications de validité
            bbox_x, bbox_y, bbox_w, bbox_h = bbox
            
            # Vérifier que la bbox est dans les limites de l'image
            if bbox_x < 0 or bbox_y < 0 or bbox_x + bbox_w > msg.width or bbox_y + bbox_h > msg.height:
                self.get_logger().error(
                    f'BBox hors limites ! Image: {msg.width}x{msg.height}, '
                    f'BBox: ({bbox_x}, {bbox_y}, {bbox_w}, {bbox_h})'
                )
                return
            
            # Vérifier dimensions minimales
            if bbox_w < 5 or bbox_h < 5:
                self.get_logger().error(f'BBox trop petite: {bbox_w}x{bbox_h}')
                return
            
            self.get_logger().info(
                f'Initialisation tracker: bbox=({bbox_x}, {bbox_y}, {bbox_w}, {bbox_h}), '
                f'image={msg.width}x{msg.height}'
            )
            
            # Créer un nouveau tracker
            self.tracker = self._create_tracker()
            if self.tracker is None:
                self.get_logger().error('Impossible de créer le tracker CSRT')
                return
            
            # Vérifier que l'image n'est pas vide
            if img_bgr is None or img_bgr.size == 0:
                self.get_logger().error('Image vide reçue !')
                return
            
            self.get_logger().info(f'Image: shape={img_bgr.shape}, dtype={img_bgr.dtype}')
            
            # OpenCV attend EXACTEMENT un tuple (x, y, w, h) avec des int
            # Pas de float, pas de numpy types
            bbox_for_opencv = (int(bbox_x), int(bbox_y), int(bbox_w), int(bbox_h))
            
            self.get_logger().info(f'BBox pour OpenCV: {bbox_for_opencv}, types: {[type(x).__name__ for x in bbox_for_opencv]}')
            
            # Initialiser le tracker
            try:
                success = self.tracker.init(img_bgr, bbox_for_opencv)
                self.get_logger().info(f'tracker.init() retourné: {success}')
                
                # Certaines versions d'OpenCV retournent None au lieu de True
                if success is None:
                    success = True
                    self.get_logger().info('tracker.init() a retourné None, considéré comme succès')
                    
            except Exception as e:
                self.get_logger().error(f'Exception lors de l\'initialisation: {type(e).__name__}: {e}')
                import traceback
                self.get_logger().error(traceback.format_exc())
                success = False
            
            if success:
                self.is_initialized = True
                self.is_tracking = True
                self.last_bbox = bbox
                self.get_logger().info(f'✓ Tracker initialisé avec succès !')
                
                # DEBUG: Sauvegarder l'image d'initialisation avec bbox
                # debug_img = img_bgr.copy()
                # cv2.rectangle(debug_img, 
                #              (int(bbox_x), int(bbox_y)), 
                #              (int(bbox_x + bbox_w), int(bbox_y + bbox_h)), 
                #              (0, 255, 0), 2)
                # cv2.putText(debug_img, f'INIT: ({bbox_x},{bbox_y}) {bbox_w}x{bbox_h}', 
                #            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                # timestamp = datetime.now().strftime('%H%M%S')
                # cv2.imwrite(f'{self.debug_dir}/init_{timestamp}.png', debug_img)
                # self.get_logger().info(f'Image d\'initialisation sauvegardée: {self.debug_dir}/init_{timestamp}.png')
            else:
                self.get_logger().error('Échec de l\'initialisation du tracker')
                self.is_initialized = False
                self.is_tracking = False
        
        # Mettre à jour le tracking si initialisé
        if self.is_initialized and self.tracker is not None:
            success, bbox = self.tracker.update(img_bgr)
            
            if success:
                new_bbox = tuple(map(int, bbox))
                
                # Contrainte de déplacement maximal pour éviter la dérive
                if self.last_bbox is not None:
                    max_disp = self.get_parameter('max_displacement_per_frame').value
                    old_x, old_y = self.last_bbox[0], self.last_bbox[1]
                    new_x, new_y = new_bbox[0], new_bbox[1]
                    
                    dx = new_x - old_x
                    dy = new_y - old_y
                    displacement = np.sqrt(dx**2 + dy**2)
                    
                    # Si le déplacement est trop grand, le limiter
                    if displacement > max_disp:
                        self.get_logger().warn(
                            f'Déplacement trop grand ({displacement:.1f}px > {max_disp}px), limité'
                        )
                        # Limiter le déplacement en gardant la direction
                        scale = max_disp / displacement
                        new_x = int(old_x + dx * scale)
                        new_y = int(old_y + dy * scale)
                        new_bbox = (new_x, new_y, new_bbox[2], new_bbox[3])
                
                self.is_tracking = True
                # old_bbox = self.last_bbox
                self.last_bbox = new_bbox
                
                # DEBUG: Logger le déplacement
                # if old_bbox is not None:
                #     old_x, old_y = old_bbox[0], old_bbox[1]
                #     new_x, new_y = self.last_bbox[0], self.last_bbox[1]
                #     dx = new_x - old_x
                #     dy = new_y - old_y
                #     self.get_logger().info(
                #         f'Frame {self.frame_count}: BBox déplacée de ({old_x},{old_y}) -> ({new_x},{new_y}), '
                #         f'delta=({dx:+d},{dy:+d})'
                #     )
                
                # Sauvegarder une frame toutes les 10 frames
                # self.frame_count += 1
                # if self.frame_count % 10 == 0:
                #     debug_img = img_bgr.copy()
                #     x, y, w, h = self.last_bbox
                #     cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                #     cv2.putText(debug_img, f'Frame {self.frame_count}: ({x},{y})', 
                #                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                #     cv2.imwrite(f'{self.debug_dir}/track_{self.frame_count:04d}.png', debug_img)
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
            
            # CORRECTION: Flipper la bbox en retour pour l'affichage
            # La bbox est dans l'espace de l'image flippée, on doit la re-flipper
            bbox_x_display = frame_msg.width - bbox_x - bbox_w
            
            # Bounding box en pixels (coordonnées pour l'affichage)
            out_msg.bbox_x = int(bbox_x_display)
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
            
            # Confiance (les trackers OpenCV ne fournissent pas de score de confiance)
            out_msg.confidence = 0.0
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
