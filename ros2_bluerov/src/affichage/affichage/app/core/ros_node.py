# =============================================================================
# ROS_NODE.PY - Nœud ROS2 pour l'interface graphique du sonar
# =============================================================================
#
# RÔLE DE CE FICHIER:
# -------------------
# Ce nœud ROS2 fait le pont entre le monde ROS2 et l'interface Qt.
#
# FLUX DE DONNÉES:
# ----------------
#
#   RÉCEPTION (ROS2 → Qt):
#   ┌─────────────────┐         ┌──────────────┐         ┌────────────┐
#   │  Autres nœuds   │ ──────► │  Callbacks   │ ──────► │  Signaux   │
#   │  ROS2 (topics)  │  msgs   │  (ce fichier)│  emit() │  Qt        │
#   └─────────────────┘         └──────────────┘         └────────────┘
#
#   ENVOI (Qt → ROS2):
#   ┌─────────────────┐         ┌──────────────┐         ┌────────────┐
#   │  Interface Qt   │ ──────► │  Publishers  │ ──────► │ Autres     │
#   │  (boutons, etc.)│ appels  │ (ce fichier) │  msgs   │ nœuds ROS2 │
#   └─────────────────┘         └──────────────┘         └────────────┘
#
# =============================================================================

import rclpy
from rclpy.node import Node  # Classe de base pour tous les nœuds ROS2

# Types de paramètres ROS2 (pour modifier les paramètres d'autres nœuds)
from rcl_interfaces.msg import ParameterType

# Messages personnalisés pour notre projet de docking
from docking_msgs.msg import Frame, FrameCartesian, DetectedLines, BBoxSelection, TrackedObject

# Service pour configurer le sonar
from docking_msgs.srv import ConfigureSonar

# Messages standards ROS2
from geometry_msgs.msg import PoseStamped  # Position + orientation
from std_msgs.msg import Bool              # Booléen simple


class SonarViewerNode(Node):
    """
    Nœud ROS2 qui gère les communications pour l'affichage du sonar.
    
    VOCABULAIRE ROS2:
    - Node     : Un "processus" ROS2 qui peut communiquer
    - Topic    : Un "canal" de communication (comme une chaîne TV)
    - Publisher: Envoie des messages sur un topic
    - Subscriber: Reçoit des messages d'un topic
    - Service  : Requête/Réponse (comme un appel de fonction à distance)
    - Callback : Fonction appelée quand un message arrive
    """

    def __init__(self, signals):
        """
        Initialise le nœud ROS2.
        
        Args:
            signals: Instance de ROSSignals pour émettre vers Qt
        """
        # super().__init__('nom') crée un nœud ROS2 avec ce nom
        # Ce nom apparaît dans `ros2 node list`
        super().__init__('sonar_viewer')
        
        # Stocke la référence aux signaux Qt pour les émettre dans les callbacks
        self.signals = signals

        # =====================================================================
        # SUBSCRIPTIONS (Réception de données)
        # =====================================================================
        # Syntaxe: create_subscription(TypeMessage, 'nom_du_topic', callback, queue_size)
        #
        # Queue size (10 ici): Nombre de messages à garder en file d'attente
        # si on ne les traite pas assez vite. Après 10, les anciens sont jetés.
        # =====================================================================
        
        # Reçoit les frames brutes du sonar (image polaire non traitée)
        self.raw_sub = self.create_subscription(
            Frame,                      # Type du message
            '/docking/sonar/raw',       # Nom du topic
            self.raw_callback,          # Fonction à appeler
            10                          # Taille de la queue
        )
        
        # Reçoit les frames cartésiennes filtrées (image en coordonnées X/Y)
        self.cartesian_filtered_sub = self.create_subscription(
            FrameCartesian, 
            '/docking/sonar/cartesian_filtered', 
            self.cartesian_filtered_callback, 
            10
        )
        
        # Subscriptions pour le système de tracking (suivi d'objets)
        # Lignes détectées dans l'image
        self.detected_lines_sub = self.create_subscription(
            DetectedLines, 
            '/docking/tracking/detected_lines', 
            self.detected_lines_callback, 
            10
        )
        
        # Objet actuellement suivi (bounding box)
        self.tracked_object_sub = self.create_subscription(
            TrackedObject, 
            '/docking/tracking/tracked_object', 
            self.tracked_object_callback, 
            10
        )
        
        # Position et orientation de la cage détectée
        self.cage_pose_sub = self.create_subscription(
            PoseStamped, 
            '/docking/tracking/cage_pose', 
            self.cage_pose_callback, 
            10
        )

        # =====================================================================
        # PUBLISHERS (Envoi de données)
        # =====================================================================
        # Syntaxe: create_publisher(TypeMessage, 'nom_du_topic', queue_size)
        # =====================================================================
        
        # Publie les sélections de bounding box faites par l'utilisateur
        self.bbox_pub = self.create_publisher(
            BBoxSelection, 
            '/docking/sonar/bbox_selection', 
            10
        )
        
        # Publie pour activer/désactiver la détection automatique
        self.auto_detect_trigger_pub = self.create_publisher(
            Bool, 
            '/docking/tracking/trigger_auto_detect', 
            10
        )
        
        # Reçoit le statut de la détection automatique
        self.auto_detect_status_sub = self.create_subscription(
            Bool, 
            '/docking/tracking/auto_detect_status', 
            self.auto_detect_status_callback, 
            10
        )

        # Affiche un message dans la console ROS2 (visible avec ros2 run ...)
        self.get_logger().info('Sonar Viewer démarré')

    # =========================================================================
    # MODIFICATION DE PARAMÈTRES D'AUTRES NŒUDS
    # =========================================================================
    # En ROS2, chaque nœud peut avoir des "paramètres" (comme des variables
    # de configuration). On peut les modifier à distance via un service.
    # =========================================================================
    
    def set_traitement_unified_parameter(self, param_name, value):
        """Change un paramètre sur le node traitement_unified_node."""
        if not hasattr(self, 'traitement_unified_param_client') or self.traitement_unified_param_client is None:
            from rcl_interfaces.srv import SetParameters
            self.traitement_unified_param_client = self.create_client(SetParameters, '/traitement_unified_node/set_parameters')

        if isinstance(value, bool):
            param_type = ParameterType.PARAMETER_BOOL
        elif isinstance(value, int):
            param_type = ParameterType.PARAMETER_INTEGER
        elif isinstance(value, float):
            param_type = ParameterType.PARAMETER_DOUBLE
        else:
            self.get_logger().error(f'Type de paramètre non supporté: {type(value)}')
            return False

        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue

        request = SetParameters.Request()
        param = ParameterMsg()
        param.name = param_name
        param.value = ParameterValue()
        param.value.type = param_type

        if param_type == ParameterType.PARAMETER_BOOL:
            param.value.bool_value = value
        elif param_type == ParameterType.PARAMETER_INTEGER:
            param.value.integer_value = value
        elif param_type == ParameterType.PARAMETER_DOUBLE:
            param.value.double_value = value

        request.parameters = [param]

        if self.traitement_unified_param_client.service_is_ready():
            self.traitement_unified_param_client.call_async(request)
            self.get_logger().debug(f'Paramètre traitement_unified.{param_name} = {value}')
            return True

        self.get_logger().warn('Service de paramètres traitement_unified_node non disponible')
        return False

    # =========================================================================
    # CALLBACKS - Fonctions appelées quand un message ROS2 arrive
    # =========================================================================
    # Chaque callback:
    # 1. Reçoit le message ROS2
    # 2. Émet un signal Qt avec emit() pour transférer les données à l'interface
    #
    # C'est ICI que se fait le pont ROS2 → Qt !
    # =========================================================================

    def raw_callback(self, msg):
        """
        Appelée quand une frame brute arrive.
        
        emit() envoie le message à tous les "slots" connectés à ce signal.
        Dans main_window.py, on a fait:
            signals.new_raw_frame.connect(self.on_raw_frame)
        Donc self.on_raw_frame() sera appelée avec msg en argument.
        """
        self.signals.new_raw_frame.emit(msg)

    def cartesian_filtered_callback(self, msg):
        """Appelée quand une frame cartésienne filtrée arrive."""
        self.signals.new_cartesian_filtered_frame.emit(msg)

    def detected_lines_callback(self, msg):
        """Appelée quand des lignes détectées arrivent."""
        self.signals.new_detected_lines.emit(msg)

    def cage_pose_callback(self, msg):
        """Appelée quand la pose de la cage arrive."""
        self.signals.new_cage_pose.emit(msg)

    def tracked_object_callback(self, msg):
        """Appelée quand un objet traqué arrive."""
        self.signals.new_tracked_object.emit(msg)
    
    # =========================================================================
    # MÉTHODES DE PUBLICATION - Envoyer des données vers ROS2
    # =========================================================================
    
    def publish_bbox_selection(self, x: int, y: int, width: int, height: int):
        """
        Publie une bounding box sélectionnée par l'utilisateur.
        
        PROCESSUS:
        1. Créer un message du bon type
        2. Remplir les champs du message
        3. Publier avec .publish()
        """
        # Crée une instance du message (comme un objet avec des attributs)
        msg = BBoxSelection()
        
        # Header standard ROS2 avec timestamp et frame de référence
        # get_clock().now() donne l'heure actuelle ROS2
        # to_msg() convertit en format message
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sonar'  # Système de coordonnées
        
        # Remplit les données de la bounding box
        msg.x = int(x)
        msg.y = int(y)
        msg.width = int(width)
        msg.height = int(height)
        msg.is_valid = True
        
        # PUBLICATION: Envoie le message sur le topic
        # Tous les subscribers de ce topic recevront ce message
        self.bbox_pub.publish(msg)
        
        # Log pour le debug (visible dans la console)
        self.get_logger().info(f'BBox publiée: ({x}, {y}, {width}, {height})')
    
    def auto_detect_status_callback(self, msg):
        """Reçoit le statut de la recherche auto-detect."""
        self.signals.auto_detect_status_changed.emit(msg.data)
    
    def publish_auto_detect_trigger(self, activate: bool):
        """Active ou désactive la recherche automatique de cage."""
        msg = Bool()
        msg.data = activate
        self.auto_detect_trigger_pub.publish(msg)
        action = 'ACTIVÉ' if activate else 'DÉSACTIVÉ'
        self.get_logger().info(f'Auto-detect {action}')

    # =========================================================================
    # SERVICES ROS2 - Communication Requête/Réponse
    # =========================================================================
    # Différence Topic vs Service:
    # - Topic: Communication continue, sans réponse (comme une radio)
    # - Service: Requête unique avec réponse (comme un appel téléphonique)
    #
    # PROCESSUS D'UN SERVICE:
    # 1. Créer un client de service
    # 2. Vérifier que le service est disponible
    # 3. Créer une requête et la remplir
    # 4. Appeler le service de façon asynchrone (call_async)
    # 5. Attendre la réponse dans un callback
    # =========================================================================
    
    def configure_sonar(self, range_m: float = 0.0, gain_percent: int = -1, 
                        master_mode: int = -1, ping_rate: int = -1):
        """
        Appelle le service de configuration du sonar.
        
        Args:
            range_m: Portée en mètres (0.0 = ne pas modifier)
            gain_percent: Gain en % [0-100] (-1 = ne pas modifier)
            master_mode: 1=LF, 2=HF (-1 = ne pas modifier)
            ping_rate: 0=auto (-1 = ne pas modifier)
        """
        if not hasattr(self, 'sonar_config_client') or self.sonar_config_client is None:
            self.sonar_config_client = self.create_client(
                ConfigureSonar, 
                '/docking/sonar/configure'
            )
        
        if not self.sonar_config_client.service_is_ready():
            self.get_logger().warn('Service /docking/sonar/configure non disponible')
            self.signals.sonar_config_response.emit(
                False, "Service sonar non disponible", 0.0, 0, 0, 0
            )
            return
        
        request = ConfigureSonar.Request()
        request.range = float(range_m)
        request.gain_percent = int(gain_percent)
        request.master_mode = int(master_mode)
        request.ping_rate = int(ping_rate)
        
        future = self.sonar_config_client.call_async(request)
        future.add_done_callback(self._sonar_config_callback)
        
        self.get_logger().info(
            f'Configuration sonar demandée: range={range_m}m, gain={gain_percent}%, '
            f'mode={master_mode}, ping_rate={ping_rate}'
        )
    
    def _sonar_config_callback(self, future):
        """Callback appelé quand le service de configuration répond."""
        try:
            response = future.result()
            self.signals.sonar_config_response.emit(
                response.success,
                response.message,
                response.current_range,
                response.current_gain,
                response.current_mode,
                response.current_ping_rate
            )
            if response.success:
                self.get_logger().info(f'Sonar configuré: {response.message}')
            else:
                self.get_logger().error(f'Erreur config sonar: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Exception config sonar: {e}')
            self.signals.sonar_config_response.emit(
                False, f"Exception: {str(e)}", 0.0, 0, 0, 0
            )
