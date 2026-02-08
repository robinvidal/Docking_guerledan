import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from docking_msgs.msg import Frame, FrameCartesian, DetectedLines, BBoxSelection, TrackedObject
from docking_msgs.srv import ConfigureSonar
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class SonarViewerNode(Node):
    """ROS2 node handling subscriptions and parameter services."""

    def __init__(self, signals):
        super().__init__('sonar_viewer')
        self.signals = signals

        # Subscriptions pour les données sonar
        self.raw_sub = self.create_subscription(Frame, '/docking/sonar/raw', self.raw_callback, 10)
        self.cartesian_filtered_sub = self.create_subscription(FrameCartesian, '/docking/sonar/cartesian_filtered', self.cartesian_filtered_callback, 10)
        
        # Subscriptions pour le tracking
        self.detected_lines_sub = self.create_subscription(DetectedLines, '/docking/tracking/detected_lines', self.detected_lines_callback, 10)
        self.tracked_object_sub = self.create_subscription(TrackedObject, '/docking/tracking/tracked_object', self.tracked_object_callback, 10)
        self.cage_pose_sub = self.create_subscription(PoseStamped, '/docking/tracking/cage_pose', self.cage_pose_callback, 10)

        # Publishers
        self.bbox_pub = self.create_publisher(BBoxSelection, '/docking/sonar/bbox_selection', 10)
        
        # Auto-detect tracking
        self.auto_detect_trigger_pub = self.create_publisher(Bool, '/docking/tracking/trigger_auto_detect', 10)
        self.auto_detect_status_sub = self.create_subscription(
            Bool, '/docking/tracking/auto_detect_status', 
            self.auto_detect_status_callback, 10
        )

        self.get_logger().info('Sonar Viewer démarré')

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

    def raw_callback(self, msg):
        self.signals.new_raw_frame.emit(msg)

    def cartesian_filtered_callback(self, msg):
        self.signals.new_cartesian_filtered_frame.emit(msg)

    def detected_lines_callback(self, msg):
        self.signals.new_detected_lines.emit(msg)

    def cage_pose_callback(self, msg):
        """Re-émet la pose de la cage pour l'interface graphique."""
        self.signals.new_cage_pose.emit(msg)

    def tracked_object_callback(self, msg):
        self.signals.new_tracked_object.emit(msg)
    
    def publish_bbox_selection(self, x: int, y: int, width: int, height: int):
        """Publie une bounding box sélectionnée manuellement."""
        msg = BBoxSelection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sonar'
        msg.x = int(x)
        msg.y = int(y)
        msg.width = int(width)
        msg.height = int(height)
        msg.is_valid = True
        
        self.bbox_pub.publish(msg)
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

    # ==================== Configuration Sonar ====================
    
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
