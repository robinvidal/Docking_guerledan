import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from docking_msgs.msg import Frame, FrameCartesian, Borders, DetectedLines, ClickPosition, BBoxSelection, PoseRelative, State, TrackedObject, RotatedBBoxInit
from std_msgs.msg import Bool


class SonarViewerNode(Node):
    """ROS2 node handling subscriptions and parameter services."""

    def __init__(self, signals):
        super().__init__('sonar_viewer')
        self.signals = signals

        self.polar_param_client = None
        self.cartesian_param_client = None
        self.sonar_mock_param_client = None
        self.hough_param_client = None

        self.raw_sub = self.create_subscription(Frame, '/docking/sonar/raw', self.raw_callback, 10)
        self.polar_filtered_sub = self.create_subscription(Frame, '/docking/sonar/polar_filtered', self.polar_filtered_callback, 10)
        self.cartesian_filtered_sub = self.create_subscription(FrameCartesian, '/docking/sonar/cartesian_filtered', self.cartesian_filtered_callback, 10)
        self.borders_sub = self.create_subscription(Borders, '/docking/tracking/borders', self.borders_callback, 10)
        self.detected_lines_sub = self.create_subscription(DetectedLines, '/docking/tracking/detected_lines', self.detected_lines_callback, 10)
        self.tracked_object_sub = self.create_subscription(TrackedObject, '/docking/tracking/tracked_object', self.tracked_object_callback, 10)
        self.pose_sub = self.create_subscription(PoseRelative, '/docking/localisation/pose', self.pose_callback, 10)
        self.state_sub = self.create_subscription(State, '/docking/mission/state', self.state_callback, 10)

        self.abort_pub = self.create_publisher(Bool, '/docking/mission/abort', 10)
        self.click_pub = self.create_publisher(ClickPosition, '/docking/sonar/click_position', 10)
        self.bbox_pub = self.create_publisher(BBoxSelection, '/docking/sonar/bbox_selection', 10)
        self.rotated_bbox_pub = self.create_publisher(RotatedBBoxInit, '/docking/sonar/rotated_bbox_init', 10)

        self.current_borders = None
        self.current_pose = None
        self.current_state = None

        self.get_logger().info('Sonar Viewer démarré')

    def set_polar_parameter(self, param_name, value):
        if not self.polar_param_client:
            from rcl_interfaces.srv import SetParameters

            self.polar_param_client = self.create_client(
                SetParameters, '/traitement_polar_node/set_parameters'
            )

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

        if self.polar_param_client.service_is_ready():
            self.polar_param_client.call_async(request)
            self.get_logger().debug(f'Polar param {param_name} = {value}')
            return True

        self.get_logger().warn('Service traitement_polar_node non disponible')
        return False

    def set_cartesian_parameter(self, param_name, value):
        if not self.cartesian_param_client:
            from rcl_interfaces.srv import SetParameters

            self.cartesian_param_client = self.create_client(
                SetParameters, '/traitement_cartesian_node/set_parameters'
            )

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

        if self.cartesian_param_client.service_is_ready():
            self.cartesian_param_client.call_async(request)
            self.get_logger().debug(f'Cartesian param {param_name} = {value}')
            return True

        self.get_logger().warn('Service traitement_cartesian_node non disponible')
        return False

    def set_sonar_mock_parameter(self, param_name, value):
        if not self.sonar_mock_param_client:
            from rcl_interfaces.srv import SetParameters

            self.sonar_mock_param_client = self.create_client(SetParameters, '/sonar_mock/set_parameters')

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

        if self.sonar_mock_param_client.service_is_ready():
            self.sonar_mock_param_client.call_async(request)
            self.get_logger().debug(f'Paramètre sonar_mock.{param_name} = {value}')
            return True

        self.get_logger().warn('Service de paramètres sonar_mock non disponible')
        return False

    def set_tracking_parameter(self, param_name, value):
        if not hasattr(self, 'tracker_param_client') or self.tracker_param_client is None:
            from rcl_interfaces.srv import SetParameters

            self.tracker_param_client = self.create_client(SetParameters, '/blob_tracker_node/set_parameters')

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

        if self.tracker_param_client.service_is_ready():
            self.tracker_param_client.call_async(request)
            self.get_logger().debug(f'Paramètre tracker.{param_name} = {value}')
            return True

        self.get_logger().warn('Service de paramètres blob_tracker_node non disponible')
        return False

    def raw_callback(self, msg):
        self.signals.new_raw_frame.emit(msg)

    def polar_filtered_callback(self, msg):
        self.signals.new_polar_filtered_frame.emit(msg)

    def cartesian_filtered_callback(self, msg):
        self.signals.new_cartesian_filtered_frame.emit(msg)

    def set_hough_parameter(self, param_name, value):
        if not self.hough_param_client:
            from rcl_interfaces.srv import SetParameters

            self.hough_param_client = self.create_client(
                SetParameters, '/hough_lines_node/set_parameters'
            )

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

        if self.hough_param_client.service_is_ready():
            self.hough_param_client.call_async(request)
            self.get_logger().debug(f'Hough param {param_name} = {value}')
            return True

        self.get_logger().warn('Service hough_lines_node non disponible')
        return False

    def borders_callback(self, msg):
        self.current_borders = msg
        self.signals.new_borders.emit(msg)

    def detected_lines_callback(self, msg):
        self.signals.new_detected_lines.emit(msg)

    def tracked_object_callback(self, msg):
        self.signals.new_tracked_object.emit(msg)

    def pose_callback(self, msg):
        self.current_pose = msg
        self.signals.new_pose.emit(msg)

    def state_callback(self, msg):
        self.current_state = msg
        self.signals.new_state.emit(msg)

    def send_abort(self):
        msg = Bool()
        msg.data = True
        self.abort_pub.publish(msg)
        self.get_logger().warn('Commande ABORT envoyée')
    
    def publish_click_position(self, x_m: float, y_m: float):
        """Publie la position d'un clic souris sur le sonar."""
        msg = ClickPosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sonar'
        msg.x = float(x_m)
        msg.y = float(y_m)
        msg.pixel_x = 0  # Non utilisé pour l'instant
        msg.pixel_y = 0
        msg.is_valid = True
        
        self.click_pub.publish(msg)
        self.get_logger().info(f'Clic publié: x={x_m:.2f}m, y={y_m:.2f}m')
    
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
    
    def publish_rotated_bbox_init(self, p1_x: float, p1_y: float, p2_x: float, p2_y: float, 
                                   p3_x: float, p3_y: float, p4_x: float, p4_y: float):
        """Publie une initialisation de bbox rotatif (4 coins)."""
        msg = RotatedBBoxInit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sonar'
        msg.p1_x = float(p1_x)
        msg.p1_y = float(p1_y)
        msg.p2_x = float(p2_x)
        msg.p2_y = float(p2_y)
        msg.p3_x = float(p3_x)
        msg.p3_y = float(p3_y)
        msg.p4_x = float(p4_x)
        msg.p4_y = float(p4_y)
        msg.is_valid = True
        
        self.rotated_bbox_pub.publish(msg)
        self.get_logger().info(
            f'Rotated BBox publiée: 4 coins - '
            f'P1({p1_x:.2f}, {p1_y:.2f}), P2({p2_x:.2f}, {p2_y:.2f}), '
            f'P3({p3_x:.2f}, {p3_y:.2f}), P4({p4_x:.2f}, {p4_y:.2f})'
        )
