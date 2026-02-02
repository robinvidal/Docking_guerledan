import numpy as np
import rclpy
from rclpy.node import Node
from docking_msgs.msg import FrameCartesian, TrackedObject, DetectedLines
from sklearn.neighbors import NearestNeighbors
from sklearn.decomposition import PCA

class ICPTrackerNode(Node):
    def __init__(self):
        super().__init__('icp_tracker_node')

        # ================= PARAMÈTRES =================
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('cage_width', 0.82)
        self.declare_parameter('cage_depth', 0.56)
        self.declare_parameter('points_per_side', 10)

        self.declare_parameter('max_iterations', 30)
        self.declare_parameter('tolerance', 0.0001)
        self.declare_parameter('intensity_threshold', 100)
        self.declare_parameter('gating_dist', 1.5)
        self.declare_parameter('max_alignment_error', 0.20)

        # <<< paramètres géométriques simples
        self.declare_parameter('perp_tol_deg', 15.0)
        self.declare_parameter('geom_latch_time', 2.0)

        self.state = np.array([0.0, 0.0, 0.0])
        self.is_initialized = False

        self.last_geom_detection_time = None  # <<< latch

        self.model_points = self._create_cage_model()

        self.sub = self.create_subscription(
            FrameCartesian,
            '/docking/sonar/cartesian_filtered',
            self.frame_callback,
            10
        )

        # <<< subscriber Hough
        self.lines_sub = self.create_subscription(
            DetectedLines,
            '/docking/tracking/detected_lines',
            self.lines_callback,
            10
        )

        self.pub = self.create_publisher(
            TrackedObject,
            '/docking/tracking/icp_object',
            10
        )

        self.get_logger().info("ICP + affichage géométrique permissif démarré")

    # ============================================================
    # HOUGH → GÉOMÉTRIE SIMPLE
    # ============================================================
    def lines_callback(self, msg: DetectedLines):
        if not msg.is_valid or msg.num_lines < 2:
            return

        tol = np.deg2rad(self.get_parameter('perp_tol_deg').value)
        thetas = msg.thetas[:msg.num_lines]

        for i in range(len(thetas)):
            for j in range(i + 1, len(thetas)):
                dtheta = abs(self._angle_diff(thetas[i], thetas[j]))
                if abs(dtheta - np.pi / 2) < tol:
                    self.last_geom_detection_time = self.get_clock().now()
                    return

    # ============================================================
    def frame_callback(self, msg: FrameCartesian):
        if not self.get_parameter('enable_tracking').value:
            return

        sonar_cloud = self._get_sonar_point_cloud(msg)

        if sonar_cloud is None or len(sonar_cloud) < 10:
            self.is_initialized = False

        else:
            nbrs_tree = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(sonar_cloud)

            # ================= INITIALISATION =================
            if not self.is_initialized:
                centroid = np.mean(sonar_cloud, axis=0)
                cx, cy = centroid
                pca_angle = self._estimate_initial_orientation(sonar_cloud)

                self.state = np.array([cx, cy, pca_angle])
                self.is_initialized = True

            # ================= ICP =================
            curr_x, curr_y, curr_theta = self.state

            for _ in range(self.get_parameter('max_iterations').value):
                model = self._transform_points(
                    self.model_points, curr_x, curr_y, curr_theta
                )

                dx, dy, dtheta = self._icp_step(model, sonar_cloud, nbrs_tree)
                if dx is None:
                    self.is_initialized = False
                    break

                curr_x += dx
                curr_y += dy
                curr_theta += dtheta

                if dx**2 + dy**2 < self.get_parameter('tolerance').value**2:
                    break

            self.state = np.array([curr_x, curr_y, curr_theta])

        # ============================================================
        # DÉCISION D’AFFICHAGE (COMME AVANT, MAIS GÉOMÉTRIQUE)
        # ============================================================
        show = False
        if self.last_geom_detection_time is not None:
            age = (self.get_clock().now() -
                   self.last_geom_detection_time).nanoseconds * 1e-9
            if age < self.get_parameter('geom_latch_time').value:
                show = True

        out = TrackedObject()
        out.header = msg.header

        if show:
            out.is_tracking = True
            out.confidence = 0.5
            out.center_x = float(self.state[0])
            out.center_y = float(self.state[1])
            out.width = self.get_parameter('cage_width').value
            out.height = self.get_parameter('cage_depth').value
        else:
            out.is_tracking = False
            out.confidence = 0.0

        self.pub.publish(out)

    # ============================================================
    # UTILS (inchangés)
    # ============================================================
    def _angle_diff(self, a, b):
        d = a - b
        while d > np.pi:
            d -= 2 * np.pi
        while d < -np.pi:
            d += 2 * np.pi
        return d

    # (les autres fonctions : _create_cage_model, _get_sonar_point_cloud,
    #  _transform_points, _estimate_initial_orientation, _icp_step
    #  restent STRICTEMENT IDENTIQUES à ton code original)

# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = ICPTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
