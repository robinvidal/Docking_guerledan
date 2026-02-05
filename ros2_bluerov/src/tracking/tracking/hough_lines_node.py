import numpy as np
import math
import rclpy
from rclpy.node import Node
from docking_msgs.msg import FrameCartesian, DetectedLines
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from collections import deque

try:
    import cv2
except ImportError:
    cv2 = None

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

class HoughLinesNode(Node):
    def __init__(self):
        super().__init__('hough_lines_node')
        
        # --- PARAMÈTRES DE DÉTECTION ---
        self.declare_parameter('enable_detection', True)
        self.declare_parameter('rho_resolution', 1.0)
        self.declare_parameter('theta_resolution', 1.0)
        self.declare_parameter('threshold', 40)
        self.declare_parameter('min_line_length', 15)
        self.declare_parameter('max_line_gap', 10)
        
        # Filtres physiques
        self.declare_parameter('filter_min_length_m', 0.2)
        self.declare_parameter('filter_max_length_m', 2.5)
        self.declare_parameter('merge_rho_tolerance', 0.3)
        self.declare_parameter('merge_theta_tolerance', 0.2)
        self.declare_parameter('cage_width', 0.82)
        self.declare_parameter('dist_tol', 0.15)
        self.declare_parameter('perp_tol', 0.25)
        self.declare_parameter('conn_tol', 0.40)

        # --- PARAMÈTRES DE FILTRAGE ---
        # Taille de la fenêtre glissante (ex: 7 mesures)
        self.declare_parameter('window_size', 7) 
        
        # Anti-Saut : Rejet si l'angle change trop vite
        self.declare_parameter('outlier_threshold_deg', 60.0) 
        self.declare_parameter('max_consecutive_outliers', 5)

        # --- INITIALISATION DU BUFFER UNIQUE ---
        win_size = self.get_parameter('window_size').value
        self.history = deque(maxlen=win_size)

        # Variables d'état
        self.last_valid_yaw = None
        self.consecutive_outliers = 0

        # --- PUBLISHERS ---
        self.subscription = self.create_subscription(
            FrameCartesian, '/docking/sonar/cartesian_filtered', self.frame_callback, 10)
        self.line_pub = self.create_publisher(DetectedLines, '/docking/tracking/detected_lines', 10)
        self.cage_pose_pub = self.create_publisher(PoseStamped, '/docking/tracking/cage_pose', 10)

        # Debug (Pour vérifier Raw vs Filtered)
        self.pub_dbg_raw = self.create_publisher(Float32, '/debug/yaw_raw', 10)
        self.pub_dbg_filtered = self.create_publisher(Float32, '/debug/yaw_filtered', 10)

        self.get_logger().info(f'Hough Lines Node Initialisé (Window: {win_size}, Anti-Outlier: Actif)')

    def merge_similar_lines(self, candidates):
        if not candidates: return []
        rho_tol = self.get_parameter('merge_rho_tolerance').value
        theta_tol = self.get_parameter('merge_theta_tolerance').value
        candidates.sort(key=lambda x: x['length'], reverse=True)
        merged = []
        while len(candidates) > 0:
            base = candidates.pop(0)
            cluster = [base]
            remaining = []
            for other in candidates:
                d_theta = abs(base['theta'] - other['theta'])
                if d_theta > np.pi/2: d_theta = abs(np.pi - d_theta)
                d_rho = abs(base['rho'] - other['rho'])
                if d_rho < rho_tol and d_theta < theta_tol:
                    cluster.append(other)
                else:
                    remaining.append(other)
            candidates = remaining
            p1 = np.mean([c['p1'] for c in cluster], axis=0)
            p2 = np.mean([c['p2'] for c in cluster], axis=0)
            length = np.linalg.norm(p2 - p1)
            angle = np.arctan2(p2[1]-p1[1], p2[0]-p1[0])
            theta = (angle + np.pi/2) % np.pi
            rho = p1[0] * np.cos(theta) + p1[1] * np.sin(theta)
            merged.append({'p1': p1, 'p2': p2, 'rho': rho, 'theta': theta, 'length': length, 'is_u': 0.0})
        return merged

    def detect_u_shape(self, lines):
        if len(lines) < 3: return None, None, lines
        width_target = self.get_parameter('cage_width').value
        dist_tol = self.get_parameter('dist_tol').value
        perp_tol = self.get_parameter('perp_tol').value
        conn_tol = self.get_parameter('conn_tol').value
        best_score = -1
        best_data = (None, None)

        for i, base in enumerate(lines):
            for j, arm1 in enumerate(lines):
                if i == j: continue
                for k, arm2 in enumerate(lines):
                    if k == i or k == j: continue
                    def check_perp(l1, l2):
                        diff = abs(l1['theta'] - l2['theta'])
                        return abs(diff - np.pi/2) < perp_tol or abs(diff - 3*np.pi/2) < perp_tol
                    if not check_perp(base, arm1) or not check_perp(base, arm2): continue
                    m1, m2 = (arm1['p1']+arm1['p2'])/2, (arm2['p1']+arm2['p2'])/2
                    dist = np.linalg.norm(m1 - m2)
                    if abs(dist - width_target) > dist_tol: continue
                    def get_min_dist(l1, l2):
                        pts = [l1['p1'], l1['p2'], l2['p1'], l2['p2']]
                        return min(np.linalg.norm(pts[0]-pts[2]), np.linalg.norm(pts[0]-pts[3]),
                                   np.linalg.norm(pts[1]-pts[2]), np.linalg.norm(pts[1]-pts[3]))
                    if get_min_dist(base, arm1) > conn_tol or get_min_dist(base, arm2) > conn_tol: continue
                    score = base['length'] + arm1['length'] + arm2['length']
                    if score > best_score:
                        best_score = score
                        bary = (base['p1'] + base['p2'] + arm1['p1'] + arm1['p2'] + arm2['p1'] + arm2['p2']) / 6
                        base_mid = (base['p1'] + base['p2']) / 2
                        angle = math.atan2(bary[1] - base_mid[1], bary[0] - base_mid[0])
                        best_data = (bary, angle)
                        for idx in [i, j, k]: lines[idx]['is_u'] = 1.0
        return best_data[0], best_data[1], lines

    def compute_average_pose(self, history_buffer):
        if not history_buffer: return None, None, None
        xs = [h[0] for h in history_buffer]
        ys = [h[1] for h in history_buffer]
        avg_x = np.mean(xs)
        avg_y = np.mean(ys)
        
        # Moyenne vectorielle pour l'angle (Crucial pour éviter les sauts +/- PI)
        angles = [h[2] for h in history_buffer]
        sin_sum = np.sum(np.sin(angles))
        cos_sum = np.sum(np.cos(angles))
        avg_yaw = np.arctan2(sin_sum, cos_sum)
        return avg_x, avg_y, avg_yaw

    def angle_diff(self, a, b):
        """ Différence angulaire normalisée entre -PI et PI """
        d = a - b
        while d > np.pi: d -= 2*np.pi
        while d < -np.pi: d += 2*np.pi
        return d

    def frame_callback(self, msg: FrameCartesian):
        if not self.get_parameter('enable_detection').value or cv2 is None: return

        # Gestion dynamique de la taille de fenêtre (si modifiée en runtime)
        current_win = self.get_parameter('window_size').value
        if self.history.maxlen != current_win:
             # On recrée la deque avec la nouvelle taille (en gardant les éléments existants)
             self.history = deque(self.history, maxlen=current_win)

        img = np.array(msg.intensities, dtype=np.uint8).reshape((msg.height, msg.width))
        rho_res = self.get_parameter('rho_resolution').value
        theta_res = self.get_parameter('theta_resolution').value * np.pi / 180.0
        
        cv_lines = cv2.HoughLinesP(img, rho=rho_res, theta=theta_res, 
                                   threshold=int(self.get_parameter('threshold').value),
                                   minLineLength=int(self.get_parameter('min_line_length').value),
                                   maxLineGap=int(self.get_parameter('max_line_gap').value))
        
        candidates = []
        if cv_lines is not None:
            for l in cv_lines:
                x1_px, y1_px, x2_px, y2_px = l[0]
                p1 = np.array([-(x1_px - msg.origin_x) * msg.resolution, (y1_px - msg.origin_y) * msg.resolution + msg.min_range])
                p2 = np.array([-(x2_px - msg.origin_x) * msg.resolution, (y2_px - msg.origin_y) * msg.resolution + msg.min_range])
                length = np.linalg.norm(p2 - p1)
                if self.get_parameter('filter_min_length_m').value < length < self.get_parameter('filter_max_length_m').value:
                    angle = np.arctan2(p2[1]-p1[1], p2[0]-p1[0])
                    theta = (angle + np.pi/2) % np.pi
                    rho = p1[0] * np.cos(theta) + p1[1] * np.sin(theta)
                    candidates.append({'p1': p1, 'p2': p2, 'rho': rho, 'theta': theta, 'length': length})

        merged = self.merge_similar_lines(candidates)
        bary, yaw, final_lines = self.detect_u_shape(merged)

        # --- LOGIQUE DE FILTRAGE ---
        if bary is not None:
            raw_x, raw_y, raw_yaw = float(bary[0]), float(bary[1]), float(yaw)
            
            accept_measurement = True
            
            # 1. Vérification Anti-Saut
            if self.last_valid_yaw is not None:
                threshold = np.radians(self.get_parameter('outlier_threshold_deg').value)
                max_outliers = self.get_parameter('max_consecutive_outliers').value
                
                diff = abs(self.angle_diff(raw_yaw, self.last_valid_yaw))
                
                if diff > threshold:
                    self.consecutive_outliers += 1
                    if self.consecutive_outliers <= max_outliers:
                        accept_measurement = False
                    else:
                        # Sécurité : On accepte si le saut persiste
                        self.get_logger().warn("Force update: Trop de rejets consécutifs.")
                        self.consecutive_outliers = 0
                else:
                    self.consecutive_outliers = 0

            # 2. Mise à jour de l'historique
            if accept_measurement:
                self.last_valid_yaw = raw_yaw
                self.history.append((raw_x, raw_y, raw_yaw))

            # 3. Calcul de la Pose Moyenne (Filtrée)
            final_x, final_y, final_yaw = self.compute_average_pose(self.history)

            # 4. Publication
            if final_x is not None:
                # Debug
                self.pub_dbg_raw.publish(Float32(data=math.degrees(raw_yaw)))
                self.pub_dbg_filtered.publish(Float32(data=math.degrees(final_yaw)))

                # Pose Officielle
                p_msg = PoseStamped()
                p_msg.header = msg.header
                p_msg.pose.position.x = float(final_x)
                p_msg.pose.position.y = float(final_y)
                q = get_quaternion_from_euler(0, 0, final_yaw)
                p_msg.pose.orientation.x, p_msg.pose.orientation.y = q[0], q[1]
                p_msg.pose.orientation.z, p_msg.pose.orientation.w = q[2], q[3]
                self.cage_pose_pub.publish(p_msg)
        else:
            # Optionnel : Conserver l'historique pour lisser la reprise, 
            # ou self.history.clear() pour repartir de zéro.
            pass

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