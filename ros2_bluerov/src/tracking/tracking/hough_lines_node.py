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
        self.declare_parameter('threshold', 0.20)
        self.declare_parameter('min_line_length', 0.2)
        self.declare_parameter('max_line_gap', 0.20)
        
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
        self.declare_parameter('window_size_ang', 7) 
        self.declare_parameter('window_size_pos', 3) 
        
        # Anti-Saut
        self.declare_parameter('outlier_threshold_deg', 60.0) 
        self.declare_parameter('outlier_threshold_pos', 0.5) # AJOUT : Seuil de saut position (0.5m par défaut)
        self.declare_parameter('max_consecutive_outliers', 5)

        # --- INITIALISATION ---
        win_pos = self.get_parameter('window_size_pos').value
        win_ang = self.get_parameter('window_size_ang').value
        
        self.history_pos = deque(maxlen=win_pos)
        self.history_ang = deque(maxlen=win_ang)

        # Variables d'état
        self.last_valid_yaw = None
        self.consecutive_outliers_yaw = 0
        self.last_valid_pos = None     # Stockera le tuple (x, y)
        self.consecutive_outliers_pos = 0

        # --- PUBLISHERS ---
        self.subscription = self.create_subscription(
            FrameCartesian, '/docking/sonar/cartesian_filtered', self.frame_callback, 10)
        self.line_pub = self.create_publisher(DetectedLines, '/docking/tracking/detected_lines', 10)
        self.cage_pose_pub = self.create_publisher(PoseStamped, '/docking/tracking/cage_pose', 10)

        self.pub_dbg_raw = self.create_publisher(Float32, '/debug/yaw_raw', 10)
        self.pub_dbg_filtered = self.create_publisher(Float32, '/debug/yaw_filtered', 10)

        self.get_logger().info(f'Hough Lines Node Initialisé (Anti-Saut Position & Angle Actif)')

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

    def compute_avg_pos(self, hist_pos):
        if not hist_pos: return None, None
        xs = [h[0] for h in hist_pos]
        ys = [h[1] for h in hist_pos]
        return np.mean(xs), np.mean(ys)

    def compute_avg_ang(self, hist_ang):
        if not hist_ang: return None
        angles = list(hist_ang)
        sin_sum = np.sum(np.sin(angles))
        cos_sum = np.sum(np.cos(angles))
        return np.arctan2(sin_sum, cos_sum)

    def angle_diff(self, a, b):
        d = a - b
        while d > np.pi: d -= 2*np.pi
        while d < -np.pi: d += 2*np.pi
        return d

    def frame_callback(self, msg: FrameCartesian):
        if not self.get_parameter('enable_detection').value or cv2 is None: return

        curr_win_pos = self.get_parameter('window_size_pos').value
        curr_win_ang = self.get_parameter('window_size_ang').value

        if self.history_pos.maxlen != curr_win_pos:
            self.history_pos = deque(self.history_pos, maxlen=curr_win_pos)
        if self.history_ang.maxlen != curr_win_ang:
            self.history_ang = deque(self.history_ang, maxlen=curr_win_ang)

        img = np.array(msg.intensities, dtype=np.uint8).reshape((msg.height, msg.width))
        res = msg.resolution 
        
        thresh_m = self.get_parameter('threshold').value
        threshold_px = int(max(1, thresh_m / res))
        min_line_m = self.get_parameter('min_line_length').value
        min_line_px = int(max(1, min_line_m / res))
        max_gap_m = self.get_parameter('max_line_gap').value
        max_gap_px = int(max(1, max_gap_m / res))

        rho_res = self.get_parameter('rho_resolution').value
        theta_res = self.get_parameter('theta_resolution').value * np.pi / 180.0
        
        cv_lines = cv2.HoughLinesP(img, rho=rho_res, theta=theta_res, 
                                   threshold=threshold_px, minLineLength=min_line_px, maxLineGap=max_gap_px)     
        
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

        # --- LOGIQUE DE FILTRAGE SÉPARÉE ET SÉCURISÉE ---
        if bary is not None:
            raw_x, raw_y, raw_yaw = float(bary[0]), float(bary[1]), float(yaw)
            max_outliers = self.get_parameter('max_consecutive_outliers').value
            
            # --- 1. GESTION POSITION (Anti-Saut Euclidien) ---
            accept_pos = True
            if self.last_valid_pos is not None:
                # Seuil en mètres (ex: 0.5 m)
                thresh_pos = self.get_parameter('outlier_threshold_pos').value
                # Calcul distance 2D
                dist = np.linalg.norm(np.array([raw_x, raw_y]) - np.array(self.last_valid_pos))
                
                if dist > thresh_pos:
                    self.consecutive_outliers_pos += 1
                    if self.consecutive_outliers_pos <= max_outliers:
                        accept_pos = False
                    else:
                        self.get_logger().warn(f"Position Force Update: saut de {dist:.2f}m accepté après persistance")
                        self.consecutive_outliers_pos = 0
                else:
                    self.consecutive_outliers_pos = 0
            
            if accept_pos:
                self.last_valid_pos = (raw_x, raw_y)
                self.history_pos.append((raw_x, raw_y))
            
            # --- 2. GESTION ANGLE (Anti-Saut Angulaire) ---
            accept_angle = True
            if self.last_valid_yaw is not None:
                thresh_ang = np.radians(self.get_parameter('outlier_threshold_deg').value)
                
                diff = abs(self.angle_diff(raw_yaw, self.last_valid_yaw))
                
                if diff > thresh_ang:
                    self.consecutive_outliers_yaw += 1
                    if self.consecutive_outliers_yaw <= max_outliers:
                        accept_angle = False
                    else:
                        self.get_logger().warn(f"Yaw Force Update: saut de {math.degrees(diff):.1f}° accepté")
                        self.consecutive_outliers_yaw = 0
                else:
                    self.consecutive_outliers_yaw = 0

            if accept_angle:
                self.last_valid_yaw = raw_yaw
                self.history_ang.append(raw_yaw)

            # --- 3. CALCUL MOYENNES & PUBLICATION ---
            avg_x, avg_y = self.compute_avg_pos(self.history_pos)
            avg_yaw = self.compute_avg_ang(self.history_ang)

            if avg_x is not None and avg_yaw is not None:
                self.pub_dbg_raw.publish(Float32(data=math.degrees(raw_yaw)))
                self.pub_dbg_filtered.publish(Float32(data=math.degrees(avg_yaw)))

                p_msg = PoseStamped()
                p_msg.header = msg.header
                p_msg.pose.position.x = float(avg_x)
                p_msg.pose.position.y = float(avg_y)
                q = get_quaternion_from_euler(0, 0, avg_yaw)
                p_msg.pose.orientation.x, p_msg.pose.orientation.y = q[0], q[1]
                p_msg.pose.orientation.z, p_msg.pose.orientation.w = q[2], q[3]
                self.cage_pose_pub.publish(p_msg)
        else:
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