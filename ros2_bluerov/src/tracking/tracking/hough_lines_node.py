import numpy as np
import math
import rclpy
from rclpy.node import Node
from docking_msgs.msg import FrameCartesian, DetectedLines
from geometry_msgs.msg import PoseStamped

try:
    import cv2
except ImportError:
    cv2 = None

def get_quaternion_from_euler(roll, pitch, yaw):
    """Convertit un angle Euler (Yaw) en Quaternion."""
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

class HoughLinesNode(Node):
    def __init__(self):
        super().__init__('hough_lines_node')
        
        # --- PARAMÈTRES ---
        self.declare_parameter('enable_detection', True)
        self.declare_parameter('rho_resolution', 1.0)
        self.declare_parameter('theta_resolution', 1.0) # Degrés
        self.declare_parameter('threshold', 40)
        self.declare_parameter('min_line_length', 15)
        self.declare_parameter('max_line_gap', 10)
        
        # Filtres physiques (mètres)
        self.declare_parameter('filter_min_length_m', 0.2)
        self.declare_parameter('filter_max_length_m', 2.5)
        self.declare_parameter('merge_rho_tolerance', 0.3)
        self.declare_parameter('merge_theta_tolerance', 0.2)

        # Géométrie Cage (U)
        self.declare_parameter('cage_width', 0.82)
        self.declare_parameter('dist_tol', 0.15)
        self.declare_parameter('perp_tol', 0.25)
        self.declare_parameter('conn_tol', 0.40)

        # Pub/Sub
        self.subscription = self.create_subscription(
            FrameCartesian, '/docking/sonar/cartesian_filtered', self.frame_callback, 10)
        self.line_pub = self.create_publisher(DetectedLines, '/docking/tracking/detected_lines', 10)
        self.cage_pose_pub = self.create_publisher(PoseStamped, '/docking/tracking/cage_pose', 10)

        self.get_logger().info('Hough Lines Node (U-Shape) initialisé.')

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
            
            # Moyenne simple des points pour la ligne fusionnée
            p1 = np.mean([c['p1'] for c in cluster], axis=0)
            p2 = np.mean([c['p2'] for c in cluster], axis=0)
            length = np.linalg.norm(p2 - p1)
            angle = np.arctan2(p2[1]-p1[1], p2[0]-p1[0])
            theta = (angle + np.pi/2) % np.pi
            rho = p1[0] * np.cos(theta) + p1[1] * np.sin(theta)

            merged.append({'p1': p1, 'p2': p2, 'rho': rho, 'theta': theta, 'length': length, 'is_u': 0.0})
        return merged

    def detect_u_shape(self, lines):
        self.get_logger().info(f"Analyse de {len(lines)} lignes candidates...", throttle_duration_sec=2.0)
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
                    
                    # 1. Vérifier perpendicularité (Base vs Bras)
                    def check_perp(l1, l2):
                        diff = abs(l1['theta'] - l2['theta'])
                        return abs(diff - np.pi/2) < perp_tol or abs(diff - 3*np.pi/2) < perp_tol

                    if not check_perp(base, arm1) or not check_perp(base, arm2): continue

                    # 2. Distance entre bras (Largeur)
                    m1, m2 = (arm1['p1']+arm1['p2'])/2, (arm2['p1']+arm2['p2'])/2
                    dist = np.linalg.norm(m1 - m2)
                    if abs(dist - width_target) > dist_tol: continue

                    # 3. Proximité (Connexion bras-base)
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
                        # Direction : du fond du U vers l'ouverture
                        angle = math.atan2(bary[1] - base_mid[1], bary[0] - base_mid[0])
                        best_data = (bary, angle)
                        # Marquer les lignes
                        for idx in [i, j, k]: lines[idx]['is_u'] = 1.0

        if best_data[0] is not None:
            deg = math.degrees(best_data[1])
            self.get_logger().info(f"✅ CAGE DÉTECTÉE | Pose: x={best_data[0][0]:.2f}m, y={best_data[0][1]:.2f}m | Yaw: {deg:.1f}°")

        return best_data[0], best_data[1], lines

    def frame_callback(self, msg: FrameCartesian):
        if not self.get_parameter('enable_detection').value or cv2 is None: return

        img = np.array(msg.intensities, dtype=np.uint8).reshape((msg.height, msg.width))
        
        # Hough Probabiliste
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
                # Pixels -> Mètres
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

        # Publication des lignes
        out_msg = DetectedLines()
        out_msg.header = msg.header
        for l in final_lines:
            out_msg.rhos.append(float(l['rho']))
            out_msg.thetas.append(float(l['theta']))
            out_msg.x1_points.append(float(l['p1'][0]))
            out_msg.y1_points.append(float(l['p1'][1]))
            out_msg.x2_points.append(float(l['p2'][0]))
            out_msg.y2_points.append(float(l['p2'][1]))
            out_msg.confidences.append(l['is_u'] if l['is_u'] > 0 else 0.4)
        self.line_pub.publish(out_msg)

        # Publication de la Pose de la cage
        if bary is not None:
            p_msg = PoseStamped()
            p_msg.header = msg.header
            p_msg.pose.position.x, p_msg.pose.position.y = float(bary[0]), float(bary[1])
            q = get_quaternion_from_euler(0, 0, yaw)
            p_msg.pose.orientation.x, p_msg.pose.orientation.y, p_msg.pose.orientation.z, p_msg.pose.orientation.w = q
            self.cage_pose_pub.publish(p_msg)


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