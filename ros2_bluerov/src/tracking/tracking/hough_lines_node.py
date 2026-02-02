"""
Nœud de détection de lignes par transformée de Hough.
Cherche spécifiquement une forme de U (Cage ouverte).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from docking_msgs.msg import FrameCartesian, DetectedLines, ClickPosition

try:
    import cv2
except ImportError:
    cv2 = None


class HoughLinesNode(Node):
    """Détecte des lignes et cherche une structure en U."""
    
    def __init__(self):
        super().__init__('hough_lines_node')
        
        # --- PARAMÈTRES DE DÉTECTION ---
        self.declare_parameter('enable_detection', True)
        self.declare_parameter('num_lines', 10)
        self.declare_parameter('rho_resolution', 0.5)  # Plus fin pour la précision
        self.declare_parameter('theta_resolution', 1.0)
        self.declare_parameter('threshold', 40)
        self.declare_parameter('min_line_length', 15)
        self.declare_parameter('max_line_gap', 15)
        self.declare_parameter('use_probabilistic', True)
        
        # --- FILTRES PHYSIQUES ---
        self.declare_parameter('filter_min_length_m', 0.3) 
        self.declare_parameter('filter_max_length_m', 2.5) 
        
        # --- FUSION DES DOUBLONS ---
        self.declare_parameter('merge_rho_tolerance', 0.3)
        self.declare_parameter('merge_theta_tolerance', 0.2)

        # --- DÉTECTION FORME EN U ---
        # Largeur attendue de la cage (distance entre les bras)
        self.declare_parameter('cage_width', 0.82) 
        # Tolérance pour la largeur (~20cm)
        self.declare_parameter('dist_tol', 0.20)
        # Tolérance pour l'angle droit (~14 degrés en radians)
        self.declare_parameter('perp_tol', 0.25)
        # Tolérance de connexion (les bras doivent toucher le fond à X m près)
        self.declare_parameter('conn_tol', 0.40)

        # Subscribers / Publishers
        self.subscription = self.create_subscription(
            FrameCartesian,
            '/docking/sonar/cartesian_filtered',
            self.frame_callback,
            10
        )
        
        self.publisher_ = self.create_publisher(
            DetectedLines,
            '/docking/tracking/detected_lines',
            10
        )

        # Publisher pour la position prédite de la cage (barycentre)
        self.pos_cage_pub = self.create_publisher(
            ClickPosition, '/docking/tracking/pos_cage', 10
        )

        # Mémo de la dernière position détectée (None si aucune)
        self.last_u_bary = None
        
        self.get_logger().info('Hough Lines Node (U-Shape Logic) démarré')
    
    def merge_similar_lines(self, candidates):
        """Fusionne les lignes géométriquement proches (doublons)."""
        if not candidates:
            return []
            
        rho_tol = self.get_parameter('merge_rho_tolerance').value
        theta_tol = self.get_parameter('merge_theta_tolerance').value
        
        # Tri par longueur décroissante
        candidates.sort(key=lambda x: x['length'], reverse=True)
        merged = []
        
        while candidates:
            base = candidates.pop(0)
            cluster = [base]
            remaining = []
            
            for other in candidates:
                d_theta = abs(base['theta'] - other['theta'])
                if d_theta > np.pi * 0.5: d_theta = np.pi - d_theta
                
                d_rho = abs(base['rho'] - other['rho'])
                
                if d_rho < rho_tol and d_theta < theta_tol:
                    cluster.append(other)
                else:
                    remaining.append(other)
            
            candidates = remaining
            
            # Moyenne pondérée des lignes fusionnées
            avg_x1 = np.mean([l['p1'][0] for l in cluster])
            avg_y1 = np.mean([l['p1'][1] for l in cluster])
            avg_x2 = np.mean([l['p2'][0] for l in cluster])
            avg_y2 = np.mean([l['p2'][1] for l in cluster])
            
            # Recalcul des propriétés géométriques
            dx = avg_x2 - avg_x1
            dy = avg_y2 - avg_y1
            length = np.sqrt(dx**2 + dy**2)
            angle = np.arctan2(dy, dx)
            theta = angle + np.pi / 2.0
            
            while theta < 0: theta += np.pi
            while theta >= np.pi: theta -= np.pi
            
            rho = avg_x1 * np.cos(theta) + avg_y1 * np.sin(theta)
            if rho < 0:
                rho = -rho
                theta = theta - np.pi if theta > 0 else theta + np.pi
                
            merged.append({
                'p1': (avg_x1, avg_y1),
                'p2': (avg_x2, avg_y2),
                'rho': rho,
                'theta': theta,
                'length': length
            })
            
        return merged
    
    def detect_u_shape(self, lines):
        """
        Analyse la liste des lignes pour trouver un motif en U.
        Retourne la liste des lignes, avec un flag 'is_u_shape' sur celles qui matchent.
        """
        if len(lines) < 3:
            return lines

        cage_width = self.get_parameter('cage_width').value
        perp_tol = self.get_parameter('perp_tol').value
        dist_tol = self.get_parameter('dist_tol').value
        conn_tol = self.get_parameter('conn_tol').value

        best_score = -1
        best_triplet = []

        # Complexité O(N^3) mais N est très petit (< 10 lignes filtrées)
        for i, base in enumerate(lines):
            for j, arm1 in enumerate(lines):
                if i == j: continue
                for k, arm2 in enumerate(lines):
                    if k == i or k == j: continue

                    # 1. Vérifier que Arm1 et Arm2 sont perpendiculaires à Base
                    def is_perp(l1, l2):
                        diff = abs(l1['theta'] - l2['theta'])
                        if diff > np.pi: diff -= np.pi
                        return abs(diff - np.pi/2) < perp_tol

                    if not is_perp(base, arm1) or not is_perp(base, arm2):
                        continue

                    # 2. Vérifier que Arm1 et Arm2 sont parallèles entre eux
                    def is_parallel(l1, l2):
                        diff = abs(l1['theta'] - l2['theta'])
                        if diff > np.pi/2: diff = np.pi - diff
                        return diff < perp_tol

                    if not is_parallel(arm1, arm2):
                        continue

                    # 3. Vérifier la distance entre Arm1 et Arm2 (Largeur cage)
                    # On prend le milieu des segments pour simplifier la distance
                    mid1 = (np.array(arm1['p1']) + np.array(arm1['p2'])) / 2
                    mid2 = (np.array(arm2['p1']) + np.array(arm2['p2'])) / 2
                    dist_arms = np.linalg.norm(mid1 - mid2)
                    self.get_logger().info(f"Test paire: Largeur vue = {dist_arms:.3f}m")
                    if abs(dist_arms - cage_width) > dist_tol:
                        continue

                    # 4. Vérifier la connexion (les bras touchent-ils la base ?)
                    # On cherche la distance min entre les points extrêmes
                    def min_dist_lines(l_base, l_arm):
                        pts_base = [np.array(l_base['p1']), np.array(l_base['p2'])]
                        pts_arm = [np.array(l_arm['p1']), np.array(l_arm['p2'])]
                        dists = []
                        for pb in pts_base:
                            for pa in pts_arm:
                                dists.append(np.linalg.norm(pb - pa))
                        return min(dists)

                    if min_dist_lines(base, arm1) > conn_tol or min_dist_lines(base, arm2) > conn_tol:
                        continue

                    # C'est un U valide ! On garde le plus grand (somme des longueurs)
                    score = base['length'] + arm1['length'] + arm2['length']
                    if score > best_score:
                        best_score = score
                        best_triplet = [i, j, k]

        # Marquage des lignes gagnantes
        if best_triplet:
            self.get_logger().info(f"✅ CAGE DÉTECTÉE ! (Score: {best_score:.2f})")
            for idx in best_triplet:
                lines[idx]['is_u_shape'] = 1.0 # Confiance max
            # Calculer barycentre des trois lignes -> on utilise le centre (milieu) de chaque segment
            pts = []
            for idx in best_triplet:
                l = lines[idx]
                mid = (np.array(l['p1']) + np.array(l['p2'])) / 2.0
                pts.append(mid)
            bary = np.mean(np.vstack(pts), axis=0)
            self.last_u_bary = bary
            # Optionnel : Ne retourner QUE le U ? 
            # Pour l'instant on retourne tout mais le tracker saura qui est qui grâce au flag.
            
        # Si aucun U détecté, réinitialiser la dernière bary
        if not best_triplet:
            self.last_u_bary = None

        return lines

    def frame_callback(self, msg: FrameCartesian):
        if not self.get_parameter('enable_detection').value or cv2 is None:
            return
        
        # --- 1. Reconstruction Image ---
        img = np.array(msg.intensities, dtype=np.uint8).reshape((msg.height, msg.width))
        
        # --- 2. Lecture Paramètres ---
        rho_res = float(self.get_parameter('rho_resolution').value)
        theta_res = float(self.get_parameter('theta_resolution').value) * np.pi / 180.0
        threshold = int(self.get_parameter('threshold').value)
        use_prob = self.get_parameter('use_probabilistic').value
        
        filt_min_m = self.get_parameter('filter_min_length_m').value
        filt_max_m = self.get_parameter('filter_max_length_m').value
        
        resolution = msg.resolution
        origin_x = msg.origin_x
        origin_y = msg.origin_y
        min_range = msg.min_range
        
        candidates = []

        # --- 3. Hough Transform ---
        if use_prob:
            min_len_px = int(self.get_parameter('min_line_length').value)
            max_gap_px = int(self.get_parameter('max_line_gap').value)
            lines = cv2.HoughLinesP(img, rho=rho_res, theta=theta_res, threshold=threshold,
                                    minLineLength=min_len_px, maxLineGap=max_gap_px)
            if lines is not None:
                for i in range(len(lines)):
                    x1_px, y1_px, x2_px, y2_px = lines[i][0]
                    
                    # Pixels -> Mètres
                    x1_m = -(float(x1_px) - origin_x) * resolution
                    y1_m = (float(y1_px) - origin_y) * resolution + min_range
                    x2_m = -(float(x2_px) - origin_x) * resolution
                    y2_m = (float(y2_px) - origin_y) * resolution + min_range
                    
                    dx, dy = x2_m - x1_m, y2_m - y1_m
                    length_m = np.sqrt(dx**2 + dy**2)
                    
                    if length_m < filt_min_m or length_m > filt_max_m: continue
                    
                    angle = np.arctan2(dy, dx)
                    theta = angle + np.pi / 2.0
                    while theta < 0: theta += np.pi
                    while theta >= np.pi: theta -= np.pi
                    
                    rho = x1_m * np.cos(theta) + y1_m * np.sin(theta)
                    if rho < 0:
                        rho = -rho
                        theta = theta - np.pi if theta > 0 else theta + np.pi
                        
                    candidates.append({
                        'p1': (x1_m, y1_m), 'p2': (x2_m, y2_m),
                        'rho': rho, 'theta': theta, 'length': length_m
                    })
        else:
            # Fallback Standard Hough si besoin (non détaillé ici pour gagner de la place, mais possible)
            pass

        # --- 4. Post-Traitement ---
        # Fusionner les doublons
        cleaned_lines = self.merge_similar_lines(candidates)
        
        # Détecter la structure en U
        final_lines = self.detect_u_shape(cleaned_lines)
        
        # --- 5. Publication ---
        out_msg = DetectedLines()
        out_msg.header = msg.header
        out_msg.is_valid = len(final_lines) > 0
        out_msg.num_lines = len(final_lines)
        
        for l in final_lines:
            out_msg.rhos.append(float(l['rho']))
            out_msg.thetas.append(float(l['theta']))
            out_msg.x1_points.append(float(l['p1'][0]))
            out_msg.y1_points.append(float(l['p1'][1]))
            out_msg.x2_points.append(float(l['p2'][0]))
            out_msg.y2_points.append(float(l['p2'][1]))
            
            # Si c'est un U, confiance = 1.0, sinon 0.5
            conf = 1.0 if l.get('is_u_shape', 0) > 0 else 0.5
            out_msg.confidences.append(float(conf))
            
        self.publisher_.publish(out_msg)

        # Publier la position prédite de la cage si disponible
        if getattr(self, 'last_u_bary', None) is not None:
            try:
                bx, by = float(self.last_u_bary[0]), float(self.last_u_bary[1])
                msg = ClickPosition()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'sonar'
                msg.x = bx
                msg.y = by
                msg.pixel_x = 0
                msg.pixel_y = 0
                msg.is_valid = True
                self.pos_cage_pub.publish(msg)
                self.get_logger().info(f"pos_cage publié: x={bx:.3f}, y={by:.3f}")
            except Exception as e:
                self.get_logger().warn(f"Erreur publication pos_cage: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HoughLinesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()