"""
Nœud de détection de lignes par transformée de Hough.
S'abonne aux données cartésiennes filtrées et détecte les n meilleures lignes.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from docking_msgs.msg import FrameCartesian, DetectedLines

try:
    import cv2
except ImportError:
    cv2 = None


class HoughLinesNode(Node):
    """Détecte des lignes avec transformée de Hough sur images cartésiennes."""
    
    def __init__(self):
        super().__init__('hough_lines_node')
        
        # Paramètres de détection
        self.declare_parameter('enable_detection', True)
        self.declare_parameter('num_lines', 5)  # Nombre de lignes à détecter
        self.declare_parameter('rho_resolution', 1.0)  # Résolution en distance (pixels)
        self.declare_parameter('theta_resolution', 1.0)  # Résolution angulaire (degrés)
        self.declare_parameter('threshold', 50)  # Seuil de votes minimum
        self.declare_parameter('min_line_length', 20)  # Longueur minimale (pixels)
        self.declare_parameter('max_line_gap', 10)  # Gap maximal (pixels)
        self.declare_parameter('use_probabilistic', True)  # Utiliser HoughLinesP
        # Taille physique de la ligne (en mètres) pour être considérée valide
        self.declare_parameter('filter_min_length_m', 0.5) 
        self.declare_parameter('filter_max_length_m', 3.0) 
        
        # Tolérances pour considérer que deux lignes sont identiques et doivent être fusionnées
        self.declare_parameter('merge_rho_tolerance', 0.3)   # 30 cm de tolérance distance
        self.declare_parameter('merge_theta_tolerance', 0.15) # ~8 degrés de tolérance angle
        # Subscription
        self.subscription = self.create_subscription(
            FrameCartesian,
            '/docking/sonar/cartesian_filtered',
            self.frame_callback,
            10
        )
        
        # Publisher
        self.publisher_ = self.create_publisher(
            DetectedLines,
            '/docking/tracking/detected_lines',
            10
        )
        
        self.get_logger().info('Hough Lines node démarré')
    
    def merge_similar_lines(self, candidates):
        """Fusionne les lignes géométriquement proches."""
        if not candidates:
            return []
            
        rho_tol = self.get_parameter('merge_rho_tolerance').value
        theta_tol = self.get_parameter('merge_theta_tolerance').value
        
        merged = []
        # On trie par longueur pour que les lignes les plus grandes absorbent les petites
        candidates.sort(key=lambda x: x['length'], reverse=True)
        
        while candidates:
            base = candidates.pop(0)
            cluster = [base]
            remaining = []
            
            for other in candidates:
                # Calcul différence d'angle (gestion modulo PI)
                d_theta = abs(base['theta'] - other['theta'])
                if d_theta > np.pi * 0.5:
                    d_theta = np.pi - d_theta
                
                d_rho = abs(base['rho'] - other['rho'])
                
                if d_rho < rho_tol and d_theta < theta_tol:
                    cluster.append(other)
                else:
                    remaining.append(other)
            
            candidates = remaining
            
            # Création de la ligne fusionnée (moyenne des coordonnées)
            # Cela est plus stable que de moyenner rho/theta directement
            avg_x1 = np.mean([l['p1'][0] for l in cluster])
            avg_y1 = np.mean([l['p1'][1] for l in cluster])
            avg_x2 = np.mean([l['p2'][0] for l in cluster])
            avg_y2 = np.mean([l['p2'][1] for l in cluster])
            
            # Recalcul rho/theta pour la moyenne
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
                'length': length,
                'confidence': min(1.0, length * len(cluster)) # Bonus confiance pour cluster
            })
            
        return merged
    
    def filter_cage_structure(self, lines_data):
        """
        Filtre les lignes en cherchant des structures de cage.
        - Cherche des paires parallèles espacées de 'cage_width' ou 'cage_depth'.
        - Cherche des paires perpendiculaires.
        Retourne la liste triée par pertinence structurelle.
        """
        if not lines_data:
            return []

        # Paramètres de la cage (à récupérer via self.get_parameter)
        # Supposons que ces paramètres existent ou sont codés en dur pour l'exemple
        cage_width = 0.7  # mètres
        cage_depth = 0.5  # mètres
        dist_tol = 0.2    # tolérance distance (m)
        angle_tol = 0.15  # tolérance angle (rad) ~8°

        # On initialise un score pour chaque ligne
        scores = [0.0] * len(lines_data)
        
        # On compare chaque ligne avec toutes les autres (Double boucle rapide car N est petit)
        for i in range(len(lines_data)):
            l1 = lines_data[i]
            
            for j in range(i + 1, len(lines_data)):
                l2 = lines_data[j]
                
                # 1. Calcul différence d'angle (modulo PI pour gérer 0° vs 180°)
                diff_theta = abs(l1['theta'] - l2['theta'])
                if diff_theta > np.pi * 0.5:
                    diff_theta = np.pi - diff_theta
                
                # 2. Calcul différence de distance (rho)
                # Attention : cela n'est valide que si les lignes sont parallèles
                diff_rho = abs(abs(l1['rho']) - abs(l2['rho']))

                # --- TEST PARALLÉLISME ---
                if diff_theta < angle_tol:
                    # Sont-elles espacées comme la LARGEUR de la cage ?
                    if abs(diff_rho - cage_width) < dist_tol:
                        scores[i] += 10.0  # Gros bonus
                        scores[j] += 10.0
                    # Sont-elles espacées comme la PROFONDEUR de la cage ?
                    elif abs(diff_rho - cage_depth) < dist_tol:
                        scores[i] += 10.0
                        scores[j] += 10.0
                    # Sont-elles juste parallèles (même barre vue en double ?)
                    else:
                        scores[i] += 1.0
                        scores[j] += 1.0

                # --- TEST PERPENDICULARITÉ ---
                elif abs(diff_theta - np.pi/2) < angle_tol:
                    # Les lignes se croisent à 90°. C'est un coin de cage potentiel.
                    scores[i] += 5.0
                    scores[j] += 5.0

        # On filtre : on ne garde que les lignes qui ont un score minimal
        # (c'est-à-dire qui participent à une structure géométrique)
        filtered_lines = []
        for i, line in enumerate(lines_data):
            # On met à jour la confiance de la ligne avec le score géométrique
            # Score de 0 = bruit isolé. Score > 10 = fait partie de la cage.
            line['structure_score'] = scores[i]
            
            # On ne garde que si elle a au moins une relation géométrique (score > 1)
            # Ou si vous voulez être permissif, gardez tout mais triez.
            if scores[i] > 2.0:
                filtered_lines.append(line)

        # Tri par score décroissant (les meilleures structures d'abord)
        filtered_lines.sort(key=lambda x: x['structure_score'], reverse=True)
        
        return filtered_lines

    def frame_callback(self, msg: FrameCartesian):
        """Traite une frame cartésienne et détecte les lignes."""
        if not self.get_parameter('enable_detection').value:
            return
        
        if cv2 is None:
            return
        
        # Reconstruction de l'image
        img = np.array(msg.intensities, dtype=np.uint8).reshape((msg.height, msg.width))
        
        # Paramètres (Lecture)
        num_lines_target = int(self.get_parameter('num_lines').value)
        rho_res = float(self.get_parameter('rho_resolution').value)
        theta_res = float(self.get_parameter('theta_resolution').value) * np.pi / 180.0
        threshold = int(self.get_parameter('threshold').value)
        use_prob = self.get_parameter('use_probabilistic').value
        
        # Nouveaux seuils physiques
        filt_min_m = self.get_parameter('filter_min_length_m').value
        filt_max_m = self.get_parameter('filter_max_length_m').value
        
        # Variables de conversion
        resolution = msg.resolution
        max_range = msg.max_range
        min_range = msg.min_range
        origin_x = msg.origin_x
        origin_y = msg.origin_y

        # Liste intermédiaire pour stocker les lignes valides en mètres
        candidates = []

        # --- BRANCHE 1 : HoughLinesP (Probabilistic) ---
        if use_prob:
            min_length_px = int(self.get_parameter('min_line_length').value)
            max_gap_px = int(self.get_parameter('max_line_gap').value)
            lines = cv2.HoughLinesP(img, rho=rho_res, theta=theta_res, threshold=threshold,
                                    minLineLength=min_length_px, maxLineGap=max_gap_px)
            
            if lines is not None:
                for i in range(len(lines)):
                    x1_px, y1_px, x2_px, y2_px = lines[i][0]
                    
                    # Conversion pixels -> mètres (Votre logique originale conservée)
                    x1_m = -(float(x1_px) - origin_x) * resolution
                    y1_m = (float(y1_px) - origin_y) * resolution + min_range
                    x2_m = -(float(x2_px) - origin_x) * resolution
                    y2_m = (float(y2_px) - origin_y) * resolution + min_range
                    
                    # Calcul longueur physique
                    dx = x2_m - x1_m
                    dy = y2_m - y1_m
                    length_m = np.sqrt(dx**2 + dy**2)
                    
                    # --- FILTRE 1 : Taille ---
                    if length_m < filt_min_m or length_m > filt_max_m:
                        continue
                        
                    # Calcul Rho/Theta pour la fusion
                    line_angle = np.arctan2(dy, dx)
                    theta = line_angle + np.pi / 2.0
                    while theta < 0: theta += np.pi
                    while theta >= np.pi: theta -= np.pi
                    
                    rho = x1_m * np.cos(theta) + y1_m * np.sin(theta)
                    if rho < 0:
                        rho = -rho
                        theta = theta - np.pi if theta > 0 else theta + np.pi # Correction bug potentiel normalisation
                        if theta < 0: theta += np.pi # Double check
                        
                    candidates.append({
                        'p1': (x1_m, y1_m), 'p2': (x2_m, y2_m),
                        'rho': rho, 'theta': theta, 'length': length_m
                    })

        # --- BRANCHE 2 : HoughLines (Standard) ---
        else:
            lines = cv2.HoughLines(img, rho=rho_res, theta=theta_res, threshold=threshold)
            
            if lines is not None:
                for i in range(len(lines)):
                    rho_px, theta_in = lines[i][0]
                    rho_m = float(rho_px) * resolution
                    
                    # Calcul intersections (Votre logique originale conservée)
                    cos_t = np.cos(theta_in)
                    sin_t = np.sin(theta_in)
                    
                    x_min, x_max = -max_range, max_range
                    y_min, y_max = min_range, max_range # Correction: Assurez-vous que y_max est correct par rapport à votre logique
                    
                    intersections = []
                    # Intersection x_min
                    if abs(sin_t) > 1e-6:
                        y = (rho_m - x_min * cos_t) / sin_t
                        if y_min <= y <= y_max: intersections.append((x_min, y))
                    # Intersection x_max
                    if abs(sin_t) > 1e-6:
                        y = (rho_m - x_max * cos_t) / sin_t
                        if y_min <= y <= y_max: intersections.append((x_max, y))
                    # Intersection y_min
                    if abs(cos_t) > 1e-6:
                        x = (rho_m - y_min * sin_t) / cos_t
                        if x_min <= x <= x_max: intersections.append((x, y_min))
                    # Intersection y_max
                    if abs(cos_t) > 1e-6: # Utiliser y_max (ou max_range selon votre logique originale)
                         x = (rho_m - max_range * sin_t) / cos_t # Supposant y_max = max_range comme avant
                         if x_min <= x <= x_max: intersections.append((x, max_range))

                    if len(intersections) >= 2:
                        p1 = intersections[0]
                        p2 = intersections[1]
                        
                        dx = p2[0] - p1[0]
                        dy = p2[1] - p1[1]
                        length_m = np.sqrt(dx**2 + dy**2)
                        
                        # --- FILTRE 1 : Taille ---
                        if length_m < filt_min_m or length_m > filt_max_m:
                            continue
                            
                        candidates.append({
                            'p1': p1, 'p2': p2,
                            'rho': rho_m, 'theta': theta_in, 'length': length_m
                        })

        # --- FILTRE 2 : FUSION ---
        cleaned_lines = self.merge_similar_lines(candidates)
        
        # --- ÉTAPE 2 : INTELLIGENCE (Structure) ---
        # On analyse les relations géométriques entre les lignes nettoyées
        # Cette fonction va trier la liste : les lignes qui forment une cage remontent en haut
        # Les lignes isolées (bruit de fond) descendent en bas
        structured_lines = self.filter_cage_structure(cleaned_lines)
        
        # --- ÉTAPE 3 : SÉLECTION ---
        # On ne garde que les N meilleures lignes (celles avec le meilleur score structurel)
        num_lines = int(self.get_parameter('num_lines').value)
        final_lines = structured_lines[:num_lines]
        final_lines_data = final_lines
        # Construction du message
        out_msg = DetectedLines()
        out_msg.header = msg.header
        
        out_msg.is_valid = len(final_lines_data) > 0
        out_msg.num_lines = len(final_lines_data)
        
        # Remplissage des vecteurs
        for l in final_lines_data:
            out_msg.rhos.append(float(l['rho']))
            out_msg.thetas.append(float(l['theta']))
            out_msg.x1_points.append(float(l['p1'][0]))
            out_msg.y1_points.append(float(l['p1'][1]))
            out_msg.x2_points.append(float(l['p2'][0]))
            out_msg.y2_points.append(float(l['p2'][1]))
            
            # Confiance
            if 'confidence' in l:
                conf = l['confidence']
            else:
                # Fallback sur votre logique originale si pas de fusion
                conf = min(1.0, l['length'] / (max_range * 0.5))
            out_msg.confidences.append(float(conf))
            
        self.publisher_.publish(out_msg)


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
