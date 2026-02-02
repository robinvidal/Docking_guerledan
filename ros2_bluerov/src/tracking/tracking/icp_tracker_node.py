"""
Tracker ICP robuste utilisant Open3D et un Filtre de Kalman Étendu (EKF).
Modèle cible : Forme en U.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from docking_msgs.msg import FrameCartesian, TrackedObject
from scipy.spatial.transform import Rotation as R

# Bibliothèques avancées
import open3d as o3d
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import sys

class CageEKF:
    """Filtre de Kalman Étendu pour lisser le tracking."""
    def __init__(self, dt=0.1):
        # État : [x, y, theta, vx, vy, vtheta]
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.dt = dt
        
        # Matrice de transition (Modèle à vitesse constante)
        self.kf.F = np.array([[1, 0, 0, dt, 0,  0],
                              [0, 1, 0, 0,  dt, 0],
                              [0, 0, 1, 0,  0,  dt],
                              [0, 0, 0, 1,  0,  0],
                              [0, 0, 0, 0,  1,  0],
                              [0, 0, 0, 0,  0,  1]])
                              
        # Matrice de mesure (On observe x, y, theta via l'ICP)
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0]])
        
        # Incertitudes
        self.kf.P *= 1000.0 # Incertitude initiale élevée
        self.kf.R = np.eye(3) * 0.05 # Bruit de mesure (ICP) faible
        self.kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=0.1, block_size=3) # Bruit process

    def predict(self):
        self.kf.predict()
        # Normalisation de l'angle theta pour rester entre -PI et PI
        self.kf.x[2] = (self.kf.x[2] + np.pi) % (2 * np.pi) - np.pi

    def update(self, z):
        self.kf.update(z)
        self.kf.x[2] = (self.kf.x[2] + np.pi) % (2 * np.pi) - np.pi

    def get_state(self):
        return self.kf.x[:3] # Retourne [x, y, theta]


class ICPTrackerNode(Node):
    def __init__(self):
        super().__init__('icp_tracker_node')

        # --- PARAMÈTRES ---
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('cage_width', 0.82)
        self.declare_parameter('cage_depth', 0.60)
        self.declare_parameter('icp_max_dist', 1.0) # Distance max correspondance
        self.declare_parameter('min_fitness', 0.5)  # Seuil confiance ICP

        # --- INITIALISATION EKF ---
        self.ekf = CageEKF(dt=0.1) # Supposons 10Hz
        self.is_initialized = False

        # --- CRÉATION MODÈLE U-SHAPE (Open3D) ---
        self.target_pcd = self._create_u_shape_pcd()

        # --- VISUALISATION (Open3D) ---
        try:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(window_name='ICP Tracker Main', width=800, height=600)

            # Préparer géométries: cible (U), source (sonar) et centre (point bleu)
            self.target_pcd.paint_uniform_color([1.0, 0.2, 0.2])
            self.source_pcd = o3d.geometry.PointCloud()
            self.source_pcd.paint_uniform_color([1.0, 1.0, 1.0])

            # Centre (point unique) en tant que PointCloud pour facilité de mise à jour
            # Centre: petit maillage sphérique visible (plus gros que des points)
            self.center_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.05, resolution=20)
            self.center_mesh.compute_vertex_normals()
            self.center_mesh.paint_uniform_color([0.0, 0.0, 1.0])
            # Position initiale au centre
            self.center_pos = np.array([0.0, 0.0, 0.0])
            self.center_mesh.translate(self.center_pos)

            self.vis.add_geometry(self.target_pcd)
            self.vis.add_geometry(self.source_pcd)
            self.vis.add_geometry(self.center_mesh)

            opt = self.vis.get_render_option()
            opt.point_size = 5.0
        except Exception as e:
            self.get_logger().warn(f"Open3D visualizer unavailable: {e}")
            self.vis = None

        # Subscribers / Publishers
        self.sub = self.create_subscription(
            FrameCartesian, '/docking/sonar/cartesian_filtered', self.frame_callback, 10)
        
        self.pub = self.create_publisher(
            TrackedObject, '/docking/tracking/icp_object', 10)

        self.get_logger().info("ICP Tracker (Open3D + EKF) démarré")

        # Timer pour rafraîchir la fenêtre Open3D (non bloquant)
        if self.vis is not None:
            self.create_timer(0.1, self._vis_update)

    def _create_u_shape_pcd(self):
        """Génère le nuage de points de référence (Cible)."""
        points = []
        w = self.get_parameter('cage_width').value
        d = self.get_parameter('cage_depth').value
        density = 50 # points par mètre

        # Fond du U (Vertical en Y, X=0)
        for y in np.linspace(-w/2, w/2, int(w * density)):
            points.append([0.0, y, 0.0])

        # Bras Gauche (Horizontal en X, Y=-w/2)
        for x in np.linspace(0, d, int(d * density)):
            points.append([x, -w/2, 0.0])

        # Bras Droit (Horizontal en X, Y=w/2)
        for x in np.linspace(0, d, int(d * density)):
            points.append([x, w/2, 0.0])

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        return pcd

    def _get_sonar_point_cloud(self, msg):
        """Convertit le message FrameCartesian en numpy array (N, 2)."""
        img = np.array(msg.intensities, dtype=np.uint8).reshape((msg.height, msg.width))
        
        # Seuil d'intensité pour ne garder que les échos forts
        threshold = 100 
        y_indices, x_indices = np.where(img > threshold)
        
        if len(x_indices) == 0:
            return None
            
        resolution = msg.resolution
        origin_x = msg.origin_x
        origin_y = msg.origin_y
        min_range = msg.min_range
        
        # Conversion pixels -> mètres
        x_m = -(x_indices.astype(float) - origin_x) * resolution
        y_m = (y_indices.astype(float) - origin_y) * resolution + min_range
        
        return np.column_stack((x_m, y_m))

    def frame_callback(self, msg: FrameCartesian):
        if not self.get_parameter('enable_tracking').value:
            return

        # 1. Prédiction EKF (Le robot a bougé depuis la dernière frame)
        self.ekf.predict()

        # 2. Récupération des données Sonar
        sonar_points_2d = self._get_sonar_point_cloud(msg)
        
        if sonar_points_2d is None or len(sonar_points_2d) < 20:
            # Perte de vue : on publie juste la prédiction (Dead Reckoning)
            self._publish_state(msg, is_tracking=False)
            return

        # Conversion Open3D (Ajout Z=0)
        z_zeros = np.zeros((sonar_points_2d.shape[0], 1))
        source_pcd = o3d.geometry.PointCloud()
        source_pcd.points = o3d.utility.Vector3dVector(np.hstack((sonar_points_2d, z_zeros)))

        if not self.is_initialized:
            # On calcule la moyenne des points (barycentre)
            centroid = np.mean(sonar_points_2d, axis=0)
            
            # On force l'état de l'EKF sur ce point
            self.ekf.x[0] = centroid[0] # X
            self.ekf.x[1] = centroid[1] # Y
            self.ekf.x[2] = 0.0         # Theta (On ne sait pas, on met 0)
            
            self.is_initialized = True
            self.get_logger().info(f"Initialisation auto: x={centroid[0]:.2f}, y={centroid[1]:.2f}")

        # --- SUITE NORMALE ---
        current_state = self.ekf.get_state() # [x, y, theta]
        
        init_trans = np.eye(4)
        init_trans[0:3, 0:3] = R.from_euler('z', current_state[2]).as_matrix()
        init_trans[0, 3] = current_state[0]
        init_trans[1, 3] = current_state[1]
        # 4. ICP Registration
        threshold = self.get_parameter('icp_max_dist').value
        reg = o3d.pipelines.registration.registration_icp(
            source_pcd, self.target_pcd, threshold, init_trans,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
        )

        # 5. Mise à jour EKF si ICP valide
        min_fitness = self.get_parameter('min_fitness').value
        
        if reg.fitness > min_fitness:
            # Extraction position corrigée
            trans = reg.transformation
            x_icp = trans[0, 3]
            y_icp = trans[1, 3]
            theta_icp = R.from_matrix(trans[0:3, 0:3]).as_euler('xyz')[2]
            
            # Correction EKF
            self.ekf.update(np.array([x_icp, y_icp, theta_icp]))
            # Log: cage détectée
            try:
                self.get_logger().info(f"Cage détectée: x={x_icp:.3f}, y={y_icp:.3f}, theta={theta_icp:.3f}")
            except Exception:
                print(f"Cage détectée: x={x_icp:.3f}, y={y_icp:.3f}, theta={theta_icp:.3f}")
            is_valid = True
        else:
            # ICP a échoué (trop loin ou mauvaise forme), on garde la prédiction
            is_valid = False

        # 6. Publication
        self._publish_state(msg, is_tracking=is_valid, fitness=reg.fitness)

        # Mettre à jour l'affichage Open3D si actif
        if self.vis is not None:
            try:
                # Mettre à jour la source (sonar)
                self.source_pcd.points = o3d.utility.Vector3dVector(np.hstack((sonar_points_2d, z_zeros)))
                self.source_pcd.colors = o3d.utility.Vector3dVector(np.tile(np.array([[1.0,1.0,1.0]]), (sonar_points_2d.shape[0],1)))
                self.vis.update_geometry(self.source_pcd)

                # Mettre à jour le centre (sphère bleue) à la position filtrée
                state = self.ekf.get_state()
                cx, cy = float(state[0]), float(state[1])
                new_center = np.array([cx, cy, 0.0])
                # Déplacer la sphère en calculant le delta
                current_center = np.asarray(self.center_mesh.get_center())
                delta = new_center - current_center
                self.center_mesh.translate(delta)
                self.vis.update_geometry(self.center_mesh)
            except Exception as e:
                self.get_logger().warn(f"Erreur mise à jour visuel Open3D: {e}")


    def _publish_state(self, header_source, is_tracking=False, fitness=0.0):
        state = self.ekf.get_state()
        
        out = TrackedObject()
        out.header = header_source.header
        out.is_tracking = is_tracking
        out.confidence = float(fitness)
        out.center_x = float(state[0])
        out.center_y = float(state[1])
        # Note : Si ton message TrackedObject a un champ angle, ajoute-le ici :
        # out.angle = float(state[2])
        out.width = self.get_parameter('cage_width').value
        out.height = self.get_parameter('cage_depth').value
        
        self.pub.publish(out)

    def _vis_update(self):
        """Timer callback pour rafraîchir la fenêtre Open3D sans bloquer le thread ROS."""
        if self.vis is None:
            return
        try:
            # Traiter les événements et redessiner
            self.vis.poll_events()
            self.vis.update_renderer()
        except Exception as e:
            # Si la fenêtre a été fermée manuellement, éviter de spammer les logs
            self.get_logger().warn(f"Open3D vis update error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ICPTrackerNode()
    rclpy.spin(node)
    # Nettoyage de la fenêtre Open3D si nécessaire
    try:
        if hasattr(node, 'vis') and node.vis is not None:
            node.vis.destroy_window()
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()