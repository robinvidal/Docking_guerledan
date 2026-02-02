"""
Nœud ROS2 pour publier les données du sonar Oculus M750d au format docking_msgs/Frame.
S'appuie sur la bibliothèque Python "oculus_python" (même API que l'exemple Divers/test_accoustic_cage_guerledan/oculus_sonar_test_v2.py).
"""

import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node

from docking_msgs.msg import Frame


class SonarNode(Node):
    """Driver sonar Oculus M750d publiant sur 
    - topic: /docking/sonar/raw
    - type: docking_msgs/Frame
    """

    def __init__(self):
        super().__init__('sonar_node')

        # Paramètres utilisateur
        self.declare_parameter('ip_address', '192.168.1.10')
        self.declare_parameter('port', 52102)
        self.declare_parameter('range', 4.0)  # m
        self.declare_parameter('gain_percent', 0)
        self.declare_parameter('master_mode', 1)  # 1: LF, 2: HF
        self.declare_parameter('ping_rate', 0)    # 0: auto
        self.declare_parameter('frame_id', 'sonar_link')
        # Suppression de bandes d'artefacts sur les bords (colonnes bearings)
        self.declare_parameter('mask_left_bearings', 5)   # nb de colonnes à gauche à masquer
        self.declare_parameter('mask_right_bearings', 0)  # nb de colonnes à droite à masquer
        self.declare_parameter('edge_mask_auto', False)    # détection auto d'une bande anormale au bord
        self.declare_parameter('edge_mask_threshold', 2.5)  # facteur * médiane des moyennes colonnes
        # Pas de mode mock: ce node ne publie que des données réelles provenant du sonar

        # Publisher
        self.publisher_ = self.create_publisher(Frame, '/docking/sonar/raw', 10)

        # État courant du sonar (rempli par callbacks)
        self._last_bearings_deg: Optional[list] = None
        self._last_ping_data: Optional[list] = None  # 2D (range, bearing)
        self._last_gains: Optional[list] = None
        self._last_range_m: Optional[float] = None
        self._last_master_mode: Optional[int] = None

        self._sonar = None
        self._config_sent = False
        self._lock = threading.Lock()

        # Initialisation du sonar réel uniquement
        self._init_sonar()

    # ---------- Intégration SDK Oculus ----------
    def _init_sonar(self):
        try:
            import oculus_python  # dépendance externe
        except Exception as e:
            self.get_logger().error(f"Impossible d'importer oculus_python : {e}")
            self.get_logger().error('Installez/activez le SDK Oculus avant d’exécuter ce node.')
            return

        try:
            self._sonar = oculus_python.OculusSonar()
            self._sonar.start()

            # Enregistrement des callbacks
            self._sonar.add_message_callback(self._message_callback)
            self._sonar.add_ping_callback(self._ping_callback)
            # self._sonar.add_status_callback(self._status_callback)

            self.get_logger().info('Sonar démarré, attente des pings...')
        except Exception as e:
            self.get_logger().error(f'Échec démarrage sonar: {e}')

    def _message_callback(self, msg):
        """Premier message -> envoi de la configuration initiale."""
        if self._config_sent or self._sonar is None:
            return

        try:
            c = self._sonar.current_config()
            c.range = float(self.get_parameter('range').get_parameter_value().double_value)
            c.masterMode = int(self.get_parameter('master_mode').get_parameter_value().integer_value)
            c.gainPercent = int(self.get_parameter('gain_percent').get_parameter_value().integer_value)
            c.pingRate = int(self.get_parameter('ping_rate').get_parameter_value().integer_value)

            self.get_logger().info(f'Envoi config sonar: range={c.range}m, master={c.masterMode}, gain={c.gainPercent}%, pingRate={c.pingRate}')
            self._sonar.send_config(c)
            self._config_sent = True
        except Exception as e:
            self.get_logger().error(f'Échec envoi config: {e}')

    def _ping_callback(self, metadata):
        """Réception d'un ping -> convertir et publier Frame."""
        try:
            # Extraire données du SDK comme dans l'exemple utilisateur
            bearings_deg = [0.01 * b for b in metadata.bearing_data()]  # degrés

            # Matrices 2D (range, bearing)
            import numpy as np
            rawPing = np.array(metadata.raw_ping_data(), dtype=np.float32)

            # Gains
            if metadata.has_gains():
                gains = np.array(metadata.gains(), dtype=np.float32)
            else:
                gains = np.ones([metadata.range_count()], dtype=np.float32)

            # Ping compensé (comme dans script) – utile si nécessaire
            pingData = np.array(metadata.ping_data(), dtype=np.float32) / np.sqrt(gains)[:, np.newaxis]

            with self._lock:
                self._last_bearings_deg = bearings_deg
                self._last_ping_data = rawPing  # utiliser intensités brutes
                self._last_gains = gains
                self._last_range_m = float(metadata.range())
                self._last_master_mode = int(metadata.master_mode())

            # Publier immédiatement
            self._publish_frame()
        except Exception as e:
            self.get_logger().error(f'Erreur ping_callback: {e}')

    def _status_callback(self, status):
        self.get_logger().info(f'STATUS: {status}')

    # ---------- Publication ----------
    def _publish_frame(self):
        with self._lock:
            if self._last_ping_data is None or self._last_bearings_deg is None or self._last_range_m is None:
                return

            data = self._last_ping_data  # numpy array (n_range, n_bearing)
            n_range, n_bearing = data.shape
            bearings_deg = self._last_bearings_deg
            range_m = self._last_range_m

        # Résolutions estimées
        # distance: supposée linéaire entre [0, range_m] sur n_range bins
        range_resolution = range_m / max(n_range, 1)

        # bearing: conversion degrés -> radians, résolution moyenne
        if len(bearings_deg) >= 2:
            bearing_resolution = abs((bearings_deg[-1] - bearings_deg[0]) / max(n_bearing - 1, 1)) * math.pi / 180.0
        else:
            bearing_resolution = 0.0

        # Construction du message Frame
        msg = Frame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        msg.range_count = n_range
        msg.bearing_count = n_bearing
        msg.range_resolution = float(range_resolution)
        msg.bearing_resolution = float(bearing_resolution)
        msg.min_range = 0.0
        msg.max_range = float(range_m)

        # Intensités: format attendu = tableau 2D aplati: intensité[bearing][range]
        # Notre array est (range, bearing). On applique un masquage éventuel des bords puis
        # on transpose et on aplatit par ordre bearing-major.
        try:
            import numpy as np
            work = np.array(data, copy=True)

            # Masquage manuel (paramètres)
            ml = int(self.get_parameter('mask_left_bearings').get_parameter_value().integer_value)
            mr = int(self.get_parameter('mask_right_bearings').get_parameter_value().integer_value)

            # Détection auto d'une bande anormalement forte sur les bords
            if self.get_parameter('edge_mask_auto').get_parameter_value().bool_value and work.shape[1] >= 3:
                col_means = work.mean(axis=0)
                # médiane des colonnes internes
                med = float(np.median(col_means[1:-1])) if work.shape[1] > 2 else float(np.median(col_means))
                thr = float(self.get_parameter('edge_mask_threshold').get_parameter_value().double_value)
                if med > 0:
                    if col_means[0] > thr * med:
                        ml = max(ml, 1)
                    if col_means[-1] > thr * med:
                        mr = max(mr, 1)

            if ml > 0:
                ml = min(ml, work.shape[1])
                work[:, :ml] = 0.0
            if mr > 0:
                mr = min(mr, work.shape[1])
                work[:, -mr:] = 0.0

            intensities = np.clip(work, 0, 255).astype(np.uint8).T.ravel()
            msg.intensities = intensities.tolist()
        except Exception:
            # fallback Python pur
            flat = []
            for j in range(n_bearing):
                for i in range(n_range):
                    v = data[i][j]
                    v = 0 if v < 0 else (255 if v > 255 else int(v))
                    flat.append(v)
            msg.intensities = flat

        # Métadonnées additionnelles (si dispo)
        # Vitesse du son non fournie par le script -> laisser à 0.0 (ou param si souhaité)
        msg.sound_speed = 0.0
        msg.gain = int(self.get_parameter('gain_percent').get_parameter_value().integer_value)

        self.publisher_.publish(msg)

    # Pas de publication synthétique dans ce node

    def destroy_node(self):
        # Nettoyage matériel
        try:
            if self._sonar is not None:
                try:
                    self._sonar.recorder_stop()
                except Exception:
                    pass
                self._sonar.stop()
        except Exception as e:
            self.get_logger().warn(f'Erreur à l’arrêt du sonar: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SonarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
