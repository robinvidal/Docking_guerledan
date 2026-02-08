"""
Nœud ROS2 Driver - Interface avec le sonar Oculus M750d.

Ce module fournit le driver ROS2 pour le sonar acoustique Oculus M750d.
Il utilise le SDK Python "oculus_python" pour communiquer avec le matériel
et publie les données au format docking_msgs/Frame.

Fonctionnalités principales:
    - Communication avec le sonar Oculus via SDK Python
    - Publication des frames sonar brutes sur /docking/sonar/raw
    - Service de reconfiguration dynamique du sonar
    - Masquage automatique des artefacts de bord

Prérequis:
    - SDK oculus_python installé et configuré
    - Sonar Oculus M750d connecté au réseau (IP: 192.168.1.10 par défaut)

Auteur: Équipe Docking Guerlédan ROB26
Date: 2026
"""

import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node

from docking_msgs.msg import Frame
from docking_msgs.srv import ConfigureSonar


class SonarNode(Node):
    """
    Driver ROS2 pour le sonar Oculus M750d.
    
    Ce nœud interface directement avec le matériel sonar via le SDK oculus_python.
    Il reçoit les pings du sonar en temps réel et les convertit au format
    docking_msgs/Frame pour publication sur le topic ROS2.
    
    Publications:
        - /docking/sonar/raw (docking_msgs/Frame): Données sonar brutes
        
    Services:
        - /docking/sonar/configure (ConfigureSonar): Configuration dynamique
    """

    def __init__(self):
        super().__init__('sonar_node')

        # ===================================================================
        # DÉCLARATION DES PARAMÈTRES ROS2
        # ===================================================================
        
        # --- Paramètres de connexion réseau ---
        self.declare_parameter('ip_address', '192.168.1.10')  # IP du sonar Oculus
        self.declare_parameter('port', 52102)                 # Port de communication
        
        # --- Paramètres de configuration du sonar ---
        self.declare_parameter('range', 15.0)        # Portée maximale (m)
        self.declare_parameter('gain_percent', 10)   # Gain acoustique (%)
        self.declare_parameter('master_mode', 2)     # Mode: 1=Basse Fréquence, 2=Haute Fréquence
        self.declare_parameter('ping_rate', 0)       # Taux de ping: 0=automatique
        self.declare_parameter('frame_id', 'sonar_link')  # Repère TF du sonar
        
        # --- Paramètres de masquage des artefacts de bord ---
        # Les bords de l'image sonar peuvent contenir des artefacts (bandes brillantes)
        self.declare_parameter('mask_left_bearings', 0)    # Colonnes à masquer à gauche
        self.declare_parameter('mask_right_bearings', 0)   # Colonnes à masquer à droite
        self.declare_parameter('edge_mask_auto', False)    # Détection automatique des bandes
        self.declare_parameter('edge_mask_threshold', 2.5) # Seuil de détection (x médiane)

        # ===================================================================
        # CONFIGURATION ROS2: PUBLISHERS ET SERVICES
        # ===================================================================
        
        # Publisher vers le topic de données sonar brutes
        self.publisher_ = self.create_publisher(Frame, '/docking/sonar/raw', 10)

        # Service pour reconfiguration dynamique du sonar
        self.config_service = self.create_service(
            ConfigureSonar,
            '/docking/sonar/configure',
            self._configure_sonar_callback
        )

        # ===================================================================
        # ÉTAT INTERNE (Données reçues du SDK)
        # ===================================================================
        # Ces variables sont mises à jour par les callbacks du SDK
        # et lues lors de la publication (accès protégé par _lock)
        self._last_bearings_deg: Optional[list] = None   # Angles azimutaux (degrés)
        self._last_ping_data: Optional[list] = None      # Intensités 2D (range, bearing)
        self._last_gains: Optional[list] = None          # Gains par bin de distance
        self._last_range_m: Optional[float] = None       # Portée configurée (m)
        self._last_master_mode: Optional[int] = None     # Mode actif (LF/HF)

        # Handle du SDK Oculus
        self._sonar = None
        self._config_sent = False  # Flag pour envoi unique de la config initiale
        self._lock = threading.Lock()  # Protection des accès concurrents

        # Initialisation de la connexion au sonar
        self._init_sonar()

    # ---------- Intégration SDK Oculus ----------
    def _init_sonar(self):
        try:
            import oculus_python  # dépendance externe
        except Exception as e:
            self.get_logger().error(f"Impossible d'importer oculus_python : {e}")
            self.get_logger().error('Installez/activez le SDK Oculus avant d’exécuter ce node.')
            self.get_logger().error('Donnez le chemin du SDK dans le PYTHONPATH ou utilisez un environnement virtuel avec oculus_python installé.')
            return

        try:
            self._sonar = oculus_python.OculusSonar()
            self._sonar.start()

            # Enregistrement des callbacks
            self._sonar.add_message_callback(self._message_callback)
            self._sonar.add_ping_callback(self._ping_callback)
            # self._sonar.add_status_callback(self._status_callback) # Optionnel pour debug

            self.get_logger().info('Sonar démarré, attente des pings...')
        except Exception as e:
            self.get_logger().error(f'Échec démarrage sonar: {e}')

    def _message_callback(self, msg):
        """
        Callback appelé lors de la réception du premier message du sonar.
        
        Utilisé pour envoyer la configuration initiale une seule fois,
        dès que la communication avec le sonar est établie.
        
        Args:
            msg: Message reçu du SDK (non utilisé directement)
        """
        if self._config_sent or self._sonar is None:
            return

        try:
            # Récupération et modification de la configuration actuelle
            c = self._sonar.current_config()
            c.range = float(self.get_parameter('range').get_parameter_value().double_value)
            c.masterMode = int(self.get_parameter('master_mode').get_parameter_value().integer_value)
            c.gainPercent = int(self.get_parameter('gain_percent').get_parameter_value().integer_value)
            c.pingRate = int(self.get_parameter('ping_rate').get_parameter_value().integer_value)

            self.get_logger().info(
                f'Envoi config sonar: range={c.range}m, master={c.masterMode}, '
                f'gain={c.gainPercent}%, pingRate={c.pingRate}'
            )
            self._sonar.send_config(c)
            self._config_sent = True
        except Exception as e:
            self.get_logger().error(f'Échec envoi config: {e}')

    def _ping_callback(self, metadata):
        """
        Callback de réception d'un ping sonar.
        
        Appelé par le SDK à chaque nouveau ping reçu. Extrait les données
        du format SDK et les stocke pour publication.
        
        STRUCTURE DES DONNÉES SDK:
        ===========================
        - bearing_data(): Angles azimutaux en centi-degrés (int)
        - raw_ping_data(): Intensités brutes, shape (n_range, n_bearing)
        - gains(): Gains par bin de distance (pour compensation)
        - range(): Portée configurée (m)
        - master_mode(): Mode actif (1=LF, 2=HF)
        
        NOTE: Les données sont stockées sous verrou pour éviter les
        accès concurrents lors de la publication.
        
        Args:
            metadata: Objet métadonnées du ping (SDK Oculus)
        """
        try:
            import numpy as np
            
            # Extraction des angles azimutaux (conversion centi-degrés → degrés)
            bearings_deg = [0.01 * b for b in metadata.bearing_data()]

            # Extraction des intensités brutes - shape (n_range, n_bearing)
            rawPing = np.array(metadata.raw_ping_data(), dtype=np.float32)

            # Extraction des gains (pour compensation optionnelle)
            if metadata.has_gains():
                gains = np.array(metadata.gains(), dtype=np.float32)
            else:
                gains = np.ones([metadata.range_count()], dtype=np.float32)

            # Calcul des données compensées (non utilisé actuellement)
            # FORMULE: pingData = raw / √gains (compensation d'atténuation)
            pingData = np.array(metadata.ping_data(), dtype=np.float32) / np.sqrt(gains)[:, np.newaxis]

            # Stockage thread-safe des données
            with self._lock:
                self._last_bearings_deg = bearings_deg
                self._last_ping_data = rawPing  # Utilisation des intensités brutes
                self._last_gains = gains
                self._last_range_m = float(metadata.range())
                self._last_master_mode = int(metadata.master_mode())

            # Publication immédiate après réception
            self._publish_frame()
        except Exception as e:
            self.get_logger().error(f'Erreur ping_callback: {e}')

    def _status_callback(self, status):
        """Callback de réception des status du sonar (debug)."""
        self.get_logger().info(f'STATUS: {status}')

    # ===================================================================
    # SERVICE DE CONFIGURATION DYNAMIQUE
    # ===================================================================
    
    def _configure_sonar_callback(self, request, response):
        """
        Callback du service pour modifier la configuration du sonar à chaud.
        
        Permet de reconfigurer le sonar sans redémarrer le nœud via:
            ros2 service call /docking/sonar/configure docking_msgs/srv/ConfigureSonar
        
        CONVENTION DES VALEURS SPÉCIALES:
        ===================================
        - range = 0.0       → ne pas modifier
        - gain_percent = -1 → ne pas modifier
        - master_mode = -1  → ne pas modifier
        - ping_rate = -1    → ne pas modifier
        
        Args:
            request: Requête ConfigureSonar avec les nouveaux paramètres
            response: Réponse avec succès/échec et configuration actuelle
            
        Returns:
            ConfigureSonar.Response avec l'état de la configuration
        """
        if self._sonar is None:
            response.success = False
            response.message = "Sonar non initialisé"
            response.current_range = 0.0
            response.current_gain = 0
            response.current_mode = 0
            response.current_ping_rate = 0
            return response

        try:
            # Récupérer la configuration actuelle
            c = self._sonar.current_config()
            changes = []

            # Modifier seulement les paramètres demandés
            if request.range > 0.0:
                c.range = float(request.range)
                changes.append(f"range={request.range}m")
            
            if request.gain_percent >= 0:
                c.gainPercent = int(request.gain_percent)
                changes.append(f"gain={request.gain_percent}%")
            
            if request.master_mode > 0:
                c.masterMode = int(request.master_mode)
                mode_name = "LF" if request.master_mode == 1 else "HF"
                changes.append(f"mode={mode_name}")
            
            if request.ping_rate >= 0:
                c.pingRate = int(request.ping_rate)
                changes.append(f"pingRate={request.ping_rate}")

            # Envoyer la nouvelle configuration
            self._sonar.send_config(c)
            
            # Mettre à jour les paramètres ROS2 aussi
            if request.range > 0.0:
                self.set_parameters([rclpy.parameter.Parameter('range', rclpy.Parameter.Type.DOUBLE, request.range)])
            if request.gain_percent >= 0:
                self.set_parameters([rclpy.parameter.Parameter('gain_percent', rclpy.Parameter.Type.INTEGER, request.gain_percent)])
            if request.master_mode > 0:
                self.set_parameters([rclpy.parameter.Parameter('master_mode', rclpy.Parameter.Type.INTEGER, request.master_mode)])
            if request.ping_rate >= 0:
                self.set_parameters([rclpy.parameter.Parameter('ping_rate', rclpy.Parameter.Type.INTEGER, request.ping_rate)])

            # Préparer la réponse
            response.success = True
            if changes:
                response.message = f"Configuration mise à jour: {', '.join(changes)}"
                self.get_logger().info(response.message)
            else:
                response.message = "Aucun paramètre modifié"
            
            response.current_range = float(c.range)
            response.current_gain = int(c.gainPercent)
            response.current_mode = int(c.masterMode)
            response.current_ping_rate = int(c.pingRate)

        except Exception as e:
            response.success = False
            response.message = f"Erreur lors de la configuration: {str(e)}"
            self.get_logger().error(response.message)
            response.current_range = 0.0
            response.current_gain = 0
            response.current_mode = 0
            response.current_ping_rate = 0

        return response

    # ===================================================================
    # PUBLICATION DES FRAMES SONAR
    # ===================================================================
    
    def _publish_frame(self):
        """
        Convertit les données SDK en message Frame et publie sur /docking/sonar/raw.
        
        PIPELINE DE TRAITEMENT:
        ========================
        1. Récupération thread-safe des données du dernier ping
        2. Calcul des résolutions (distance et azimut)
        3. Application du masquage des bords (artefacts)
        4. Transformation T1: Transpose pour format bearing-major
        5. Publication du message Frame
        
        TRANSFORMATION T1 (CRITIQUE):
        ==============================
        Les données SDK sont en format (n_range, n_bearing) - range-major.
        Le message Frame attend un format (n_bearing, n_range) - bearing-major.
        
        On applique une transposition (.T) puis un aplatissement (.ravel())
        en ordre row-major (C-order) pour obtenir:
            intensities[i * n_range + j] = pixel(bearing_i, range_j)
        
        Voir COORDINATE_TRANSFORMS.md pour plus de détails.
        """
        with self._lock:
            if self._last_ping_data is None or self._last_bearings_deg is None or self._last_range_m is None:
                return

            data = self._last_ping_data  # numpy array (n_range, n_bearing)
            n_range, n_bearing = data.shape
            bearings_deg = self._last_bearings_deg
            range_m = self._last_range_m

        # --- Calcul des résolutions ---
        # Distance: distribution linéaire entre [0, range_m] sur n_range bins
        range_resolution = range_m / max(n_range, 1)

        # Azimut: conversion degrés → radians, résolution moyenne
        if len(bearings_deg) >= 2:
            bearing_resolution = abs((bearings_deg[-1] - bearings_deg[0]) / max(n_bearing - 1, 1)) * math.pi / 180.0
        else:
            bearing_resolution = 0.0

        # --- Construction du message Frame ---
        msg = Frame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        msg.range_count = n_range
        msg.bearing_count = n_bearing
        msg.range_resolution = float(range_resolution)
        msg.bearing_resolution = float(bearing_resolution)
        msg.min_range = 0.0
        msg.max_range = float(range_m)

        # --- Transformation T1: Transpose + masquage + aplatissement ---
        try:
            import numpy as np
            work = np.array(data, copy=True)

            # Masquage manuel des colonnes de bord (paramètres)
            ml = int(self.get_parameter('mask_left_bearings').get_parameter_value().integer_value)
            mr = int(self.get_parameter('mask_right_bearings').get_parameter_value().integer_value)

            # Détection automatique des bandes anormalement intenses sur les bords
            # Critère: colonne de bord > seuil × médiane des colonnes internes
            if self.get_parameter('edge_mask_auto').get_parameter_value().bool_value and work.shape[1] >= 3:
                col_means = work.mean(axis=0)
                med = float(np.median(col_means[1:-1])) if work.shape[1] > 2 else float(np.median(col_means))
                thr = float(self.get_parameter('edge_mask_threshold').get_parameter_value().double_value)
                if med > 0:
                    if col_means[0] > thr * med:
                        ml = max(ml, 1)
                    if col_means[-1] > thr * med:
                        mr = max(mr, 1)

            # Application du masquage (mise à zéro des colonnes de bord)
            if ml > 0:
                ml = min(ml, work.shape[1])
                work[:, :ml] = 0.0
            if mr > 0:
                mr = min(mr, work.shape[1])
                work[:, -mr:] = 0.0

            # T1: Transpose (range,bearing) → (bearing,range) puis flatten
            # NOTE CRITIQUE: .T pour transposer, .ravel() pour aplatir en C-order
            intensities = np.clip(work, 0, 255).astype(np.uint8).T.ravel()
            msg.intensities = intensities.tolist()
        except Exception:
            # Fallback Python pur en cas d'échec NumPy
            flat = []
            for j in range(n_bearing):
                for i in range(n_range):
                    v = data[i][j]
                    v = 0 if v < 0 else (255 if v > 255 else int(v))
                    flat.append(v)
            msg.intensities = flat

        # --- Métadonnées acoustiques ---
        msg.sound_speed = 0.0  # Non fourni par le SDK, laisser à 0
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
