"""
Visualiseur de données sonar en temps réel.
Affiche les frames sonar brutes et filtrées avec overlay des détections.
"""

import sys
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QTabWidget, QPushButton, QGroupBox)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap
import pyqtgraph as pg

import rclpy
from rclpy.node import Node
from docking_msgs.msg import Frame, Borders, PoseRelative, State
from std_msgs.msg import Bool


class ROSSignals(QObject):
    """Signaux Qt pour communication thread-safe entre ROS et Qt."""
    new_raw_frame = pyqtSignal(object)
    new_filtered_frame = pyqtSignal(object)
    new_borders = pyqtSignal(object)
    new_pose = pyqtSignal(object)
    new_state = pyqtSignal(object)


class SonarViewerNode(Node):
    """Nœud ROS2 qui souscrit aux topics et émet des signaux Qt."""
    
    def __init__(self, signals):
        super().__init__('sonar_viewer')
        self.signals = signals
        
        # Subscriptions
        self.raw_sub = self.create_subscription(
            Frame, '/docking/sonar/raw', self.raw_callback, 10
        )
        self.filtered_sub = self.create_subscription(
            Frame, '/docking/sonar/filtered', self.filtered_callback, 10
        )
        self.borders_sub = self.create_subscription(
            Borders, '/docking/tracking/borders', self.borders_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseRelative, '/docking/localisation/pose', self.pose_callback, 10
        )
        self.state_sub = self.create_subscription(
            State, '/docking/mission/state', self.state_callback, 10
        )
        
        # Publisher pour abort
        self.abort_pub = self.create_publisher(Bool, '/docking/mission/abort', 10)
        
        # Données actuelles
        self.current_borders = None
        self.current_pose = None
        self.current_state = None
        
        self.get_logger().info('Sonar Viewer démarré')
    
    def raw_callback(self, msg):
        """Callback pour frames sonar brutes."""
        # Log rapide pour debug: moyenne d'intensité
        try:
            arr = np.array(msg.intensities, dtype=np.float32)
            mean_int = float(arr.mean()) if arr.size > 0 else 0.0
        except Exception:
            mean_int = 0.0
        self.get_logger().info(f"raw_callback: mean intensity={mean_int:.1f}")
        self.signals.new_raw_frame.emit(msg)
    
    def filtered_callback(self, msg):
        """Callback pour frames sonar filtrées."""
        # Log rapide pour debug: moyenne d'intensité
        try:
            arr = np.array(msg.intensities, dtype=np.float32)
            mean_int = float(arr.mean()) if arr.size > 0 else 0.0
        except Exception:
            mean_int = 0.0
        self.get_logger().info(f"filtered_callback: mean intensity={mean_int:.1f}")
        self.signals.new_filtered_frame.emit(msg)
    
    def borders_callback(self, msg):
        """Callback pour bords détectés."""
        self.current_borders = msg
        self.signals.new_borders.emit(msg)
    
    def pose_callback(self, msg):
        """Callback pour pose relative."""
        self.current_pose = msg
        self.signals.new_pose.emit(msg)
    
    def state_callback(self, msg):
        """Callback pour état mission."""
        self.current_state = msg
        self.signals.new_state.emit(msg)
    
    def send_abort(self):
        """Envoie commande d'abort."""
        msg = Bool()
        msg.data = True
        self.abort_pub.publish(msg)
        self.get_logger().warn('Commande ABORT envoyée')


class SonarCartesianWidget(pg.PlotWidget):
    """Widget d'affichage sonar en vue cartésienne 2D (top-down)."""
    
    def __init__(self, title="Sonar"):
        super().__init__()
        self.setTitle(title)
        self.setLabel('bottom', 'X (latéral, m)', units='m')
        self.setLabel('left', 'Y (frontal, m)', units='m')
        self.setAspectLocked(True)
        
        # Scatter plot pour les points sonar
        self.scatter = pg.ScatterPlotItem(size=3, pen=pg.mkPen(None))
        self.addItem(self.scatter)
        
        # Overlay pour bords détectés (plus gros et rouge)
        self.borders_scatter = pg.ScatterPlotItem(size=15, pen=pg.mkPen(None), 
                                                 brush=pg.mkBrush(255, 0, 0, 255))
        self.addItem(self.borders_scatter)
        
        # Ligne centrale (axe de symétrie)
        self.center_line = pg.PlotCurveItem(pen=pg.mkPen('w', width=1, style=Qt.DashLine))
        self.addItem(self.center_line)
        self.center_line.setData([0, 0], [0, 50])
        
        # Marqueur position ROV (origine)
        self.rov_marker = pg.ScatterPlotItem(
            pos=[(0, 0)], size=20, symbol='t', 
            pen=pg.mkPen('g', width=2), brush=pg.mkBrush(0, 255, 0, 100)
        )
        self.addItem(self.rov_marker)
        
        # Grille
        self.showGrid(x=True, y=True, alpha=0.3)
        
        # Colormap custom dorée
        positions = [0.0, 0.25, 0.5, 0.75, 1.0]
        colors = [
            (15, 10, 5),       # noir brunâtre (fond)
            (80, 60, 20),      # bronze foncé
            (180, 140, 50),    # or sombre
            (230, 190, 80),    # or moyen
            (255, 230, 140)    # or brillant
        ]
        self.custom_colormap = pg.ColorMap(positions, colors)
        
    def update_image(self, frame_msg):
        """Convertit et affiche les données sonar en vue cartésienne."""
        # Reconstruction grille polaire
        img = np.array(frame_msg.intensities, dtype=np.uint8).reshape(
            (frame_msg.bearing_count, frame_msg.range_count)
        )
        
        # Grilles polaires
        ranges = np.linspace(frame_msg.min_range, frame_msg.max_range, frame_msg.range_count)
        # Calcul correct des angles à partir de bearing_resolution
        total_angle = frame_msg.bearing_resolution * frame_msg.bearing_count
        bearings = np.linspace(-total_angle/2, total_angle/2, frame_msg.bearing_count)
        
        # Conversion polaire -> cartésien
        points = []
        intensities = []
        
        # Sous-échantillonnage pour performance (tous les N pixels)
        step = 2
        
        for i in range(0, frame_msg.bearing_count, step):
            for j in range(0, frame_msg.range_count, step):
                intensity = img[i, j]
                
                # Filtrer les intensités faibles pour clarté
                if intensity > 30:  # Seuil minimal
                    r = ranges[j]
                    theta = bearings[i]
                    
                    # Conversion polaire -> cartésien
                    x = r * np.sin(theta)
                    y = r * np.cos(theta)
                    
                    points.append([x, y])
                    intensities.append(intensity)
        
        if points:
            points = np.array(points)
            intensities = np.array(intensities)
            
            # Colormap custom dorée
            colors = self.custom_colormap.mapToQColor(intensities / 255.0)
            brushes = [pg.mkBrush(c) for c in colors]
            
            self.scatter.setData(pos=points, brush=brushes)
        else:
            self.scatter.setData([], [])
        
    def update_borders(self, borders_msg):
        """Met à jour l'overlay des bords détectés en coordonnées cartésiennes."""
        if not borders_msg or not borders_msg.is_valid:
            self.borders_scatter.setData([], [])
            return
        
        # Conversion polaire -> cartésien pour chaque bord
        points = []
        for r, theta in zip(borders_msg.ranges, borders_msg.bearings):
            x = r * np.sin(theta)
            y = r * np.cos(theta)
            points.append([x, y])
        
        if points:
            self.borders_scatter.setData(pos=np.array(points))


class MainWindow(QMainWindow):
    """Fenêtre principale de l'application."""
    
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        
        self.setWindowTitle('Sonar Viewer - Système de Docking')
        self.setGeometry(100, 100, 1400, 900)
        
        # Widget central
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # === Panneau supérieur: État mission ===
        status_group = QGroupBox("État Mission")
        status_layout = QHBoxLayout()
        
        self.state_label = QLabel("État: IDLE")
        self.state_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        status_layout.addWidget(self.state_label)
        
        self.pose_label = QLabel("Pose: N/A")
        status_layout.addWidget(self.pose_label)
        
        self.confidence_label = QLabel("Confiance: N/A")
        status_layout.addWidget(self.confidence_label)
        
        status_layout.addStretch()
        
        # Bouton Abort
        self.abort_btn = QPushButton("🛑 ABORT")
        self.abort_btn.setStyleSheet("background-color: #ff4444; color: white; "
                                     "font-size: 14px; font-weight: bold; padding: 10px;")
        self.abort_btn.clicked.connect(self.on_abort_clicked)
        status_layout.addWidget(self.abort_btn)
        
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)
        
        # === Onglets pour différentes vues ===
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)
        
        # Onglet 1: Sonar brut (vue cartésienne)
        self.raw_widget = SonarCartesianWidget("Sonar Brut - Vue Cartésienne")
        self.tabs.addTab(self.raw_widget, "📡 Sonar Brut")
        
        # Onglet 2: Sonar filtré (vue cartésienne)
        self.filtered_widget = SonarCartesianWidget("Sonar Filtré - Vue Cartésienne")
        self.tabs.addTab(self.filtered_widget, "🔍 Sonar Filtré")
        
        # Onglet 3: Comparaison côte à côte
        compare_widget = QWidget()
        compare_layout = QHBoxLayout(compare_widget)
        self.raw_compare = SonarCartesianWidget("Brut")
        self.filtered_compare = SonarCartesianWidget("Filtré")
        compare_layout.addWidget(self.raw_compare)
        compare_layout.addWidget(self.filtered_compare)
        self.tabs.addTab(compare_widget, "⚖️ Comparaison")
        
        # Onglet 4: Graphes pose
        graphs_widget = QWidget()
        graphs_layout = QVBoxLayout(graphs_widget)
        
        self.pose_plot = pg.PlotWidget(title="Position (x, y)")
        self.pose_plot.setLabel('bottom', 'Temps (échantillons)')
        self.pose_plot.setLabel('left', 'Position (m)')
        self.pose_plot.addLegend()
        self.x_curve = self.pose_plot.plot(pen='r', name='X (latéral)')
        self.y_curve = self.pose_plot.plot(pen='g', name='Y (frontal)')
        
        self.yaw_plot = pg.PlotWidget(title="Orientation (yaw)")
        self.yaw_plot.setLabel('bottom', 'Temps (échantillons)')
        self.yaw_plot.setLabel('left', 'Angle (°)')
        self.yaw_curve = self.yaw_plot.plot(pen='b', name='Yaw')
        
        graphs_layout.addWidget(self.pose_plot)
        graphs_layout.addWidget(self.yaw_plot)
        self.tabs.addTab(graphs_widget, "📊 Graphes Pose")
        
        # Historique des données pour graphes
        self.pose_history_x = []
        self.pose_history_y = []
        self.pose_history_yaw = []
        self.max_history = 200  # Nombre de points à afficher
        
        # Stockage dernier frame pour overlay
        self.last_raw_frame = None
        self.last_filtered_frame = None
        
        # Connexion signaux
        self.ros_node.signals.new_raw_frame.connect(self.on_raw_frame)
        self.ros_node.signals.new_filtered_frame.connect(self.on_filtered_frame)
        self.ros_node.signals.new_borders.connect(self.on_borders)
        self.ros_node.signals.new_pose.connect(self.on_pose)
        self.ros_node.signals.new_state.connect(self.on_state)
        
    def on_raw_frame(self, msg):
        """Mise à jour frame brute."""
        self.last_raw_frame = msg
        self.raw_widget.update_image(msg)
        self.raw_compare.update_image(msg)
        
        # Overlay bords si disponibles
        if self.ros_node.current_borders:
            self.raw_widget.update_borders(self.ros_node.current_borders)
        # Mettre à jour le titre de l'onglet avec la moyenne d'intensité (aide debug)
        try:
            arr = np.array(msg.intensities, dtype=np.float32)
            mean_int = float(arr.mean()) if arr.size > 0 else 0.0
        except Exception:
            mean_int = 0.0
        self.tabs.setTabText(self.tabs.indexOf(self.raw_widget), f"📡 Sonar Brut ({mean_int:.0f})")
    
    def on_filtered_frame(self, msg):
        """Mise à jour frame filtrée."""
        self.last_filtered_frame = msg
        self.filtered_widget.update_image(msg)
        self.filtered_compare.update_image(msg)
        
        # Overlay bords si disponibles
        if self.ros_node.current_borders:
            self.filtered_widget.update_borders(self.ros_node.current_borders)
        # Mettre à jour le titre de l'onglet avec la moyenne d'intensité (aide debug)
        try:
            arr = np.array(msg.intensities, dtype=np.float32)
            mean_int = float(arr.mean()) if arr.size > 0 else 0.0
        except Exception:
            mean_int = 0.0
        self.tabs.setTabText(self.tabs.indexOf(self.filtered_widget), f"🔍 Sonar Filtré ({mean_int:.0f})")
    
    def on_borders(self, msg):
        """Mise à jour bords détectés."""
        self.raw_widget.update_borders(msg)
        self.filtered_widget.update_borders(msg)
        self.raw_compare.update_borders(msg)
        self.filtered_compare.update_borders(msg)
    
    def on_pose(self, msg):
        """Mise à jour pose."""
        if msg.is_valid:
            pose_text = f"Pose: x={msg.x:.2f}m, y={msg.y:.2f}m, yaw={np.rad2deg(msg.yaw):.1f}°"
            self.pose_label.setText(pose_text)
            self.confidence_label.setText(f"Confiance: {msg.confidence:.2%}")
            
            # Historique pour graphes
            self.pose_history_x.append(msg.x)
            self.pose_history_y.append(msg.y)
            self.pose_history_yaw.append(np.rad2deg(msg.yaw))
            
            # Limiter taille historique
            if len(self.pose_history_x) > self.max_history:
                self.pose_history_x.pop(0)
                self.pose_history_y.pop(0)
                self.pose_history_yaw.pop(0)
            
            # Mise à jour graphes
            self.x_curve.setData(self.pose_history_x)
            self.y_curve.setData(self.pose_history_y)
            self.yaw_curve.setData(self.pose_history_yaw)
        else:
            self.pose_label.setText("Pose: INVALIDE")
            self.confidence_label.setText("Confiance: 0%")
    
    def on_state(self, msg):
        """Mise à jour état mission."""
        state_names = {
            0: "IDLE",
            1: "LOCK_ON",
            2: "APPROACH",
            3: "DOCKING",
            4: "DOCKED",
            5: "RECOVERY",
            6: "ABORT"
        }
        
        state_name = state_names.get(msg.current_state, f"UNKNOWN({msg.current_state})")
        
        # Couleur selon état
        colors = {
            "IDLE": "#888888",
            "LOCK_ON": "#ffaa00",
            "APPROACH": "#00aaff",
            "DOCKING": "#ff8800",
            "DOCKED": "#00ff00",
            "RECOVERY": "#ffff00",
            "ABORT": "#ff0000"
        }
        color = colors.get(state_name, "#888888")
        
        self.state_label.setText(f"État: {state_name} ({msg.progress:.0%})")
        self.state_label.setStyleSheet(
            f"font-size: 16px; font-weight: bold; color: {color};"
        )
    
    def on_abort_clicked(self):
        """Gestion du bouton abort."""
        self.ros_node.send_abort()


def main(args=None):
    """Point d'entrée principal."""
    # Initialisation ROS
    rclpy.init(args=args)
    
    # Création application Qt
    app = QApplication(sys.argv)
    
    # Signaux Qt
    signals = ROSSignals()
    
    # Nœud ROS
    ros_node = SonarViewerNode(signals)
    
    # Fenêtre principale
    window = MainWindow(ros_node)
    window.show()
    
    # Timer Qt pour spinner ROS
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)  # 100 Hz
    
    # Lancement application
    try:
        exit_code = app.exec_()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
