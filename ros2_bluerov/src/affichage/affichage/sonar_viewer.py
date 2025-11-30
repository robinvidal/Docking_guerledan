"""
Visualiseur de données sonar en temps réel.
Affiche les frames sonar brutes et filtrées avec overlay des détections.
"""

import sys
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QTabWidget, QPushButton, QGroupBox,
                             QCheckBox, QSlider, QDoubleSpinBox, QSpinBox, QScrollArea,
                             QFormLayout, QFileDialog, QMessageBox)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap
import pyqtgraph as pg

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
from docking_msgs.msg import Frame, Borders, PoseRelative, State
from std_msgs.msg import Bool
import yaml
import os


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
        
        # Client pour accéder aux paramètres du nœud traitement
        self.traitement_param_client = None
        # Client pour accéder aux paramètres du nœud sonar_mock
        self.sonar_mock_param_client = None
        
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
    
    def set_traitement_parameter(self, param_name, value):
        """Modifie un paramètre du nœud traitement_node."""
        if not self.traitement_param_client:
            # Lazy initialization du client
            from rclpy.parameter import Parameter
            self.traitement_param_client = self.create_client(
                self.get_parameter_service_type(),
                '/traitement_node/set_parameters'
            )
        
        # Déterminer le type de paramètre
        if isinstance(value, bool):
            param_type = ParameterType.PARAMETER_BOOL
        elif isinstance(value, int):
            param_type = ParameterType.PARAMETER_INTEGER
        elif isinstance(value, float):
            param_type = ParameterType.PARAMETER_DOUBLE
        else:
            self.get_logger().error(f'Type de paramètre non supporté: {type(value)}')
            return False
        
        # Créer la requête de modification
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue
        
        request = SetParameters.Request()
        param = ParameterMsg()
        param.name = param_name
        param.value = ParameterValue()
        param.value.type = param_type
        
        if param_type == ParameterType.PARAMETER_BOOL:
            param.value.bool_value = value
        elif param_type == ParameterType.PARAMETER_INTEGER:
            param.value.integer_value = value
        elif param_type == ParameterType.PARAMETER_DOUBLE:
            param.value.double_value = value
        
        request.parameters = [param]
        
        # Envoyer la requête (non bloquant)
        if self.traitement_param_client.service_is_ready():
            future = self.traitement_param_client.call_async(request)
            self.get_logger().debug(f'Paramètre {param_name} = {value} envoyé')
            return True
        else:
            self.get_logger().warn('Service de paramètres traitement_node non disponible')
            return False
    
    def get_parameter_service_type(self):
        """Retourne le type de service pour les paramètres."""
        from rcl_interfaces.srv import SetParameters
        return SetParameters
    
    def set_sonar_mock_parameter(self, param_name, value):
        """Modifie un paramètre du nœud sonar_mock."""
        if not self.sonar_mock_param_client:
            # Lazy initialization du client
            from rcl_interfaces.srv import SetParameters
            self.sonar_mock_param_client = self.create_client(
                SetParameters,
                '/sonar_mock/set_parameters'
            )
        
        # Déterminer le type de paramètre
        if isinstance(value, bool):
            param_type = ParameterType.PARAMETER_BOOL
        elif isinstance(value, int):
            param_type = ParameterType.PARAMETER_INTEGER
        elif isinstance(value, float):
            param_type = ParameterType.PARAMETER_DOUBLE
        else:
            self.get_logger().error(f'Type de paramètre non supporté: {type(value)}')
            return False
        
        # Créer la requête de modification
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue
        
        request = SetParameters.Request()
        param = ParameterMsg()
        param.name = param_name
        param.value = ParameterValue()
        param.value.type = param_type
        
        if param_type == ParameterType.PARAMETER_BOOL:
            param.value.bool_value = value
        elif param_type == ParameterType.PARAMETER_INTEGER:
            param.value.integer_value = value
        elif param_type == ParameterType.PARAMETER_DOUBLE:
            param.value.double_value = value
        
        request.parameters = [param]
        
        # Envoyer la requête (non bloquant)
        if self.sonar_mock_param_client.service_is_ready():
            future = self.sonar_mock_param_client.call_async(request)
            self.get_logger().debug(f'Paramètre sonar_mock.{param_name} = {value}')
            return True
        else:
            self.get_logger().warn('Service de paramètres sonar_mock non disponible')
            return False
    
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


class TraitementControlWidget(QWidget):
    """Panneau de contrôle des paramètres de traitement en temps réel."""
    
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.param_widgets = {}  # Stockage des widgets pour récupération valeurs
        
        # Layout principal avec scroll
        main_layout = QVBoxLayout(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        
        # === En-tête avec boutons d'action ===
        header_layout = QHBoxLayout()
        
        self.save_btn = QPushButton("💾 Sauvegarder dans YAML")
        self.save_btn.setStyleSheet("background-color: #4CAF50; color: white; "
                                   "font-weight: bold; padding: 8px;")
        self.save_btn.clicked.connect(self.save_to_yaml)
        header_layout.addWidget(self.save_btn)
        
        self.reset_btn = QPushButton("🔄 Réinitialiser")
        self.reset_btn.clicked.connect(self.reset_to_defaults)
        header_layout.addWidget(self.reset_btn)
        
        header_layout.addStretch()
        scroll_layout.addLayout(header_layout)
        
        # === 1. TVG Correction ===
        tvg_group = self.create_group_box(
            "🎚️ Correction TVG (Time Varying Gain)",
            [
                ('enable_histogram_eq', 'Égalisation histogramme (CLAHE)', 'bool', True),
                ('enable_tvg_correction', 'Activer correction TVG', 'bool', False),
                ('tvg_alpha', 'Coefficient atténuation α (Np/m)', 'double', 0.0002, 0.0, 0.001, 0.00001),
                ('tvg_spreading_loss', 'Perte étalement (dB)', 'double', 20.0, 10.0, 40.0, 1.0),
            ]
        )
        scroll_layout.addWidget(tvg_group)
        
        # === 2. Filtrage de base ===
        filter_group = self.create_group_box(
            "🔍 Filtrage de base",
            [
                ('enable_median', 'Filtre médian', 'bool', True),
                ('median_kernel', 'Taille kernel médian', 'int', 3, 3, 11, 2),  # step=2 (impair)
                ('enable_gaussian', 'Filtre gaussien', 'bool', True),
                ('gaussian_sigma', 'Sigma gaussien', 'double', 1.0, 0.1, 5.0, 0.1),
            ]
        )
        scroll_layout.addWidget(filter_group)
        
        # === 3. SO-CFAR ===
        cfar_group = self.create_group_box(
            "🎯 SO-CFAR (Smallest Of - Constant False Alarm Rate)",
            [
                ('enable_so_cfar', 'Activer SO-CFAR', 'bool', True),
                ('cfar_guard_cells', 'Cellules de garde', 'int', 5, 1, 20, 1),
                ('cfar_window_size', 'Taille fenêtre référence', 'int', 10, 5, 30, 1),
                ('cfar_alpha', 'Facteur seuil α', 'double', 10.0, 1.0, 20.0, 0.5),
            ]
        )
        scroll_layout.addWidget(cfar_group)
        
        # === 4. Seuillage adaptatif ===
        adt_group = self.create_group_box(
            "✂️ Seuillage Adaptatif Final (ADT)",
            [
                ('enable_adaptive_threshold', 'Activer ADT', 'bool', False),
                ('adt_block_size', 'Taille bloc', 'int', 15, 5, 51, 2),  # impair
                ('adt_c', 'Constante C', 'int', 2, 0, 10, 1),
            ]
        )
        scroll_layout.addWidget(adt_group)
        
        # === Informations en bas ===
        info_label = QLabel(
            "ℹ️ Les modifications sont appliquées en temps réel au nœud traitement_node.\n"
            "Utilisez 'Sauvegarder dans YAML' pour rendre les changements permanents."
        )
        info_label.setStyleSheet("background-color: #e3f2fd; padding: 10px; "
                                 "border-radius: 5px; color: #1565c0;")
        info_label.setWordWrap(True)
        scroll_layout.addWidget(info_label)
        
        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)
    
    def create_group_box(self, title, params):
        """Crée un groupe de paramètres avec des widgets appropriés."""
        group = QGroupBox(title)
        layout = QFormLayout()
        
        for param_info in params:
            if param_info[2] == 'bool':
                # Checkbox pour booléen
                param_name, label, _, default = param_info
                checkbox = QCheckBox()
                checkbox.setChecked(default)
                checkbox.stateChanged.connect(
                    lambda state, name=param_name: self.on_param_changed(name, state == Qt.Checked)
                )
                layout.addRow(label + ':', checkbox)
                self.param_widgets[param_name] = checkbox
                
            elif param_info[2] == 'int':
                # Slider + SpinBox pour entier
                param_name, label, _, default, min_val, max_val, step = param_info
                
                widget_layout = QHBoxLayout()
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(min_val)
                slider.setMaximum(max_val)
                slider.setValue(default)
                slider.setSingleStep(step)
                
                spinbox = QSpinBox()
                spinbox.setMinimum(min_val)
                spinbox.setMaximum(max_val)
                spinbox.setValue(default)
                spinbox.setSingleStep(step)
                
                # Synchronisation slider <-> spinbox
                slider.valueChanged.connect(spinbox.setValue)
                spinbox.valueChanged.connect(slider.setValue)
                spinbox.valueChanged.connect(
                    lambda value, name=param_name: self.on_param_changed(name, value)
                )
                
                widget_layout.addWidget(slider, 3)
                widget_layout.addWidget(spinbox, 1)
                layout.addRow(label + ':', widget_layout)
                self.param_widgets[param_name] = spinbox
                
            elif param_info[2] == 'double':
                # Slider + DoubleSpinBox pour flottant
                param_name, label, _, default, min_val, max_val, step = param_info
                
                widget_layout = QHBoxLayout()
                
                # Slider avec résolution x1000 pour précision
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(int(min_val / step))
                slider.setMaximum(int(max_val / step))
                slider.setValue(int(default / step))
                
                spinbox = QDoubleSpinBox()
                spinbox.setMinimum(min_val)
                spinbox.setMaximum(max_val)
                spinbox.setValue(default)
                spinbox.setSingleStep(step)
                spinbox.setDecimals(len(str(step).split('.')[-1]) if '.' in str(step) else 0)
                
                # Synchronisation slider <-> spinbox
                slider.valueChanged.connect(lambda v, sb=spinbox, s=step: sb.setValue(v * s))
                spinbox.valueChanged.connect(lambda v, sl=slider, s=step: sl.setValue(int(v / s)))
                spinbox.valueChanged.connect(
                    lambda value, name=param_name: self.on_param_changed(name, value)
                )
                
                widget_layout.addWidget(slider, 3)
                widget_layout.addWidget(spinbox, 1)
                layout.addRow(label + ':', widget_layout)
                self.param_widgets[param_name] = spinbox
        
        group.setLayout(layout)
        return group
    
    def on_param_changed(self, param_name, value):
        """Callback appelé quand un paramètre change."""
        # Envoyer au nœud ROS
        success = self.ros_node.set_traitement_parameter(param_name, value)
        if success:
            self.ros_node.get_logger().info(f'Paramètre {param_name} = {value}')
    
    def get_current_params(self):
        """Récupère les valeurs actuelles de tous les paramètres."""
        params = {}
        for name, widget in self.param_widgets.items():
            if isinstance(widget, QCheckBox):
                params[name] = widget.isChecked()
            elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                params[name] = widget.value()
        return params
    
    def save_to_yaml(self):
        """Sauvegarde les paramètres actuels dans un fichier YAML."""
        # Récupérer valeurs actuelles
        params = self.get_current_params()
        
        # Construire structure YAML
        yaml_content = {
            'traitement_node': {
                'ros__parameters': params
            }
        }
        
        # Dialogue de sauvegarde
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Sauvegarder paramètres de traitement",
            "traitement_params.yaml",
            "YAML Files (*.yaml *.yml)"
        )
        
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    yaml.dump(yaml_content, f, default_flow_style=False, sort_keys=False)
                
                QMessageBox.information(
                    self,
                    "Sauvegarde réussie",
                    f"Paramètres sauvegardés dans:\n{file_path}\n\n"
                    "Pour les utiliser au démarrage:\n"
                    "ros2 run traitement traitement_node --ros-args "
                    f"--params-file {file_path}"
                )
                self.ros_node.get_logger().info(f'Paramètres sauvegardés: {file_path}')
            except Exception as e:
                QMessageBox.critical(
                    self,
                    "Erreur de sauvegarde",
                    f"Impossible de sauvegarder:\n{str(e)}"
                )
                self.ros_node.get_logger().error(f'Erreur sauvegarde: {e}')
    
    def reset_to_defaults(self):
        """Réinitialise tous les paramètres aux valeurs par défaut."""
        reply = QMessageBox.question(
            self,
            "Confirmer réinitialisation",
            "Voulez-vous vraiment réinitialiser tous les paramètres aux valeurs par défaut ?",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            # Valeurs par défaut (à synchroniser avec traitement_params.yaml)
            defaults = {
                'enable_histogram_eq': True,
                'enable_tvg_correction': False,
                'tvg_alpha': 0.0002,
                'tvg_spreading_loss': 20.0,
                'enable_median': True,
                'median_kernel': 3,
                'enable_gaussian': True,
                'gaussian_sigma': 1.0,
                'enable_so_cfar': True,
                'cfar_guard_cells': 5,
                'cfar_window_size': 10,
                'cfar_alpha': 10.0,
                'enable_adaptive_threshold': False,
                'adt_block_size': 15,
                'adt_c': 2,
            }
            
            # Appliquer aux widgets
            for name, value in defaults.items():
                if name in self.param_widgets:
                    widget = self.param_widgets[name]
                    if isinstance(widget, QCheckBox):
                        widget.setChecked(value)
                    elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                        widget.setValue(value)


class SonarMockControlWidget(QWidget):
    """Panneau de contrôle des paramètres du sonar_mock en temps réel."""
    
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.param_widgets = {}
        
        # Layout principal avec scroll
        main_layout = QVBoxLayout(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        
        # En-tête
        header_label = QLabel(
            "⚙️ <b>Contrôle Simulateur Sonar</b><br>"
            "<small>Ajustez les paramètres du sonar_mock en temps réel.</small>"
        )
        header_label.setStyleSheet("background-color: #e8f5e9; padding: 10px; "
                                   "border-radius: 5px; color: #2e7d32;")
        header_label.setWordWrap(True)
        scroll_layout.addWidget(header_label)
        
        # 1. Bruit
        noise_group = QGroupBox("🔊 Bruit de Fond")
        noise_layout = QFormLayout()
        noise_info = QLabel(
            "⚠️ Cage toujours à 200-255 (non affectée)\n"
            "• 3-5: Très propre  • 10-15: Modéré  • 20-30: Très bruité"
        )
        noise_info.setStyleSheet("color: #666; font-size: 10px; font-style: italic;")
        noise_info.setWordWrap(True)
        noise_layout.addRow(noise_info)
        
        noise_layout_h = QHBoxLayout()
        noise_slider = QSlider(Qt.Horizontal)
        noise_slider.setMinimum(0)
        noise_slider.setMaximum(100)
        noise_slider.setValue(20)
        noise_spin = QDoubleSpinBox()
        noise_spin.setMinimum(0.0)
        noise_spin.setMaximum(50.0)
        noise_spin.setValue(10.0)
        noise_spin.setSingleStep(0.5)
        noise_spin.setDecimals(1)
        noise_slider.valueChanged.connect(lambda v: noise_spin.setValue(v / 2.0))
        noise_spin.valueChanged.connect(lambda v: noise_slider.setValue(int(v * 2)))
        noise_spin.valueChanged.connect(lambda v: self.on_param_changed('noise_level', v))
        noise_layout_h.addWidget(noise_slider, 3)
        noise_layout_h.addWidget(noise_spin, 1)
        noise_layout.addRow('Niveau bruit:', noise_layout_h)
        self.param_widgets['noise_level'] = noise_spin
        noise_group.setLayout(noise_layout)
        scroll_layout.addWidget(noise_group)
        
        # 2. Filtres
        filters_group = QGroupBox("🔍 Filtres Pré-appliqués")
        filters_layout = QFormLayout()
        
        filters_info = QLabel(
            "⚠️ Ces filtres s'appliquent AVANT publication sur /docking/sonar/raw\n"
            "Le contraste est ESSENTIEL pour voir le bruit après filtrage médian/gaussien"
        )
        filters_info.setStyleSheet("color: #666; font-size: 10px; font-style: italic; background-color: #fff3cd; padding: 5px;")
        filters_info.setWordWrap(True)
        filters_layout.addRow(filters_info)
        
        median_check = QCheckBox()
        median_check.setChecked(True)
        median_check.stateChanged.connect(lambda s: self.on_param_changed('enable_median', s == Qt.Checked))
        filters_layout.addRow('Filtre médian:', median_check)
        self.param_widgets['enable_median'] = median_check
        
        median_layout_h = QHBoxLayout()
        median_slider = QSlider(Qt.Horizontal)
        median_slider.setMinimum(3)
        median_slider.setMaximum(11)
        median_slider.setValue(3)
        median_slider.setSingleStep(2)
        median_spin = QSpinBox()
        median_spin.setMinimum(3)
        median_spin.setMaximum(11)
        median_spin.setValue(3)
        median_spin.setSingleStep(2)
        median_slider.valueChanged.connect(median_spin.setValue)
        median_spin.valueChanged.connect(median_slider.setValue)
        median_spin.valueChanged.connect(lambda v: self.on_param_changed('median_kernel', v))
        median_layout_h.addWidget(median_slider, 3)
        median_layout_h.addWidget(median_spin, 1)
        filters_layout.addRow('  Kernel:', median_layout_h)
        self.param_widgets['median_kernel'] = median_spin
        
        gaussian_check = QCheckBox()
        gaussian_check.setChecked(True)
        gaussian_check.stateChanged.connect(lambda s: self.on_param_changed('enable_gaussian', s == Qt.Checked))
        filters_layout.addRow('Filtre gaussien:', gaussian_check)
        self.param_widgets['enable_gaussian'] = gaussian_check
        
        gaussian_layout_h = QHBoxLayout()
        gaussian_slider = QSlider(Qt.Horizontal)
        gaussian_slider.setMinimum(1)
        gaussian_slider.setMaximum(50)
        gaussian_slider.setValue(24)  # 2.4 * 10
        gaussian_spin = QDoubleSpinBox()
        gaussian_spin.setMinimum(0.1)
        gaussian_spin.setMaximum(5.0)
        gaussian_spin.setValue(2.4)  # Valeur de sonar_params.yaml
        gaussian_spin.setSingleStep(0.1)
        gaussian_spin.setDecimals(1)
        gaussian_slider.valueChanged.connect(lambda v: gaussian_spin.setValue(v / 10.0))
        gaussian_spin.valueChanged.connect(lambda v: gaussian_slider.setValue(int(v * 10)))
        gaussian_spin.valueChanged.connect(lambda v: self.on_param_changed('gaussian_sigma', v))
        gaussian_layout_h.addWidget(gaussian_slider, 3)
        gaussian_layout_h.addWidget(gaussian_spin, 1)
        filters_layout.addRow('  Sigma:', gaussian_layout_h)
        self.param_widgets['gaussian_sigma'] = gaussian_spin
        
        contrast_check = QCheckBox()
        contrast_check.setChecked(True)
        contrast_check.stateChanged.connect(lambda s: self.on_param_changed('enable_contrast', s == Qt.Checked))
        filters_layout.addRow('Amélioration contraste:', contrast_check)
        self.param_widgets['enable_contrast'] = contrast_check
        
        contrast_layout_h = QHBoxLayout()
        contrast_slider = QSlider(Qt.Horizontal)
        contrast_slider.setMinimum(1)   # 0.05 * 20
        contrast_slider.setMaximum(100) # 5.0 * 20
        contrast_slider.setValue(7)     # 0.35 * 20 = 7
        contrast_spin = QDoubleSpinBox()
        contrast_spin.setMinimum(0.05)
        contrast_spin.setMaximum(5.0)
        contrast_spin.setValue(0.35)  # Valeur de sonar_params.yaml (CRITIQUE pour bruit visible)
        contrast_spin.setSingleStep(0.05)
        contrast_spin.setDecimals(2)
        contrast_slider.valueChanged.connect(lambda v: contrast_spin.setValue(v / 20.0))
        contrast_spin.valueChanged.connect(lambda v: contrast_slider.setValue(int(v * 20)))
        contrast_spin.valueChanged.connect(lambda v: self.on_param_changed('contrast_clip', v))
        contrast_layout_h.addWidget(contrast_slider, 3)
        contrast_layout_h.addWidget(contrast_spin, 1)
        filters_layout.addRow('  Clip (%):', contrast_layout_h)
        self.param_widgets['contrast_clip'] = contrast_spin
        
        range_comp_check = QCheckBox()
        range_comp_check.setChecked(False)
        range_comp_check.stateChanged.connect(lambda s: self.on_param_changed('enable_range_comp', s == Qt.Checked))
        filters_layout.addRow('Compensation portée:', range_comp_check)
        self.param_widgets['enable_range_comp'] = range_comp_check
        
        range_comp_layout_h = QHBoxLayout()
        range_comp_slider = QSlider(Qt.Horizontal)
        range_comp_slider.setMinimum(0)
        range_comp_slider.setMaximum(100)
        range_comp_slider.setValue(10)
        range_comp_spin = QDoubleSpinBox()
        range_comp_spin.setMinimum(0.0)
        range_comp_spin.setMaximum(0.1)
        range_comp_spin.setValue(0.01)
        range_comp_spin.setSingleStep(0.001)
        range_comp_spin.setDecimals(3)
        range_comp_slider.valueChanged.connect(lambda v: range_comp_spin.setValue(v / 1000.0))
        range_comp_spin.valueChanged.connect(lambda v: range_comp_slider.setValue(int(v * 1000)))
        range_comp_spin.valueChanged.connect(lambda v: self.on_param_changed('range_comp_alpha', v))
        range_comp_layout_h.addWidget(range_comp_slider, 3)
        range_comp_layout_h.addWidget(range_comp_spin, 1)
        filters_layout.addRow('  Alpha (m⁻¹):', range_comp_layout_h)
        self.param_widgets['range_comp_alpha'] = range_comp_spin
        
        filters_group.setLayout(filters_layout)
        scroll_layout.addWidget(filters_group)
        
        # 3. Géométrie
        geom_group = QGroupBox("📐 Géométrie Sonar")
        geom_layout = QFormLayout()
        
        bearing_layout_h = QHBoxLayout()
        bearing_slider = QSlider(Qt.Horizontal)
        bearing_slider.setMinimum(60)
        bearing_slider.setMaximum(180)
        bearing_slider.setValue(140)
        bearing_spin = QDoubleSpinBox()
        bearing_spin.setMinimum(60.0)
        bearing_spin.setMaximum(180.0)
        bearing_spin.setValue(140.0)
        bearing_spin.setSingleStep(5.0)
        bearing_slider.valueChanged.connect(bearing_spin.setValue)
        bearing_spin.valueChanged.connect(lambda v: bearing_slider.setValue(int(v)))
        bearing_spin.valueChanged.connect(lambda v: self.on_param_changed('bearing_angle', v))
        bearing_layout_h.addWidget(bearing_slider, 3)
        bearing_layout_h.addWidget(bearing_spin, 1)
        geom_layout.addRow('Ouverture (°):', bearing_layout_h)
        self.param_widgets['bearing_angle'] = bearing_spin
        
        geom_group.setLayout(geom_layout)
        scroll_layout.addWidget(geom_group)
        
        # Boutons
        btn_layout = QHBoxLayout()
        save_btn = QPushButton("💾 Sauvegarder")
        save_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 8px;")
        save_btn.clicked.connect(self.save_to_yaml)
        btn_layout.addWidget(save_btn)
        
        reset_btn = QPushButton("🔄 Réinitialiser")
        reset_btn.clicked.connect(self.reset_to_defaults)
        btn_layout.addWidget(reset_btn)
        scroll_layout.addLayout(btn_layout)
        
        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)
    
    def on_param_changed(self, param_name, value):
        success = self.ros_node.set_sonar_mock_parameter(param_name, value)
        if success:
            self.ros_node.get_logger().info(f'Sonar: {param_name}={value}')
    
    def get_current_params(self):
        params = {}
        for name, widget in self.param_widgets.items():
            if isinstance(widget, QCheckBox):
                params[name] = widget.isChecked()
            elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                params[name] = widget.value()
        return params
    
    def save_to_yaml(self):
        params = self.get_current_params()
        yaml_content = {'sonar_mock': {'ros__parameters': params}}
        
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Sauvegarder paramètres sonar_mock", 
            "sonar_params.yaml", "YAML Files (*.yaml *.yml)"
        )
        
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    yaml.dump(yaml_content, f, default_flow_style=False, sort_keys=False)
                QMessageBox.information(self, "Sauvegarde réussie", f"Sauvegardé: {file_path}")
            except Exception as e:
                QMessageBox.critical(self, "Erreur", f"Erreur: {str(e)}")
    
    def reset_to_defaults(self):
        if QMessageBox.question(self, "Confirmer", "Réinitialiser?") == QMessageBox.Yes:
            self.param_widgets['noise_level'].setValue(10.0)
            self.param_widgets['enable_median'].setChecked(True)
            self.param_widgets['median_kernel'].setValue(3)
            self.param_widgets['enable_gaussian'].setChecked(True)
            self.param_widgets['gaussian_sigma'].setValue(2.4)  # sonar_params.yaml
            self.param_widgets['enable_contrast'].setChecked(True)
            self.param_widgets['contrast_clip'].setValue(0.35)  # sonar_params.yaml
            self.param_widgets['enable_range_comp'].setChecked(False)
            self.param_widgets['range_comp_alpha'].setValue(0.01)
            self.param_widgets['bearing_angle'].setValue(140.0)


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
        
        # Onglet 5: Contrôle traitement
        self.control_widget = TraitementControlWidget(self.ros_node)
        self.tabs.addTab(self.control_widget, "⚙️ Contrôle Traitement")
        
        # Onglet 6: Contrôle sonar mock
        self.sonar_control_widget = SonarMockControlWidget(self.ros_node)
        self.tabs.addTab(self.sonar_control_widget, "🎛️ Contrôle Sonar Mock")
        
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
