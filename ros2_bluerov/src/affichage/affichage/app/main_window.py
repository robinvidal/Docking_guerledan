import numpy as np
from PyQt5.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QStackedWidget,
    QPushButton,
    QSplitter,
)
from PyQt5.QtCore import Qt

from .widgets.sonar_panels import (
    RawSonarPanel, 
    CartesianFilteredSonarPanel,
    CompareSonarPanel
)
from .widgets.tracker_control import TrackerControlWidget
from .core.config_loader import get_cage_dimensions


class MainWindow(QMainWindow):
    """Main window assembling stacked panels and wiring signals."""

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.last_raw_frame = None
        self.last_cartesian_filtered_frame = None

        self.setWindowTitle('Sonar Viewer - Système de Docking')
        self.setMinimumSize(1200, 800)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        splitter = QSplitter(Qt.Horizontal)

        left_container = QWidget()
        left_layout = QVBoxLayout(left_container)

        left_switch = QHBoxLayout()
        self.raw_btn = QPushButton('📡 Brut')
        self.cartesian_filtered_btn = QPushButton('🟢 Filtré')
        self.compare_btn = QPushButton('⚖️ Comparaison')
        for btn in (self.raw_btn, self.cartesian_filtered_btn, self.compare_btn):
            btn.setCheckable(True)
        left_switch.addWidget(self.raw_btn)
        left_switch.addWidget(self.cartesian_filtered_btn)
        left_switch.addWidget(self.compare_btn)
        left_switch.addStretch()
        left_layout.addLayout(left_switch)

        self.left_stack = QStackedWidget()
        self.raw_panel = RawSonarPanel()
        self.cartesian_filtered_panel = CartesianFilteredSonarPanel()
        self.compare_panel = CompareSonarPanel()
        self.left_stack.addWidget(self.raw_panel)
        self.left_stack.addWidget(self.cartesian_filtered_panel)
        self.left_stack.addWidget(self.compare_panel)
        left_layout.addWidget(self.left_stack)
        
        # Connecter les signaux de clic sur les vues cartésiennes
        self.cartesian_filtered_panel.viewer.click_position.connect(self.on_click_position)
        self.compare_panel.cartesian_viewer.click_position.connect(self.on_click_position)
        
        # Connecter les signaux de sélection de bbox
        self.cartesian_filtered_panel.viewer.bbox_selected.connect(self.on_bbox_selected)
        self.compare_panel.cartesian_viewer.bbox_selected.connect(self.on_bbox_selected)
        
        # Connecter les signaux de sélection rotatif (3 points)
        self.cartesian_filtered_panel.viewer.rotated_bbox_selected.connect(self.on_rotated_bbox_selected)
        self.compare_panel.cartesian_viewer.rotated_bbox_selected.connect(self.on_rotated_bbox_selected)
        
        # Charger les dimensions de la cage depuis le YAML et les appliquer aux vues
        cage_width, cage_height = get_cage_dimensions()
        self.cartesian_filtered_panel.viewer.set_cage_dimensions(cage_width, cage_height)
        self.compare_panel.cartesian_viewer.set_cage_dimensions(cage_width, cage_height)

        splitter.addWidget(left_container)

        # Panneau Tracker directement (sans onglets)
        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)
        self.tracker_widget = TrackerControlWidget(self.ros_node)
        right_layout.addWidget(self.tracker_widget)
        
        # Connecter le bouton de sélection du tracker aux vues cartésiennes
        self.tracker_widget.bbox_selection_requested.connect(
            lambda enabled: self.cartesian_filtered_panel.viewer.set_bbox_selection_mode(enabled)
        )
        self.tracker_widget.bbox_selection_requested.connect(
            lambda enabled: self.compare_panel.cartesian_viewer.set_bbox_selection_mode(enabled)
        )
        
        # Connecter le bouton de sélection rotatif (3 points)
        self.tracker_widget.rotated_selection_requested.connect(
            lambda enabled: self.cartesian_filtered_panel.viewer.set_rotated_selection_mode(enabled)
        )
        self.tracker_widget.rotated_selection_requested.connect(
            lambda enabled: self.compare_panel.cartesian_viewer.set_rotated_selection_mode(enabled)
        )

        splitter.addWidget(right_container)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        main_layout.addWidget(splitter)

        self.raw_btn.clicked.connect(lambda: self.set_left_view(0))
        self.cartesian_filtered_btn.clicked.connect(lambda: self.set_left_view(1))
        self.compare_btn.clicked.connect(lambda: self.set_left_view(2))

        # default selection: start on comparaison
        self.set_left_view(2)

        self.ros_node.signals.new_raw_frame.connect(self.on_raw_frame)
        self.ros_node.signals.new_cartesian_filtered_frame.connect(self.on_cartesian_filtered_frame)
        self.ros_node.signals.new_detected_lines.connect(self.on_detected_lines)
        self.ros_node.signals.new_tracked_object.connect(self.on_tracked_object)

    def on_raw_frame(self, msg):
        self.last_raw_frame = msg
        self.raw_panel.update_frame(msg)
        self.compare_panel.update_raw(msg)

    def on_cartesian_filtered_frame(self, msg):
        self.last_cartesian_filtered_frame = msg
        self.cartesian_filtered_panel.update_frame(msg)
        self.compare_panel.update_cartesian(msg)
    
    def on_detected_lines(self, msg):
        self.cartesian_filtered_panel.update_detected_lines(msg)
        self.compare_panel.update_detected_lines(msg)
    
    def on_tracked_object(self, msg):
        """Affiche la bounding box du tracker CSRT."""
        self.cartesian_filtered_panel.update_tracked_object(msg)
        self.compare_panel.update_tracked_object(msg)

    def set_left_view(self, index):
        self.left_stack.setCurrentIndex(index)
        buttons = (self.raw_btn, self.cartesian_filtered_btn, self.compare_btn)
        for i, btn in enumerate(buttons):
            btn.setChecked(i == index)
    
    def on_click_position(self, x_m, y_m):
        """Gère un clic sur le sonar cartésien."""
        self.ros_node.publish_click_position(x_m, y_m)
    
    def on_bbox_selected(self, x, y, width, height):
        """Gère la sélection d'une bbox sur le sonar cartésien."""
        self.ros_node.publish_bbox_selection(x, y, width, height)
    
    def on_rotated_bbox_selected(self, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, p4_x, p4_y):
        """Gère la sélection rotatif (4 coins) sur le sonar cartésien."""
        self.ros_node.publish_rotated_bbox_init(p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, p4_x, p4_y)
