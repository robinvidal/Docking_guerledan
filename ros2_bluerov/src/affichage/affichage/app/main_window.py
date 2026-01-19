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
    PolarFilteredSonarPanel, 
    CartesianFilteredSonarPanel,
    CompareSonarPanel
)
from .widgets.pose_graphs import PoseGraphsWidget
from .widgets.right_controls import RightControlsPanel
from .widgets.status_header import StatusHeader
from .core.config_loader import get_cage_dimensions


class MainWindow(QMainWindow):
    """Main window assembling stacked panels and wiring signals."""

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.last_raw_frame = None
        self.last_polar_filtered_frame = None
        self.last_cartesian_filtered_frame = None

        self.setWindowTitle('Sonar Viewer - Système de Docking')
        self.setMinimumSize(1200, 800)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        self.status_header = StatusHeader()
        self.status_header.abort_clicked.connect(self.ros_node.send_abort)
        main_layout.addWidget(self.status_header)

        splitter = QSplitter(Qt.Horizontal)

        left_container = QWidget()
        left_layout = QVBoxLayout(left_container)

        left_switch = QHBoxLayout()
        self.raw_btn = QPushButton('📡 Brut')
        self.polar_filtered_btn = QPushButton('🔵 Polaire')
        self.cartesian_filtered_btn = QPushButton('🟢 Cartésien')
        self.compare_btn = QPushButton('⚖️ Comparaison')
        for btn in (self.raw_btn, self.polar_filtered_btn, self.cartesian_filtered_btn, self.compare_btn):
            btn.setCheckable(True)
        left_switch.addWidget(self.raw_btn)
        left_switch.addWidget(self.polar_filtered_btn)
        left_switch.addWidget(self.cartesian_filtered_btn)
        left_switch.addWidget(self.compare_btn)
        left_switch.addStretch()
        left_layout.addLayout(left_switch)

        self.left_stack = QStackedWidget()
        self.raw_panel = RawSonarPanel()
        self.polar_filtered_panel = PolarFilteredSonarPanel()
        self.cartesian_filtered_panel = CartesianFilteredSonarPanel()
        self.compare_panel = CompareSonarPanel()
        self.left_stack.addWidget(self.raw_panel)
        self.left_stack.addWidget(self.polar_filtered_panel)
        self.left_stack.addWidget(self.cartesian_filtered_panel)
        self.left_stack.addWidget(self.compare_panel)
        left_layout.addWidget(self.left_stack)
        
        # Connecter les signaux de clic sur les vues cartésiennes
        self.cartesian_filtered_panel.viewer.click_position.connect(self.on_click_position)
        self.compare_panel.cartesian_viewer.click_position.connect(self.on_click_position)
        
        # Connecter les signaux de sélection de bbox
        self.cartesian_filtered_panel.viewer.bbox_selected.connect(self.on_bbox_selected)
        self.compare_panel.cartesian_viewer.bbox_selected.connect(self.on_bbox_selected)
        
        # Charger les dimensions de la cage depuis le YAML et les appliquer aux vues
        cage_width, cage_height = get_cage_dimensions()
        self.cartesian_filtered_panel.viewer.set_cage_dimensions(cage_width, cage_height)
        self.compare_panel.cartesian_viewer.set_cage_dimensions(cage_width, cage_height)

        splitter.addWidget(left_container)

        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)

        right_switch = QHBoxLayout()
        self.graphs_btn = QPushButton('📊 Graphes Pose')
        self.controls_btn = QPushButton('🎛️ Contrôles')
        for btn in (self.graphs_btn, self.controls_btn):
            btn.setCheckable(True)
        right_switch.addWidget(self.graphs_btn)
        right_switch.addWidget(self.controls_btn)
        right_switch.addStretch()
        right_layout.addLayout(right_switch)

        self.right_stack = QStackedWidget()
        self.graphs_panel = PoseGraphsWidget()
        self.controls_panel = RightControlsPanel(self.ros_node)
        self.right_stack.addWidget(self.graphs_panel)
        self.right_stack.addWidget(self.controls_panel)
        right_layout.addWidget(self.right_stack)
        
        # Connecter le bouton de sélection du tracker aux vues cartésiennes (après création du controls_panel)
        self.controls_panel.tracker_widget.bbox_selection_requested.connect(
            lambda enabled: self.cartesian_filtered_panel.viewer.set_bbox_selection_mode(enabled)
        )
        self.controls_panel.tracker_widget.bbox_selection_requested.connect(
            lambda enabled: self.compare_panel.cartesian_viewer.set_bbox_selection_mode(enabled)
        )

        splitter.addWidget(right_container)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        main_layout.addWidget(splitter)

        self.raw_btn.clicked.connect(lambda: self.set_left_view(0))
        self.polar_filtered_btn.clicked.connect(lambda: self.set_left_view(1))
        self.cartesian_filtered_btn.clicked.connect(lambda: self.set_left_view(2))
        self.compare_btn.clicked.connect(lambda: self.set_left_view(3))
        self.graphs_btn.clicked.connect(lambda: self.set_right_view(0))
        self.controls_btn.clicked.connect(lambda: self.set_right_view(1))

        # default selections: start on comparaison (left) and controls (right)
        self.set_left_view(3)
        self.set_right_view(1)

        self.ros_node.signals.new_raw_frame.connect(self.on_raw_frame)
        self.ros_node.signals.new_polar_filtered_frame.connect(self.on_polar_filtered_frame)
        self.ros_node.signals.new_cartesian_filtered_frame.connect(self.on_cartesian_filtered_frame)
        self.ros_node.signals.new_borders.connect(self.on_borders)
        self.ros_node.signals.new_detected_lines.connect(self.on_detected_lines)
        self.ros_node.signals.new_tracked_object.connect(self.on_tracked_object)
        self.ros_node.signals.new_pose.connect(self.on_pose)
        self.ros_node.signals.new_state.connect(self.on_state)

    def on_raw_frame(self, msg):
        self.last_raw_frame = msg
        self.raw_panel.update_frame(msg)
        self.compare_panel.update_raw(msg)

        if self.ros_node.current_borders:
            self.raw_panel.update_borders(self.ros_node.current_borders)

    def on_polar_filtered_frame(self, msg):
        self.last_polar_filtered_frame = msg
        self.polar_filtered_panel.update_frame(msg)

        if self.ros_node.current_borders:
            self.polar_filtered_panel.update_borders(self.ros_node.current_borders)

    def on_cartesian_filtered_frame(self, msg):
        self.last_cartesian_filtered_frame = msg
        self.cartesian_filtered_panel.update_frame(msg)
        self.compare_panel.update_cartesian(msg)

        if self.ros_node.current_borders:
            self.cartesian_filtered_panel.update_borders(self.ros_node.current_borders)

    def on_borders(self, msg):
        self.raw_panel.update_borders(msg)
        self.filtered_panel.update_borders(msg)
        self.compare_panel.update_borders(msg)
    
    def on_detected_lines(self, msg):
        self.cartesian_filtered_panel.update_detected_lines(msg)
        self.compare_panel.update_detected_lines(msg)
    
    def on_tracked_object(self, msg):
        """Affiche la bounding box du tracker CSRT."""
        self.cartesian_filtered_panel.update_tracked_object(msg)
        self.compare_panel.update_tracked_object(msg)

    def on_pose(self, msg):
        self.status_header.update_pose(msg)
        self.graphs_panel.update_pose(msg)

    def on_state(self, msg):
        self.status_header.update_state(msg)

    def set_left_view(self, index):
        self.left_stack.setCurrentIndex(index)
        buttons = (self.raw_btn, self.polar_filtered_btn, self.cartesian_filtered_btn, self.compare_btn)
        for i, btn in enumerate(buttons):
            btn.setChecked(i == index)

    def set_right_view(self, index):
        self.right_stack.setCurrentIndex(index)
        buttons = (self.graphs_btn, self.controls_btn)
        for i, btn in enumerate(buttons):
            btn.setChecked(i == index)
    
    def on_click_position(self, x_m, y_m):
        """Gère un clic sur le sonar cartésien."""
        self.ros_node.publish_click_position(x_m, y_m)
    
    def on_bbox_selected(self, x, y, width, height):
        """Gère la sélection d'une bbox sur le sonar cartésien."""
        self.ros_node.publish_bbox_selection(x, y, width, height)
