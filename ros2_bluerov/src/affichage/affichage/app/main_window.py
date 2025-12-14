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

from .widgets.sonar_panels import RawSonarPanel, FilteredSonarPanel, CompareSonarPanel
from .widgets.pose_graphs import PoseGraphsWidget
from .widgets.right_controls import RightControlsPanel
from .widgets.status_header import StatusHeader


class MainWindow(QMainWindow):
    """Main window assembling stacked panels and wiring signals."""

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.last_raw_frame = None
        self.last_filtered_frame = None

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
        self.raw_btn = QPushButton('📡 Sonar Brut')
        self.filtered_btn = QPushButton('🔍 Sonar Filtré')
        self.compare_btn = QPushButton('⚖️ Comparaison')
        for btn in (self.raw_btn, self.filtered_btn, self.compare_btn):
            btn.setCheckable(True)
        left_switch.addWidget(self.raw_btn)
        left_switch.addWidget(self.filtered_btn)
        left_switch.addWidget(self.compare_btn)
        left_switch.addStretch()
        left_layout.addLayout(left_switch)

        self.left_stack = QStackedWidget()
        self.raw_panel = RawSonarPanel()
        self.filtered_panel = FilteredSonarPanel()
        self.compare_panel = CompareSonarPanel()
        self.left_stack.addWidget(self.raw_panel)
        self.left_stack.addWidget(self.filtered_panel)
        self.left_stack.addWidget(self.compare_panel)
        left_layout.addWidget(self.left_stack)

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

        splitter.addWidget(right_container)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        main_layout.addWidget(splitter)

        self.raw_btn.clicked.connect(lambda: self.set_left_view(0))
        self.filtered_btn.clicked.connect(lambda: self.set_left_view(1))
        self.compare_btn.clicked.connect(lambda: self.set_left_view(2))
        self.graphs_btn.clicked.connect(lambda: self.set_right_view(0))
        self.controls_btn.clicked.connect(lambda: self.set_right_view(1))

        # default selections: start on comparaison (left) and controls/traitement (right)
        self.set_left_view(2)
        self.set_right_view(1)

        self.ros_node.signals.new_raw_frame.connect(self.on_raw_frame)
        self.ros_node.signals.new_filtered_frame.connect(self.on_filtered_frame)
        self.ros_node.signals.new_borders.connect(self.on_borders)
        self.ros_node.signals.new_pose.connect(self.on_pose)
        self.ros_node.signals.new_state.connect(self.on_state)

    def on_raw_frame(self, msg):
        self.last_raw_frame = msg
        self.raw_panel.update_frame(msg)
        self.compare_panel.update_raw(msg)

        if self.ros_node.current_borders:
            self.raw_panel.update_borders(self.ros_node.current_borders)

    def on_filtered_frame(self, msg):
        self.last_filtered_frame = msg
        self.filtered_panel.update_frame(msg)
        self.compare_panel.update_filtered(msg)

        if self.ros_node.current_borders:
            self.filtered_panel.update_borders(self.ros_node.current_borders)

    def on_borders(self, msg):
        self.raw_panel.update_borders(msg)
        self.filtered_panel.update_borders(msg)
        self.compare_panel.update_borders(msg)

    def on_pose(self, msg):
        self.status_header.update_pose(msg)
        self.graphs_panel.update_pose(msg)

    def on_state(self, msg):
        self.status_header.update_state(msg)

    def set_left_view(self, index):
        self.left_stack.setCurrentIndex(index)
        buttons = (self.raw_btn, self.filtered_btn, self.compare_btn)
        for i, btn in enumerate(buttons):
            btn.setChecked(i == index)

    def set_right_view(self, index):
        self.right_stack.setCurrentIndex(index)
        buttons = (self.graphs_btn, self.controls_btn)
        for i, btn in enumerate(buttons):
            btn.setChecked(i == index)
