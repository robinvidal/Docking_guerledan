from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QStackedWidget

from .traitement_polar_control import TraitementPolarControlWidget
from .traitement_cartesian_control import TraitementCartesianControlWidget
from .sonar_mock_control import SonarMockControlWidget
from .tracker_control import TrackerControlWidget
from .hough_lines_control import HoughLinesControlWidget


class RightControlsPanel(QWidget):
    """Panneau de contrôles empilés pour filtres polaires, cartésiens, sonar mock, tracker et Hough."""

    def __init__(self, ros_node):
        super().__init__()
        layout = QVBoxLayout(self)

        switch_layout = QHBoxLayout()
        self.polar_btn = QPushButton("Filtres Polaires")
        self.cartesian_btn = QPushButton("Filtres Cartésiens")
        self.sonar_btn = QPushButton("Sonar Mock")
        self.tracker_btn = QPushButton("Tracker")
        self.hough_btn = QPushButton("Hough Lines")
        
        for btn in (self.polar_btn, self.cartesian_btn, self.sonar_btn, self.tracker_btn, self.hough_btn):
            btn.setCheckable(True)
            switch_layout.addWidget(btn)
        
        switch_layout.addStretch()
        layout.addLayout(switch_layout)

        self.stack = QStackedWidget()
        self.polar_widget = TraitementPolarControlWidget(ros_node)
        self.cartesian_widget = TraitementCartesianControlWidget(ros_node)
        self.sonar_widget = SonarMockControlWidget(ros_node)
        self.tracker_widget = TrackerControlWidget(ros_node)
        self.hough_widget = HoughLinesControlWidget(ros_node)
        
        # Connecter les changements de paramètres Hough au ROS node
        self.hough_widget.parameter_changed.connect(
            lambda name, value: ros_node.set_hough_parameter(name, value)
        )
        
        self.stack.addWidget(self.polar_widget)
        self.stack.addWidget(self.cartesian_widget)
        self.stack.addWidget(self.sonar_widget)
        self.stack.addWidget(self.tracker_widget)
        self.stack.addWidget(self.hough_widget)
        layout.addWidget(self.stack)

        self.polar_btn.clicked.connect(lambda: self.set_view(0))
        self.cartesian_btn.clicked.connect(lambda: self.set_view(1))
        self.sonar_btn.clicked.connect(lambda: self.set_view(2))
        self.tracker_btn.clicked.connect(lambda: self.set_view(3))
        self.hough_btn.clicked.connect(lambda: self.set_view(4))

        # default selection: filtres polaires
        self.set_view(0)

    def set_view(self, index):
        self.stack.setCurrentIndex(index)
        buttons = (self.polar_btn, self.cartesian_btn, self.sonar_btn, self.tracker_btn, self.hough_btn)
        for i, btn in enumerate(buttons):
            btn.setChecked(i == index)
