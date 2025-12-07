from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QStackedWidget

from .traitement_control import TraitementControlWidget
from .sonar_mock_control import SonarMockControlWidget
from .tracker_control import TrackerControlWidget


class RightControlsPanel(QWidget):
    """Stacked controls for traitement and sonar mock."""

    def __init__(self, ros_node):
        super().__init__()
        layout = QVBoxLayout(self)

        switch_layout = QHBoxLayout()
        self.traitement_btn = QPushButton("Traitement")
        self.sonar_btn = QPushButton("Sonar Mock")
        for btn in (self.traitement_btn, self.sonar_btn):
            btn.setCheckable(True)
        switch_layout.addWidget(self.traitement_btn)
        switch_layout.addWidget(self.sonar_btn)
        switch_layout.addStretch()
        layout.addLayout(switch_layout)

        self.stack = QStackedWidget()
        self.traitement_widget = TraitementControlWidget(ros_node)
        self.sonar_widget = SonarMockControlWidget(ros_node)
        self.tracker_widget = TrackerControlWidget(ros_node)
        self.stack.addWidget(self.traitement_widget)
        self.stack.addWidget(self.sonar_widget)
        self.stack.addWidget(self.tracker_widget)
        layout.addWidget(self.stack)

        self.traitement_btn.clicked.connect(lambda: self.set_view(0))
        self.sonar_btn.clicked.connect(lambda: self.set_view(1))
        # Add tracker button and handler
        self.tracker_btn = QPushButton("Tracker")
        self.tracker_btn.setCheckable(True)
        switch_layout.insertWidget(2, self.tracker_btn)
        self.tracker_btn.clicked.connect(lambda: self.set_view(2))

        # default selection
        self.set_view(0)

    def set_view(self, index):
        self.stack.setCurrentIndex(index)
        buttons = (self.traitement_btn, self.sonar_btn)
        for i, btn in enumerate(buttons):
            btn.setChecked(i == index)
