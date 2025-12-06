from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QStackedWidget

from .traitement_control import TraitementControlWidget
from .sonar_mock_control import SonarMockControlWidget


class RightControlsPanel(QWidget):
    """Stacked controls for traitement and sonar mock."""

    def __init__(self, ros_node):
        super().__init__()
        layout = QVBoxLayout(self)

        switch_layout = QHBoxLayout()
        self.traitement_btn = QPushButton("Traitement")
        self.sonar_btn = QPushButton("Sonar Mock")
        switch_layout.addWidget(self.traitement_btn)
        switch_layout.addWidget(self.sonar_btn)
        switch_layout.addStretch()
        layout.addLayout(switch_layout)

        self.stack = QStackedWidget()
        self.traitement_widget = TraitementControlWidget(ros_node)
        self.sonar_widget = SonarMockControlWidget(ros_node)
        self.stack.addWidget(self.traitement_widget)
        self.stack.addWidget(self.sonar_widget)
        layout.addWidget(self.stack)

        self.traitement_btn.clicked.connect(lambda: self.stack.setCurrentIndex(0))
        self.sonar_btn.clicked.connect(lambda: self.stack.setCurrentIndex(1))
