from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout

from .sonar_display import SonarCartesianWidget
from .sonar_cartesian_display import SonarCartesianImageWidget


class RawSonarPanel(QWidget):
    """Vue sonar brut (polaire)."""
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        self.viewer = SonarCartesianWidget("Sonar Brut")
        layout.addWidget(self.viewer)

    def update_frame(self, frame_msg):
        self.viewer.update_image(frame_msg)


class CartesianFilteredSonarPanel(QWidget):
    """Vue sonar cartésien filtré."""
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        self.viewer = SonarCartesianImageWidget("Sonar Cartésien Filtré")
        layout.addWidget(self.viewer)

    def update_frame(self, frame_msg):
        self.viewer.update_cartesian_frame(frame_msg)
    
    def update_detected_lines(self, lines_msg):
        self.viewer.update_detected_lines(lines_msg)
    
    def update_tracked_object(self, tracked_msg):
        self.viewer.update_tracked_object(tracked_msg)

    def update_cage_pose(self, pose_msg):
        self.viewer.update_cage_pose(pose_msg)


class CompareSonarPanel(QWidget):
    """Comparaison brut vs cartésien filtré."""
    def __init__(self):
        super().__init__()
        layout = QHBoxLayout(self)
        self.raw_viewer = SonarCartesianWidget("Brut")
        self.cartesian_viewer = SonarCartesianImageWidget("Cartésien Filtré")
        layout.addWidget(self.raw_viewer)
        layout.addWidget(self.cartesian_viewer)

    def update_raw(self, frame_msg):
        self.raw_viewer.update_image(frame_msg)

    def update_cartesian(self, frame_msg):
        self.cartesian_viewer.update_cartesian_frame(frame_msg)
    
    def update_detected_lines(self, lines_msg):
        self.cartesian_viewer.update_detected_lines(lines_msg)
    
    def update_tracked_object(self, tracked_msg):
        self.cartesian_viewer.update_tracked_object(tracked_msg)

    def update_cage_pose(self, pose_msg):
        self.cartesian_viewer.update_cage_pose(pose_msg)
