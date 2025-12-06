from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout

from .sonar_display import SonarCartesianWidget


class RawSonarPanel(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        self.viewer = SonarCartesianWidget("Sonar Brut - Vue Cartésienne")
        layout.addWidget(self.viewer)

    def update_frame(self, frame_msg):
        self.viewer.update_image(frame_msg)

    def update_borders(self, borders_msg):
        self.viewer.update_borders(borders_msg)


class FilteredSonarPanel(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        self.viewer = SonarCartesianWidget("Sonar Filtré - Vue Cartésienne")
        layout.addWidget(self.viewer)

    def update_frame(self, frame_msg):
        self.viewer.update_image(frame_msg)

    def update_borders(self, borders_msg):
        self.viewer.update_borders(borders_msg)


class CompareSonarPanel(QWidget):
    def __init__(self):
        super().__init__()
        layout = QHBoxLayout(self)
        self.raw_viewer = SonarCartesianWidget("Brut")
        self.filtered_viewer = SonarCartesianWidget("Filtré")
        layout.addWidget(self.raw_viewer)
        layout.addWidget(self.filtered_viewer)

    def update_raw(self, frame_msg):
        self.raw_viewer.update_image(frame_msg)

    def update_filtered(self, frame_msg):
        self.filtered_viewer.update_image(frame_msg)

    def update_borders(self, borders_msg):
        self.raw_viewer.update_borders(borders_msg)
        self.filtered_viewer.update_borders(borders_msg)
