import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QWidget, QVBoxLayout


class PoseGraphsWidget(QWidget):
    """Graph panel for pose history."""

    def __init__(self, max_history=200):
        super().__init__()
        self.max_history = max_history
        self.pose_history_x = []
        self.pose_history_y = []
        self.pose_history_yaw = []

        layout = QVBoxLayout(self)

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

        layout.addWidget(self.pose_plot)
        layout.addWidget(self.yaw_plot)

    def update_pose(self, msg):
        if not msg.is_valid:
            return

        self.pose_history_x.append(msg.x)
        self.pose_history_y.append(msg.y)
        self.pose_history_yaw.append(np.rad2deg(msg.yaw))

        if len(self.pose_history_x) > self.max_history:
            self.pose_history_x.pop(0)
            self.pose_history_y.pop(0)
            self.pose_history_yaw.pop(0)

        self.x_curve.setData(self.pose_history_x)
        self.y_curve.setData(self.pose_history_y)
        self.yaw_curve.setData(self.pose_history_yaw)
