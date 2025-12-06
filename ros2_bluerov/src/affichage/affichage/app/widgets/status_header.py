from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QGroupBox, QHBoxLayout, QLabel, QPushButton
import numpy as np


class StatusHeader(QWidget):
    """Mission status header with state, pose and abort."""

    abort_clicked = pyqtSignal()

    def __init__(self):
        super().__init__()
        group = QGroupBox("État Mission")
        layout = QHBoxLayout(group)

        self.state_label = QLabel("État: IDLE")
        self.state_label.setObjectName("stateLabel")
        layout.addWidget(self.state_label)

        self.pose_label = QLabel("Pose: N/A")
        self.pose_label.setObjectName("poseLabel")
        layout.addWidget(self.pose_label)

        self.confidence_label = QLabel("Confiance: N/A")
        self.confidence_label.setObjectName("confidenceLabel")
        layout.addWidget(self.confidence_label)

        layout.addStretch()

        self.abort_btn = QPushButton("🛑 ABORT")
        self.abort_btn.setObjectName("abortButton")
        self.abort_btn.clicked.connect(self.abort_clicked.emit)
        layout.addWidget(self.abort_btn)

        root_layout = QHBoxLayout(self)
        root_layout.addWidget(group)

    def update_state(self, msg):
        state_names = {
            0: "IDLE",
            1: "LOCK_ON",
            2: "APPROACH",
            3: "DOCKING",
            4: "DOCKED",
            5: "RECOVERY",
            6: "ABORT",
        }
        state_name = state_names.get(msg.current_state, f"UNKNOWN({msg.current_state})")
        self.state_label.setText(f"État: {state_name} ({msg.progress:.0%})")

    def update_pose(self, msg):
        if msg.is_valid:
            pose_text = f"Pose: x={msg.x:.2f}m, y={msg.y:.2f}m, yaw={np.rad2deg(msg.yaw):.1f}°"
            self.pose_label.setText(pose_text)
            self.confidence_label.setText(f"Confiance: {msg.confidence:.2%}")
        else:
            self.pose_label.setText("Pose: INVALIDE")
            self.confidence_label.setText("Confiance: 0%")
