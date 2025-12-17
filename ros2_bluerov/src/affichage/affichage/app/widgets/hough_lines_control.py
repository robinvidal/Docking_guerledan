"""
Widget de contrôle pour la détection de lignes par Hough.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QSpinBox,
    QCheckBox, QGroupBox, QPushButton, QDoubleSpinBox
)
from PyQt5.QtCore import Qt, pyqtSignal


class HoughLinesControlWidget(QWidget):
    """Panneau de contrôle pour les paramètres de détection Hough."""
    
    # Signaux pour changements de paramètres
    parameter_changed = pyqtSignal(str, object)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # === Activation ===
        enable_group = QGroupBox("Activation")
        enable_layout = QVBoxLayout()
        
        self.enable_check = QCheckBox("Activer détection Hough")
        self.enable_check.setChecked(True)
        self.enable_check.stateChanged.connect(
            lambda: self.parameter_changed.emit('enable_detection', self.enable_check.isChecked())
        )
        enable_layout.addWidget(self.enable_check)
        
        enable_group.setLayout(enable_layout)
        layout.addWidget(enable_group)
        
        # === Nombre de lignes ===
        lines_group = QGroupBox("Nombre de lignes")
        lines_layout = QHBoxLayout()
        
        self.num_lines_slider = QSlider(Qt.Horizontal)
        self.num_lines_slider.setMinimum(1)
        self.num_lines_slider.setMaximum(20)
        self.num_lines_slider.setValue(5)
        self.num_lines_slider.setTickPosition(QSlider.TicksBelow)
        self.num_lines_slider.setTickInterval(1)
        
        self.num_lines_spin = QSpinBox()
        self.num_lines_spin.setMinimum(1)
        self.num_lines_spin.setMaximum(20)
        self.num_lines_spin.setValue(5)
        
        # Synchronisation
        self.num_lines_slider.valueChanged.connect(self.num_lines_spin.setValue)
        self.num_lines_spin.valueChanged.connect(self.num_lines_slider.setValue)
        self.num_lines_slider.valueChanged.connect(
            lambda v: self.parameter_changed.emit('num_lines', v)
        )
        
        lines_layout.addWidget(QLabel("n ="))
        lines_layout.addWidget(self.num_lines_slider, 3)
        lines_layout.addWidget(self.num_lines_spin, 1)
        
        lines_group.setLayout(lines_layout)
        layout.addWidget(lines_group)
        
        # === Paramètres de détection ===
        detection_group = QGroupBox("Paramètres de détection")
        detection_layout = QVBoxLayout()
        
        # Seuil de votes
        threshold_layout = QHBoxLayout()
        self.threshold_slider = QSlider(Qt.Horizontal)
        self.threshold_slider.setMinimum(10)
        self.threshold_slider.setMaximum(200)
        self.threshold_slider.setValue(50)
        self.threshold_spin = QSpinBox()
        self.threshold_spin.setMinimum(10)
        self.threshold_spin.setMaximum(200)
        self.threshold_spin.setValue(50)
        self.threshold_slider.valueChanged.connect(self.threshold_spin.setValue)
        self.threshold_spin.valueChanged.connect(self.threshold_slider.setValue)
        self.threshold_slider.valueChanged.connect(
            lambda v: self.parameter_changed.emit('threshold', v)
        )
        threshold_layout.addWidget(QLabel("Seuil votes:"))
        threshold_layout.addWidget(self.threshold_slider, 3)
        threshold_layout.addWidget(self.threshold_spin, 1)
        detection_layout.addLayout(threshold_layout)
        
        # Résolution rho
        rho_layout = QHBoxLayout()
        self.rho_spin = QDoubleSpinBox()
        self.rho_spin.setMinimum(0.1)
        self.rho_spin.setMaximum(10.0)
        self.rho_spin.setSingleStep(0.1)
        self.rho_spin.setValue(1.0)
        self.rho_spin.valueChanged.connect(
            lambda v: self.parameter_changed.emit('rho_resolution', v)
        )
        rho_layout.addWidget(QLabel("Résolution ρ (px):"))
        rho_layout.addWidget(self.rho_spin)
        rho_layout.addStretch()
        detection_layout.addLayout(rho_layout)
        
        # Résolution theta
        theta_layout = QHBoxLayout()
        self.theta_spin = QDoubleSpinBox()
        self.theta_spin.setMinimum(0.1)
        self.theta_spin.setMaximum(10.0)
        self.theta_spin.setSingleStep(0.1)
        self.theta_spin.setValue(1.0)
        self.theta_spin.valueChanged.connect(
            lambda v: self.parameter_changed.emit('theta_resolution', v)
        )
        theta_layout.addWidget(QLabel("Résolution θ (°):"))
        theta_layout.addWidget(self.theta_spin)
        theta_layout.addStretch()
        detection_layout.addLayout(theta_layout)
        
        detection_group.setLayout(detection_layout)
        layout.addWidget(detection_group)
        
        # === Mode de détection ===
        mode_group = QGroupBox("Mode")
        mode_layout = QVBoxLayout()
        
        self.prob_check = QCheckBox("Utiliser HoughLinesP (probabiliste)")
        self.prob_check.setChecked(True)
        self.prob_check.stateChanged.connect(
            lambda: self.parameter_changed.emit('use_probabilistic', self.prob_check.isChecked())
        )
        mode_layout.addWidget(self.prob_check)
        
        # Paramètres probabilistes
        self.prob_params_widget = QWidget()
        prob_params_layout = QVBoxLayout(self.prob_params_widget)
        prob_params_layout.setContentsMargins(20, 0, 0, 0)
        
        # Longueur minimale
        min_length_layout = QHBoxLayout()
        self.min_length_spin = QSpinBox()
        self.min_length_spin.setMinimum(1)
        self.min_length_spin.setMaximum(500)
        self.min_length_spin.setValue(20)
        self.min_length_spin.valueChanged.connect(
            lambda v: self.parameter_changed.emit('min_line_length', v)
        )
        min_length_layout.addWidget(QLabel("Longueur min (px):"))
        min_length_layout.addWidget(self.min_length_spin)
        min_length_layout.addStretch()
        prob_params_layout.addLayout(min_length_layout)
        
        # Gap maximal
        max_gap_layout = QHBoxLayout()
        self.max_gap_spin = QSpinBox()
        self.max_gap_spin.setMinimum(1)
        self.max_gap_spin.setMaximum(100)
        self.max_gap_spin.setValue(10)
        self.max_gap_spin.valueChanged.connect(
            lambda v: self.parameter_changed.emit('max_line_gap', v)
        )
        max_gap_layout.addWidget(QLabel("Gap max (px):"))
        max_gap_layout.addWidget(self.max_gap_spin)
        max_gap_layout.addStretch()
        prob_params_layout.addLayout(max_gap_layout)
        
        mode_layout.addWidget(self.prob_params_widget)
        self.prob_check.stateChanged.connect(self.prob_params_widget.setVisible)
        
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)
        
        # Boutons de gestion
        buttons_layout = QHBoxLayout()
        
        reset_btn = QPushButton("Réinitialiser")
        reset_btn.clicked.connect(self.reset_parameters)
        buttons_layout.addWidget(reset_btn)
        
        layout.addLayout(buttons_layout)
        layout.addStretch()
    
    def reset_parameters(self):
        """Réinitialise tous les paramètres aux valeurs par défaut."""
        self.enable_check.setChecked(True)
        self.num_lines_slider.setValue(5)
        self.threshold_slider.setValue(50)
        self.rho_spin.setValue(1.0)
        self.theta_spin.setValue(1.0)
        self.prob_check.setChecked(True)
        self.min_length_spin.setValue(20)
        self.max_gap_spin.setValue(10)
    
    def get_parameters(self):
        """Retourne un dictionnaire de tous les paramètres."""
        return {
            'enable_detection': self.enable_check.isChecked(),
            'num_lines': self.num_lines_slider.value(),
            'threshold': self.threshold_slider.value(),
            'rho_resolution': self.rho_spin.value(),
            'theta_resolution': self.theta_spin.value(),
            'use_probabilistic': self.prob_check.isChecked(),
            'min_line_length': self.min_length_spin.value(),
            'max_line_gap': self.max_gap_spin.value(),
        }
