"""
Widget de contrôle pour les paramètres du nœud traitement_polar_node.
"""

from pathlib import Path
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QScrollArea, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QFormLayout, QCheckBox, QSlider, QSpinBox, QDoubleSpinBox,
    QFileDialog, QMessageBox,
)
import yaml

from ..core.utils import load_yaml_params


class TraitementPolarControlWidget(QWidget):
    """Panneau de contrôle pour les filtres polaires."""

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.param_widgets = {}

        main_layout = QVBoxLayout(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)

        # ========== FILTRES POLAIRES ==========
        polar_gaussian_group = self.create_group_box(
            "🌀 Filtre Gaussien",
            [
                ('polar_enable_gaussian', 'Activer', 'bool', True),
                ('polar_gaussian_sigma', 'Sigma', 'double', 1.0, 0.1, 10.0, 0.1),
            ],
        )
        scroll_layout.addWidget(polar_gaussian_group)

        polar_median_group = self.create_group_box(
            "📊 Filtre Médian",
            [
                ('polar_enable_median', 'Activer', 'bool', True),
                ('polar_median_kernel', 'Taille kernel', 'int', 3, 3, 15, 2),
            ],
        )
        scroll_layout.addWidget(polar_median_group)

        polar_loss_group = self.create_group_box(
            "🔧 Filtre Loss (Lee)",
            [
                ('polar_enable_loss', 'Activer', 'bool', False),
                ('polar_loss_window_size', 'Taille fenêtre', 'int', 5, 3, 15, 2),
            ],
        )
        scroll_layout.addWidget(polar_loss_group)

        polar_frost_group = self.create_group_box(
            "❄️ Filtre Frost",
            [
                ('polar_enable_frost', 'Activer', 'bool', False),
                ('polar_frost_window_size', 'Taille fenêtre', 'int', 5, 3, 15, 2),
                ('polar_frost_damping', 'Damping factor', 'double', 1.0, 0.1, 5.0, 0.1),
            ],
        )
        scroll_layout.addWidget(polar_frost_group)

        polar_log_group = self.create_group_box(
            "📈 Compression Log",
            [
                ('polar_enable_log_compression', 'Activer', 'bool', False),
                ('polar_log_scale', 'Facteur échelle', 'double', 30.0, 1.0, 100.0, 1.0),
            ],
        )
        scroll_layout.addWidget(polar_log_group)

        info_label = QLabel(
            "ℹ️ Filtres appliqués sur données polaires (bearing × range).\n"
            "Modifications appliquées en temps réel à traitement_polar_node."
        )
        info_label.setWordWrap(True)
        scroll_layout.addWidget(info_label)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)

        # Footer buttons
        footer_layout = QHBoxLayout()
        footer_layout.addStretch()
        self.save_btn = QPushButton("💾 Sauvegarder")
        self.save_btn.clicked.connect(self.save_to_yaml)
        footer_layout.addWidget(self.save_btn)
        self.reset_btn = QPushButton("🔄 Réinitialiser")
        self.reset_btn.clicked.connect(self.reset_to_defaults)
        footer_layout.addWidget(self.reset_btn)
        main_layout.addLayout(footer_layout)

        self._apply_yaml_defaults()

    def create_group_box(self, title, params):
        group = QGroupBox(title)
        layout = QFormLayout()

        for param_info in params:
            if param_info[2] == 'bool':
                param_name, label, _, default = param_info
                checkbox = QCheckBox()
                checkbox.setChecked(default)
                checkbox.stateChanged.connect(
                    lambda state, name=param_name: self.on_param_changed(name, state == Qt.Checked)
                )
                layout.addRow(label + ':', checkbox)
                self.param_widgets[param_name] = checkbox

            elif param_info[2] == 'int':
                param_name, label, _, default, min_val, max_val, step = param_info
                widget_layout = QHBoxLayout()
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(min_val)
                slider.setMaximum(max_val)
                slider.setValue(default)
                slider.setSingleStep(step)
                spinbox = QSpinBox()
                spinbox.setMinimum(min_val)
                spinbox.setMaximum(max_val)
                spinbox.setValue(default)
                spinbox.setSingleStep(step)
                slider.valueChanged.connect(spinbox.setValue)
                spinbox.valueChanged.connect(slider.setValue)
                spinbox.valueChanged.connect(
                    lambda value, name=param_name: self.on_param_changed(name, value)
                )
                widget_layout.addWidget(slider, 3)
                widget_layout.addWidget(spinbox, 1)
                layout.addRow(label + ':', widget_layout)
                self.param_widgets[param_name] = spinbox

            elif param_info[2] == 'double':
                param_name, label, _, default, min_val, max_val, step = param_info
                widget_layout = QHBoxLayout()
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(int(min_val / step))
                slider.setMaximum(int(max_val / step))
                slider.setValue(int(default / step))
                spinbox = QDoubleSpinBox()
                spinbox.setMinimum(min_val)
                spinbox.setMaximum(max_val)
                spinbox.setValue(default)
                spinbox.setSingleStep(step)
                spinbox.setDecimals(len(str(step).split('.')[-1]) if '.' in str(step) else 0)
                slider.valueChanged.connect(lambda v, sb=spinbox, s=step: sb.setValue(v * s))
                spinbox.valueChanged.connect(lambda v, sl=slider, s=step: sl.setValue(int(v / s)))
                spinbox.valueChanged.connect(
                    lambda value, name=param_name: self.on_param_changed(name, value)
                )
                widget_layout.addWidget(slider, 3)
                widget_layout.addWidget(spinbox, 1)
                layout.addRow(label + ':', widget_layout)
                self.param_widgets[param_name] = spinbox

        group.setLayout(layout)
        return group

    def on_param_changed(self, param_name, value):
        success = self.ros_node.set_polar_parameter(param_name, value)
        if success:
            self.ros_node.get_logger().info(f'Polar: {param_name} = {value}')

    def _apply_yaml_defaults(self):
        params = load_yaml_params('traitement', 'traitement_polar_params.yaml', self.ros_node.get_logger())
        sub = params.get('traitement_polar_node', {}).get('ros__parameters', {}) if isinstance(params, dict) else {}
        if not sub:
            return
        for name, value in sub.items():
            if name not in self.param_widgets:
                continue
            widget = self.param_widgets[name]
            try:
                if isinstance(widget, QCheckBox):
                    widget.setChecked(bool(value))
                elif isinstance(widget, QSpinBox):
                    widget.setValue(int(value))
                elif isinstance(widget, QDoubleSpinBox):
                    widget.setValue(float(value))
            except Exception as exc:
                self.ros_node.get_logger().debug(f'Param {name} ignore: {exc}')

    def get_current_params(self):
        params = {}
        for name, widget in self.param_widgets.items():
            if isinstance(widget, QCheckBox):
                params[name] = widget.isChecked()
            elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                params[name] = widget.value()
        return params

    def save_to_yaml(self):
        params = self.get_current_params()
        yaml_content = {'traitement_polar_node': {'ros__parameters': params}}
        default_path = (
            self._find_ros2_root() / "src" / "traitement" / "config" / "traitement_polar_params.yaml"
        )
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Sauvegarder paramètres polaires", str(default_path), "YAML Files (*.yaml *.yml)",
        )
        if file_path:
            try:
                with open(file_path, 'w', encoding='utf-8') as f:
                    yaml.dump(yaml_content, f, default_flow_style=False, sort_keys=False)
                QMessageBox.information(self, "Sauvegarde réussie", f"Paramètres sauvegardés: {file_path}")
            except Exception as exc:
                QMessageBox.critical(self, "Erreur", f"Impossible de sauvegarder: {exc}")

    def reset_to_defaults(self):
        reply = QMessageBox.question(
            self, "Réinitialiser", "Réinitialiser aux valeurs par défaut ?",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            defaults = {
                'polar_enable_gaussian': True,
                'polar_gaussian_sigma': 1.0,
                'polar_enable_median': True,
                'polar_median_kernel': 3,
                'polar_enable_loss': False,
                'polar_loss_window_size': 5,
                'polar_enable_frost': False,
                'polar_frost_window_size': 5,
                'polar_frost_damping': 1.0,
                'polar_enable_log_compression': False,
                'polar_log_scale': 30.0,
            }
            for name, value in defaults.items():
                if name in self.param_widgets:
                    widget = self.param_widgets[name]
                    if isinstance(widget, QCheckBox):
                        widget.setChecked(value)
                    elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                        widget.setValue(value)

    def _find_ros2_root(self) -> Path:
        for parent in Path(__file__).resolve().parents:
            if parent.name == "ros2_bluerov":
                return parent
        return Path(__file__).resolve().parents[5]
