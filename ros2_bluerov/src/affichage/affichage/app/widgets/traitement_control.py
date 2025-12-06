from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QScrollArea,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QGroupBox,
    QFormLayout,
    QCheckBox,
    QSlider,
    QSpinBox,
    QDoubleSpinBox,
    QFileDialog,
    QMessageBox,
)
import yaml

from ..core.utils import load_yaml_params


class TraitementControlWidget(QWidget):
    """Control panel for traitement parameters with live updates."""

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.param_widgets = {}

        main_layout = QVBoxLayout(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)

        header_layout = QHBoxLayout()

        self.save_btn = QPushButton("💾 Sauvegarder dans YAML")
        self.save_btn.clicked.connect(self.save_to_yaml)
        header_layout.addWidget(self.save_btn)

        self.reset_btn = QPushButton("🔄 Réinitialiser")
        self.reset_btn.clicked.connect(self.reset_to_defaults)
        header_layout.addWidget(self.reset_btn)

        header_layout.addStretch()
        scroll_layout.addLayout(header_layout)

        tvg_group = self.create_group_box(
            "🎚️ Correction TVG",
            [
                ('enable_histogram_eq', 'Égalisation histogramme (CLAHE)', 'bool', True),
                ('enable_tvg_correction', 'Activer correction TVG', 'bool', False),
                ('tvg_alpha', 'Coefficient atténuation α (Np/m)', 'double', 0.0002, 0.0, 0.001, 0.00001),
                ('tvg_spreading_loss', 'Perte étalement (dB)', 'double', 20.0, 10.0, 40.0, 1.0),
            ],
        )
        scroll_layout.addWidget(tvg_group)

        filter_group = self.create_group_box(
            "🔍 Filtrage de base",
            [
                ('enable_median', 'Filtre médian', 'bool', True),
                ('median_kernel', 'Taille kernel médian', 'int', 3, 3, 11, 2),
                ('enable_gaussian', 'Filtre gaussien', 'bool', True),
                ('gaussian_sigma', 'Sigma gaussien', 'double', 1.0, 0.1, 5.0, 0.1),
            ],
        )
        scroll_layout.addWidget(filter_group)

        advanced_group = self.create_group_box(
            "🧪 Filtres avancés",
            [
                ('enable_bilateral', 'Filtre bilatéral', 'bool', False),
                ('bilateral_d', 'Diamètre voisinage (d)', 'int', 5, 1, 25, 1),
                ('bilateral_sigma_color', 'Sigma couleur', 'double', 25.0, 1.0, 150.0, 1.0),
                ('bilateral_sigma_space', 'Sigma espace', 'double', 5.0, 1.0, 50.0, 1.0),
                ('enable_tophat', 'Top-hat morphologique', 'bool', False),
                ('tophat_kernel', 'Taille noyau top-hat', 'int', 5, 1, 31, 2),
                ('enable_log_enhance', 'Renforcement LoG', 'bool', False),
                ('log_sigma', 'Sigma LoG', 'double', 1.0, 0.2, 10.0, 0.2),
                ('enable_dog_enhance', 'Renforcement DoG', 'bool', False),
                ('dog_sigma1', 'Sigma1 DoG', 'double', 1.0, 0.2, 10.0, 0.2),
                ('dog_sigma2', 'Sigma2 DoG', 'double', 2.0, 0.2, 10.0, 0.2),
                ('enable_matched_filter', 'Filtre adapté (PSF gaussien)', 'bool', False),
                ('mf_sigma', 'Sigma PSF', 'double', 1.2, 0.2, 10.0, 0.2),
                ('mf_kernel_size', 'Taille noyau PSF', 'int', 9, 3, 51, 2),
            ],
        )
        scroll_layout.addWidget(advanced_group)

        cfar_group = self.create_group_box(
            "🎯 SO-CFAR",
            [
                ('enable_so_cfar', 'Activer SO-CFAR', 'bool', True),
                ('cfar_guard_cells', 'Cellules de garde', 'int', 5, 1, 20, 1),
                ('cfar_window_size', 'Taille fenêtre référence', 'int', 10, 5, 30, 1),
                ('cfar_alpha', 'Facteur seuil α', 'double', 10.0, 1.0, 20.0, 0.5),
            ],
        )
        scroll_layout.addWidget(cfar_group)

        adt_group = self.create_group_box(
            "✂️ Seuillage ADT",
            [
                ('enable_adaptive_threshold', 'Activer ADT', 'bool', False),
                ('adt_block_size', 'Taille bloc', 'int', 15, 5, 51, 2),
                ('adt_c', 'Constante C', 'int', 2, 0, 10, 1),
            ],
        )
        scroll_layout.addWidget(adt_group)

        info_label = QLabel(
            "ℹ️ Les modifications sont appliquées en temps réel au nœud traitement_node.\n"
            "Utilisez 'Sauvegarder dans YAML' pour rendre les changements permanents."
        )
        info_label.setWordWrap(True)
        scroll_layout.addWidget(info_label)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)

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
        success = self.ros_node.set_traitement_parameter(param_name, value)
        if success:
            self.ros_node.get_logger().info(f'Paramètre {param_name} = {value}')

    def _apply_yaml_defaults(self):
        params = load_yaml_params('traitement', 'traitement_params.yaml', self.ros_node.get_logger())
        sub = params.get('traitement_node', {}).get('ros__parameters', {}) if isinstance(params, dict) else {}
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
            except Exception as exc:  # noqa: BLE001
                self.ros_node.get_logger().debug(f'Param {name} ignore (val={value}): {exc}')

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
        yaml_content = {'traitement_node': {'ros__parameters': params}}

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Sauvegarder paramètres de traitement",
            "traitement_params.yaml",
            "YAML Files (*.yaml *.yml)",
        )

        if file_path:
            try:
                with open(file_path, 'w', encoding='utf-8') as handle:
                    yaml.dump(yaml_content, handle, default_flow_style=False, sort_keys=False)

                QMessageBox.information(
                    self,
                    "Sauvegarde réussie",
                    f"Paramètres sauvegardés dans:\n{file_path}\n\n"
                    "Pour les utiliser au démarrage:\n"
                    "ros2 run traitement traitement_node --ros-args "
                    f"--params-file {file_path}",
                )
                self.ros_node.get_logger().info(f'Paramètres sauvegardés: {file_path}')
            except Exception as exc:  # noqa: BLE001
                QMessageBox.critical(self, "Erreur de sauvegarde", f"Impossible de sauvegarder:\n{str(exc)}")
                self.ros_node.get_logger().error(f'Erreur sauvegarde: {exc}')

    def reset_to_defaults(self):
        reply = QMessageBox.question(
            self,
            "Confirmer réinitialisation",
            "Voulez-vous vraiment réinitialiser tous les paramètres aux valeurs par défaut ?",
            QMessageBox.Yes | QMessageBox.No,
        )

        if reply == QMessageBox.Yes:
            defaults = {
                'enable_histogram_eq': True,
                'enable_tvg_correction': False,
                'tvg_alpha': 0.0002,
                'tvg_spreading_loss': 20.0,
                'enable_median': True,
                'median_kernel': 3,
                'enable_gaussian': True,
                'gaussian_sigma': 1.0,
                'enable_bilateral': False,
                'bilateral_d': 5,
                'bilateral_sigma_color': 25.0,
                'bilateral_sigma_space': 5.0,
                'enable_tophat': False,
                'tophat_kernel': 5,
                'enable_log_enhance': False,
                'log_sigma': 1.0,
                'enable_dog_enhance': False,
                'dog_sigma1': 1.0,
                'dog_sigma2': 2.0,
                'enable_matched_filter': False,
                'mf_sigma': 1.2,
                'mf_kernel_size': 9,
                'enable_so_cfar': True,
                'cfar_guard_cells': 5,
                'cfar_window_size': 10,
                'cfar_alpha': 10.0,
                'enable_adaptive_threshold': False,
                'adt_block_size': 15,
                'adt_c': 2,
            }

            for name, value in defaults.items():
                if name in self.param_widgets:
                    widget = self.param_widgets[name]
                    if isinstance(widget, QCheckBox):
                        widget.setChecked(value)
                    elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                        widget.setValue(value)
