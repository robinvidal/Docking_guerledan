from pathlib import Path

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QScrollArea,
    QLabel,
    QGroupBox,
    QFormLayout,
    QSlider,
    QDoubleSpinBox,
    QCheckBox,
    QSpinBox,
    QHBoxLayout,
    QPushButton,
    QFileDialog,
    QMessageBox,
)
import yaml

from ..core.utils import load_yaml_params


class SonarMockControlWidget(QWidget):
    """Control panel for sonar_mock parameters."""

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.param_widgets = {}

        main_layout = QVBoxLayout(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)

        header_label = QLabel(
            "⚙️ <b>Contrôle Simulateur Sonar</b><br>"
            "<small>Ajustez les paramètres du sonar_mock en temps réel.</small>"
        )
        header_label.setWordWrap(True)
        scroll_layout.addWidget(header_label)

        noise_group = QGroupBox("🔊 Bruit de Fond")
        noise_layout = QFormLayout()
        noise_info = QLabel(
            "⚠️ Cage toujours à 200-255 (non affectée)\n"
            "• 3-5: Très propre  • 10-15: Modéré  • 20-30: Très bruité"
        )
        noise_info.setWordWrap(True)
        noise_layout.addRow(noise_info)

        noise_layout_h = QHBoxLayout()
        noise_slider = QSlider(Qt.Horizontal)
        noise_slider.setMinimum(0)
        noise_slider.setMaximum(100)
        noise_slider.setValue(20)
        noise_spin = QDoubleSpinBox()
        noise_spin.setMinimum(0.0)
        noise_spin.setMaximum(50.0)
        noise_spin.setValue(10.0)
        noise_spin.setSingleStep(0.5)
        noise_spin.setDecimals(1)
        noise_slider.valueChanged.connect(lambda v: noise_spin.setValue(v / 2.0))
        noise_spin.valueChanged.connect(lambda v: noise_slider.setValue(int(v * 2)))
        noise_spin.valueChanged.connect(lambda v: self.on_param_changed('noise_level', v))
        noise_layout_h.addWidget(noise_slider, 3)
        noise_layout_h.addWidget(noise_spin, 1)
        noise_layout.addRow('Niveau bruit:', noise_layout_h)
        self.param_widgets['noise_level'] = noise_spin
        noise_group.setLayout(noise_layout)
        scroll_layout.addWidget(noise_group)

        # Point-cloud noise parameters
        point_group = QGroupBox("🌌 Bruit par Points")
        point_layout = QFormLayout()

        npoints_spin = QSpinBox()
        npoints_spin.setMinimum(0)
        npoints_spin.setMaximum(100000)
        npoints_spin.setValue(200)
        npoints_spin.setSingleStep(50)
        npoints_spin.valueChanged.connect(lambda v: self.on_param_changed('noise_point_count', v))
        point_layout.addRow('Nombre de points:', npoints_spin)
        self.param_widgets['noise_point_count'] = npoints_spin

        i_min_spin = QDoubleSpinBox()
        i_min_spin.setMinimum(0.0)
        i_min_spin.setMaximum(255.0)
        i_min_spin.setValue(5.0)
        i_min_spin.setSingleStep(1.0)
        i_min_spin.valueChanged.connect(lambda v: self.on_param_changed('noise_i_min', v))
        point_layout.addRow('Intensité min:', i_min_spin)
        self.param_widgets['noise_i_min'] = i_min_spin

        i_max_spin = QDoubleSpinBox()
        i_max_spin.setMinimum(0.0)
        i_max_spin.setMaximum(255.0)
        i_max_spin.setValue(120.0)
        i_max_spin.setSingleStep(1.0)
        i_max_spin.valueChanged.connect(lambda v: self.on_param_changed('noise_i_max', v))
        point_layout.addRow('Intensité max:', i_max_spin)
        self.param_widgets['noise_i_max'] = i_max_spin

        blur_sigma_spin = QDoubleSpinBox()
        blur_sigma_spin.setMinimum(0.0)
        blur_sigma_spin.setMaximum(10.0)
        blur_sigma_spin.setValue(1.2)
        blur_sigma_spin.setSingleStep(0.1)
        blur_sigma_spin.setDecimals(2)
        blur_sigma_spin.valueChanged.connect(lambda v: self.on_param_changed('noise_blur_sigma', v))
        point_layout.addRow('Flou sigma:', blur_sigma_spin)
        self.param_widgets['noise_blur_sigma'] = blur_sigma_spin

        blur_iters_spin = QSpinBox()
        blur_iters_spin.setMinimum(1)
        blur_iters_spin.setMaximum(10)
        blur_iters_spin.setValue(1)
        blur_iters_spin.setSingleStep(1)
        blur_iters_spin.valueChanged.connect(lambda v: self.on_param_changed('noise_blur_iters', v))
        point_layout.addRow('Itérations flou:', blur_iters_spin)
        self.param_widgets['noise_blur_iters'] = blur_iters_spin

        point_group.setLayout(point_layout)
        scroll_layout.addWidget(point_group)

        # Geometry / portée
        geom_group = QGroupBox("📐 Géométrie / Portée")
        geom_layout = QFormLayout()

        bearing_spin = QDoubleSpinBox()
        bearing_spin.setMinimum(10.0)
        bearing_spin.setMaximum(360.0)
        bearing_spin.setValue(140.0)
        bearing_spin.setSingleStep(1.0)
        bearing_spin.setDecimals(1)
        bearing_spin.valueChanged.connect(lambda v: self.on_param_changed('bearing_angle', v))
        geom_layout.addRow("Ouverture (deg):", bearing_spin)
        self.param_widgets['bearing_angle'] = bearing_spin

        min_range_spin = QDoubleSpinBox()
        min_range_spin.setMinimum(0.1)
        min_range_spin.setMaximum(1000.0)
        min_range_spin.setValue(1.0)
        min_range_spin.setSingleStep(0.0)
        min_range_spin.setDecimals(2)
        min_range_spin.valueChanged.connect(lambda v: self.on_param_changed('min_range', v))
        geom_layout.addRow("Portée min (m):", min_range_spin)
        self.param_widgets['min_range'] = min_range_spin

        max_range_spin = QDoubleSpinBox()
        max_range_spin.setMinimum(0.5)
        max_range_spin.setMaximum(2000.0)
        max_range_spin.setValue(40.0)
        max_range_spin.setSingleStep(0.5)
        max_range_spin.setDecimals(2)
        max_range_spin.valueChanged.connect(lambda v: self.on_param_changed('max_range', v))
        geom_layout.addRow("Portée max (m):", max_range_spin)
        self.param_widgets['max_range'] = max_range_spin

        geom_group.setLayout(geom_layout)
        scroll_layout.addWidget(geom_group)

        filters_group = QGroupBox("🔍 Filtres Pré-appliqués")
        filters_layout = QFormLayout()

        filters_info = QLabel(
            "⚠️ Ces filtres s'appliquent AVANT publication sur /docking/sonar/raw\n"
            "Le contraste est ESSENTIEL pour voir le bruit après filtrage médian/gaussien"
        )
        filters_info.setWordWrap(True)
        filters_layout.addRow(filters_info)

        median_check = QCheckBox()
        median_check.setChecked(True)
        median_check.stateChanged.connect(lambda s: self.on_param_changed('enable_median', s == Qt.Checked))
        filters_layout.addRow('Filtre médian:', median_check)
        self.param_widgets['enable_median'] = median_check

        median_layout_h = QHBoxLayout()
        median_slider = QSlider(Qt.Horizontal)
        median_slider.setMinimum(3)
        median_slider.setMaximum(11)
        median_slider.setValue(3)
        median_slider.setSingleStep(2)
        median_spin = QSpinBox()
        median_spin.setMinimum(3)
        median_spin.setMaximum(11)
        median_spin.setValue(3)
        median_spin.setSingleStep(2)
        median_slider.valueChanged.connect(median_spin.setValue)
        median_spin.valueChanged.connect(median_slider.setValue)
        median_spin.valueChanged.connect(lambda v: self.on_param_changed('median_kernel', v))
        median_layout_h.addWidget(median_slider, 3)
        median_layout_h.addWidget(median_spin, 1)
        filters_layout.addRow('  Kernel:', median_layout_h)
        self.param_widgets['median_kernel'] = median_spin

        gaussian_check = QCheckBox()
        gaussian_check.setChecked(True)
        gaussian_check.stateChanged.connect(lambda s: self.on_param_changed('enable_gaussian', s == Qt.Checked))
        filters_layout.addRow('Filtre gaussien:', gaussian_check)
        self.param_widgets['enable_gaussian'] = gaussian_check

        gaussian_layout_h = QHBoxLayout()
        gaussian_slider = QSlider(Qt.Horizontal)
        gaussian_slider.setMinimum(1)
        gaussian_slider.setMaximum(50)
        gaussian_slider.setValue(24)
        gaussian_spin = QDoubleSpinBox()
        gaussian_spin.setMinimum(0.1)
        gaussian_spin.setMaximum(5.0)
        gaussian_spin.setValue(2.4)
        gaussian_spin.setSingleStep(0.1)
        gaussian_spin.setDecimals(1)
        gaussian_slider.valueChanged.connect(lambda v: gaussian_spin.setValue(v / 10.0))
        gaussian_spin.valueChanged.connect(lambda v: gaussian_slider.setValue(int(v * 10)))
        gaussian_spin.valueChanged.connect(lambda v: self.on_param_changed('gaussian_sigma', v))
        gaussian_layout_h.addWidget(gaussian_slider, 3)
        gaussian_layout_h.addWidget(gaussian_spin, 1)
        filters_layout.addRow('  Sigma:', gaussian_layout_h)
        self.param_widgets['gaussian_sigma'] = gaussian_spin

        contrast_check = QCheckBox()
        contrast_check.setChecked(True)
        contrast_check.stateChanged.connect(lambda s: self.on_param_changed('enable_contrast', s == Qt.Checked))
        filters_layout.addRow('Amélioration contraste:', contrast_check)
        self.param_widgets['enable_contrast'] = contrast_check

        contrast_layout_h = QHBoxLayout()
        contrast_slider = QSlider(Qt.Horizontal)
        contrast_slider.setMinimum(1)
        contrast_slider.setMaximum(100)
        contrast_slider.setValue(7)
        contrast_spin = QDoubleSpinBox()
        contrast_spin.setMinimum(0.00)
        contrast_spin.setMaximum(1.0)
        contrast_spin.setValue(0.1)
        contrast_spin.setSingleStep(0.002)
        contrast_spin.setDecimals(3)
        contrast_slider.valueChanged.connect(lambda v: contrast_spin.setValue(v / 20.0))
        contrast_spin.valueChanged.connect(lambda v: contrast_slider.setValue(int(v * 20)))
        contrast_spin.valueChanged.connect(lambda v: self.on_param_changed('contrast_clip', v))
        contrast_layout_h.addWidget(contrast_slider, 3)
        contrast_layout_h.addWidget(contrast_spin, 1)
        filters_layout.addRow('  Clip (%):', contrast_layout_h)
        self.param_widgets['contrast_clip'] = contrast_spin

        range_comp_check = QCheckBox()
        range_comp_check.setChecked(False)
        range_comp_check.stateChanged.connect(lambda s: self.on_param_changed('enable_range_comp', s == Qt.Checked))
        filters_layout.addRow('Compensation portée:', range_comp_check)
        self.param_widgets['enable_range_comp'] = range_comp_check

        range_comp_layout_h = QHBoxLayout()
        range_comp_slider = QSlider(Qt.Horizontal)
        range_comp_slider.setMinimum(0)
        range_comp_slider.setMaximum(100)
        range_comp_slider.setValue(10)
        range_comp_spin = QDoubleSpinBox()
        range_comp_spin.setMinimum(0.0)
        range_comp_spin.setMaximum(0.1)
        range_comp_spin.setValue(0.01)
        range_comp_spin.setSingleStep(0.001)
        range_comp_spin.setDecimals(3)
        range_comp_slider.valueChanged.connect(lambda v: range_comp_spin.setValue(v / 1000.0))
        range_comp_spin.valueChanged.connect(lambda v: range_comp_slider.setValue(int(v * 1000)))
        range_comp_spin.valueChanged.connect(lambda v: self.on_param_changed('range_comp_alpha', v))
        range_comp_layout_h.addWidget(range_comp_slider, 3)
        range_comp_layout_h.addWidget(range_comp_spin, 1)
        filters_layout.addRow('  Alpha (m⁻¹):', range_comp_layout_h)
        self.param_widgets['range_comp_alpha'] = range_comp_spin

        posts_group = QGroupBox("📍 Montants de la cage")
        posts_layout = QFormLayout()

        post_left_spin = QDoubleSpinBox()
        post_left_spin.setMinimum(0.0)
        post_left_spin.setMaximum(255.0)
        post_left_spin.setValue(230.0)
        post_left_spin.setSingleStep(5.0)
        post_left_spin.valueChanged.connect(lambda v: self.on_param_changed('post_intensity_left', v))
        posts_layout.addRow('Intensité montant gauche:', post_left_spin)
        self.param_widgets['post_intensity_left'] = post_left_spin

        post_right_spin = QDoubleSpinBox()
        post_right_spin.setMinimum(0.0)
        post_right_spin.setMaximum(255.0)
        post_right_spin.setValue(230.0)
        post_right_spin.setSingleStep(5.0)
        post_right_spin.valueChanged.connect(lambda v: self.on_param_changed('post_intensity_right', v))
        posts_layout.addRow('Intensité montant droit:', post_right_spin)
        self.param_widgets['post_intensity_right'] = post_right_spin

        posts_group.setLayout(posts_layout)
        scroll_layout.addWidget(posts_group)

        filters_group.setLayout(filters_layout)
        scroll_layout.addWidget(filters_group)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)

        # footer buttons fixed under parameters
        footer_layout = QHBoxLayout()
        footer_layout.addStretch()
        save_btn = QPushButton("💾 Sauvegarder")
        save_btn.clicked.connect(self.save_to_yaml)
        footer_layout.addWidget(save_btn)

        reset_btn = QPushButton("🔄 Réinitialiser")
        reset_btn.clicked.connect(self.reset_to_defaults)
        footer_layout.addWidget(reset_btn)
        main_layout.addLayout(footer_layout)

        self._apply_yaml_defaults()

    def on_param_changed(self, param_name, value):
        success = self.ros_node.set_sonar_mock_parameter(param_name, value)
        if success:
            self.ros_node.get_logger().info(f'Sonar: {param_name}={value}')

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
        yaml_content = {'sonar_mock': {'ros__parameters': params}}

        default_path = (
            self._find_ros2_root()
            / "src"
            / "sonar"
            / "config"
            / "sonar_params.yaml"
        )

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Sauvegarder paramètres sonar_mock",
            str(default_path),
            "YAML Files (*.yaml *.yml)",
        )

        if file_path:
            try:
                with open(file_path, 'w', encoding='utf-8') as handle:
                    yaml.dump(yaml_content, handle, default_flow_style=False, sort_keys=False)
                QMessageBox.information(self, "Sauvegarde réussie", f"Sauvegardé: {file_path}")
            except Exception as exc:  # noqa: BLE001
                QMessageBox.critical(self, "Erreur", f"Erreur: {str(exc)}")

    def reset_to_defaults(self):
        if QMessageBox.question(self, "Confirmer", "Réinitialiser?") == QMessageBox.Yes:
            self.param_widgets['noise_level'].setValue(10.0)
            self.param_widgets['noise_point_count'].setValue(200)
            self.param_widgets['noise_i_min'].setValue(5.0)
            self.param_widgets['noise_i_max'].setValue(120.0)
            self.param_widgets['noise_blur_sigma'].setValue(1.2)
            self.param_widgets['noise_blur_iters'].setValue(1)
            self.param_widgets['bearing_angle'].setValue(140.0)
            self.param_widgets['min_range'].setValue(1.0)
            self.param_widgets['max_range'].setValue(40.0)
            self.param_widgets['enable_median'].setChecked(True)
            self.param_widgets['median_kernel'].setValue(3)
            self.param_widgets['enable_gaussian'].setChecked(True)
            self.param_widgets['gaussian_sigma'].setValue(2.4)
            self.param_widgets['enable_contrast'].setChecked(True)
            self.param_widgets['contrast_clip'].setValue(0.35)
            self.param_widgets['enable_range_comp'].setChecked(False)
            self.param_widgets['range_comp_alpha'].setValue(0.01)
            self.param_widgets['post_intensity_left'].setValue(230.0)
            self.param_widgets['post_intensity_right'].setValue(230.0)

    def _find_ros2_root(self) -> Path:
        """Find ros2_bluerov root regardless of nesting depth."""
        for parent in Path(__file__).resolve().parents:
            if parent.name == "ros2_bluerov":
                return parent
        return Path(__file__).resolve().parents[5]

    def _apply_yaml_defaults(self):
        params = load_yaml_params('sonar', 'sonar_params.yaml', self.ros_node.get_logger())
        sub = params.get('sonar_mock', {}).get('ros__parameters', {}) if isinstance(params, dict) else {}
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
                self.ros_node.get_logger().debug(f'Param sonar {name} ignore (val={value}): {exc}')
