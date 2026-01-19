from pathlib import Path

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QScrollArea,
    QLabel,
    QGroupBox,
    QFormLayout,
    QCheckBox,
    QSpinBox,
    QDoubleSpinBox,
    QHBoxLayout,
    QSlider,
    QPushButton,
    QFileDialog,
    QMessageBox,
)
import yaml

from ..core.utils import load_yaml_params


class TrackerControlWidget(QWidget):
    """Control panel for blob tracker parameters."""
    
    # Signal émis pour activer/désactiver le mode sélection de bbox
    bbox_selection_requested = pyqtSignal(bool)  # True = activer, False = désactiver

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.param_widgets = {}

        main_layout = QVBoxLayout(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)

        header = QLabel("🎛️ <b>Contrôles Tracker</b><br><small>Réglez les paramètres du tracker en temps réel.</small>")
        header.setWordWrap(True)
        scroll_layout.addWidget(header)
        
        # Bouton de sélection de cage (CSRT)
        selection_layout = QHBoxLayout()
        self.select_bbox_btn = QPushButton("📦 Sélectionner Cage (CSRT)")
        self.select_bbox_btn.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71;
                color: white;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #27ae60;
            }
            QPushButton:pressed {
                background-color: #229954;
            }
            QPushButton:checked {
                background-color: #e74c3c;
            }
        """)
        self.select_bbox_btn.setCheckable(True)
        self.select_bbox_btn.clicked.connect(self.on_select_bbox_clicked)
        selection_layout.addWidget(self.select_bbox_btn)
        scroll_layout.addLayout(selection_layout)
        
        help_label = QLabel(
            "<small><b>Mode sélection:</b> Cliquez sur le bouton, puis dessinez un rectangle "
            "avec la souris sur l'image cartésienne. Le tracker démarre automatiquement.</small>"
        )
        help_label.setWordWrap(True)
        help_label.setStyleSheet("color: #95a5a6; padding: 5px;")
        scroll_layout.addWidget(help_label)

        general = QGroupBox("⚙️ Général")
        gform = QFormLayout()
        enable_cb = QCheckBox()
        enable_cb.setChecked(True)
        enable_cb.stateChanged.connect(lambda s: self.on_param_changed('enable_tracker', s == Qt.Checked))
        gform.addRow('Activer tracker:', enable_cb)
        self.param_widgets['enable_tracker'] = enable_cb

        template_spin = QSpinBox()
        template_spin.setMinimum(3)
        template_spin.setMaximum(101)
        template_spin.setValue(21)
        template_spin.setSingleStep(2)
        template_spin.valueChanged.connect(lambda v: self.on_param_changed('template_size', v))
        gform.addRow('Taille template (px):', template_spin)
        self.param_widgets['template_size'] = template_spin

        search_spin = QSpinBox()
        search_spin.setMinimum(1)
        search_spin.setMaximum(200)
        search_spin.setValue(24)
        search_spin.valueChanged.connect(lambda v: self.on_param_changed('search_radius', v))
        gform.addRow('Rayon recherche (px):', search_spin)
        self.param_widgets['search_radius'] = search_spin

        init_mode_check = QCheckBox()
        init_mode_check.setChecked(False)
        init_mode_check.stateChanged.connect(lambda s: self.on_param_changed('init_mode', 'center' if s == Qt.Checked else 'max'))
        gform.addRow("Init centre (sinon 'max'):", init_mode_check)

        intensity_spin = QSpinBox()
        intensity_spin.setMinimum(0)
        intensity_spin.setMaximum(255)
        intensity_spin.setValue(120)
        intensity_spin.valueChanged.connect(lambda v: self.on_param_changed('intensity_threshold', v))
        gform.addRow('Seuil intensité init:', intensity_spin)
        self.param_widgets['intensity_threshold'] = intensity_spin

        general.setLayout(gform)
        scroll_layout.addWidget(general)

        geom = QGroupBox('📐 Géométrie')
        g2 = QFormLayout()
        dist_spin = QDoubleSpinBox()
        dist_spin.setMinimum(0.0)
        dist_spin.setMaximum(100.0)
        dist_spin.setDecimals(3)
        dist_spin.setValue(2.0)
        dist_spin.valueChanged.connect(lambda v: self.on_param_changed('distance_expected', v))
        g2.addRow('Distance attendue (m):', dist_spin)
        self.param_widgets['distance_expected'] = dist_spin

        eps_spin = QDoubleSpinBox()
        eps_spin.setMinimum(0.0)
        eps_spin.setMaximum(10.0)
        eps_spin.setDecimals(3)
        eps_spin.setValue(0.3)
        eps_spin.valueChanged.connect(lambda v: self.on_param_changed('distance_epsilon', v))
        g2.addRow('Epsilon anneau (m):', eps_spin)
        self.param_widgets['distance_epsilon'] = eps_spin

        sigma_spin = QDoubleSpinBox()
        sigma_spin.setMinimum(0.0)
        sigma_spin.setMaximum(10.0)
        sigma_spin.setDecimals(3)
        sigma_spin.setValue(0.2)
        sigma_spin.valueChanged.connect(lambda v: self.on_param_changed('distance_sigma', v))
        g2.addRow('Sigma distance (m):', sigma_spin)
        self.param_widgets['distance_sigma'] = sigma_spin

        geom.setLayout(g2)
        scroll_layout.addWidget(geom)

        scoring = QGroupBox('🏷️ Scoring')
        sform = QFormLayout()
        ncc_spin = QDoubleSpinBox()
        ncc_spin.setMinimum(0.0)
        ncc_spin.setMaximum(1.0)
        ncc_spin.setDecimals(3)
        ncc_spin.setSingleStep(0.01)
        ncc_spin.setValue(0.5)
        ncc_spin.valueChanged.connect(lambda v: self.on_param_changed('ncc_threshold', v))
        sform.addRow('NCC threshold:', ncc_spin)
        self.param_widgets['ncc_threshold'] = ncc_spin

        w1 = QDoubleSpinBox(); w1.setDecimals(2); w1.setRange(0.0,1.0); w1.setValue(0.6)
        w1.valueChanged.connect(lambda v: self.on_param_changed('weight_ncc', v))
        sform.addRow('Poids NCC (w1):', w1); self.param_widgets['weight_ncc'] = w1

        w2 = QDoubleSpinBox(); w2.setDecimals(2); w2.setRange(0.0,1.0); w2.setValue(0.25)
        w2.valueChanged.connect(lambda v: self.on_param_changed('weight_stability', v))
        sform.addRow('Poids stabilité (w2):', w2); self.param_widgets['weight_stability'] = w2

        w3 = QDoubleSpinBox(); w3.setDecimals(2); w3.setRange(0.0,1.0); w3.setValue(0.15)
        w3.valueChanged.connect(lambda v: self.on_param_changed('weight_distance', v))
        sform.addRow('Poids distance (w3):', w3); self.param_widgets['weight_distance'] = w3

        min_tot = QDoubleSpinBox(); min_tot.setDecimals(3); min_tot.setRange(0.0,1.0); min_tot.setValue(0.4)
        min_tot.valueChanged.connect(lambda v: self.on_param_changed('min_total_score', v))
        sform.addRow('Score min total:', min_tot); self.param_widgets['min_total_score'] = min_tot

        scoring.setLayout(sform)
        scroll_layout.addWidget(scoring)

        recovery = QGroupBox('🔁 Recovery & Stabilité')
        rform = QFormLayout()
        stab_win = QSpinBox(); stab_win.setRange(1, 50); stab_win.setValue(5)
        stab_win.valueChanged.connect(lambda v: self.on_param_changed('stability_window', v))
        rform.addRow('Fenêtre stabilité (frames):', stab_win); self.param_widgets['stability_window'] = stab_win

        stab_thr = QDoubleSpinBox(); stab_thr.setDecimals(3); stab_thr.setRange(0.0,1.0); stab_thr.setValue(0.4)
        stab_thr.valueChanged.connect(lambda v: self.on_param_changed('stability_threshold', v))
        rform.addRow('Seuil stabilité:', stab_thr); self.param_widgets['stability_threshold'] = stab_thr

        max_jump = QDoubleSpinBox(); max_jump.setDecimals(3); max_jump.setRange(0.0, 10.0); max_jump.setValue(1.0)
        max_jump.valueChanged.connect(lambda v: self.on_param_changed('max_jump_m', v))
        rform.addRow('Max jump (m):', max_jump); self.param_widgets['max_jump_m'] = max_jump

        topk = QSpinBox(); topk.setRange(2, 200); topk.setValue(20)
        topk.valueChanged.connect(lambda v: self.on_param_changed('top_k_candidates', v))
        rform.addRow('Top K candidates:', topk); self.param_widgets['top_k_candidates'] = topk

        rec_cb = QCheckBox(); rec_cb.setChecked(True)
        rec_cb.stateChanged.connect(lambda s: self.on_param_changed('recovery_enabled', s == Qt.Checked))
        rform.addRow('Recovery activé:', rec_cb); self.param_widgets['recovery_enabled'] = rec_cb

        recovery.setLayout(rform)
        scroll_layout.addWidget(recovery)

        # Footer
        footer = QHBoxLayout()
        footer.addStretch()
        save = QPushButton('💾 Sauvegarder')
        save.clicked.connect(self.save_to_yaml)
        footer.addWidget(save)
        scroll_layout.addLayout(footer)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)

        self._apply_yaml_defaults()
    
    def on_select_bbox_clicked(self, checked):
        """Gère le clic sur le bouton de sélection de cage."""
        if checked:
            self.select_bbox_btn.setText("❌ Annuler Sélection")
        else:
            self.select_bbox_btn.setText("📦 Sélectionner Cage (CSRT)")
        
        # Émettre le signal pour activer/désactiver le mode sélection
        self.bbox_selection_requested.emit(checked)

    def on_param_changed(self, name, value):
        success = self.ros_node.set_tracking_parameter(name, value)
        if success:
            self.ros_node.get_logger().info(f'Tracker: {name}={value}')

    def _apply_yaml_defaults(self):
        params = load_yaml_params('tracking', 'tracking_params.yaml', self.ros_node.get_logger())
        sub = params.get('blob_tracker_node', {}).get('ros__parameters', {}) if isinstance(params, dict) else {}
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
            except Exception:
                self.ros_node.get_logger().debug(f'Param tracker {name} ignore (val={value})')

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
        yaml_content = {'blob_tracker_node': {'ros__parameters': params}}

        default_path = (
            self._find_ros2_root()
            / 'src'
            / 'tracking'
            / 'config'
            / 'tracking_params.yaml'
        )

        file_path, _ = QFileDialog.getSaveFileName(self, 'Sauvegarder paramètres tracker', str(default_path), 'YAML Files (*.yaml *.yml)')
        if file_path:
            try:
                with open(file_path, 'w', encoding='utf-8') as f:
                    yaml.dump(yaml_content, f, default_flow_style=False, sort_keys=False)
                QMessageBox.information(self, 'Sauvegarde réussie', f'Sauvegardé: {file_path}')
            except Exception as exc:
                QMessageBox.critical(self, 'Erreur', f'Erreur: {exc}')

    def _find_ros2_root(self) -> Path:
        for parent in Path(__file__).resolve().parents:
            if parent.name == 'ros2_bluerov':
                return parent
        return Path(__file__).resolve().parents[5]
