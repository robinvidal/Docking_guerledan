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
    """Control panel for tracker with unified traitement parameters."""
    
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

        header = QLabel("🎛️ <b>Contrôles Tracker & Traitement Unified</b><br><small>Réglez les paramètres de traitement en temps réel.</small>")
        header.setWordWrap(True)
        scroll_layout.addWidget(header)
        
        # ==================== BOUTONS TRACKER ====================
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
        
        # Bouton Auto Tracking (détection automatique de cage)
        self.auto_tracking_btn = QPushButton("🔍 Auto Tracking")
        self.auto_tracking_btn.setStyleSheet("""
            QPushButton {
                background-color: #9b59b6;
                color: white;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #8e44ad;
            }
            QPushButton:pressed {
                background-color: #7d3c98;
            }
            QPushButton:checked {
                background-color: #e74c3c;
            }
        """)
        self.auto_tracking_btn.setCheckable(True)
        self.auto_tracking_btn.clicked.connect(self.on_auto_tracking_clicked)
        selection_layout.addWidget(self.auto_tracking_btn)
        
        scroll_layout.addLayout(selection_layout)
        
        help_label = QLabel(
            "<small><b>Mode sélection:</b> Cliquez sur le bouton, puis dessinez un rectangle "
            "avec la souris sur l'image cartésienne. Le tracker démarre automatiquement.<br>"
            "<b>Auto Tracking:</b> Détection automatique de la cage en U via Hough Lines. "
            "Une fois détectée, le tracker CSRT prend le relai. "
            "Si le tracker perd la cage, la détection reprend automatiquement.</small>"
        )
        help_label.setWordWrap(True)
        help_label.setStyleSheet("color: #95a5a6; padding: 5px;")
        scroll_layout.addWidget(help_label)

        # ==================== FILTRES POLAIRES ====================
        polar_median_group = self._create_group_box(
            "📊 Filtre Médian (Polaire)",
            [
                ('polar_enable_median', 'Activer Médian', 'bool', True),
                ('polar_median_kernel', 'Taille kernel', 'int', 3, 3, 15, 2),
            ],
        )
        scroll_layout.addWidget(polar_median_group)

        polar_frost_group = self._create_group_box(
            "❄️ Filtre Frost (Polaire)",
            [
                ('polar_enable_frost', 'Activer Frost', 'bool', False),
                ('polar_frost_window_size', 'Taille fenêtre', 'int', 3, 3, 15, 2),
                ('polar_frost_damping', 'Damping factor', 'double', 3.4, 0.1, 10.0, 0.1),
            ],
        )
        scroll_layout.addWidget(polar_frost_group)

        # ==================== PARAMÈTRES CARTÉSIENS ====================
        cartesian_general_group = self._create_group_box(
            "🔄 Paramètres Cartésiens",
            [
                ('cartesian_scale_factor', 'Facteur échelle', 'double', 2.0, 0.5, 4.0, 0.1),
                ('enable_spatial_filter', 'Filtre spatial', 'bool', False),
                ('spatial_filter_radius', 'Rayon filtre spatial (m)', 'double', 2.0, 0.1, 10.0, 0.1),
                ('spatial_filter_sigma', 'Sigma filtre spatial', 'double', 0.8, 0.1, 5.0, 0.1),
            ],
        )
        scroll_layout.addWidget(cartesian_general_group)

        # ==================== FILTRES CARTÉSIENS ====================
        cart_median_group = self._create_group_box(
            "🧹 Médian Cartésien (Denoising)",
            [
                ('cart_enable_median', 'Activer Médian', 'bool', False),
                ('cart_median_kernel_size', 'Taille kernel', 'int', 3, 3, 15, 2),
            ],
        )
        scroll_layout.addWidget(cart_median_group)

        cart_clahe_group = self._create_group_box(
            "📈 CLAHE (Contraste Adaptatif)",
            [
                ('cart_enable_clahe', 'Activer CLAHE', 'bool', False),
                ('cart_clahe_clip_limit', 'Clip limit', 'double', 2.0, 1.0, 10.0, 0.5),
                ('cart_clahe_tile_grid_size', 'Taille grille', 'int', 8, 2, 32, 1),
            ],
        )
        scroll_layout.addWidget(cart_clahe_group)

        cart_threshold_group = self._create_group_box(
            "🎚️ Seuil Bas (Threshold)",
            [
                ('cart_enable_threshold', 'Activer seuil', 'bool', False),
                ('cart_min_intensity_threshold', 'Seuil intensité min', 'int', 0, 0, 255, 5),
            ],
        )
        scroll_layout.addWidget(cart_threshold_group)

        cart_morph_group = self._create_group_box(
            "🔲 Morphologie (Closing)",
            [
                ('cart_enable_morphology', 'Activer morphologie', 'bool', False),
                ('cart_morph_kernel_size', 'Taille kernel', 'int', 3, 3, 15, 2),
                ('cart_morph_iterations', 'Itérations', 'int', 1, 1, 10, 1),
            ],
        )
        scroll_layout.addWidget(cart_morph_group)

        cart_flip_group = self._create_group_box(
            "🔄 Flip/Miroir",
            [
                ('cart_flip_horizontal', 'Flip horizontal', 'bool', False),
                ('cart_flip_vertical', 'Flip vertical', 'bool', False),
            ],
        )
        scroll_layout.addWidget(cart_flip_group)

        cart_percentile_group = self._create_group_box(
            "✂️ Binarisation Percentile",
            [
                ('cart_enable_percentile_binarize', 'Activer binarisation', 'bool', False),
                ('cart_percentile_keep_percent', 'Garder X% intenses', 'double', 10.0, 0.1, 100.0, 0.5),
            ],
        )
        scroll_layout.addWidget(cart_percentile_group)

        cart_opening_closing_group = self._create_group_box(
            "🔳 Opening-Closing (Nettoyage)",
            [
                ('cart_enable_opening_closing', 'Activer Opening-Closing', 'bool', False),
                ('cart_opening_kernel_size', 'Kernel Opening', 'int', 3, 3, 15, 2),
                ('cart_closing_kernel_size', 'Kernel Closing', 'int', 3, 3, 15, 2),
                ('cart_opening_iterations', 'Itérations Opening', 'int', 1, 0, 10, 1),
                ('cart_closing_iterations', 'Itérations Closing', 'int', 1, 0, 10, 1),
            ],
        )
        scroll_layout.addWidget(cart_opening_closing_group)

        # Info
        info_label = QLabel(
            "ℹ️ Les modifications sont appliquées en temps réel au nœud traitement_unified_node.\n"
            "Utilisez 'Sauvegarder' pour rendre les changements permanents."
        )
        info_label.setWordWrap(True)
        scroll_layout.addWidget(info_label)

        # Footer
        footer = QHBoxLayout()
        footer.addStretch()
        
        reset_btn = QPushButton('🔄 Réinitialiser')
        reset_btn.clicked.connect(self.reset_to_defaults)
        footer.addWidget(reset_btn)
        
        save_btn = QPushButton('💾 Sauvegarder')
        save_btn.clicked.connect(self.save_to_yaml)
        footer.addWidget(save_btn)
        
        scroll_layout.addLayout(footer)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)

        self._apply_yaml_defaults()

    def _create_group_box(self, title, params):
        """Crée un groupe de paramètres avec widgets appropriés."""
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
                spinbox.setDecimals(len(str(step).split('.')[-1]) if '.' in str(step) else 1)

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
    
    def on_select_bbox_clicked(self, checked):
        """Gère le clic sur le bouton de sélection de cage."""
        if checked:
            self.select_bbox_btn.setText("❌ Annuler Sélection")
            # Désactiver l'autre bouton
            self.auto_tracking_btn.setChecked(False)
            self.auto_tracking_btn.setText("🔍 Auto Tracking")
            # Arrêter l'auto-tracking si actif
            self.ros_node.publish_auto_detect_trigger(False)
        else:
            self.select_bbox_btn.setText("📦 Sélectionner Cage (CSRT)")
        
        # Émettre le signal pour activer/désactiver le mode sélection
        self.bbox_selection_requested.emit(checked)
    
    def on_auto_tracking_clicked(self, checked):
        """Gère le clic sur le bouton Auto Tracking."""
        if checked:
            self.auto_tracking_btn.setText("⏹️ Arrêter Auto Tracking")
            # Désactiver l'autre bouton
            self.select_bbox_btn.setChecked(False)
            self.select_bbox_btn.setText("📦 Sélectionner Cage (CSRT)")
            self.bbox_selection_requested.emit(False)
        else:
            self.auto_tracking_btn.setText("🔍 Auto Tracking")
        
        # Publier le trigger pour activer/désactiver l'auto-detect
        self.ros_node.publish_auto_detect_trigger(checked)
    
    def on_auto_detect_status_changed(self, is_searching: bool):
        """Callback quand le statut auto-detect change (cage trouvée ou perdue)."""
        if not is_searching and self.auto_tracking_btn.isChecked():
            # La cage a été trouvée, le tracking CSRT a pris le relai
            self.auto_tracking_btn.setText("✅ Tracking Actif (Auto)")
        elif is_searching and self.auto_tracking_btn.isChecked():
            # La recherche est en cours (relance après perte)
            self.auto_tracking_btn.setText("🔍 Recherche en cours...")

    def on_param_changed(self, name, value):
        """Envoie le paramètre modifié au nœud traitement_unified_node."""
        success = self.ros_node.set_traitement_unified_parameter(name, value)
        if success:
            self.ros_node.get_logger().info(f'Traitement Unified: {name}={value}')

    def _apply_yaml_defaults(self):
        """Charge les valeurs par défaut depuis le fichier YAML."""
        params = load_yaml_params('traitement', 'traitement_unified_params.yaml', self.ros_node.get_logger())
        sub = params.get('traitement_unified_node', {}).get('ros__parameters', {}) if isinstance(params, dict) else {}
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
                self.ros_node.get_logger().debug(f'Param unified {name} ignore (val={value})')

    def get_current_params(self):
        """Récupère tous les paramètres actuels."""
        params = {}
        for name, widget in self.param_widgets.items():
            if isinstance(widget, QCheckBox):
                params[name] = widget.isChecked()
            elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                params[name] = widget.value()
        return params

    def reset_to_defaults(self):
        """Réinitialise tous les paramètres aux valeurs par défaut."""
        reply = QMessageBox.question(
            self,
            "Confirmer réinitialisation",
            "Voulez-vous vraiment réinitialiser tous les paramètres aux valeurs par défaut ?",
            QMessageBox.Yes | QMessageBox.No,
        )

        if reply == QMessageBox.Yes:
            defaults = {
                # Filtres polaires
                'polar_enable_median': True,
                'polar_median_kernel': 3,
                'polar_enable_frost': False,
                'polar_frost_window_size': 3,
                'polar_frost_damping': 3.4,
                # Paramètres cartésiens
                'cartesian_scale_factor': 2.0,
                'enable_spatial_filter': False,
                'spatial_filter_radius': 2.0,
                'spatial_filter_sigma': 0.8,
                # Filtres cartésiens
                'cart_enable_median': False,
                'cart_median_kernel_size': 3,
                'cart_enable_clahe': False,
                'cart_clahe_clip_limit': 2.0,
                'cart_clahe_tile_grid_size': 8,
                'cart_enable_threshold': False,
                'cart_min_intensity_threshold': 0,
                'cart_enable_morphology': False,
                'cart_morph_kernel_size': 3,
                'cart_morph_iterations': 1,
                'cart_flip_horizontal': False,
                'cart_flip_vertical': False,
                'cart_enable_percentile_binarize': False,
                'cart_percentile_keep_percent': 10.0,
                'cart_enable_opening_closing': False,
                'cart_opening_kernel_size': 3,
                'cart_closing_kernel_size': 3,
                'cart_opening_iterations': 1,
                'cart_closing_iterations': 1,
            }

            for name, value in defaults.items():
                if name in self.param_widgets:
                    widget = self.param_widgets[name]
                    if isinstance(widget, QCheckBox):
                        widget.setChecked(value)
                    elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                        widget.setValue(value)

    def save_to_yaml(self):
        """Sauvegarde les paramètres actuels dans un fichier YAML."""
        params = self.get_current_params()
        yaml_content = {'traitement_unified_node': {'ros__parameters': params}}

        default_path = (
            self._find_ros2_root()
            / 'src'
            / 'traitement'
            / 'config'
            / 'traitement_unified_params.yaml'
        )

        file_path, _ = QFileDialog.getSaveFileName(
            self, 
            'Sauvegarder paramètres Traitement Unified', 
            str(default_path), 
            'YAML Files (*.yaml *.yml)'
        )
        if file_path:
            try:
                with open(file_path, 'w', encoding='utf-8') as f:
                    yaml.dump(yaml_content, f, default_flow_style=False, sort_keys=False)
                QMessageBox.information(
                    self, 
                    'Sauvegarde réussie', 
                    f'Paramètres sauvegardés dans:\n{file_path}\n\n'
                    'Pour les utiliser au démarrage:\n'
                    f'ros2 run traitement traitement_unified_node --ros-args --params-file {file_path}'
                )
            except Exception as exc:
                QMessageBox.critical(self, 'Erreur', f'Erreur: {exc}')

    def _find_ros2_root(self) -> Path:
        """Trouve le répertoire racine ros2_bluerov."""
        for parent in Path(__file__).resolve().parents:
            if parent.name == 'ros2_bluerov':
                return parent
        return Path(__file__).resolve().parents[5]
