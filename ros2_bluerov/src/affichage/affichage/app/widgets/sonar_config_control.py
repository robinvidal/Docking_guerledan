"""Widget de contrôle pour la configuration dynamique du sonar Oculus."""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QScrollArea,
    QLabel,
    QGroupBox,
    QFormLayout,
    QHBoxLayout,
    QSpinBox,
    QDoubleSpinBox,
    QComboBox,
    QPushButton,
    QFrame,
)


class SonarConfigControlWidget(QWidget):
    """Panneau de contrôle pour la configuration du sonar en temps réel."""

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        
        # Valeurs actuelles (seront mises à jour par les réponses du service)
        self.current_range = 15.0
        self.current_gain = 0
        self.current_mode = 1
        self.current_ping_rate = 0
        
        main_layout = QVBoxLayout(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)

        # Header
        header = QLabel(
            "🔊 <b>Configuration Sonar</b><br>"
            "<small>Modifiez les paramètres du sonar Oculus M750d en temps réel.</small>"
        )
        header.setWordWrap(True)
        scroll_layout.addWidget(header)

        # Status
        self.status_label = QLabel("⏳ En attente de connexion au sonar...")
        self.status_label.setStyleSheet("""
            QLabel {
                background-color: #34495e;
                color: #ecf0f1;
                padding: 8px;
                border-radius: 5px;
                font-weight: bold;
            }
        """)
        scroll_layout.addWidget(self.status_label)

        # ==================== Groupe Portée ====================
        range_group = QGroupBox("📏 Portée (Range)")
        range_layout = QFormLayout()
        
        # Portée en mètres
        range_h_layout = QHBoxLayout()
        self.range_spin = QDoubleSpinBox()
        self.range_spin.setMinimum(1.0)
        self.range_spin.setMaximum(120.0)
        self.range_spin.setValue(15.0)
        self.range_spin.setSingleStep(1.0)
        self.range_spin.setSuffix(" m")
        self.range_spin.setStyleSheet("font-size: 14px; padding: 5px;")
        range_h_layout.addWidget(self.range_spin)
        
        # Boutons de raccourci pour la portée
        for val in [5, 10, 15, 20, 30]:
            btn = QPushButton(f"{val}m")
            btn.setMaximumWidth(50)
            btn.clicked.connect(lambda checked, v=val: self.set_range_quick(v))
            range_h_layout.addWidget(btn)
        
        range_layout.addRow("Portée:", range_h_layout)
        range_group.setLayout(range_layout)
        scroll_layout.addWidget(range_group)

        # ==================== Groupe Gain ====================
        gain_group = QGroupBox("🔆 Gain")
        gain_layout = QFormLayout()
        
        gain_h_layout = QHBoxLayout()
        self.gain_spin = QSpinBox()
        self.gain_spin.setMinimum(0)
        self.gain_spin.setMaximum(100)
        self.gain_spin.setValue(0)
        self.gain_spin.setSuffix(" %")
        self.gain_spin.setStyleSheet("font-size: 14px; padding: 5px;")
        gain_h_layout.addWidget(self.gain_spin)
        
        # Boutons de raccourci pour le gain
        for val in [0, 25, 50, 75, 100]:
            btn = QPushButton(f"{val}%")
            btn.setMaximumWidth(50)
            btn.clicked.connect(lambda checked, v=val: self.set_gain_quick(v))
            gain_h_layout.addWidget(btn)
        
        gain_layout.addRow("Gain:", gain_h_layout)
        gain_group.setLayout(gain_layout)
        scroll_layout.addWidget(gain_group)

        # ==================== Groupe Mode ====================
        mode_group = QGroupBox("📡 Mode Fréquence")
        mode_layout = QFormLayout()
        
        mode_h_layout = QHBoxLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("🔵 Basse Fréquence (LF)", 1)
        self.mode_combo.addItem("🔴 Haute Fréquence (HF)", 2)
        self.mode_combo.setStyleSheet("""
            QComboBox {
                font-size: 14px;
                padding: 5px;
                color: white;
                background-color: #34495e;
            }
            QComboBox QAbstractItemView {
                color: white;
                background-color: #2c3e50;
                selection-color: white;
                selection-background-color: #2c3e50;
            }
            QComboBox QAbstractItemView::item {
                color: white;
            }
            QComboBox QAbstractItemView::item:hover {
                color: white;
                background-color: #2c3e50;
            }
            QComboBox QAbstractItemView::item:selected {
                color: white;
                background-color: #2c3e50;
            }
        """)
        mode_h_layout.addWidget(self.mode_combo)
        
        self.lf_btn = QPushButton("LF")
        self.lf_btn.setStyleSheet("""
            QPushButton {
                background-color: #3498db;
                color: white;
                font-weight: bold;
                padding: 8px 15px;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #2980b9; }
        """)
        self.lf_btn.clicked.connect(lambda: self.set_mode_quick(1))
        mode_h_layout.addWidget(self.lf_btn)
        
        self.hf_btn = QPushButton("HF")
        self.hf_btn.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                font-weight: bold;
                padding: 8px 15px;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #c0392b; }
        """)
        self.hf_btn.clicked.connect(lambda: self.set_mode_quick(2))
        mode_h_layout.addWidget(self.hf_btn)
        
        mode_layout.addRow("Mode:", mode_h_layout)
        
        # Info sur les modes
        mode_info = QLabel(
            "<small><b>LF:</b> Plus grande portée, résolution réduite<br>"
            "<b>HF:</b> Meilleure résolution, portée réduite</small>"
        )
        mode_info.setStyleSheet("color: #7f8c8d; padding: 5px;")
        mode_layout.addRow(mode_info)
        
        mode_group.setLayout(mode_layout)
        scroll_layout.addWidget(mode_group)

        # ==================== Groupe Ping Rate ====================
        ping_group = QGroupBox("⏱️ Taux de Ping")
        ping_layout = QFormLayout()
        
        self.ping_combo = QComboBox()
        self.ping_combo.addItem("🔄 Auto", 0)
        self.ping_combo.addItem("📊 Normal", 1)
        self.ping_combo.addItem("⚡ High", 2)
        self.ping_combo.addItem("🚀 Highest", 3)
        self.ping_combo.addItem("🐢 Low", 4)
        self.ping_combo.addItem("🦥 Lowest", 5)
        self.ping_combo.setStyleSheet("""
            QComboBox {
                font-size: 14px;
                padding: 5px;
                color: white;
                background-color: #34495e;
            }
            QComboBox QAbstractItemView {
                color: white;
                background-color: #2c3e50;
                selection-color: white;
                selection-background-color: #2c3e50;
            }
            QComboBox QAbstractItemView::item {
                color: white;
            }
            QComboBox QAbstractItemView::item:hover {
                color: white;
                background-color: #2c3e50;
            }
            QComboBox QAbstractItemView::item:selected {
                color: white;
                background-color: #2c3e50;
            }
        """)
        ping_layout.addRow("Taux:", self.ping_combo)
        
        ping_group.setLayout(ping_layout)
        scroll_layout.addWidget(ping_group)

        # ==================== Bouton Appliquer ====================
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        scroll_layout.addWidget(separator)
        
        apply_layout = QHBoxLayout()
        
        self.apply_btn = QPushButton("✅ Appliquer Configuration")
        self.apply_btn.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                font-weight: bold;
                font-size: 16px;
                padding: 12px;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #2ecc71;
            }
            QPushButton:pressed {
                background-color: #229954;
            }
            QPushButton:disabled {
                background-color: #95a5a6;
            }
        """)
        self.apply_btn.clicked.connect(self.on_apply_clicked)
        apply_layout.addWidget(self.apply_btn)
        
        self.refresh_btn = QPushButton("🔄 Rafraîchir")
        self.refresh_btn.setStyleSheet("""
            QPushButton {
                background-color: #3498db;
                color: white;
                font-weight: bold;
                padding: 12px;
                border-radius: 8px;
            }
            QPushButton:hover { background-color: #2980b9; }
        """)
        self.refresh_btn.clicked.connect(self.on_refresh_clicked)
        apply_layout.addWidget(self.refresh_btn)
        
        scroll_layout.addLayout(apply_layout)

        # ==================== Presets ====================
        presets_group = QGroupBox("🎯 Préréglages Rapides")
        presets_layout = QHBoxLayout()
        
        preset_close = QPushButton("🔍 Proche\n(5m, HF)")
        preset_close.clicked.connect(lambda: self.apply_preset(5.0, 50, 2, 0))
        preset_close.setStyleSheet("padding: 10px;")
        presets_layout.addWidget(preset_close)
        
        preset_medium = QPushButton("📍 Moyen\n(15m, LF)")
        preset_medium.clicked.connect(lambda: self.apply_preset(15.0, 25, 1, 0))
        preset_medium.setStyleSheet("padding: 10px;")
        presets_layout.addWidget(preset_medium)
        
        preset_far = QPushButton("🌊 Loin\n(30m, LF)")
        preset_far.clicked.connect(lambda: self.apply_preset(30.0, 0, 1, 0))
        preset_far.setStyleSheet("padding: 10px;")
        presets_layout.addWidget(preset_far)
        
        preset_docking = QPushButton("🎯 Docking\n(10m, HF)")
        preset_docking.clicked.connect(lambda: self.apply_preset(10.0, 30, 2, 2))
        preset_docking.setStyleSheet("""
            QPushButton {
                background-color: #9b59b6;
                color: white;
                font-weight: bold;
                padding: 10px;
            }
            QPushButton:hover { background-color: #8e44ad; }
        """)
        presets_layout.addWidget(preset_docking)
        
        presets_group.setLayout(presets_layout)
        scroll_layout.addWidget(presets_group)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)

        # Connecter le signal de réponse du service
        if hasattr(ros_node, 'signals'):
            ros_node.signals.sonar_config_response.connect(self.on_config_response)

    def set_range_quick(self, value):
        """Définit rapidement la portée."""
        self.range_spin.setValue(value)
    
    def set_gain_quick(self, value):
        """Définit rapidement le gain."""
        self.gain_spin.setValue(value)
    
    def set_mode_quick(self, mode):
        """Définit rapidement le mode (1=LF, 2=HF)."""
        self.mode_combo.setCurrentIndex(mode - 1)
        self.apply_single_param(master_mode=mode)
    
    def apply_preset(self, range_m, gain, mode, ping_rate):
        """Applique un préréglage complet."""
        self.range_spin.setValue(range_m)
        self.gain_spin.setValue(gain)
        self.mode_combo.setCurrentIndex(mode - 1)
        self.ping_combo.setCurrentIndex(ping_rate)
        self.on_apply_clicked()
    
    def apply_single_param(self, range_m=0.0, gain_percent=-1, master_mode=-1, ping_rate=-1):
        """Applique un seul paramètre immédiatement."""
        self.ros_node.configure_sonar(
            range_m=range_m,
            gain_percent=gain_percent,
            master_mode=master_mode,
            ping_rate=ping_rate
        )
        self.status_label.setText("⏳ Configuration en cours...")
        self.status_label.setStyleSheet("""
            QLabel {
                background-color: #f39c12;
                color: white;
                padding: 8px;
                border-radius: 5px;
                font-weight: bold;
            }
        """)

    def on_apply_clicked(self):
        """Applique tous les paramètres actuels."""
        range_m = self.range_spin.value()
        gain = self.gain_spin.value()
        mode = self.mode_combo.currentData()
        ping_rate = self.ping_combo.currentData()
        
        self.ros_node.configure_sonar(
            range_m=range_m,
            gain_percent=gain,
            master_mode=mode,
            ping_rate=ping_rate
        )
        
        self.status_label.setText("⏳ Configuration en cours...")
        self.status_label.setStyleSheet("""
            QLabel {
                background-color: #f39c12;
                color: white;
                padding: 8px;
                border-radius: 5px;
                font-weight: bold;
            }
        """)
        self.apply_btn.setEnabled(False)
    
    def on_refresh_clicked(self):
        """Demande la configuration actuelle au sonar (sans modifier)."""
        # Envoyer une requête avec toutes les valeurs à -1/0 pour juste obtenir l'état
        self.ros_node.configure_sonar(
            range_m=0.0,
            gain_percent=-1,
            master_mode=-1,
            ping_rate=-1
        )
        self.status_label.setText("🔄 Récupération de la configuration...")

    def on_config_response(self, success, message, current_range, current_gain, current_mode, current_ping_rate):
        """Callback quand le service répond."""
        self.apply_btn.setEnabled(True)
        
        if success:
            # Mettre à jour les valeurs affichées
            self.current_range = current_range
            self.current_gain = current_gain
            self.current_mode = current_mode
            self.current_ping_rate = current_ping_rate
            
            # Mettre à jour les widgets si les valeurs ont changé
            if current_range > 0:
                self.range_spin.setValue(current_range)
            if current_gain >= 0:
                self.gain_spin.setValue(current_gain)
            if current_mode > 0:
                self.mode_combo.setCurrentIndex(current_mode - 1)
            if current_ping_rate >= 0:
                # Trouver l'index correspondant
                for i in range(self.ping_combo.count()):
                    if self.ping_combo.itemData(i) == current_ping_rate:
                        self.ping_combo.setCurrentIndex(i)
                        break
            
            mode_str = "LF" if current_mode == 1 else "HF"
            self.status_label.setText(
                f"✅ {message}\n"
                f"Range: {current_range:.1f}m | Gain: {current_gain}% | Mode: {mode_str}"
            )
            self.status_label.setStyleSheet("""
                QLabel {
                    background-color: #27ae60;
                    color: white;
                    padding: 8px;
                    border-radius: 5px;
                    font-weight: bold;
                }
            """)
        else:
            self.status_label.setText(f"❌ Erreur: {message}")
            self.status_label.setStyleSheet("""
                QLabel {
                    background-color: #e74c3c;
                    color: white;
                    padding: 8px;
                    border-radius: 5px;
                    font-weight: bold;
                }
            """)
