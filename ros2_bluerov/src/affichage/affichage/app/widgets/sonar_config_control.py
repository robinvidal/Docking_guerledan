"""
Widget de contrôle pour la configuration dynamique du sonar Oculus.

=============================================================================
GUIDE PyQt5 - SONAR CONFIG CONTROL
=============================================================================

CE WIDGET PERMET DE :
- Modifier les paramètres du sonar Oculus M750d en temps réel
- Appliquer des préréglages rapides
- Visualiser l'état de la configuration

INTERACTION AVEC ROS2 :
Ce widget envoie des requêtes au service ROS2 /configure_sonar.
Quand le service répond, un signal Qt est émis pour mettre à jour l'interface.

WIDGETS UTILISÉS :
- QSpinBox/QDoubleSpinBox : Entrées numériques avec min/max
- QComboBox : Menu déroulant pour les choix multiples
- QPushButton : Boutons d'action
- QGroupBox : Groupes visuels avec titre
- QScrollArea : Zone scrollable pour les longs formulaires
"""

# ============================================================================
# IMPORTS PyQt5
# ============================================================================
from PyQt5.QtCore import Qt  # Constantes et flags Qt
from PyQt5.QtWidgets import (
    QWidget,           # Classe de base des widgets
    QVBoxLayout,       # Layout vertical
    QScrollArea,       # Zone scrollable
    QLabel,            # Affichage de texte
    QGroupBox,         # Groupe avec titre et bordure
    QFormLayout,       # Layout formulaire (label: widget)
    QHBoxLayout,       # Layout horizontal
    QSpinBox,          # Entrée entière avec flèches
    QDoubleSpinBox,    # Entrée flottante avec flèches
    QComboBox,         # Menu déroulant
    QPushButton,       # Bouton cliquable
    QFrame,            # Cadre/séparateur visuel
)


class SonarConfigControlWidget(QWidget):
    """
    Panneau de contrôle pour la configuration du sonar en temps réel.
    
    ARCHITECTURE :
    ┌─────────────────────────────────────────┐
    │  Header (titre + description)           │
    ├─────────────────────────────────────────┤
    │  Status Label (état de la connexion)    │
    ├─────────────────────────────────────────┤
    │  📏 Groupe Portée (Range)               │
    │    └─ QDoubleSpinBox + boutons rapides  │
    ├─────────────────────────────────────────┤
    │  🔆 Groupe Gain                         │
    │    └─ QSpinBox + boutons rapides        │
    ├─────────────────────────────────────────┤
    │  📡 Groupe Mode (LF/HF)                 │
    │    └─ QComboBox + boutons LF/HF         │
    ├─────────────────────────────────────────┤
    │  ⏱️ Groupe Ping Rate                    │
    │    └─ QComboBox                         │
    ├─────────────────────────────────────────┤
    │  [Appliquer]  [Rafraîchir]              │
    ├─────────────────────────────────────────┤
    │  🎯 Préréglages rapides                 │
    └─────────────────────────────────────────┘
    """

    def __init__(self, ros_node):
        """
        Initialise le widget de configuration du sonar.
        
        Args:
            ros_node: Nœud ROS2 pour communiquer avec le service sonar
        """
        super().__init__()  # Appelle le constructeur de QWidget
        self.ros_node = ros_node
        
        # =====================================================================
        # ÉTAT INTERNE
        # =====================================================================
        # Ces valeurs sont mises à jour par les réponses du service ROS2
        self.current_range = 15.0    # Portée en mètres
        self.current_gain = 0        # Gain en %
        self.current_mode = 1        # 1=LF (basse fréq), 2=HF (haute fréq)
        self.current_ping_rate = 0   # Taux de ping
        
        # =====================================================================
        # LAYOUT PRINCIPAL + SCROLL
        # =====================================================================
        main_layout = QVBoxLayout(self)
        
        # QScrollArea : permet de scroller si le contenu dépasse
        # Très utile pour les longs formulaires
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)  # Le contenu s'adapte à la largeur
        
        # Widget qui va contenir tout le contenu scrollable
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)

        # =====================================================================
        # HEADER - Titre et description
        # =====================================================================
        header = QLabel(
            "🔊 <b>Configuration Sonar</b><br>"
            "<small>Modifiez les paramètres du sonar Oculus M750d en temps réel.</small>"
        )
        header.setWordWrap(True)  # Retour à la ligne automatique
        scroll_layout.addWidget(header)

        # =====================================================================
        # STATUS LABEL - Affiche l'état de la connexion/configuration
        # =====================================================================
        # Ce label change de couleur selon l'état (orange=attente, vert=ok, rouge=erreur)
        self.status_label = QLabel("⏳ En attente de connexion au sonar...")
        
        # CSS STYLING :
        # PyQt5 supporte un sous-ensemble de CSS pour styliser les widgets
        # Ici on définit : fond gris, texte blanc, padding, coins arrondis
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

        # =====================================================================
        # GROUPE PORTÉE (Range)
        # =====================================================================
        # QGroupBox : crée une boîte avec un titre et une bordure
        range_group = QGroupBox("📏 Portée (Range)")
        range_layout = QFormLayout()  # Layout formulaire
        
        # Layout horizontal pour mettre spinbox et boutons côte à côte
        range_h_layout = QHBoxLayout()
        
        # QDoubleSpinBox : entrée pour nombres décimaux
        # Différent de QSpinBox qui n'accepte que des entiers
        self.range_spin = QDoubleSpinBox()
        self.range_spin.setMinimum(1.0)      # Minimum 1 mètre
        self.range_spin.setMaximum(120.0)    # Maximum 120 mètres
        self.range_spin.setValue(15.0)       # Valeur par défaut
        self.range_spin.setSingleStep(1.0)   # Incrément par clic
        self.range_spin.setSuffix(" m")      # Affiche " m" après la valeur
        self.range_spin.setStyleSheet("font-size: 14px; padding: 5px;")
        range_h_layout.addWidget(self.range_spin)
        
        # BOUTONS RACCOURCI :
        # Boucle pour créer des boutons de valeurs prédéfinies
        for val in [5, 10, 15, 20, 30]:
            btn = QPushButton(f"{val}m")
            btn.setMaximumWidth(50)  # Limite la largeur
            # Lambda avec capture : v=val évite le problème de fermeture
            btn.clicked.connect(lambda checked, v=val: self.set_range_quick(v))
            range_h_layout.addWidget(btn)
        
        range_layout.addRow("Portée:", range_h_layout)
        range_group.setLayout(range_layout)
        scroll_layout.addWidget(range_group)

        # =====================================================================
        # GROUPE GAIN
        # =====================================================================
        # Le gain contrôle l'amplification du signal sonar
        gain_group = QGroupBox("🔆 Gain")
        gain_layout = QFormLayout()
        
        gain_h_layout = QHBoxLayout()
        
        # QSpinBox : entrée pour entiers (0-100%)
        self.gain_spin = QSpinBox()
        self.gain_spin.setMinimum(0)
        self.gain_spin.setMaximum(100)
        self.gain_spin.setValue(0)
        self.gain_spin.setSuffix(" %")  # Affiche " %" après la valeur
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

        # =====================================================================
        # GROUPE MODE FRÉQUENCE
        # =====================================================================
        # Le sonar peut fonctionner en deux modes :
        # - LF (Low Frequency)  : Longue portée, résolution moindre
        # - HF (High Frequency) : Courte portée, haute résolution
        mode_group = QGroupBox("📡 Mode Fréquence")
        mode_layout = QFormLayout()
        
        mode_h_layout = QHBoxLayout()
        
        # QComboBox : menu déroulant avec choix multiples
        # addItem(texte_affiché, donnée_associée)
        # La donnée associée est récupérable via currentData()
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("🔵 Basse Fréquence (LF)", 1)  # Valeur 1
        self.mode_combo.addItem("🔴 Haute Fréquence (HF)", 2)  # Valeur 2
        
        # STYLING COMPLEXE DU COMBOBOX :
        # On stylise à la fois le widget principal et sa liste déroulante
        self.mode_combo.setStyleSheet("""
            QComboBox {
                font-size: 14px;
                padding: 5px;
                color: white;
                background-color: #34495e;
            }
            /* QAbstractItemView = la liste déroulante */
            QComboBox QAbstractItemView {
                color: white;
                background-color: #2c3e50;
                selection-color: white;
                selection-background-color: #2c3e50;
            }
            /* ::item = chaque élément de la liste */
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
        
        # BOUTON LF - Style bleu
        self.lf_btn = QPushButton("LF")
        self.lf_btn.setStyleSheet("""
            QPushButton {
                background-color: #3498db;  /* Bleu */
                color: white;
                font-weight: bold;
                padding: 8px 15px;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #2980b9; }  /* Plus foncé au survol */
        """)
        self.lf_btn.clicked.connect(lambda: self.set_mode_quick(1))
        mode_h_layout.addWidget(self.lf_btn)
        
        # BOUTON HF - Style rouge
        self.hf_btn = QPushButton("HF")
        self.hf_btn.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;  /* Rouge */
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
        
        # Label d'information sur les modes
        mode_info = QLabel(
            "<small><b>LF:</b> Plus grande portée, résolution réduite<br>"
            "<b>HF:</b> Meilleure résolution, portée réduite</small>"
        )
        mode_info.setStyleSheet("color: #7f8c8d; padding: 5px;")
        mode_layout.addRow(mode_info)
        
        mode_group.setLayout(mode_layout)
        scroll_layout.addWidget(mode_group)

        # =====================================================================
        # GROUPE PING RATE
        # =====================================================================
        # Contrôle la fréquence d'émission des pings sonar
        # Plus rapide = plus de données mais plus de bruit
        ping_group = QGroupBox("⏱️ Taux de Ping")
        ping_layout = QFormLayout()
        
        # ComboBox avec plusieurs options de vitesse
        self.ping_combo = QComboBox()
        self.ping_combo.addItem("🔄 Auto", 0)       # Automatique
        self.ping_combo.addItem("📊 Normal", 1)    # Normal
        self.ping_combo.addItem("⚡ High", 2)       # Rapide
        self.ping_combo.addItem("🚀 Highest", 3)   # Très rapide
        self.ping_combo.addItem("🐢 Low", 4)        # Lent
        self.ping_combo.addItem("🦥 Lowest", 5)    # Très lent
        
        # Même style que le mode combo
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

        # =====================================================================
        # BOUTONS D'ACTION PRINCIPAUX
        # =====================================================================
        
        # QFrame comme séparateur visuel (ligne horizontale)
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)   # Ligne horizontale
        separator.setFrameShadow(QFrame.Sunken) # Effet enfoncé
        scroll_layout.addWidget(separator)
        
        apply_layout = QHBoxLayout()
        
        # BOUTON APPLIQUER - Style vert
        self.apply_btn = QPushButton("✅ Appliquer Configuration")
        self.apply_btn.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;  /* Vert */
                color: white;
                font-weight: bold;
                font-size: 16px;
                padding: 12px;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #2ecc71;  /* Plus clair au survol */
            }
            QPushButton:pressed {
                background-color: #229954;  /* Plus foncé quand pressé */
            }
            QPushButton:disabled {
                background-color: #95a5a6;  /* Gris quand désactivé */
            }
        """)
        self.apply_btn.clicked.connect(self.on_apply_clicked)
        apply_layout.addWidget(self.apply_btn)
        
        # BOUTON RAFRAÎCHIR - Pour lire la config actuelle du sonar
        self.refresh_btn = QPushButton("🔄 Rafraîchir")
        self.refresh_btn.setStyleSheet("""
            QPushButton {
                background-color: #3498db;  /* Bleu */
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

        # =====================================================================
        # PRESETS - Préréglages rapides
        # =====================================================================
        # Permet d'appliquer une configuration complète en un clic
        presets_group = QGroupBox("🎯 Préréglages Rapides")
        presets_layout = QHBoxLayout()
        
        # PRESET PROCHE : courte portée, haute résolution
        preset_close = QPushButton("🔍 Proche\n(5m, HF)")
        preset_close.clicked.connect(lambda: self.apply_preset(5.0, 50, 2, 0))
        preset_close.setStyleSheet("padding: 10px;")
        presets_layout.addWidget(preset_close)
        
        # PRESET MOYEN : portée moyenne, basse fréquence
        preset_medium = QPushButton("📍 Moyen\n(15m, LF)")
        preset_medium.clicked.connect(lambda: self.apply_preset(15.0, 25, 1, 0))
        preset_medium.setStyleSheet("padding: 10px;")
        presets_layout.addWidget(preset_medium)
        
        # PRESET LOIN : longue portée
        preset_far = QPushButton("🌊 Loin\n(30m, LF)")
        preset_far.clicked.connect(lambda: self.apply_preset(30.0, 0, 1, 0))
        preset_far.setStyleSheet("padding: 10px;")
        presets_layout.addWidget(preset_far)
        
        # PRESET DOCKING : optimisé pour l'amarrage (style violet différent)
        preset_docking = QPushButton("🎯 Docking\n(10m, HF)")
        preset_docking.clicked.connect(lambda: self.apply_preset(10.0, 30, 2, 2))
        preset_docking.setStyleSheet("""
            QPushButton {
                background-color: #9b59b6;  /* Violet */
                color: white;
                font-weight: bold;
                padding: 10px;
            }
            QPushButton:hover { background-color: #8e44ad; }
        """)
        presets_layout.addWidget(preset_docking)
        
        presets_group.setLayout(presets_layout)
        scroll_layout.addWidget(presets_group)

        scroll_layout.addStretch()  # Pousse tout vers le haut
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)

        # =====================================================================
        # CONNEXION DU SIGNAL ROS2 → Qt
        # =====================================================================
        # Quand le service sonar répond, le signal est émis
        # et cette méthode est appelée pour mettre à jour l'interface
        if hasattr(ros_node, 'signals'):
            ros_node.signals.sonar_config_response.connect(self.on_config_response)

    # =========================================================================
    # MÉTHODES HELPER - Raccourcis pour les boutons
    # =========================================================================
    
    def set_range_quick(self, value):
        """
        Définit rapidement la portée.
        Appelé par les boutons 5m, 10m, 15m, 20m, 30m.
        
        Args:
            value: Portée en mètres
        """
        self.range_spin.setValue(value)
    
    def set_gain_quick(self, value):
        """
        Définit rapidement le gain.
        Appelé par les boutons 0%, 25%, 50%, 75%, 100%.
        
        Args:
            value: Gain en pourcentage
        """
        self.gain_spin.setValue(value)
    
    def set_mode_quick(self, mode):
        """
        Définit rapidement le mode et l'applique immédiatement.
        Appelé par les boutons LF et HF.
        
        Args:
            mode: 1=LF (basse fréquence), 2=HF (haute fréquence)
        """
        self.mode_combo.setCurrentIndex(mode - 1)  # Index 0 ou 1
        self.apply_single_param(master_mode=mode)  # Applique immédiatement
    
    def apply_preset(self, range_m, gain, mode, ping_rate):
        """
        Applique un préréglage complet.
        
        Met à jour tous les widgets puis appelle on_apply_clicked().
        
        Args:
            range_m: Portée en mètres
            gain: Gain en %
            mode: 1=LF, 2=HF
            ping_rate: Taux de ping (0-5)
        """
        self.range_spin.setValue(range_m)
        self.gain_spin.setValue(gain)
        self.mode_combo.setCurrentIndex(mode - 1)
        self.ping_combo.setCurrentIndex(ping_rate)
        self.on_apply_clicked()  # Envoie au sonar
    
    def apply_single_param(self, range_m=0.0, gain_percent=-1, master_mode=-1, ping_rate=-1):
        """
        Applique un seul paramètre immédiatement au sonar.
        
        Les valeurs négatives ou nulles signifient "ne pas modifier".
        
        Args:
            range_m: Portée en mètres (0.0 = ne pas modifier)
            gain_percent: Gain en % (-1 = ne pas modifier)
            master_mode: Mode fréquence (-1 = ne pas modifier)
            ping_rate: Taux de ping (-1 = ne pas modifier)
        """
        # Appelle le service ROS2 via le nœud
        self.ros_node.configure_sonar(
            range_m=range_m,
            gain_percent=gain_percent,
            master_mode=master_mode,
            ping_rate=ping_rate
        )
        
        # Met à jour le status label pour indiquer que la config est en cours
        self.status_label.setText("⏳ Configuration en cours...")
        self.status_label.setStyleSheet("""
            QLabel {
                background-color: #f39c12;  /* Orange = en attente */
                color: white;
                padding: 8px;
                border-radius: 5px;
                font-weight: bold;
            }
        """)

    # =========================================================================
    # CALLBACKS - Méthodes appelées par les actions utilisateur
    # =========================================================================
    
    def on_apply_clicked(self):
        """
        Callback du bouton "Appliquer Configuration".
        
        Récupère les valeurs de tous les widgets et les envoie
        au sonar via le service ROS2.
        """
        # Récupère les valeurs des widgets
        range_m = self.range_spin.value()         # QDoubleSpinBox.value()
        gain = self.gain_spin.value()             # QSpinBox.value()
        mode = self.mode_combo.currentData()      # QComboBox.currentData() = valeur associée
        ping_rate = self.ping_combo.currentData()
        
        # Envoie au service ROS2
        self.ros_node.configure_sonar(
            range_m=range_m,
            gain_percent=gain,
            master_mode=mode,
            ping_rate=ping_rate
        )
        
        # Met à jour l'interface pour montrer que c'est en cours
        self.status_label.setText("⏳ Configuration en cours...")
        self.status_label.setStyleSheet("""
            QLabel {
                background-color: #f39c12;  /* Orange */
                color: white;
                padding: 8px;
                border-radius: 5px;
                font-weight: bold;
            }
        """)
        
        # Désactive le bouton pour éviter les clics multiples
        self.apply_btn.setEnabled(False)
    
    def on_refresh_clicked(self):
        """
        Callback du bouton "Rafraîchir".
        
        Demande la configuration actuelle au sonar sans la modifier.
        Envoie des valeurs "nulles" au service qui retourne juste l'état.
        """
        # Envoyer une requête avec toutes les valeurs à -1/0 pour juste obtenir l'état
        self.ros_node.configure_sonar(
            range_m=0.0,
            gain_percent=-1,
            master_mode=-1,
            ping_rate=-1
        )
        self.status_label.setText("🔄 Récupération de la configuration...")

    def on_config_response(self, success, message, current_range, current_gain, current_mode, current_ping_rate):
        """
        Callback appelé quand le service ROS2 répond.
        
        Ce slot est connecté au signal sonar_config_response.
        Appelé de manière thread-safe depuis le thread ROS2.
        
        FLUX DE DONNÉES :
        Service ROS2 répond → ros_node.
        ros_node émet signal → cette méthode met à jour l'UI.
        
        Args:
            success: True si la configuration a réussi
            message: Message décrivant le résultat
            current_range: Portée actuelle du sonar
            current_gain: Gain actuel
            current_mode: Mode actuel (1=LF, 2=HF)
            current_ping_rate: Taux de ping actuel
        """
        # Réactive le bouton maintenant qu'on a la réponse
        self.apply_btn.setEnabled(True)
        
        if success:
            # Sauvegarde les nouvelles valeurs dans l'état interne
            self.current_range = current_range
            self.current_gain = current_gain
            self.current_mode = current_mode
            self.current_ping_rate = current_ping_rate
            
            # Met à jour les widgets avec les vraies valeurs du sonar
            # (elles peuvent différer de ce qu'on a demandé)
            if current_range > 0:
                self.range_spin.setValue(current_range)
            if current_gain >= 0:
                self.gain_spin.setValue(current_gain)
            if current_mode > 0:
                self.mode_combo.setCurrentIndex(current_mode - 1)
            if current_ping_rate >= 0:
                # Trouve l'index correspondant à la valeur
                for i in range(self.ping_combo.count()):
                    if self.ping_combo.itemData(i) == current_ping_rate:
                        self.ping_combo.setCurrentIndex(i)
                        break
            
            # Affiche le succès avec les valeurs actuelles
            mode_str = "LF" if current_mode == 1 else "HF"
            self.status_label.setText(
                f"✅ {message}\n"
                f"Range: {current_range:.1f}m | Gain: {current_gain}% | Mode: {mode_str}"
            )
            # Style vert = succès
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
            # Affiche l'erreur
            self.status_label.setText(f"❌ Erreur: {message}")
            # Style rouge = erreur
            self.status_label.setStyleSheet("""
                QLabel {
                    background-color: #e74c3c;
                    color: white;
                    padding: 8px;
                    border-radius: 5px;
                    font-weight: bold;
                }
            """)
