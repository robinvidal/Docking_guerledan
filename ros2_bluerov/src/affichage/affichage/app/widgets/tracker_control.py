# =============================================================================
# TRACKER_CONTROL.PY - Panneau de contrôle pour le tracker et les filtres
# =============================================================================
#
# CE QUE FAIT CE FICHIER :
# ------------------------
# Fournit une interface graphique pour :
# 1. Initialiser le tracker (sélection manuelle ou auto-détection)
# 2. Régler les paramètres de traitement d'image en temps réel
# 3. Sauvegarder/charger les configurations
#
# WIDGETS UTILISÉS :
# ------------------
# - QScrollArea : Zone scrollable (pour beaucoup de paramètres)
# - QGroupBox : Boîte avec titre pour regrouper des widgets
# - QFormLayout : Layout pour formulaires (label: widget)
# - QCheckBox : Case à cocher (booléen)
# - QSpinBox : Entrée numérique entière
# - QDoubleSpinBox : Entrée numérique décimale
# - QSlider : Curseur coulissant
# - QPushButton : Bouton
#
# SIGNAL PERSONNALISÉ :
# ---------------------
# bbox_selection_requested : émis quand l'utilisateur veut sélectionner une zone
#
# =============================================================================

from pathlib import Path

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QScrollArea,         # Zone scrollable
    QLabel,
    QGroupBox,           # Boîte avec titre
    QFormLayout,         # Layout label: widget
    QCheckBox,           # Case à cocher
    QSpinBox,            # Entrée entière
    QDoubleSpinBox,      # Entrée décimale
    QHBoxLayout,
    QSlider,             # Curseur
    QPushButton,
    QFileDialog,         # Boîte de dialogue fichier
    QMessageBox,         # Boîte de dialogue message
)
import yaml  # Pour lire/écrire les fichiers de config

from ..core.utils import load_yaml_params  # Fonction utilitaire locale


class TrackerControlWidget(QWidget):
    """
    Panneau de contrôle pour le tracker et les paramètres de traitement.
    
    FONCTIONNALITÉS :
    1. Boutons pour démarrer/arrêter le tracking
    2. Sliders et spinbox pour régler les filtres en temps réel
    3. Sauvegarde/chargement des paramètres
    
    ARCHITECTURE :
    - Les widgets de paramètres sont créés dynamiquement via _create_group_box()
    - Chaque changement déclenche on_param_changed() qui envoie à ROS2
    - Les valeurs initiales sont chargées depuis un fichier YAML
    """
    
    # =========================================================================
    # SIGNAL PERSONNALISÉ
    # =========================================================================
    # Ce signal est émis quand l'utilisateur clique sur "Sélectionner Cage".
    # Il transporte un bool : True = activer le mode sélection, False = désactiver
    #
    # Dans main_window.py, ce signal est connecté aux viewers pour activer
    # leur mode sélection de bbox.
    # =========================================================================
    bbox_selection_requested = pyqtSignal(bool)

    def __init__(self, ros_node):
        """
        Initialise le panneau de contrôle.
        
        STRUCTURE DE L'INTERFACE :
        ┌──────────────────────────────────────┐
        │  Header (titre)                    │
        ├──────────────────────────────────────┤
        │  [Sélectionner] [Auto Tracking]   │
        ├──────────────────────────────────────┤
        │  ┌─ Filtre Médian (Polaire) ───┐   │
        │  │  [✓] Activer                 │   │
        │  │  Taille: [===●===] [3]      │   │
        │  └───────────────────────────┘   │
        │  ┌─ Filtre Frost (Polaire) ───┐   │
        │  │  ...                        │   │
        │  └───────────────────────────┘   │
        │  ... (autres groupes)              │
        ├──────────────────────────────────────┤
        │  [Réinitialiser] [Sauvegarder]    │
        └──────────────────────────────────────┘
        
        Args:
            ros_node: Le nœud ROS2 pour envoyer les paramètres
        """
        super().__init__()
        self.ros_node = ros_node
        
        # Dictionnaire pour stocker les références aux widgets de paramètres
        # Clé = nom du paramètre, Valeur = widget (QCheckBox, QSpinBox, etc.)
        self.param_widgets = {}

        # =====================================================================
        # LAYOUT PRINCIPAL + ZONE SCROLLABLE
        # =====================================================================
        # On utilise QScrollArea car il y a beaucoup de paramètres.
        # Si la fenêtre est trop petite, on peut scroller.
        
        main_layout = QVBoxLayout(self)
        
        # QScrollArea : conteneur scrollable
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)  # Le contenu s'adapte à la largeur
        
        # Widget qui sera dans le scroll (contient le vrai contenu)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)

        # =====================================================================
        # HEADER - Titre avec HTML
        # =====================================================================
        # QLabel peut afficher du HTML basique : <b>gras</b>, <small>petit</small>
        header = QLabel(
            "🎛️ <b>Contrôles Tracker & Traitement Unified</b><br>"
            "<small>Réglez les paramètres de traitement en temps réel.</small>"
        )
        header.setWordWrap(True)  # Retour à la ligne automatique
        scroll_layout.addWidget(header)
        
        # =====================================================================
        # BOUTONS DE CONTRÔLE DU TRACKER
        # =====================================================================
        # Deux modes pour initialiser le tracker :
        # 1. Sélection manuelle : l'utilisateur dessine un rectangle
        # 2. Auto Tracking : détection automatique de la cage
        
        selection_layout = QHBoxLayout()  # Les deux boutons côte à côte
        
        # Bouton "Sélectionner Cage" - Active le mode sélection manuelle
        self.select_bbox_btn = QPushButton("📦 Sélectionner Cage (CSRT)")
        
        # setStyleSheet() applique du CSS-like pour personnaliser l'apparence
        # On peut définir des styles pour différents états (:hover, :pressed, :checked)
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
        
        # setCheckable(True) : le bouton peut rester "enfoncé" (comme un toggle)
        self.select_bbox_btn.setCheckable(True)
        
        # Connexion : quand on clique, appelle on_select_bbox_clicked
        # Le bool "checked" est passé automatiquement
        self.select_bbox_btn.clicked.connect(self.on_select_bbox_clicked)
        selection_layout.addWidget(self.select_bbox_btn)
        
        # Bouton "Auto Tracking" - Détection automatique de la cage
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
        self.auto_tracking_btn.setCheckable(True)  # Bouton toggle
        self.auto_tracking_btn.clicked.connect(self.on_auto_tracking_clicked)
        selection_layout.addWidget(self.auto_tracking_btn)
        
        scroll_layout.addLayout(selection_layout)  # Ajoute au layout vertical
        
        # Label d'aide pour expliquer les modes
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

        # =====================================================================
        # GROUPES DE PARAMÈTRES
        # =====================================================================
        # Chaque groupe est créé avec _create_group_box() qui génère
        # automatiquement les widgets (checkbox, slider, spinbox) selon le type.
        #
        # Format des paramètres :
        # - Bool : (nom, label, 'bool', valeur_defaut)
        # - Int  : (nom, label, 'int', valeur_defaut, min, max, step)
        # - Float: (nom, label, 'double', valeur_defaut, min, max, step)
        # =====================================================================
        
        # ----- Filtres appliqués sur l'image POLAIRE (avant conversion) -----
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

        # ----- Paramètres de la conversion cartésienne -----
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

        # ----- Filtres appliqués sur l'image CARTÉSIENNE -----
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

        # Label informatif
        info_label = QLabel(
            "ℹ️ Les modifications sont appliquées en temps réel au nœud traitement_unified_node.\n"
            "Utilisez 'Sauvegarder' pour rendre les changements permanents."
        )
        info_label.setWordWrap(True)
        scroll_layout.addWidget(info_label)

        # =====================================================================
        # FOOTER - Boutons de contrôle global
        # =====================================================================
        footer = QHBoxLayout()
        footer.addStretch()  # Pousse les boutons à droite
        
        # Bouton "Réinitialiser" - Remet les valeurs par défaut
        reset_btn = QPushButton('🔄 Réinitialiser')
        reset_btn.clicked.connect(self.reset_to_defaults)
        footer.addWidget(reset_btn)
        
        # Bouton "Sauvegarder" - Écrit dans un fichier YAML
        save_btn = QPushButton('💾 Sauvegarder')
        save_btn.clicked.connect(self.save_to_yaml)
        footer.addWidget(save_btn)
        
        scroll_layout.addLayout(footer)

        # addStretch() à la fin pousse tout vers le haut
        scroll_layout.addStretch()
        
        # Finalise le scroll : met scroll_content dans scroll
        scroll.setWidget(scroll_content)
        
        # Ajoute le scroll au layout principal
        main_layout.addWidget(scroll)

        # Charge les valeurs initiales depuis le fichier YAML
        self._apply_yaml_defaults()

    def _create_group_box(self, title, params):
        """
        Crée un groupe de paramètres avec les widgets appropriés.
        
        FACTORY PATTERN :
        Cette méthode génère automatiquement les widgets selon le type.
        Ça évite de répéter le même code pour chaque paramètre.
        
        TYPES SUPPORTÉS :
        - 'bool'   : QCheckBox (case à cocher)
        - 'int'    : QSlider + QSpinBox couplés
        - 'double' : QSlider + QDoubleSpinBox couplés
        
        COUPLAGE SLIDER/SPINBOX :
        Quand on bouge le slider, le spinbox se met à jour et vice-versa.
        C'est fait via des connexions croisées de signaux.
        
        Args:
            title: Titre du groupe (affiché en haut de la boîte)
            params: Liste de tuples définissant les paramètres
        
        Returns:
            QGroupBox: Le groupe avec tous ses widgets
        """
        # QGroupBox : boîte avec un titre et une bordure
        group = QGroupBox(title)
        
        # QFormLayout : layout spécial pour les formulaires
        # Chaque ligne a un label à gauche et un widget à droite
        layout = QFormLayout()

        for param_info in params:
            # ================================================================
            # TYPE BOOLÉEN : QCheckBox
            # ================================================================
            if param_info[2] == 'bool':
                param_name, label, _, default = param_info
                
                checkbox = QCheckBox()
                checkbox.setChecked(default)  # Valeur initiale
                
                # Connexion : quand la case change, envoie le paramètre à ROS2
                # Le lambda capture le nom du paramètre (name=param_name)
                # state == Qt.Checked convertit l'état en booléen
                checkbox.stateChanged.connect(
                    lambda state, name=param_name: self.on_param_changed(name, state == Qt.Checked)
                )
                
                # addRow() ajoute une ligne au formulaire : "Label: [widget]"
                layout.addRow(label + ':', checkbox)
                
                # Stocke la référence pour pouvoir lire/écrire la valeur plus tard
                self.param_widgets[param_name] = checkbox

            # ================================================================
            # TYPE ENTIER : QSlider + QSpinBox couplés
            # ================================================================
            elif param_info[2] == 'int':
                param_name, label, _, default, min_val, max_val, step = param_info
                
                # Layout horizontal pour mettre slider et spinbox côte à côte
                widget_layout = QHBoxLayout()
                
                # QSlider : curseur coulissant
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(min_val)
                slider.setMaximum(max_val)
                slider.setValue(default)
                slider.setSingleStep(step)  # Incrément quand on clique

                # QSpinBox : entrée numérique avec flèches
                spinbox = QSpinBox()
                spinbox.setMinimum(min_val)
                spinbox.setMaximum(max_val)
                spinbox.setValue(default)
                spinbox.setSingleStep(step)

                # COUPLAGE : les deux widgets se synchronisent
                # Quand slider change → spinbox se met à jour
                slider.valueChanged.connect(spinbox.setValue)
                # Quand spinbox change → slider se met à jour
                spinbox.valueChanged.connect(slider.setValue)
                
                # Quand spinbox change → envoie à ROS2
                spinbox.valueChanged.connect(
                    lambda value, name=param_name: self.on_param_changed(name, value)
                )

                # Ajoute au layout avec proportions (3:1 = slider plus large)
                widget_layout.addWidget(slider, 3)
                widget_layout.addWidget(spinbox, 1)
                layout.addRow(label + ':', widget_layout)
                self.param_widgets[param_name] = spinbox  # Stocke le spinbox

            # ================================================================
            # TYPE FLOTTANT : QSlider + QDoubleSpinBox couplés
            # ================================================================
            elif param_info[2] == 'double':
                param_name, label, _, default, min_val, max_val, step = param_info
                widget_layout = QHBoxLayout()

                # Pour les doubles, le slider travaille en "unités de step"
                # Ex: si step=0.1, slider va de 0 à 100 pour représenter 0.0 à 10.0
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(int(min_val / step))
                slider.setMaximum(int(max_val / step))
                slider.setValue(int(default / step))

                spinbox = QDoubleSpinBox()
                spinbox.setMinimum(min_val)
                spinbox.setMaximum(max_val)
                spinbox.setValue(default)
                spinbox.setSingleStep(step)
                
                # Calcule le nombre de décimales à afficher
                spinbox.setDecimals(len(str(step).split('.')[-1]) if '.' in str(step) else 1)

                # COUPLAGE avec conversion (slider = valeur * step)
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
        """
        Callback quand le bouton "Sélectionner Cage" est cliqué.
        
        LOGIQUE :
        - Si activé : change le texte, désactive l'autre mode, émet le signal
        - Si désactivé : remet le texte normal
        
        Args:
            checked: True si le bouton est maintenant enfoncé
        """
        if checked:
            self.select_bbox_btn.setText("❌ Annuler Sélection")
            
            # Désactive l'autre bouton (mutuellement exclusifs)
            self.auto_tracking_btn.setChecked(False)
            self.auto_tracking_btn.setText("🔍 Auto Tracking")
            
            # Arrête l'auto-tracking via ROS2
            self.ros_node.publish_auto_detect_trigger(False)
        else:
            self.select_bbox_btn.setText("📦 Sélectionner Cage (CSRT)")
        
        # Émet le signal pour activer/désactiver le mode sélection
        # dans les viewers cartésiens
        self.bbox_selection_requested.emit(checked)
    
    def on_auto_tracking_clicked(self, checked):
        """
        Callback quand le bouton "Auto Tracking" est cliqué.
        
        Active la détection automatique de cage via Hough Lines.
        
        Args:
            checked: True si le bouton est maintenant enfoncé
        """
        if checked:
            self.auto_tracking_btn.setText("⏹️ Arrêter Auto Tracking")
            
            # Désactive l'autre bouton
            self.select_bbox_btn.setChecked(False)
            self.select_bbox_btn.setText("📦 Sélectionner Cage (CSRT)")
            self.bbox_selection_requested.emit(False)  # Désactive mode sélection
        else:
            self.auto_tracking_btn.setText("🔍 Auto Tracking")
        
        # Publie vers ROS2 pour activer/désactiver l'auto-detect
        self.ros_node.publish_auto_detect_trigger(checked)
    
    def on_auto_detect_status_changed(self, is_searching: bool):
        """
        Callback appelé quand le statut de l'auto-detect change.
        
        Reçu via signal ROS2 → Qt.
        Met à jour le texte du bouton pour indiquer l'état.
        
        Args:
            is_searching: True = en recherche, False = cage trouvée
        """
        if not is_searching and self.auto_tracking_btn.isChecked():
            # La cage a été trouvée, le tracker CSRT est actif
            self.auto_tracking_btn.setText("✅ Tracking Actif (Auto)")
        elif is_searching and self.auto_tracking_btn.isChecked():
            # En recherche (au démarrage ou après perte de la cage)
            self.auto_tracking_btn.setText("🔍 Recherche en cours...")

    def on_param_changed(self, name, value):
        """
        Callback appelé quand un paramètre change.
        
        Envoie la nouvelle valeur au nœud ROS2 traitement_unified_node
        via le service de paramètres.
        
        TEMPS RÉEL :
        Les changements sont appliqués immédiatement, pas besoin de cliquer "Appliquer".
        
        Args:
            name: Nom du paramètre (ex: 'polar_enable_median')
            value: Nouvelle valeur (bool, int, ou float)
        """
        success = self.ros_node.set_traitement_unified_parameter(name, value)
        if success:
            self.ros_node.get_logger().info(f'Traitement Unified: {name}={value}')

    def _apply_yaml_defaults(self):
        """
        Charge les valeurs par défaut depuis le fichier YAML de configuration.
        
        Appelée au démarrage pour initialiser les widgets avec les valeurs
        du fichier de config ROS2.
        """
        # Charge le fichier YAML
        params = load_yaml_params('traitement', 'traitement_unified_params.yaml', self.ros_node.get_logger())
        
        # Extrait les paramètres du nœud
        sub = params.get('traitement_unified_node', {}).get('ros__parameters', {}) if isinstance(params, dict) else {}
        if not sub:
            return
            
        # Applique chaque valeur au widget correspondant
        for name, value in sub.items():
            if name not in self.param_widgets:
                continue  # Ignore les paramètres sans widget
                
            widget = self.param_widgets[name]
            try:
                # Selon le type de widget, utilise la bonne méthode
                if isinstance(widget, QCheckBox):
                    widget.setChecked(bool(value))
                elif isinstance(widget, QSpinBox):
                    widget.setValue(int(value))
                elif isinstance(widget, QDoubleSpinBox):
                    widget.setValue(float(value))
            except Exception:
                self.ros_node.get_logger().debug(f'Param unified {name} ignore (val={value})')

    def get_current_params(self):
        """
        Récupère les valeurs actuelles de tous les paramètres.
        
        Lit les valeurs depuis les widgets (pas depuis ROS2).
        Utilisé pour la sauvegarde.
        
        Returns:
            dict: {nom_param: valeur}
        """
        params = {}
        for name, widget in self.param_widgets.items():
            if isinstance(widget, QCheckBox):
                params[name] = widget.isChecked()  # Retourne bool
            elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                params[name] = widget.value()  # Retourne int ou float
        return params

    def reset_to_defaults(self):
        """
        Réinitialise tous les paramètres aux valeurs par défaut.
        
        Affiche une boîte de dialogue de confirmation avant.
        """
        # QMessageBox.question() : boîte de dialogue Oui/Non
        reply = QMessageBox.question(
            self,                           # Parent
            "Confirmer réinitialisation",  # Titre
            "Voulez-vous vraiment réinitialiser tous les paramètres aux valeurs par défaut ?",
            QMessageBox.Yes | QMessageBox.No,  # Boutons
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
        """
        Sauvegarde les paramètres actuels dans un fichier YAML.
        
        PROCESSUS :
        1. Récupère toutes les valeurs des widgets
        2. Construit la structure YAML ROS2
        3. Ouvre une boîte de dialogue pour choisir le fichier
        4. Écrit le fichier YAML
        
        Le fichier peut ensuite être utilisé au lancement du nœud ROS2.
        """
        # Récupère les valeurs actuelles
        params = self.get_current_params()
        
        # Structure YAML pour ROS2
        # Format: nom_du_noeud/ros__parameters/nom_param: valeur
        yaml_content = {'traitement_unified_node': {'ros__parameters': params}}

        # Chemin par défaut : dans le dossier config du package
        default_path = (
            self._find_ros2_root()
            / 'src'
            / 'traitement'
            / 'config'
            / 'traitement_unified_params.yaml'
        )

        # QFileDialog.getSaveFileName() : boîte de dialogue "Enregistrer sous"
        # Retourne (chemin_fichier, filtre_sélectionné)
        file_path, _ = QFileDialog.getSaveFileName(
            self, 
            'Sauvegarder paramètres Traitement Unified', 
            str(default_path), 
            'YAML Files (*.yaml *.yml)'  # Filtre de fichiers
        )
        
        if file_path:  # Si l'utilisateur n'a pas annulé
            try:
                # Écrit le fichier YAML
                with open(file_path, 'w', encoding='utf-8') as f:
                    yaml.dump(yaml_content, f, default_flow_style=False, sort_keys=False)
                
                # Affiche un message de succès
                QMessageBox.information(
                    self, 
                    'Sauvegarde réussie', 
                    f'Paramètres sauvegardés dans:\n{file_path}\n\n'
                    'Pour les utiliser au démarrage:\n'
                    f'ros2 run traitement traitement_unified_node --ros-args --params-file {file_path}'
                )
            except Exception as exc:
                # Affiche une erreur si la sauvegarde échoue
                QMessageBox.critical(self, 'Erreur', f'Erreur: {exc}')

    def _find_ros2_root(self) -> Path:
        """
        Trouve le répertoire racine ros2_bluerov.
        
        Remonte l'arborescence des dossiers jusqu'à trouver 'ros2_bluerov'.
        
        Returns:
            Path: Chemin vers la racine du workspace ROS2
        """
        # Path(__file__).resolve().parents : liste de tous les dossiers parents
        for parent in Path(__file__).resolve().parents:
            if parent.name == 'ros2_bluerov':
                return parent
        # Fallback si non trouvé
        return Path(__file__).resolve().parents[5]
