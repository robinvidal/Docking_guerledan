# =============================================================================
# MAIN_WINDOW.PY - Fenêtre principale de l'interface graphique
# =============================================================================
#
# HIÉRARCHIE DES WIDGETS PyQt:
# ----------------------------
# En PyQt, tout est un "Widget" (élément graphique).
# Les widgets peuvent contenir d'autres widgets (comme des poupées russes).
#
# STRUCTURE DE CETTE FENÊTRE:
# ┌─────────────────────────────────────────────────────────────────────────┐
# │  MainWindow (QMainWindow)                                               │
# │  ┌───────────────────────────────────────────────────────────────────┐  │
# │  │  Central Widget (QWidget)                                         │  │
# │  │  ┌─────────────────────────────────────────────────────────────┐  │  │
# │  │  │  Splitter (QSplitter) - Divise en 2 zones redimensionnables │  │  │
# │  │  │  ┌──────────────────────┐  ┌─────────────────────────────┐  │  │  │
# │  │  │  │  Panneau Gauche      │  │  Panneau Droit              │  │  │  │
# │  │  │  │  ┌────────────────┐  │  │  ┌───────────────────────┐  │  │  │  │
# │  │  │  │  │ Boutons tabs   │  │  │  │ Boutons tabs          │  │  │  │  │
# │  │  │  │  ├────────────────┤  │  │  ├───────────────────────┤  │  │  │  │
# │  │  │  │  │ QStackedWidget │  │  │  │ QStackedWidget        │  │  │  │  │
# │  │  │  │  │(pages empilées)│  │  │  │ (pages empilées)      │  │  │  │  │
# │  │  │  │  └────────────────┘  │  │  └───────────────────────┘  │  │  │  │
# │  │  │  └──────────────────────┘  └─────────────────────────────┘  │  │  │
# │  │  └─────────────────────────────────────────────────────────────┘  │  │
# │  └───────────────────────────────────────────────────────────────────┘  │
# └─────────────────────────────────────────────────────────────────────────┘
#
# LAYOUTS PyQt (Disposition des widgets):
# - QVBoxLayout: Empile les widgets VERTICALEMENT (V = Vertical)
# - QHBoxLayout: Empile les widgets HORIZONTALEMENT (H = Horizontal)
# - QGridLayout: Grille de lignes/colonnes (non utilisé ici)
#
# =============================================================================

import numpy as np

# Imports des widgets PyQt5
from PyQt5.QtWidgets import (
    QMainWindow,      # Fenêtre principale avec menu, barre d'outils, etc.
    QWidget,          # Widget de base (conteneur générique)
    QVBoxLayout,      # Layout vertical (empile de haut en bas)
    QHBoxLayout,      # Layout horizontal (empile de gauche à droite)
    QStackedWidget,   # Conteneur de pages empilées (une seule visible à la fois)
    QPushButton,      # Bouton cliquable
    QSplitter,        # Séparateur redimensionnable entre widgets
)
from PyQt5.QtCore import Qt  # Constantes Qt (orientations, alignements, etc.)

# Nos widgets personnalisés pour l'affichage du sonar
from .widgets.sonar_panels import (
    RawSonarPanel,                  # Affichage brut du sonar
    CartesianFilteredSonarPanel,    # Affichage cartésien filtré
    CompareSonarPanel               # Comparaison des deux vues
)
from .widgets.tracker_control import TrackerControlWidget
from .widgets.sonar_config_control import SonarConfigControlWidget
from .core.config_loader import get_cage_dimensions


class MainWindow(QMainWindow):
    """
    Fenêtre principale de l'application.
    
    QMainWindow est spéciale car elle a:
    - Une barre de menu (optionnelle)
    - Une barre d'outils (optionnelle)
    - Un widget central (OBLIGATOIRE)
    - Une barre de statut (optionnelle)
    
    Ici on utilise seulement le widget central.
    """

    def __init__(self, ros_node):
        """
        Initialise la fenêtre principale.
        
        Args:
            ros_node: Le nœud ROS2 pour communiquer avec le robot
        """
        # super().__init__() appelle le constructeur de QMainWindow
        # C'est OBLIGATOIRE quand on hérite d'une classe Qt
        super().__init__()
        
        # Garde une référence au nœud ROS2 pour pouvoir l'utiliser
        self.ros_node = ros_node

        # Configure la fenêtre elle-même
        self.setWindowTitle('Sonar Viewer - Système de Docking')  # Titre
        self.setMinimumSize(1200, 800)  # Taille minimum en pixels

        # =====================================================================
        # CRÉATION DU WIDGET CENTRAL
        # =====================================================================
        # QMainWindow EXIGE un widget central. On crée un QWidget vide
        # puis on lui ajoute un layout pour organiser son contenu.
        
        central_widget = QWidget()  # Conteneur principal
        self.setCentralWidget(central_widget)  # L'assigne comme widget central
        
        # Crée un layout vertical et l'assigne au widget central
        # Le layout gère automatiquement la position et la taille des enfants
        main_layout = QVBoxLayout(central_widget)

        # =====================================================================
        # SPLITTER - Divise la fenêtre en zones redimensionnables
        # =====================================================================
        # QSplitter permet à l'utilisateur de redimensionner les zones
        # en faisant glisser la barre de séparation.
        # Qt.Horizontal = séparation verticale (gauche | droite)
        splitter = QSplitter(Qt.Horizontal)

        # =====================================================================
        # PANNEAU GAUCHE - Affichage du sonar
        # =====================================================================
        
        # Conteneur pour tout le panneau gauche
        left_container = QWidget()
        left_layout = QVBoxLayout(left_container)

        # ----- Barre de boutons pour changer de vue -----
        # QHBoxLayout empile les boutons horizontalement
        left_switch = QHBoxLayout()
        
        # Crée les 3 boutons d'onglets
        self.raw_btn = QPushButton('📡 Brut')
        self.cartesian_filtered_btn = QPushButton('🟢 Filtré')
        self.compare_btn = QPushButton('⚖️ Comparaison')
        
        # setCheckable(True) = le bouton reste "enfoncé" quand on clique
        # (comme un bouton radio, pas comme un bouton normal)
        for btn in (self.raw_btn, self.cartesian_filtered_btn, self.compare_btn):
            btn.setCheckable(True)
        
        # addWidget() ajoute chaque bouton au layout
        left_switch.addWidget(self.raw_btn)
        left_switch.addWidget(self.cartesian_filtered_btn)
        left_switch.addWidget(self.compare_btn)
        
        # addStretch() ajoute un espace élastique qui pousse tout à gauche
        left_switch.addStretch()
        
        # Ajoute la barre de boutons au layout vertical
        left_layout.addLayout(left_switch)

        # ----- QStackedWidget - Pages empilées -----
        # QStackedWidget contient plusieurs widgets mais n'en affiche qu'UN
        # C'est comme un deck de cartes : on ne voit que celle du dessus
        # setCurrentIndex(n) change la page visible
        
        self.left_stack = QStackedWidget()
        
        # Crée les 3 panneaux d'affichage
        self.raw_panel = RawSonarPanel()                      # Index 0
        self.cartesian_filtered_panel = CartesianFilteredSonarPanel()  # Index 1
        self.compare_panel = CompareSonarPanel()              # Index 2
        
        # addWidget() les ajoute dans l'ordre (l'index est automatique)
        self.left_stack.addWidget(self.raw_panel)
        self.left_stack.addWidget(self.cartesian_filtered_panel)
        self.left_stack.addWidget(self.compare_panel)
        
        # Ajoute le stack au layout vertical
        left_layout.addWidget(self.left_stack)
        
        # =====================================================================
        # CONNEXION DES SIGNAUX - Le cœur du pattern Signal/Slot
        # =====================================================================
        # signal.connect(slot) : Quand le signal est émis, le slot est appelé
        #
        # Ici, quand l'utilisateur sélectionne une bbox dans le viewer,
        # le signal bbox_selected est émis, et on_bbox_selected() est appelée.
        # =====================================================================
        
        # Pour dessiner une bbox, l'utilisateur clique sur "Sélectionner" dans le tracker,
        # puis clique-glisse sur le sonar. Les deux viewers cartésiens passent en mode sélection,
        # et quand la sélection est faite, on_bbox_selected() reçoit les coordonnées.
        self.cartesian_filtered_panel.viewer.bbox_selected.connect(self.on_bbox_selected)
        self.compare_panel.cartesian_viewer.bbox_selected.connect(self.on_bbox_selected)
        

        # Ajoute le conteneur gauche au splitter
        splitter.addWidget(left_container)

        # =====================================================================
        # PANNEAU DROIT - Contrôles (Tracker + Config Sonar)
        # =====================================================================
        # Même structure que le panneau gauche : conteneur + layout + tabs + stack
        
        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)
        
        # Barre de boutons d'onglets
        right_switch = QHBoxLayout()
        self.tracker_tab_btn = QPushButton('🎯 Tracker')
        self.sonar_config_tab_btn = QPushButton('🔊 Sonar')
        for btn in (self.tracker_tab_btn, self.sonar_config_tab_btn):
            btn.setCheckable(True)
        right_switch.addWidget(self.tracker_tab_btn)
        right_switch.addWidget(self.sonar_config_tab_btn)
        right_switch.addStretch()
        right_layout.addLayout(right_switch)
        
        # Stack de pages pour les contrôles
        self.right_stack = QStackedWidget()
        self.tracker_widget = TrackerControlWidget(self.ros_node)  # Index 0
        self.sonar_config_widget = SonarConfigControlWidget(self.ros_node)  # Index 1
        self.right_stack.addWidget(self.tracker_widget)
        self.right_stack.addWidget(self.sonar_config_widget)
        right_layout.addWidget(self.right_stack)
        
        # =====================================================================
        # CONNEXION BOUTONS → ACTIONS avec lambda
        # =====================================================================
        # clicked est un SIGNAL émis quand le bouton est cliqué
        # lambda: ... crée une fonction anonyme (pratique pour passer des args)
        # 
        # Sans lambda: btn.clicked.connect(self.set_right_view)  # ERREUR!
        #              Car clicked ne passe pas d'argument, mais set_right_view en attend un
        #
        # Avec lambda: btn.clicked.connect(lambda: self.set_right_view(0))  # OK!
        #              La lambda "capture" la valeur 0 et la passe à set_right_view
        # =====================================================================
        
        self.tracker_tab_btn.clicked.connect(lambda: self.set_right_view(0))
        self.sonar_config_tab_btn.clicked.connect(lambda: self.set_right_view(1))
        self.set_right_view(0)  # Affiche l'onglet Tracker par défaut
        
        # Connecte le signal de demande de sélection bbox aux deux viewers
        # Quand l'utilisateur clique "Sélectionner" dans le tracker,
        # les deux viewers cartésiens passent en mode sélection
        self.tracker_widget.bbox_selection_requested.connect(
            lambda enabled: self.cartesian_filtered_panel.viewer.set_bbox_selection_mode(enabled)
        )
        self.tracker_widget.bbox_selection_requested.connect(
            lambda enabled: self.compare_panel.cartesian_viewer.set_bbox_selection_mode(enabled)
        )
        
        # Connecte le signal de statut auto-detect (ROS2) au widget tracker (Qt)
        self.ros_node.signals.auto_detect_status_changed.connect(
            self.tracker_widget.on_auto_detect_status_changed
        )

        # Ajoute le panneau droit au splitter
        splitter.addWidget(right_container)
        
        # setStretchFactor(index, facteur) définit comment l'espace est réparti
        # Ici: panneau gauche = 3 parts, panneau droit = 2 parts
        # Donc gauche prend 60% de l'espace (3/(3+2)), droite 40%
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        # Ajoute le splitter au layout principal
        main_layout.addWidget(splitter)

        # =====================================================================
        # CONNEXION DES BOUTONS D'ONGLETS GAUCHE
        # =====================================================================
        self.raw_btn.clicked.connect(lambda: self.set_left_view(0))
        self.cartesian_filtered_btn.clicked.connect(lambda: self.set_left_view(1))
        self.compare_btn.clicked.connect(lambda: self.set_left_view(2))

        # Sélectionne l'onglet "Comparaison" par défaut au démarrage
        self.set_left_view(2)

        # =====================================================================
        # CONNEXION DES SIGNAUX ROS2 → MÉTHODES Qt
        # =====================================================================
        # Quand ROS2 reçoit des données, les signaux sont émis.
        # Ces connexions font que nos méthodes on_xxx() sont appelées
        # pour mettre à jour l'interface.
        #
        # C'EST ICI QUE L'INTERFACE SE MET À JOUR EN TEMPS RÉEL !
        # =====================================================================
        
        self.ros_node.signals.new_raw_frame.connect(self.on_raw_frame)
        self.ros_node.signals.new_cartesian_filtered_frame.connect(self.on_cartesian_filtered_frame)
        self.ros_node.signals.new_detected_lines.connect(self.on_detected_lines)
        self.ros_node.signals.new_tracked_object.connect(self.on_tracked_object)
        self.ros_node.signals.new_cage_pose.connect(self.on_cage_pose)

    # =========================================================================
    # SLOTS - Méthodes appelées quand les signaux sont émis
    # =========================================================================
    # Ces méthodes reçoivent les données ROS2 et mettent à jour l'interface.
    # Elles sont appelées automatiquement grâce aux connexions .connect()
    # =========================================================================

    def on_raw_frame(self, msg):
        """
        Slot appelé quand une nouvelle frame brute arrive.
        
        Met à jour:
        - Le panneau brut (si visible)
        - La partie brute du panneau comparaison
        """
        self.raw_panel.update_frame(msg)
        self.compare_panel.update_raw(msg)

    def on_cartesian_filtered_frame(self, msg):
        """Slot appelé quand une frame cartésienne filtrée arrive."""
        self.cartesian_filtered_panel.update_frame(msg)
        self.compare_panel.update_cartesian(msg)
    
    def on_detected_lines(self, msg):
        """Slot appelé quand des lignes détectées arrivent."""
        self.cartesian_filtered_panel.update_detected_lines(msg)
        self.compare_panel.update_detected_lines(msg)
    
    def on_tracked_object(self, msg):
        """Slot appelé quand un objet traqué arrive."""
        self.cartesian_filtered_panel.update_tracked_object(msg)
        self.compare_panel.update_tracked_object(msg)

    def on_cage_pose(self, msg):
        """Slot appelé quand la pose de la cage arrive."""
        self.cartesian_filtered_panel.update_cage_pose(msg)
        self.compare_panel.update_cage_pose(msg)

    # =========================================================================
    # MÉTHODES DE NAVIGATION - Changement d'onglets
    # =========================================================================

    def set_left_view(self, index):
        """
        Change l'onglet visible dans le panneau gauche.
        
        Args:
            index: 0=Brut, 1=Filtré, 2=Comparaison
        """
        # Change la page visible du QStackedWidget
        self.left_stack.setCurrentIndex(index)
        
        # Met à jour l'état des boutons (un seul "checked" à la fois)
        buttons = (self.raw_btn, self.cartesian_filtered_btn, self.compare_btn)
        for i, btn in enumerate(buttons):
            btn.setChecked(i == index)  # True seulement pour le bouton actif
    
    def set_right_view(self, index):
        """
        Change l'onglet visible dans le panneau droit.
        
        Args:
            index: 0=Tracker, 1=Config Sonar
        """
        self.right_stack.setCurrentIndex(index)
        buttons = (self.tracker_tab_btn, self.sonar_config_tab_btn)
        for i, btn in enumerate(buttons):
            btn.setChecked(i == index)
    
    # =========================================================================
    # HANDLERS D'ÉVÉNEMENTS UTILISATEUR
    # =========================================================================
    
    def on_bbox_selected(self, x, y, width, height):
        """
        Slot appelé quand l'utilisateur dessine une bbox sur le sonar.
        
        Cette méthode fait le pont: Qt → ROS2
        Elle appelle le nœud ROS2 pour publier la sélection.
        """
        self.ros_node.publish_bbox_selection(x, y, width, height)
