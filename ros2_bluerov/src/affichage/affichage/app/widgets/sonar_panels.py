# =============================================================================
# SONAR_PANELS.PY - Panneaux conteneurs pour les vues sonar
# =============================================================================
#
# CE FICHIER EST UN "WRAPPER" (enveloppe)
# ---------------------------------------
# Ces classes sont des conteneurs simples qui :
# 1. Créent un layout
# 2. Ajoutent un widget d'affichage (le vrai travail est fait ailleurs)
# 3. Délèguent les appels aux widgets internes
#
# PATTERN "COMPOSITION" :
# - Au lieu de tout mettre dans un seul fichier énorme,
#   on compose plusieurs petits objets spécialisés
# - RawSonarPanel CONTIENT un SonarCartesianWidget (il ne l'est pas)
#
# HIÉRARCHIE :
# ┌────────────────────────────────────────────────────────┐
# │  MainWindow                                            │
# │     └── QStackedWidget (onglets)                        │
# │           ├── RawSonarPanel          ← CE FICHIER     │
# │           │      └── SonarCartesianWidget              │
# │           ├── CartesianFilteredSonarPanel            │
# │           │      └── SonarCartesianImageWidget         │
# │           └── CompareSonarPanel                      │
# │                  ├── SonarCartesianWidget              │
# │                  └── SonarCartesianImageWidget         │
# └────────────────────────────────────────────────────────┘
#
# =============================================================================

from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout

# Les vrais widgets d'affichage (font le travail complexe)
from .sonar_display import SonarCartesianWidget          # Affichage polaire → cartésien
from .sonar_cartesian_display import SonarCartesianImageWidget  # Affichage cartésien direct


class RawSonarPanel(QWidget):
    """
    Panneau pour afficher le sonar brut (données polaires converties).
    
    C'est un simple conteneur qui:
    - Hérite de QWidget (conteneur générique)
    - Contient un SonarCartesianWidget qui fait le vrai travail
    """
    
    def __init__(self):
        super().__init__()  # TOUJOURS appeler le constructeur parent
        
        # Crée un layout vertical et l'assigne à ce widget
        # En passant 'self' au constructeur, le layout est automatiquement
        # attaché à ce widget
        layout = QVBoxLayout(self)
        
        # Crée le widget d'affichage sonar avec un titre
        self.viewer = SonarCartesianWidget("Sonar Brut")
        
        # Ajoute le viewer au layout (il prendra tout l'espace)
        layout.addWidget(self.viewer)

    def update_frame(self, frame_msg):
        """
        Met à jour l'affichage avec une nouvelle frame sonar.
        
        Cette méthode est un "pass-through" (elle ne fait que transférer).
        Elle reçoit le message ROS2 et le passe au viewer interne.
        
        Args:
            frame_msg: Message ROS2 de type Frame (données polaires)
        """
        # Délègue au widget interne qui sait comment afficher
        self.viewer.update_image(frame_msg)


class CartesianFilteredSonarPanel(QWidget):
    """
    Panneau pour afficher le sonar cartésien filtré.
    
    Utilise SonarCartesianImageWidget qui affiche directement
    des images déjà en coordonnées cartésiennes (pas de conversion).
    """
    
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        self.viewer = SonarCartesianImageWidget("Sonar Cartésien Filtré")
        layout.addWidget(self.viewer)

    def update_frame(self, frame_msg):
        """Met à jour l'image cartésienne."""
        self.viewer.update_cartesian_frame(frame_msg)
    
    def update_detected_lines(self, lines_msg):
        """Met à jour les lignes détectées (Hough)."""
        self.viewer.update_detected_lines(lines_msg)
    
    def update_tracked_object(self, tracked_msg):
        """Met à jour la bounding box du tracker."""
        self.viewer.update_tracked_object(tracked_msg)

    def update_cage_pose(self, pose_msg):
        """Met à jour la pose de la cage détectée."""
        self.viewer.update_cage_pose(pose_msg)


class CompareSonarPanel(QWidget):
    """
    Panneau de comparaison: brut à gauche, cartésien à droite.
    
    Utilise un QHBoxLayout pour mettre les deux vues côte à côte.
    """
    
    def __init__(self):
        super().__init__()
        
        # Layout HORIZONTAL pour mettre les deux vues côte à côte
        layout = QHBoxLayout(self)
        
        # Deux viewers différents
        self.raw_viewer = SonarCartesianWidget("Brut")
        self.cartesian_viewer = SonarCartesianImageWidget("Cartésien Filtré")
        
        # Les ajoute au layout (50% chacun par défaut)
        layout.addWidget(self.raw_viewer)
        layout.addWidget(self.cartesian_viewer)

    def update_raw(self, frame_msg):
        """Met à jour la vue brute (gauche)."""
        self.raw_viewer.update_image(frame_msg)

    def update_cartesian(self, frame_msg):
        """Met à jour la vue cartésienne (droite)."""
        self.cartesian_viewer.update_cartesian_frame(frame_msg)
    
    def update_detected_lines(self, lines_msg):
        """Met à jour les lignes sur la vue cartésienne."""
        self.cartesian_viewer.update_detected_lines(lines_msg)
    
    def update_tracked_object(self, tracked_msg):
        """Met à jour le tracker sur la vue cartésienne."""
        self.cartesian_viewer.update_tracked_object(tracked_msg)

    def update_cage_pose(self, pose_msg):
        """Met à jour la pose cage sur la vue cartésienne."""
        self.cartesian_viewer.update_cage_pose(pose_msg)
