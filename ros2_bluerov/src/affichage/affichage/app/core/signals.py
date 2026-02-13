# =============================================================================
# SIGNALS.PY - Communication thread-safe entre ROS2 et Qt
# =============================================================================
# 
# CONCEPT CLÉ: Les signaux Qt (pyqtSignal)
# -----------------------------------------
# En PyQt, les signaux sont un mécanisme de communication entre objets.
# Ils permettent de notifier qu'un événement s'est produit.
#
# POURQUOI C'EST IMPORTANT ?
# - ROS2 reçoit des données dans un thread séparé
# - Qt (l'interface graphique) tourne dans le thread principal
# - On NE PEUT PAS modifier l'interface depuis un autre thread !
# - Les signaux Qt sont "thread-safe" : ils passent les données
#   d'un thread à l'autre de manière sécurisée.
#
# PATTERN SIGNAL/SLOT:
# - Signal : "Je crie que quelque chose s'est passé !"
# - Slot   : "J'écoute et je réagis quand ça arrive"
# - Connexion : signal.connect(slot) → lie les deux
# =============================================================================

from PyQt5.QtCore import QObject, pyqtSignal


class ROSSignals(QObject):
    """
    Classe contenant tous les signaux pour la communication ROS → Qt.
    
    POURQUOI hériter de QObject ?
    - pyqtSignal ne peut être utilisé QUE dans une classe qui hérite de QObject
    - QObject est la classe de base de tous les objets Qt
    """

    # =========================================================================
    # DÉCLARATION DES SIGNAUX
    # =========================================================================
    # Syntaxe: nom_du_signal = pyqtSignal(types_des_données)
    # 
    # pyqtSignal(object) → peut transporter n'importe quel objet Python
    # pyqtSignal(int)    → transporte un entier
    # pyqtSignal(str)    → transporte une chaîne
    # pyqtSignal(bool, str, float) → transporte plusieurs valeurs
    # =========================================================================
    
    # Signal émis quand une nouvelle frame brute arrive du sonar
    # "object" permet de passer le message ROS2 complet
    new_raw_frame = pyqtSignal(object)
    
    # Signal pour les frames cartésiennes filtrées
    new_cartesian_filtered_frame = pyqtSignal(object)
    
    # Signal pour les lignes détectées par le tracking
    new_detected_lines = pyqtSignal(object)
    
    # Signal pour la pose (position + orientation) de la cage
    new_cage_pose = pyqtSignal(object)
    
    # Signal pour l'objet traqué (bounding box du tracker CSRT)
    new_tracked_object = pyqtSignal(object)
    
    # Signal indiquant si la recherche automatique est active
    # bool: True = en recherche, False = pas en recherche
    auto_detect_status_changed = pyqtSignal(bool)
    
    # Signal de réponse de configuration du sonar
    # Transporte plusieurs valeurs: (succès, message, portée, gain, mode, ping_rate)
    sonar_config_response = pyqtSignal(bool, str, float, int, int, int)
