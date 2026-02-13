# =============================================================================
# MAIN.PY - Point d'entrée de l'application PyQt5 + ROS2
# =============================================================================
#
# ARCHITECTURE DE L'APPLICATION:
# ------------------------------
#     ┌──────────────────────────────────────────────────────────────┐
#     │                    Thread Principal (Qt)                     │
#     │  ┌───────────────┐     ┌──────────────────────────────────┐  │
#     │  │  QApplication │────►│         MainWindow               │  │
#     │  └───────────────┘     │  (Interface graphique)           │  │
#     │         │              └──────────────────────────────────┘  │
#     │         │                          ▲                         │
#     │         ▼                          │ (signaux Qt)            │
#     │  ┌───────────────┐          ┌──────┴───────┐                 │
#     │  │    QTimer     │─────────►│  ROSSignals  │                 │
#     │  │ (toutes 10ms) │          └──────────────┘                 │
#     │  └───────────────┘                 ▲                         │
#     │         │                          │                         │
#     │         ▼                          │                         │
#     │  ┌─────────────────────────────────┴──────────────────────┐  │
#     │  │              SonarViewerNode (ROS2)                    │  │
#     │  │  - Reçoit les messages ROS2                            │  │
#     │  │  - Émet les signaux Qt                                 │  │
#     │  └────────────────────────────────────────────────────────┘  │
#     └──────────────────────────────────────────────────────────────┘
#
# =============================================================================

import os
import sys

# QApplication : Classe principale de toute application Qt
# C'est le "chef d'orchestre" qui gère la boucle d'événements
from PyQt5.QtWidgets import QApplication

# QTimer : Permet d'exécuter du code périodiquement
# Ici, utilisé pour faire tourner ROS2 dans la boucle Qt
from PyQt5.QtCore import QTimer

# rclpy : Bibliothèque Python pour ROS2
import rclpy

# Imports locaux de notre application
from .core.signals import ROSSignals        # Nos signaux de communication
from .core.ros_node import SonarViewerNode  # Notre nœud ROS2
from .main_window import MainWindow         # Notre fenêtre principale


def apply_global_style(app):
    """
    Applique une feuille de style CSS à toute l'application.
    
    PyQt utilise QSS (Qt Style Sheets), très similaire au CSS web.
    Ça permet de personnaliser l'apparence de TOUS les widgets.
    
    Exemple de QSS:
        QPushButton {
            background-color: #3498db;
            border-radius: 5px;
        }
    """
    style_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'style.qss')
    if os.path.exists(style_path):
        with open(style_path, 'r', encoding='utf-8') as handle:
            # setStyleSheet() applique le style à l'app entière
            app.setStyleSheet(handle.read())


def main(args=None):
    """
    Fonction principale qui démarre l'application.
    
    ÉTAPES:
    1. Initialiser ROS2
    2. Créer l'application Qt
    3. Créer le nœud ROS2 avec les signaux
    4. Créer et afficher la fenêtre
    5. Configurer le timer pour ROS2
    6. Lancer la boucle d'événements
    7. Nettoyer à la fermeture
    """
    
    # =========================================================================
    # ÉTAPE 1: Initialiser ROS2
    # =========================================================================
    # rclpy.init() doit être appelé AVANT de créer des nœuds ROS2
    rclpy.init(args=args)

    # =========================================================================
    # ÉTAPE 2: Créer l'application Qt
    # =========================================================================
    # QApplication est OBLIGATOIRE pour toute application PyQt
    # sys.argv permet de passer des arguments en ligne de commande
    # Il ne peut y avoir qu'UNE SEULE QApplication par programme !
    app = QApplication(sys.argv)
    
    # Appliquer le style visuel personnalisé
    apply_global_style(app)

    # =========================================================================
    # ÉTAPE 3: Créer les signaux et le nœud ROS2
    # =========================================================================
    # Les signaux sont créés EN PREMIER car le nœud ROS2 en a besoin
    signals = ROSSignals()
    
    # Le nœud ROS2 reçoit les signaux pour pouvoir les émettre
    # quand il reçoit des messages ROS2
    ros_node = SonarViewerNode(signals)

    # =========================================================================
    # ÉTAPE 4: Créer et afficher la fenêtre principale
    # =========================================================================
    # La fenêtre reçoit le nœud ROS2 pour pouvoir:
    # - Se connecter aux signaux (recevoir les données)
    # - Appeler des méthodes du nœud (envoyer des commandes)
    window = MainWindow(ros_node)
    
    # show() rend la fenêtre visible à l'écran
    # Sans ça, la fenêtre existe mais est invisible !
    window.show()

    # =========================================================================
    # ÉTAPE 5: Configurer le timer pour intégrer ROS2 dans Qt
    # =========================================================================
    # PROBLÈME: Qt a sa propre boucle d'événements (app.exec_())
    #           ROS2 aussi a besoin de tourner (rclpy.spin())
    #           On ne peut pas avoir DEUX boucles infinies !
    #
    # SOLUTION: Utiliser un QTimer qui appelle rclpy.spin_once()
    #           toutes les 10 millisecondes.
    #           Ainsi, ROS2 traite ses messages dans la boucle Qt.
    #
    timer = QTimer()
    
    # timeout est un SIGNAL émis quand le timer expire
    # On le connecte à une fonction lambda qui fait tourner ROS2
    # timeout_sec=0 signifie "ne pas bloquer, retourner immédiatement"
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    
    # start(10) = le timer se déclenche toutes les 10ms
    # = 100 fois par seconde, suffisant pour une interface réactive
    timer.start(10)

    # =========================================================================
    # ÉTAPE 6: Lancer la boucle d'événements Qt
    # =========================================================================
    try:
        # app.exec_() démarre la boucle d'événements Qt
        # Cette fonction BLOQUE jusqu'à ce que l'application se ferme
        # Elle retourne un code de sortie (0 = succès, autre = erreur)
        exit_code = app.exec_()
    finally:
        # =====================================================================
        # ÉTAPE 7: Nettoyage à la fermeture
        # =====================================================================
        # "finally" garantit que ce code s'exécute même en cas d'erreur
        # C'est IMPORTANT de bien nettoyer les ressources ROS2 !
        ros_node.destroy_node()  # Détruit le nœud proprement
        rclpy.shutdown()          # Arrête ROS2

    # Quitte le programme avec le code de sortie
    sys.exit(exit_code)


# Point d'entrée standard Python
# __name__ == '__main__' signifie "ce fichier est exécuté directement"
# (pas importé comme module)
if __name__ == '__main__':
    main()
