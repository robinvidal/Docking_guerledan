# Noeuds — Résumé rapide

Ce dossier contient les paquets Python (probablement utilisés avec ROS) qui gèrent la commande, l'interface, le lancement et le suivi du projet Docking_guerledan.
Le but de ce README réduit est d'expliquer brièvement à quoi sert chaque package et ce qu'on y trouve de principal.

Paquets présents

- bluerov_control0 : utilitaires de contrôle.
   - Contient : `utils_0.py`, `utils_pid.py` (PID et fonctions utilitaires), tests.
   - Rôle : calcul des lois de commande et fonctions d'assistance pour convertir/filtrer les mesures.

- bluerov_ihm0 : interface homme‑machine.
   - Contient : `ihm.py` (logique d'IHM), packaging et tests.
   - Rôle : supervision et/ou interface graphique pour visualiser l'état et piloter manuellement.

- bluerov_launch0 : scripts et configurations de lancement.
   - Contient : `config/parametre_inky.yaml`, `launch/px4_inky_launch.py` et métadonnées.
   - Rôle : orchestration du démarrage des nœuds et centralisation des paramètres (PID, options de run).

- bluerov_tracking0 : nœud de tracking.
   - Contient : `tracking_node_vs0.py` (nœud principal), tests.
   - Rôle : estimer/traiter la position/trajectoire et publier des consignes pour le suivi.

Comment l'utiliser (essentiel)

- Installer le paquet en mode développement :
   - Depuis le répertoire du paquet : `pip install -e .` ou `python setup.py develop`.
- Lancer un nœud Python simple : ex. `python -m bluerov_tracking0.tracking_node_vs0` (si empaqueté pour être exécuté ainsi).
- Si vous utilisez ROS/ROS2 : préférez `roslaunch` / `ros2 launch` ou le script dans `launch/` pour démarrer l'ensemble.

Tests et qualité (essentiel)

- Chaque paquet contient un dossier `test/` avec des contrôles de style (flake8), docstrings (PEP257) et checks de licence.
- Exécuter `pytest` dans le paquet concerné pour lancer les tests.

Conseils rapides

- Lire `config/parametre_inky.yaml` pour connaître les paramètres critiques (PID, gains).
- Activer le logging (niveau DEBUG) pour le débogage local.
- Avant de pousser : lancer les tests et le linting.

Contact

- Pour des questions, ouvrir une issue dans le dépôt. Consultez l'historique Git pour identifier les mainteneurs.

Fichier condensé pour une lecture rapide — pour plus de détails, on peut rétablir la version longue ou ajouter des exemples de lancement spécifiques ROS1/ROS2.
