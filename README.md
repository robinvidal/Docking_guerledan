# Docking Autonome BlueROV - Lac de Guerlédan

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python 3.10](https://img.shields.io/badge/Python-3.10-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

Système de docking autonome pour ROV BlueROV2 utilisant un sonar imageur Oculus M750d. Le robot doit détecter et entrer dans une cage sous-marine en s'appuyant uniquement sur les données sonar, sans vision optique.

## Vue d'ensemble

Ce projet implémente un pipeline complet de perception et contrôle permettant à un BlueROV2 d'effectuer un docking autonome :

1. **Acquisition sonar** - Driver Oculus M750d (données polaires)
2. **Traitement d'image** - Filtrage et conversion polaire → cartésien
3. **Détection** - Tracking des bords de cage (CSRT, Hough)
4. **Localisation** - Estimation de pose relative (range, bearing, orientation)
5. **Contrôle** - Asservissement PID (surge, sway, yaw)
6. **Mission** - Machine d'états supervisée (7 états)

### Architecture du système

```
┌──────────────┐     ┌─────────────┐     ┌──────────────┐
│ Sonar Oculus │────▶│ Traitement  │────▶│   Tracking   │
│   M750d      │     │  (Filtres)  │     │ (Bords cage) │
└──────────────┘     └─────────────┘     └───────┬──────┘
                                                  │
       ┌──────────────────────────────────────────┘
       │
       ▼
┌──────────────┐     ┌─────────────┐     ┌──────────────┐
│ Localisation │────▶│   Contrôle  │────▶│   Mission    │
│  (Pose 3D)   │     │  (PID x/y/ψ)│     │ (États FSM)  │
└──────────────┘     └─────────────┘     └──────────────┘
```

## Structure du projet

```
Docking_guerledan/                 # Racine du dépôt GitHub
├── ros2_bluerov/                  # Workspace ROS 2 principal
│   ├── src/
│   │   ├── sonar/                 # Driver et simulation sonar Oculus
│   │   ├── traitement/            # Filtrage images sonar (polar/cartesian)
│   │   ├── tracking/              # Détection bords cage (CSRT, Hough)
│   │   ├── localisation/          # Estimation pose relative
│   │   ├── control/               # Contrôleurs PID
│   │   ├── mission/               # Machine d'états docking
│   │   ├── affichage/             # IHM Qt/PyQtGraph
│   │   ├── bringup/               # Launch files orchestration
│   │   ├── docking_msgs/          # Messages ROS custom
│   │   └── docking_utils/         # Utilitaires communs
│   ├── build/                     # Dossier de compilation (généré)
│   └── install/                   # Dossier d'installation (généré)
│
├── simulation/                    # Simulation 2D Python (tests contrôle)
├── docs/                          # Documentation et rapport LaTeX
├── divers/                        # Archives codes expérimentaux
├── bluerov2/                      # Description URDF et contrôleurs
└── bluerov_sim/                   # Simulateur simple
```

## Installation

### Prérequis

- **Ubuntu 22.04 LTS**
- **ROS 2 Humble** ([guide installation](https://docs.ros.org/en/humble/Installation.html))
- **Python 3.10+**
- **QGroundControl** ([téléchargement](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html))

### Dépendances système

```bash
# MAVROS2 pour communication BlueROV
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Joystick support
sudo apt install ros-humble-joy

# Outils supplémentaires
sudo apt install sshpass ffmpeg
```

### Environnement Python

```bash
pip install numpy==1.24.0 opencv-python==4.2.0 opencv-contrib-python \
    scipy pyautogui imutils vidgear brping PyQt5 pyqtgraph
```

### Compilation du workspace

```bash
# Cloner le dépôt
git clone https://github.com/<votre-organisation>/Docking_guerledan.git
cd Docking_guerledan

# Compiler le workspace ROS 2
cd ros2_bluerov
colcon build --symlink-install
source install/setup.bash

# Ajouter au ~/.bashrc pour persistance
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
```

### Installation GeographicLib (MAVROS)

```bash
# Script fourni dans le dépôt (depuis la racine du projet)
cd Docking_guerledan
bash install_geographiclib_datasets.sh
```

## Utilisation

### 1. Pipeline complet (Sonar réel)

Lance tous les nœuds avec le sonar Oculus physique :

```bash
ros2 launch bringup complete_pipeline.launch.py
```

**Nœuds lancés :** sonar_driver, traitement, tracking, localisation, contrôle, mission, affichage

### 2. Pipeline avec rosbag (Replay)

Rejoue des données sonar enregistrées :

```bash
ros2 launch bringup rosbag_pipeline.launch.py
```

### 3. Test avec sonar simulé

Idéal pour développement sans matériel :

```bash
ros2 launch bringup user_pipeline.launch.py
```

**Sonar mock :** génère une cage synthétique avec bruit gaussien (configurable)

### 4. Mission de docking

Une fois le pipeline lancé, démarrer la mission :

```bash
# Via topic ROS
ros2 topic pub /docking/mission/start std_msgs/Bool "data: true"

# Annuler la mission
ros2 topic pub /docking/mission/abort std_msgs/Bool "data: true"
```

### 5. Monitoring en temps réel

```bash
# État de la mission
ros2 topic echo /docking/mission/state

# Pose estimée
ros2 topic echo /docking/localisation/pose

# Commandes contrôle
ros2 topic echo /cmd_vel

# Graphe des nœuds
rqt_graph
```

## Topics ROS principaux

| Topic | Type | Description |
|-------|------|-------------|
| `/docking/sonar/raw` | `docking_msgs/Frame` | Données sonar brutes (polaire) |
| `/docking/sonar/filtered` | `docking_msgs/Frame` | Image filtrée (cartésien) |
| `/docking/tracking/borders` | `docking_msgs/Borders` | Bords cage détectés |
| `/docking/localisation/pose` | `docking_msgs/PoseRelative` | Pose 3D estimée (x, y, ψ) |
| `/docking/mission/state` | `docking_msgs/State` | État machine FSM |
| `/cmd_vel` | `geometry_msgs/Twist` | Consignes vélocité ROV |

## Configuration

Chaque package possède son fichier de paramètres YAML :

```
sonar/config/sonar_params.yaml          # Fréquence, résolution
traitement/config/traitement_params.yaml # Filtres (Gaussian, CLAHE, etc.)
tracking/config/tracking_params.yaml     # Algorithmes détection
localisation/config/localisation_params.yaml # Modèle géométrique
control/config/control_params.yaml       # Gains PID (Kp, Ki, Kd)
mission/config/mission_params.yaml       # Timeouts, seuils FSM
```

Exemple : modifier les gains PID

```yaml
# control/config/control_params.yaml
pid_surge:
  kp: 0.5
  ki: 0.01
  kd: 0.1
```

## Tests et simulation

### Simulation 2D Python

Test rapide des lois de commande sans ROS :

```bash
# Depuis la racine du projet
cd Docking_guerledan/simulation
python3 simulation_bluerov.py
```

**Fonctionnalités :**
- Visualisation 2D du ROV et de la cage
- Comparaison contrôleurs (PID, LQR, SMC)
- Génération de trajectoires

### Tests unitaires ROS

```bash
# Depuis la racine du projet
cd Docking_guerledan/ros2_bluerov
colcon test
colcon test-result --verbose
```

## Packages détaillés

### sonar
- `sonar_driver_node` : Interface Oculus M750d (TCP)
- `sonar_mock_node` : Générateur cage synthétique

### traitement
- `traitement_polar_node` : Filtres sur données polaires
- `traitement_cartesian_node` : Conversion polar→cartesian + filtres

### tracking
- `csrt_tracker_node` : Tracking CSRT (OpenCV) **[Utilisé]**
- `hough_lines_node` : Détection Hough + filtrage **[Utilisé]**

### localisation
- `localisation_node` : Triangulation géométrique (range/bearing → pose 3D)

### control
- `control_node` : PID cascade (surge, sway, yaw)

### mission
- `mission_node` : FSM 7 états (IDLE, LOCK_ON, APPROACH, DOCKING, DOCKED, RECOVERY, ABORT)

### affichage
- `ihm_node` : Interface Qt temps réel (sonar, tracking, pose, graphes)

## État de la mission (FSM)

```
IDLE → LOCK_ON → APPROACH → DOCKING → DOCKED
         ↓           ↓          ↓
       RECOVERY ←────┴──────────┘
         ↓
       ABORT
```

**Transitions :**
- `IDLE→LOCK_ON` : Cage détectée
- `LOCK_ON→APPROACH` : Pose valide acquise (timeout 10s sinon RECOVERY)
- `APPROACH→DOCKING` : Distance < 1m ET aligné (|ψ| < 6°)
- `DOCKING→DOCKED` : Contact (y < 0.3m)
- `*→RECOVERY` : Perte pose (timeout 30s sinon ABORT)
- `*→ABORT` : Commande manuelle ou erreur critique

## Développement

### Ajouter un nouveau tracker

1. Créer le nœud dans `tracking/tracking/`
2. Hériter de `rclpy.node.Node`
3. Souscrire à `/docking/sonar/filtered`
4. Publier sur `/docking/tracking/borders`
5. Ajouter l'entry point dans `tracking/setup.py`
6. Rebuild : `colcon build --packages-select tracking`

### Debug mode

```bash
# Activer logs détaillés
ros2 run tracking csrt_tracker_node --ros-args --log-level debug

# Enregistrer topics pour replay
ros2 bag record /docking/sonar/raw /docking/tracking/borders
```

## Troubleshooting

### Sonar non détecté
```bash
# Vérifier connexion réseau (IP : 10.0.0.1)
ping 10.0.0.1

# Tester driver
ros2 run sonar sonar_driver_node
```

### Pas de détection de cage
- Vérifier paramètres tracking dans `tracking/config/tracking_params.yaml`
- Visualiser l'image filtrée dans l'IHM
- Réduire seuils de détection

### Contrôle instable
- Réduire gains PID dans `control/config/control_params.yaml`
- Vérifier orientation ROV (yaw)
- Tester en simulation 2D d'abord

## Documentation

- **Rapport complet** : `docs/Rapport_Docking/rapport.pdf`
- **READMEs détaillés** : Chaque package contient son propre README
- **Analyse transformations** : `ANALYSE_TRANSFORMATIONS.md`

## Contributeurs

Projet académique ENSTA Bretagne - Guerlédan 2025/2026

Auteurs : Bourgeois Thomas, Dunot Clément, Lefèvre Maxime, Vidal Robin

## Licence

Apache License 2.0

---

**Contact :** thomas.bourgeois@ensta.fr, clement.dunot@ensta.fr, maxime.lefevre@ensta.fr, robin.vidal@ensta.fr

**Ressources :**
- [Documentation ROS 2 Humble](https://docs.ros.org/en/humble/)
- [Blueprint Subsea Oculus](https://www.blueprintsubsea.com/oculus/)
- [BlueROV2 Documentation](https://bluerobotics.com/learn/bluerov2/)

