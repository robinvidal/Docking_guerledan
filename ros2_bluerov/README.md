# ROS 2 Workspace - BlueROV Docking

> **Documentation complète** : Voir [README principal](../README.md) pour installation, utilisation, troubleshooting et détails techniques.

Ce workspace implémente le pipeline ROS 2 pour le docking autonome du BlueROV2.

## Structure

```
ros2_bluerov/
├── src/
│   ├── sonar/              # Acquisition sonar
│   ├── traitement/         # Filtrage images
│   ├── tracking/           # Détection bords cage
│   ├── localisation/       # Estimation pose 3D
│   ├── control/            # Contrôleurs PID
│   ├── mission/            # Machine d'états FSM
│   ├── affichage/          # IHM Qt
│   ├── docking_msgs/       # Messages custom
│   ├── docking_utils/      # Utilitaires Python
│   └── bringup/            # Launch files
├── build/                  # Généré par colcon
└── install/                # Généré par colcon
```

## Compilation

```bash
cd ros2_bluerov
colcon build --symlink-install
source install/setup.bash
```

En cas d'erreur de génération IDL :
```bash
colcon build --packages-select docking_msgs && source install/setup.bash
colcon build --symlink-install && source install/setup.bash
```

## Packages

| Package | Description |
|---------|-------------|
| **sonar** | Driver Oculus M750d + mock |
| **traitement** | Filtrage images sonar |
| **tracking** | Détection montants cage (CSRT, Hough, Blob) |
| **localisation** | Estimation pose relative |
| **control** | PID cascade (surge, sway, yaw) |
| **mission** | FSM 7 états |
| **affichage** | Interface Qt temps réel |
| **docking_msgs** | Messages ROS custom |
| **docking_utils** | Bibliothèque utilitaires |
| **bringup** | Orchestration launch files |

Documentation détaillée : `src/<package>/README.md`

## Lancement

```bash
# Simulation complète
ros2 launch bringup user_pipeline.launch.py

# Sonar réel Oculus
ros2 launch bringup sonar_pipeline.launch.py

# Replay rosbag
ros2 launch bringup rosbag_pipeline.launch.py

# Système complet BlueROV
ros2 launch bringup complete_pipeline.launch.py
```

---

**Projet ENSTA Bretagne** - Guerlédan 2025/2026
