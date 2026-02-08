# ROS 2 Workspace - BlueROV Docking

Workspace ROS2 pour le docking autonome du BlueROV2 sur une cage aquacole. Ce projet utilise un sonar Oculus M750d pour détecter et suivre la cage, permettant un amarrage précis.

## Pipeline

Le système fonctionne en 4 étapes séquentielles :

```
┌─────────┐    ┌──────────────┐    ┌────────────┐    ┌─────────────┐
│ 1.SONAR │───▶│ 2.TRAITEMENT │───▶│ 3.TRACKING │───▶│ 4.AFFICHAGE │
└─────────┘    └──────────────┘    └────────────┘    └─────────────┘
```

1. **Sonar** : Acquisition des données sonar (réel ou mock) → publie l'image polaire brute
2. **Traitement** : Filtrage de l'image (Frost, médian, CLAHE, morpho) → conversion polaire → cartésien
3. **Tracking** : Détection de la cage par Hough (forme U) ou suivi visuel CSRT → publie la pose
4. **Affichage** : Visualisation temps réel PyQt5 + sélection manuelle de la bbox

## Packages

| Package | Description |
|---------|-------------|
| [sonar](src/sonar/) | Driver Oculus M750d + simulateur mock pour tests |
| [traitement](src/traitement/) | Pipeline de filtrage d'image (polaire → cartésien) |
| [tracking](src/tracking/) | Détection cage (Hough) et suivi visuel (CSRT) |
| [affichage](src/affichage/) | Interface PyQt5 de visualisation et contrôle |
| [docking_msgs](src/docking_msgs/) | Messages ROS2 custom (`Frame`, `TrackedObject`, etc.) |
| [bringup](src/bringup/) | Launch files pour orchestrer les nœuds |

> 📖 Chaque package contient son propre `README.md` avec les détails techniques.  
> 🚀 Voir [bringup/launch/](src/bringup/launch/) pour toutes les commandes de lancement.

## Installation

### Prérequis

- Ubuntu 22.04
- ROS2 Humble ([installation officielle](https://docs.ros.org/en/humble/Installation.html))
- Python 3.10

### 1. Cloner le projet

```bash
git clone <url-du-repo> ~/Desktop/Docking_guerledan
cd ~/Desktop/Docking_guerledan/ros2_bluerov
```

### 2. Installer les dépendances Python

```bash
pip install -r requirements.txt
```

### 3. Installer le SDK Oculus (requis pour le sonar réel)


Suivre les instructions : [oculus_python v1.2.1](https://github.com/ENSTABretagneRobotics/oculus_driver/tree/v1.2.1/python)


### 4. Compiler le workspace

```bash
cd ~/Desktop/Docking_guerledan/ros2_bluerov
colcon build --symlink-install
source install/setup.bash
```

> 💡 En cas d'erreur IDL, compiler d'abord les messages :
> ```bash
> colcon build --packages-select docking_msgs && source install/setup.bash
> colcon build --symlink-install
> ```

### 5. Tester l'installation

**Sans sonar (replay d'un rosbag) :**
```bash
source install/setup.bash
ros2 launch bringup replay_mission.launch.py bag_path:=<chemin_vers_rosbag>
```

**Avec sonar réel :**
```bash
source install/setup.bash
ros2 launch bringup sonar_pipeline.launch.py
```

---

**Projet ENSTA Bretagne** - Guerlédan 2025/2026
