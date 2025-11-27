# Quick Start - Workspace ROS2 Docking

Guide de démarrage rapide pour le workspace de docking autonome.

## 🚀 Démarrage en 5 minutes

### 1. Vérifier l'installation ROS2

```bash
# Vérifier version ROS2
echo $ROS_DISTRO  # Doit afficher: humble, iron, ou jazzy

# Si vide, sourcer ROS2
source /opt/ros/humble/setup.zsh  # Adapter selon votre distro
```

### 2. Build le workspace

```bash
cd ros2_bluerov
colcon build --symlink-install
```

**Temps estimé**: 1-2 minutes

### 3. Sourcer l'environnement

```bash
source install/setup.zsh  # ou setup.bash
```

### 4. Lancer la simulation

```bash
ros2 launch bringup mock_pipeline.launch.py
```

**Vous devriez voir**:
```
[sonar_mock]: Sonar mock démarré: 10.0 Hz, cage @ 8.0m
[traitement_node]: Traitement node démarré
[tracking_node]: Tracking node démarré
[localisation_node]: Localisation node démarré
[control_node]: Contrôle activé
[mission_node]: Transition: IDLE -> LOCK_ON (cage détectée)
[mission_node]: Transition: LOCK_ON -> APPROACH (pose valide acquise)
```

### 5. Observer les données

Dans un nouveau terminal:

```bash
# Sourcer d'abord
source ros2_bluerov/install/setup.zsh

# Voir l'état de la mission
ros2 topic echo /docking/mission/state

# Voir la pose du ROV
ros2 topic echo /docking/localisation/pose

# Voir les commandes envoyées
ros2 topic echo /cmd_vel
```

## ✅ Vérifications

### Tous les topics sont publiés

```bash
ros2 topic list
```

Doit contenir:
- `/docking/sonar/raw`
- `/docking/sonar/filtered`
- `/docking/tracking/borders`
- `/docking/localisation/pose`
- `/docking/mission/state`
- `/cmd_vel`

### Les nœuds sont actifs

```bash
ros2 node list
```

Doit afficher:
- `/sonar_mock`
- `/traitement_node`
- `/tracking_node`
- `/localisation_node`
- `/control_node`
- `/mission_node`

### La mission progresse

```bash
ros2 topic echo /docking/mission/state | grep current_state
```

Vous devriez voir la progression:
```
current_state: 0  # IDLE
current_state: 1  # LOCK_ON
current_state: 2  # APPROACH
current_state: 3  # DOCKING
current_state: 4  # DOCKED
```

## 📊 Visualisation

### Graphe des nœuds

```bash
rqt_graph
```

### PlotJuggler (analyser signaux)

```bash
# Installer si nécessaire
sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros

# Lancer
ros2 run plotjuggler plotjuggler
```

Puis:
1. Streaming → ROS2 Topics
2. Sélectionner `/docking/localisation/pose`
3. Glisser `x`, `y`, `yaw` vers le graphe

## 🛠️ Commandes utiles

### Tester un seul package

```bash
# Build
colcon build --packages-select docking_utils

# Test
colcon test --packages-select docking_utils
colcon test-result --verbose
```

### Modifier paramètres

```bash
# Éditer
nano src/sonar/config/sonar_params.yaml

# Changer par exemple cage_distance: 15.0

# Relancer (pas besoin de rebuild)
ros2 launch bringup mock_pipeline.launch.py
```

### Nettoyer et rebuild

```bash
rm -rf build/ install/ log/
colcon build --symlink-install
```

## 🐛 Problèmes courants

### Erreur: `colcon: command not found`

**Solution**:
```bash
sudo apt install python3-colcon-common-extensions
```

### Erreur: `package 'docking_msgs' not found`

**Solution**:
```bash
# Build messages d'abord
colcon build --packages-select docking_msgs
source install/setup.zsh
# Puis build le reste
colcon build
```

### Erreur: `No module named 'docking_utils'`

**Solution**: Environnement ROS2 non sourcé
```bash
source ros2_bluerov/install/setup.zsh
```

### Aucun topic publié

**Causes possibles**:
1. Nœud n'a pas démarré → vérifier logs
2. Mauvais namespace → vérifier launch file
3. Problème de build → rebuild le package

**Debug**:
```bash
ros2 node list  # Nœuds actifs?
ros2 topic list  # Topics créés?
ros2 run <package> <node>  # Lancer nœud directement avec logs
```

## 📚 Aller plus loin

### Documentation complète

- Workspace: [ros2_bluerov/README_WORKSPACE.md](ros2_bluerov/README_WORKSPACE.md)
- Messages: [ros2_bluerov/src/docking_msgs/README.md](ros2_bluerov/src/docking_msgs/README.md)
- Utilitaires: [ros2_bluerov/src/docking_utils/README.md](ros2_bluerov/src/docking_utils/README.md)
- Bringup: [ros2_bluerov/src/bringup/README.md](ros2_bluerov/src/bringup/README.md)

### Modifier le code

1. **Éditer un nœud**: `src/<package>/<package>/<node>.py`
2. **Rebuild**: `colcon build --packages-select <package> --symlink-install`
3. **Tester**: `ros2 run <package> <node>`

Avec `--symlink-install`, pas besoin de rebuild après modification Python!

### Lancer un scénario spécifique

```bash
# Seulement détection (sonar → tracking)
ros2 launch bringup detection_pipeline.launch.py

# Seulement sonar
ros2 launch bringup sonar_only.launch.py
```

### Utiliser VSCode

Ouvrir le workspace dans VSCode:
```bash
code /Users/maximelefevre/Desktop/Docking_guerledan
```

Puis:
- `Cmd+Shift+B` → Choisir tâche (build, test, launch)
- Terminal intégré déjà configuré

## 🎯 Prochaines étapes

1. ✅ Familiarisation avec pipeline mock
2. Modifier paramètres cage (distance, largeur)
3. Visualiser données avec PlotJuggler
4. Tester pipeline de détection seul
5. Enregistrer session avec `ros2 bag`
6. Implémenter amélioration (ex: nouveau filtre)

---

**Bon développement! 🚁💧**
