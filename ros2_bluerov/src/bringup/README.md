# Bringup Package

Package de lancement et configuration pour le système de docking autonome.

## Description

Ce package regroupe les fichiers de lancement (launch files) et les configurations (YAML) pour démarrer l'ensemble du pipeline de docking. Il permet de lancer tous les nœuds avec leurs paramètres de manière coordonnée.

## Launch files

### `mock_pipeline.launch.py`

Lance le pipeline complet en mode simulation (avec sonar mock).

**Nœuds lancés:**
1. `sonar_mock` - Simulation du sonar Oculus M750d
2. `traitement_node` - Filtrage des images sonar
3. `tracking_node` - Détection des bords de cage
4. `localisation_node` - Calcul de pose relative
5. `control_node` - Asservissement PID
6. `mission_node` - Machine d'états

**Usage:**
```bash
cd ~/ros2_bluerov
source install/setup.bash
ros2 launch bringup mock_pipeline.launch.py
```

**Tous les nœuds démarrent automatiquement avec leurs configurations par défaut.**

### Configuration des paramètres

Chaque nœud charge ses paramètres depuis un fichier YAML dans le package correspondant:

- `sonar/config/sonar_params.yaml`
- `traitement/config/traitement_params.yaml`
- `tracking/config/tracking_params.yaml`
- `localisation/config/localisation_params.yaml`
- `control/config/control_params.yaml`
- `mission/config/mission_params.yaml`

## Topics principaux

```
/docking/sonar/raw          [docking_msgs/Frame]
    ↓
/docking/sonar/filtered     [docking_msgs/Frame]
    ↓
/docking/tracking/borders   [docking_msgs/Borders]
    ↓
/docking/localisation/pose  [docking_msgs/PoseRelative]
    ↓ + /docking/mission/state
/cmd_vel                    [geometry_msgs/Twist]
```

## Démarrage rapide

### 1. Build du workspace

```bash
cd ~/ros2_bluerov
colcon build
source install/setup.bash
```

### 2. Lancement du pipeline mock

```bash
ros2 launch bringup mock_pipeline.launch.py
```

Vous devriez voir:
```
[sonar_mock]: Sonar mock démarré: 10.0 Hz, cage @ 8.0m
[traitement_node]: Traitement node démarré
[tracking_node]: Tracking node démarré
[localisation_node]: Localisation node démarré
[control_node]: Control node démarré
[mission_node]: Mission node démarré (état: IDLE)
```

### 3. Monitoring

Dans un autre terminal:
```bash
# Observer l'état de la mission
ros2 topic echo /docking/mission/state

# Observer la pose estimée
ros2 topic echo /docking/localisation/pose

# Observer les commandes de contrôle
ros2 topic echo /cmd_vel

# Visualiser les topics
ros2 run rqt_graph rqt_graph
```

## Modes de fonctionnement

### Mode Mock (simulation complète)
```bash
ros2 launch bringup mock_pipeline.launch.py
```
- Sonar simulé avec cage virtuelle
- Développement sans matériel
- Test des algorithmes

### Mode Hardware (TODO)
```bash
ros2 launch bringup hardware_pipeline.launch.py
```
- Sonar réel Oculus M750d
- Interface BlueROV
- Déploiement terrain

## Debug

### Problème: Nœuds ne démarrent pas

Vérifiez:
```bash
# Dépendances installées
pip install numpy scipy

# Workspace buildé
colcon build --packages-select docking_msgs docking_utils sonar traitement tracking localisation control mission

# Source setup
source install/setup.bash
```

### Problème: Pas de détection de cage

```bash
# Vérifier données sonar
ros2 topic hz /docking/sonar/raw
ros2 topic echo /docking/sonar/raw --once

# Vérifier tracking
ros2 topic echo /docking/tracking/borders
```

### Problème: Contrôle ne fonctionne pas

```bash
# Vérifier état mission
ros2 topic echo /docking/mission/state

# Le contrôle n'est actif que dans les états APPROACH et DOCKING
```

## Visualisation

### RViz (TODO)
```bash
ros2 launch bringup rviz.launch.py
```

### PlotJuggler
```bash
ros2 run plotjuggler plotjuggler
# Charger les topics: /docking/localisation/pose, /cmd_vel, etc.
```

## Architecture complète

```
┌──────────────┐
│  sonar_mock  │ Génère frames synthétiques
└──────┬───────┘
       │ /docking/sonar/raw
       ▼
┌──────────────────┐
│ traitement_node  │ Filtre les images
└──────┬───────────┘
       │ /docking/sonar/filtered
       ▼
┌──────────────────┐
│  tracking_node   │ Détecte les bords
└──────┬───────────┘
       │ /docking/tracking/borders
       ▼
┌────────────────────┐
│ localisation_node  │ Calcule la pose
└──────┬─────────────┘
       │ /docking/localisation/pose
       ▼
┌──────────────┐         ┌──────────────┐
│ mission_node │────────▶│ control_node │
└──────────────┘         └──────┬───────┘
 /docking/mission/state          │ /cmd_vel
                                 ▼
                          [ ROV Thrusters ]
```

## TODO

- [ ] Launch file pour hardware (sonar réel)
- [ ] Configuration RViz avec visualisation sonar
- [ ] Scripts de diagnostic automatique
- [ ] Recording/replay de données réelles
- [ ] Launch file pour tests unitaires
