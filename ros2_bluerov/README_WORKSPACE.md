# Workspace ROS2 - Docking Autonome BlueROV

Workspace ROS2 modulaire pour la mission de docking autonome du BlueROV dans une cage acoustique immergée (Lac de Guerlédan).

## Architecture

```
ros2_bluerov/
├── src/
│   ├── docking_msgs/       # Messages communs (Frame, Borders, PoseRelative, State)
│   ├── docking_utils/      # Bibliothèque utilitaires (conversions, filtres, géométrie)
│   ├── sonar/             # Driver Oculus M750d + mock
│   ├── traitement/        # Filtrage données sonar
│   ├── tracking/          # Détection bords cage
│   ├── localisation/      # Calcul pose relative
│   ├── control/           # Asservissement PID
│   ├── mission/           # Machine d'états
│   └── bringup/           # Launch files orchestration
```

## Flux de données

```
sonar → traitement → tracking → localisation → control → /cmd_vel
                                                  ↓
                                               mission ← état système
```

## Installation

### Prérequis

- ROS 2 Humble ou Iron
- Python 3.8+
- Dépendances Python: numpy, scipy

```bash
# Installer dépendances ROS2
sudo apt install ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-geometry-msgs

# Installer dépendances Python
pip3 install numpy scipy
```

### Build

```bash
cd ros2_bluerov
colcon build --symlink-install
source install/setup.zsh  # ou setup.bash
```

**Ordre de build recommandé** (si problèmes):
```bash
colcon build --packages-select docking_msgs
colcon build --packages-select docking_utils
colcon build  # Tous les autres
```

## Démarrage rapide

### 1. Pipeline complet en mode simulation

```bash
# Terminal 1: Lancer tous les nœuds
ros2 launch bringup mock_pipeline.launch.py

# Terminal 2: Surveiller l'état de la mission
ros2 topic echo /docking/mission/state

# Terminal 3: Visualiser le graphe
rqt_graph
```

### 2. Test de détection seule

```bash
ros2 launch bringup detection_pipeline.launch.py

# Vérifier détection bords
ros2 topic echo /docking/tracking/borders
```

### 3. Utiliser les tasks VSCode

Appuyer sur `Cmd+Shift+B` (macOS) ou `Ctrl+Shift+B` (Linux/Windows), puis choisir:
- `colcon: build all` - Build tout
- `colcon: test all` - Exécuter tests
- `launch: mock pipeline` - Lancer simulation

## Packages

### Fondations

#### docking_msgs
Messages ROS2 personnalisés pour le système.

**Messages**:
- `Frame`: Données sonar brutes
- `Borders`: Bords cage détectés
- `PoseRelative`: Pose 6DOF du ROV
- `State`: État machine d'états

**Voir**: [src/docking_msgs/README.md](src/docking_msgs/README.md)

#### docking_utils
Bibliothèque Python réutilisable.

**Modules**:
- `conversions`: Polaire↔cartésien, angles, repères
- `filters`: Médian, gaussien, morpho, Wiener, contraste
- `geometry`: Validation cage, calculs pose, trajectoires
- `tf_utils`: Transformations TF2, quaternions

**Voir**: [src/docking_utils/README.md](src/docking_utils/README.md)

### Nœuds métiers

#### sonar
- `sonar_mock`: Génère frames synthétiques (développement)
- `sonar_node`: Driver Oculus M750d (TODO)

**Publie**: `/docking/sonar/raw`

#### traitement
- `traitement_node`: Filtrage (médian, gaussien, compensation portée, contraste)

**Souscrit**: `/docking/sonar/raw`  
**Publie**: `/docking/sonar/filtered`

#### tracking
- `tracking_node`: Détecte 4 montants verticaux de la cage

**Souscrit**: `/docking/sonar/filtered`  
**Publie**: `/docking/tracking/borders`

#### localisation
- `localisation_node`: Calcule pose 6DOF avec covariance

**Souscrit**: `/docking/tracking/borders`  
**Publie**: `/docking/localisation/pose`

#### control
- `control_node`: Asservissement PID (x, y, yaw)

**Souscrit**: `/docking/localisation/pose`, `/docking/mission/state`  
**Publie**: `/cmd_vel`

#### mission
- `mission_node`: Machine d'états (IDLE → LOCK_ON → APPROACH → DOCKING → DOCKED)

**Souscrit**: `/docking/tracking/borders`, `/docking/localisation/pose`  
**Publie**: `/docking/mission/state`

#### bringup
Launch files pour différents scénarios.

**Launch files**:
- `mock_pipeline.launch.py`: Pipeline complet mock
- `detection_pipeline.launch.py`: Sonar + traitement + tracking
- `sonar_only.launch.py`: Sonar seul

## Tests

### Tests unitaires

```bash
# Tous les tests
colcon test
colcon test-result --verbose

# Package spécifique
colcon test --packages-select docking_utils
```

### Tests manuels

```bash
# Vérifier fréquence publication
ros2 topic hz /docking/sonar/raw

# Afficher un message
ros2 topic echo /docking/localisation/pose --once

# Lister paramètres d'un nœud
ros2 param list /tracking_node
```

### Enregistrement/replay

```bash
# Enregistrer session
ros2 bag record -a  # Tous les topics
# ou
ros2 bag record /docking/sonar/raw /docking/tracking/borders

# Rejouer
ros2 bag play <nom_du_bag>
```

## Configuration

Chaque package a ses paramètres dans `config/<package>_params.yaml`.

**Paramètres clés**:

- **sonar_mock**: `cage_distance`, `cage_width`, `noise_level`
- **tracking**: `intensity_threshold`, `min_confidence`
- **localisation**: `cage_width`, `cage_depth`
- **control**: PID gains (`pid_x_kp`, `pid_y_kp`, `pid_yaw_kp`)
- **mission**: `approach_distance`, `docking_distance`, `alignment_threshold`

Modifier et relancer:
```bash
ros2 launch bringup mock_pipeline.launch.py
```

## Développement

### Ajouter un nouveau filtre dans docking_utils

1. Éditer `src/docking_utils/docking_utils/filters.py`
2. Ajouter fonction avec docstring + exemple
3. Créer test dans `test/test_filters.py`
4. Build et tester:
```bash
colcon build --packages-select docking_utils
pytest src/docking_utils/test/test_filters.py -v
```

### Modifier un nœud

1. Éditer `src/<package>/<package>/<node>.py`
2. Rebuild:
```bash
colcon build --packages-select <package> --symlink-install
```
3. Relancer (pas besoin de rebuild si `--symlink-install` utilisé)

### Lint

```bash
# Python
flake8 src/ --exclude build,install,log

# ROS2 lint
colcon test --packages-select <package> --ctest-args -R lint
```

## Troubleshooting

### `colcon build` échoue sur docking_msgs

**Cause**: rosidl_default_generators manquant

**Solution**:
```bash
sudo apt install ros-${ROS_DISTRO}-rosidl-default-generators
```

### Imports Python ne fonctionnent pas

**Cause**: Environnement ROS2 non sourcé

**Solution**:
```bash
source ros2_bluerov/install/setup.zsh
```

### Nœuds ne communiquent pas

**Vérifier**:
```bash
ros2 topic list  # Topics publiés?
ros2 node list   # Nœuds actifs?
ros2 topic info /docking/sonar/raw  # Publishers/Subscribers?
```

### Pipeline mock ne détecte pas la cage

**Vérifier**:
1. `ros2 topic echo /docking/sonar/raw` → données publiées?
2. `ros2 topic echo /docking/tracking/borders` → `is_valid: true`?
3. Ajuster `intensity_threshold` dans `tracking/config/tracking_params.yaml`

## Prochaines étapes

### Implémentation prioritaire

1. **Driver Oculus réel** (`sonar/sonar_node.py`)
   - Intégrer SDK Oculus
   - Conversion données propriétaires → Frame

2. **Amélioration tracking** (`tracking/tracking_node.py`)
   - Algorithme robuste (template matching, Hough transform)
   - Tracking temporel (filtre particules)

3. **Filtrage Kalman** (`localisation/localisation_node.py`)
   - Fusion pose sonar + IMU + DVL
   - Prédiction/correction

4. **Tuning PID** (`control/control_node.py`)
   - Auto-tuning Ziegler-Nichols
   - Anti-windup amélioré
   - Feedforward

5. **Tests en bassin**
   - Validation avec cage réelle
   - Réglage paramètres environnement réel

### Fonctionnalités avancées

- Package `affichage`: Visualisation temps réel (RViz/Qt)
- Services ROS2: start_mission, abort_mission, set_target_pose
- Diagnostics: `/diagnostics` pour monitoring santé système
- Safety: watchdog, failsafes automatiques

## Contribution

### Workflow Git

```bash
# Créer branche feature
git checkout -b feature/nom-feature

# Développer, committer
git add .
git commit -m "Description"

# Pousser et créer PR
git push origin feature/nom-feature
```

### Standards

- **Code**: PEP8 (Python), Google Style (C++)
- **Commits**: Messages descriptifs (type: description)
- **Tests**: Couverture > 80% pour docking_utils
- **Docs**: Docstrings + README par package

## Licence

Apache 2.0

## Contact

Maxime Lefevre - maxime.lefevre@example.com
