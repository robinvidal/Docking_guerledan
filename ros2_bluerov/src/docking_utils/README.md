# docking_utils

Bibliothèque de fonctions utilitaires communes pour le système de docking autonome du BlueROV.

## Modules

### conversions.py
Transformations de coordonnées et conversions géométriques:
- `polar_to_cartesian()`: Polaire → Cartésien 2D
- `cartesian_to_polar()`: Cartésien → Polaire 2D
- `normalize_angle()`: Normalisation angles dans [-π, π]
- `sonar_to_body_frame()`: Transformation repère sonar → corps ROV
- `interpolate_sonar_data()`: Interpolation angulaire données sonar

### filters.py
Filtres de traitement de signal pour images sonar:
- `median_filter()`: Réduction bruit impulsionnel
- `gaussian_filter()`: Lissage gaussien
- `morphological_opening()`: Élimination petits objets
- `morphological_closing()`: Remplissage trous
- `adaptive_threshold()`: Seuillage adaptatif local
- `wiener_filter()`: Débruitage avec préservation contours
- `contrast_enhancement()`: Amélioration contraste
- `range_compensation()`: Compensation atténuation distance

### geometry.py
Validation et calculs géométriques cage:
- `validate_cage_geometry()`: Vérification cohérence 4 bords
- `compute_cage_center()`: Position centre cage
- `compute_cage_orientation()`: Orientation (yaw) cage
- `check_collision_risk()`: Détection risque collision
- `estimate_approach_trajectory()`: Génération waypoints approche

### tf_utils.py
Utilitaires transformations TF2 ROS:
- `euler_to_quaternion()`: Euler → Quaternion
- `quaternion_to_euler()`: Quaternion → Euler
- `create_transform()`: Création TransformStamped
- `pose_from_xyyaw()`: Pose 2D + yaw → Pose ROS
- `transform_point()`: Application transformation à point 3D

## Installation

```bash
cd ros2_bluerov
colcon build --packages-select docking_utils
source install/setup.zsh
```

## Dépendances

- Python 3.8+
- NumPy
- SciPy
- ROS 2 (rclpy, geometry_msgs)
- docking_msgs

## Tests

```bash
# Tous les tests
colcon test --packages-select docking_utils
colcon test-result --verbose

# Tests spécifiques
pytest src/docking_utils/test/test_conversions.py -v
pytest src/docking_utils/test/test_filters.py -v
pytest src/docking_utils/test/test_geometry.py -v
```

## Usage

### Exemple: Conversion polaire/cartésien

```python
from docking_utils.conversions import polar_to_cartesian, cartesian_to_polar

# Sonar détecte objet à 10m, 30° gauche
range_m = 10.0
bearing_rad = np.deg2rad(30)

x, y = polar_to_cartesian(range_m, bearing_rad)
print(f"Position cartésienne: x={x:.2f}m, y={y:.2f}m")

# Conversion inverse
r, theta = cartesian_to_polar(x, y)
print(f"Vérifié: range={r:.2f}m, bearing={np.rad2deg(theta):.1f}°")
```

### Exemple: Filtrage image sonar

```python
from docking_utils.filters import median_filter, contrast_enhancement
import numpy as np

# Image sonar bruitée (shape: [bearings, ranges])
sonar_image = np.random.randint(0, 255, (256, 512), dtype=np.uint8)

# Pipeline de filtrage
filtered = median_filter(sonar_image, kernel_size=5)
enhanced = contrast_enhancement(filtered, clip_limit=2.0)
```

### Exemple: Validation géométrie cage

```python
from docking_utils.geometry import validate_cage_geometry, compute_cage_center

# Bords détectés par tracking
ranges = np.array([5.2, 5.0, 5.1, 5.3])  # Distances (m)
bearings = np.deg2rad(np.array([-8, -4, 4, 8]))  # Angles (rad)

# Validation
valid, msg = validate_cage_geometry(ranges, bearings, 
                                   expected_width=2.0, 
                                   expected_depth=2.0)
if valid:
    x_c, y_c = compute_cage_center(ranges, bearings)
    print(f"Centre cage: x={x_c:.2f}m, y={y_c:.2f}m")
else:
    print(f"Géométrie invalide: {msg}")
```

### Exemple: Transformations TF

```python
from docking_utils.tf_utils import euler_to_quaternion, pose_from_xyyaw

# Créer une pose 2D
x, y, yaw = 5.0, 10.0, np.deg2rad(45)
pose = pose_from_xyyaw(x, y, yaw)

# Ou quaternion directement
q = euler_to_quaternion(0.0, 0.0, yaw)
print(f"Quaternion: w={q.w:.3f}, z={q.z:.3f}")
```

## Contribution

Les fonctions utilitaires doivent:
- Être pures (sans état) autant que possible
- Inclure docstrings avec exemples
- Avoir des tests unitaires (couverture > 80%)
- Gérer les cas limites (divisions par zéro, arrays vides, etc.)

## Architecture

Ce package est une dépendance **de bibliothèque uniquement** (pas de nœuds ROS). Il peut être importé par tous les packages métiers (sonar, traitement, tracking, localisation, control) sans créer de dépendances circulaires.

```
docking_msgs ──┐
               ├──> docking_utils ──> [sonar, traitement, tracking, ...]
std_msgs ──────┘
```

## License

Apache 2.0
