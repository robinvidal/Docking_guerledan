# Traitement Package

Package de prétraitement et filtrage des données sonar pour le système de docking.

## Description

Ce package applique une chaîne de filtrage multi-étapes sur les frames sonar :
1. **Filtrage polaire** - Traitement sur données brutes (bearing × range)
2. **Conversion cartésienne** - Transformation polaire → cartésien (x × y)
3. **Filtres cartésiens** - Filtres spatiaux et morphologiques
4. **Filtre spatial gaussien** - Isolation de la zone trackée (nouveau - Étape 1)
5. **Opérations morphologiques** - Nettoyage de l'image (nouveau - Étape 2)

## Nœuds

### `traitement_polar_node`

Applique des filtres sur les frames sonar en coordonnées polaires.

**Topics souscrits:**
- `/docking/sonar/raw` (`docking_msgs/Frame`) - Frames sonar brutes (polaires)

**Topics publiés:**
- `/docking/sonar/polar_filtered` (`docking_msgs/Frame`) - Frames filtrées (polaires)

**Paramètres principaux:**
- `enable_median` (bool, défaut: false) - Filtre médian anti-bruit
- `enable_gaussian` (bool, défaut: false) - Lissage gaussien
- `enable_dog_enhance` (bool, défaut: true) - Amélioration DoG (Difference of Gaussians)
- `enable_so_cfar` (bool, défaut: false) - Détection CFAR (Constant False Alarm Rate)

### `traitement_cartesian_node` ⭐ **Nouveau**

Convertit les données polaires en cartésiennes et applique des filtres avancés.

**Topics souscrits:**
- `/docking/sonar/polar_filtered` (`docking_msgs/Frame`) - Frames filtrées polaires
- `/docking/tracking/tracked_object` (`docking_msgs/TrackedObject`) - Position de la cage trackée

**Topics publiés:**
- `/docking/sonar/cartesian_filtered` (`docking_msgs/FrameCartesian`) - Image cartésienne filtrée

**Paramètres principaux:**

*Conversion:*
- `cartesian_scale_factor` (float, défaut: 2.0) - Facteur d'échelle largeur image
- `subscribe_to_filtered` (bool, défaut: true) - S'abonner aux données filtrées polaires

*Filtre spatial gaussien (ÉTAPE 1):*
- `enable_spatial_filter` (bool, défaut: false) - Activer le filtre spatial
- `spatial_filter_radius` (float, défaut: 2.0 m) - Rayon d'effet
- `spatial_filter_sigma` (float, défaut: 0.8 m) - Écart-type gaussien (contrôle la transition)

*Opérations morphologiques (ÉTAPE 2):*
- `cart_enable_morphology` (bool, défaut: false) - Activer nettoyage morphologique
- `cart_morphology_kernel_size` (int, défaut: 3) - Taille kernel (impair)
- `cart_morphology_iterations` (int, défaut: 2) - Nombre d'itérations closing

*Détection de contours:*
- `cart_enable_canny` (bool, défaut: false) - Détection contours Canny
- `cart_canny_threshold1` (float, défaut: 50.0) - Seuil bas Canny
- `cart_canny_threshold2` (float, défaut: 150.0) - Seuil haut Canny

*Binarisation:*
- `cart_enable_percentile_binarization` (bool, défaut: false) - Binarisation par centiles
- `cart_percentile_threshold` (float, défaut: 90.0) - Centile (garde les N% plus lumineux)

## Pipeline de filtrage complet

```
sonar_node/sonar_mock
  ↓ /docking/sonar/raw (Frame polaire)
traitement_polar_node
  ↓ /docking/sonar/polar_filtered (Frame polaire filtrée)
traitement_cartesian_node
  ├─ Conversion polaire → cartésien
  ├─ Filtre spatial gaussien (si tracker actif) ← ÉTAPE 1
  ├─ Opérations morphologiques (closing/opening) ← ÉTAPE 2
  ├─ Détection Canny (optionnel)
  └─ Binarisation (optionnel)
  ↓ /docking/sonar/cartesian_filtered (FrameCartesian)
csrt_tracker_node / hough_lines_node
```

## Filtre Spatial Gaussien (ÉTAPE 1)

Le filtre spatial gaussien permet d'**isoler la zone d'intérêt autour de la cage trackée** :

- **Principe** : Applique un masque gaussien centré sur la position de la cage
- **Effet** : Atténue progressivement les intensités en dehors de la zone (transition douce)
- **Activation** : Nécessite que le tracker soit actif et publie sur `/docking/tracking/tracked_object`
- **Formule** : `mask = exp(-(dist² / (2σ²)))` où dist = distance à la cage

**Cas d'usage** : Éliminer les échos parasites et focaliser la détection Hough sur la cage uniquement.

## Opérations Morphologiques (ÉTAPE 2)

Nettoyage de l'image binaire avant détection de lignes :

- **Closing** : Ferme les petits trous dans les objets (dilatation puis érosion)
- **Opening** : Supprime les petits bruits isolés (érosion puis dilatation)

**Cas d'usage** : Améliorer la robustesse de la transformée de Hough en éliminant les discontinuités.

## Lancement

### Pipeline complet (avec mock sonar)

```bash
# Terminal 1 - Lancer le pipeline
source install/setup.bash
ros2 launch bringup user_pipeline.launch.py
```

### Activation dynamique des filtres

```bash
# Terminal 2 - Activer le filtre spatial après avoir initialisé le tracker
ros2 param set /traitement_cartesian_node enable_spatial_filter true
ros2 param set /traitement_cartesian_node spatial_filter_sigma 1.0

# Terminal 3 - Activer la morphologie
ros2 param set /traitement_cartesian_node cart_enable_morphology true
```

### Nœud individuel

```bash
# Traitement polaire seul
ros2 run traitement traitement_polar_node --ros-args \
  --params-file install/traitement/share/traitement/config/traitement_params.yaml

# Traitement cartésien seul
ros2 run traitement traitement_cartesian_node --ros-args \
  --params-file install/traitement/share/traitement/config/traitement_params.yaml
```

## Configuration

Fichier: `config/traitement_params.yaml`

```yaml
traitement_polar_node:
  ros__parameters:
    enable_dog_enhance: true
    dog_sigma1: 1.8
    dog_sigma2: 10.0

traitement_cartesian_node:
  ros__parameters:
    # Conversion
    cartesian_scale_factor: 2.0
    subscribe_to_filtered: true
    
    # Filtre spatial gaussien (ÉTAPE 1)
    enable_spatial_filter: false        # Activer quand tracker utilisé
    spatial_filter_radius: 2.0          # Rayon d'effet en mètres
    spatial_filter_sigma: 0.8           # Transition progressive
    
    # Opérations morphologiques (ÉTAPE 2)
    cart_enable_morphology: false
    cart_morphology_kernel_size: 3
    cart_morphology_iterations: 2
    
    # Détection contours
    cart_enable_canny: false
    cart_canny_threshold1: 50.0
    cart_canny_threshold2: 150.0
```

## Dépendances

- `docking_msgs` - Messages custom (Frame, FrameCartesian, TrackedObject)
- `docking_utils` - Filtres de traitement d'image
- `scipy` - Conversion polaire→cartésien (map_coordinates)
- `opencv-python` (cv2) - Morphologie et Canny (optionnel)

## Notes de développement

### Système de coordonnées

⚠️ **Important** : Le tracker travaille sur une image flippée horizontalement (`cv2.flip`), donc :
- `TrackedObject.center_x` est dans l'espace de l'image flippée
- Le filtre spatial applique la correction : `center_x_real = -center_x_tracker`

### Cache d'optimisation

Le filtre spatial utilise un cache pour éviter de recalculer le masque gaussien :
- Invalidation si position change de >5cm ou si dimension image change
- Gain de performance : ~70% pour position stable

