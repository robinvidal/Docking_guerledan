# Traitement Package

Package de prétraitement et filtrage des données sonar brutes.

## Description

Ce package applique une chaîne de filtrage sur les frames sonar pour améliorer le rapport signal/bruit et préparer les données pour la détection des bords. Il utilise les filtres du package `docking_utils`.

## Nœuds

### `traitement_node`

Applique des filtres de traitement d'image sur les frames sonar brutes.

**Topics souscrits:**
- `/docking/sonar/raw` (`docking_msgs/Frame`) - Frames sonar brutes

**Topics publiés:**
- `/docking/sonar/filtered` (`docking_msgs/Frame`) - Frames filtrées

**Paramètres:**
- `enable_median` (bool, défaut: true) - Activer le filtre médian
- `median_kernel` (int, défaut: 3) - Taille du noyau médian
- `enable_gaussian` (bool, défaut: true) - Activer le lissage gaussien
- `gaussian_sigma` (float, défaut: 1.5) - Écart-type du filtre gaussien
- `enable_contrast` (bool, défaut: true) - Activer l'amélioration du contraste
- `contrast_clip` (float, défaut: 2.0) - Limite de clipping pour le contraste (%)
- `enable_range_comp` (bool, défaut: true) - Activer la compensation de distance
- `range_comp_alpha` (float, défaut: 0.0001) - Coefficient d'atténuation volumique (m⁻¹)

## Pipeline de filtrage

Le traitement s'effectue dans l'ordre suivant:

1. **Filtre médian** - Réduit le bruit impulsionnel (speckle)
2. **Filtre gaussien** - Lissage pour réduire le bruit haute fréquence
3. **Compensation de distance** - Corrige l'atténuation due à la propagation
4. **Amélioration du contraste** - Étirement d'histogramme pour maximiser la dynamique

Chaque étape peut être activée/désactivée indépendamment via paramètres.

## Filtres disponibles (via `docking_utils`)

- **`median_filter`** - Filtre médian pour bruit impulsionnel
- **`gaussian_filter`** - Lissage gaussien
- **`range_compensation`** - Correction spreading loss + atténuation
- **`contrast_enhancement`** - Étirement d'histogramme avec clipping
- **`morphological_opening`** - Nettoyage morphologique (disponible, non utilisé actuellement)

## Lancement

```bash
# Traitement seul
# Lancer le noeud traitement (chemin relatif dans le dépôt)
ros2 run traitement traitement_node --ros-args --params-file src/traitement/config/traitement_params.yaml

# Après `colcon build` (fichier installé dans `install/`)
ros2 run traitement traitement_node --ros-args --params-file install/traitement/share/traitement/config/traitement_params.yaml

# Avec le pipeline complet
ros2 launch bringup mock_pipeline.launch.py
```

## Configuration

Fichier: `config/traitement_params.yaml`

```yaml
traitement_node:
  ros__parameters:
    enable_median: true
    median_kernel: 3
    enable_gaussian: true
    gaussian_sigma: 1.5
    enable_contrast: true
    contrast_clip: 2.0
    enable_range_comp: true
    range_comp_alpha: 0.0001
```

## Performance

- Fréquence de traitement: ~10 Hz (synchronisé avec sonar)
- Latence ajoutée: <10 ms pour image 256×512

## TODO

- [ ] Ajout de filtres adaptatifs selon conditions (turbidité, etc.)
- [ ] Optimisation GPU pour temps réel haute résolution
- [ ] Métriques de qualité d'image en sortie
