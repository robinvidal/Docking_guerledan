# Tracking Package

Package de détection et suivi des bords de la cage dans les images sonar.

## Description

Ce package détecte les 2 montants verticaux de la cage d'amarrage à partir des frames sonar filtrées. Il utilise une approche par projection angulaire et détection de pics pour identifier les bords.

## Nœuds

### `tracking_node`

Détecte les bords de la cage dans les frames sonar et publie leurs positions polaires.

**Topics souscrits:**
- `/docking/sonar/filtered` (`docking_msgs/Frame`) - Frames sonar filtrées

**Topics publiés:**
- `/docking/tracking/borders` (`docking_msgs/Borders`) - Bords détectés avec positions et confidences

**Paramètres:**
- `intensity_threshold` (int, défaut: 150) - Seuil d'intensité pour détection de pics
- `min_confidence` (float, défaut: 0.7) - Confiance minimale pour valider la détection
- `expected_cage_width` (float, défaut: 2.0) - Largeur attendue de la cage (m)
- `search_range_min` (float, défaut: 2.0) - Distance minimale de recherche (m)
- `search_range_max` (float, défaut: 20.0) - Distance maximale de recherche (m)

## Algorithme de détection

1. **Limitation de zone** - Restriction aux distances [search_range_min, search_range_max]
2. **Projection angulaire** - Somme des intensités sur les ranges pour chaque bearing
3. **Détection de pics** - Identification des maxima locaux dépassant le seuil
2. **Sélection des 2 montants** - Tri par intensité et sélection des 2 pics les plus forts
5. **Estimation de distance** - Pour chaque bearing, détection du pic radial
6. **Calcul de confiance** - Basé sur l'intensité normalisée de chaque montant
7. **Validation géométrique** - Vérification cohérence avec largeur attendue

## Message de sortie

Le message `Borders` contient:
- **ranges[]** - Distances des 2 montants (m)
- **bearings[]** - Angles des 2 montants (rad)
- **confidences[]** - Confiance de détection pour chaque montant [0-1]
- **is_valid** - Validité globale de la détection (booléen)
- **cage_width** - Largeur estimée de la cage (m)
- **cage_depth** - Distance moyenne de la cage (m)

## Lancement

```bash
# Tracking seul
ros2 run tracking tracking_node --ros-args --params-file config/tracking_params.yaml

# Avec le pipeline complet
ros2 launch bringup mock_pipeline.launch.py
```

## Configuration

Fichier: `config/tracking_params.yaml`

```yaml
tracking_node:
  ros__parameters:
    intensity_threshold: 150
    min_confidence: 0.7
    expected_cage_width: 2.0
    search_range_min: 2.0
    search_range_max: 20.0
```

## Performance

- Fréquence: ~10 Hz (synchronisé avec sonar)
- Robustesse: Détection stable jusqu'à 15m en conditions normales
- Faux positifs: Rares grâce au filtrage par largeur attendue

## TODO

- [ ] Filtrage temporel (Kalman) pour lisser les détections
- [ ] Sélection initiale par opérateur (région d'intérêt)
- [ ] Détection multi-hypothèses pour environnements encombrés
- [ ] Métriques de qualité de tracking
