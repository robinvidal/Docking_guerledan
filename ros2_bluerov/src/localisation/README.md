# Localisation Package

Package de calcul de la pose relative du ROV par rapport à la cage d'amarrage.

## Description

Ce package convertit les détections de bords (coordonnées polaires) en une pose 6DOF relative du ROV dans le repère de la cage. Il valide la géométrie détectée et estime la position, l'orientation et les incertitudes associées.

## Nœuds

### `localisation_node`

Calcule la pose relative à partir des bords détectés par le tracking.

**Topics souscrits:**
- `/docking/tracking/borders` (`docking_msgs/Borders`) - Bords détectés

**Topics publiés:**
- `/docking/localisation/pose` (`docking_msgs/PoseRelative`) - Pose relative 6DOF avec covariance

**Paramètres:**
- `cage_width` (float, défaut: 2.0) - Largeur nominale de la cage (m)
- `cage_depth` (float, défaut: 2.0) - Profondeur nominale de la cage (m)
- `min_borders_confidence` (float, défaut: 0.7) - Confiance minimale pour valider la pose

## Algorithme de localisation

1. **Validation des entrées** - Vérification de 2 bords valides avec confiance suffisante
2. **Validation géométrique** - Cohérence avec dimensions attendues (via `docking_utils.geometry`)
3. **Conversion polaire → cartésien** - Transformation en coordonnées métriques
4. **Calcul du centre** - Position moyenne des 2 montants
5. **Calcul d'orientation** - Angle de la cage par régression linéaire sur les colonnes
6. **Transformation de repère** - Inversion pour obtenir pose ROV dans repère cage
7. **Estimation de covariance** - Matrice 6×6 basée sur distance et confiance

## Message de sortie

Le message `PoseRelative` contient:
- **Position** - (x, y, z) en mètres dans le repère cage
  - x: décalage latéral (+ = droite)
  - y: distance frontale (+ = devant cage)
  - z: décalage vertical (0 car sonar 2D)
- **Orientation** - (roll, pitch, yaw) en radians
  - yaw: angle d'approche (0 = frontal)
  - roll/pitch: non disponibles avec sonar 2D
- **confidence** - Confiance globale [0-1]
- **is_valid** - Validité de la pose (booléen)
- **covariance[]** - Matrice 6×6 flatten (incertitudes)

## Repères et conventions

```
Repère cage (target):
  x: latéral (+ = droite)
  y: frontal (+ = vers l'avant de la cage)
  z: vertical (+ = haut)

Repère ROV (body):
  Convention NED locale
```

## Lancement

```bash
# Localisation seule
ros2 run localisation localisation_node --ros-args --params-file config/localisation_params.yaml

# Avec le pipeline complet
ros2 launch bringup mock_pipeline.launch.py
```

## Configuration

Fichier: `config/localisation_params.yaml`

```yaml
localisation_node:
  ros__parameters:
    cage_width: 2.0
    cage_depth: 2.0
    min_borders_confidence: 0.7
```

## Précision

- **Latérale (x):** ±10cm + 1% de la distance
- **Frontale (y):** ±10cm + 1% de la distance
- **Angulaire (yaw):** ±3° en conditions normales

La précision se dégrade avec:
- Distance (>15m)
- Mauvais alignement (>45°)
- Faible confiance de tracking

## TODO

