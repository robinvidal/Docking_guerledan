# Sonar Package

Package pour l'interface avec le sonar Oculus M750d et simulation de données sonar.

## Description

Ce package gère l'acquisition des données sonar pour la détection de la cage d'amarrage. Il fournit à la fois une interface pour le sonar réel et un mock pour le développement sans matériel.

## Nœuds

### `sonar_mock`

Simule un sonar Oculus M750d en générant des frames synthétiques avec une cage virtuelle.

**Topics publiés:**
- `/docking/sonar/raw` (`docking_msgs/Frame`) - Frames sonar synthétiques à ~10 Hz

**Paramètres:**
- `publish_rate` (float, défaut: 10.0) - Fréquence de publication (Hz)
- `range_count` (int, défaut: 512) - Nombre de bins de distance
- `bearing_count` (int, défaut: 256) - Nombre de faisceaux angulaires
- `min_range` (float, défaut: 1.0) - Distance minimale (m)
- `max_range` (float, défaut: 40.0) - Distance maximale (m)
- `cage_distance` (float, défaut: 8.0) - Distance de la cage simulée (m)
- `cage_width` (float, défaut: 2.0) - Largeur de la cage (m)
- `noise_level` (float, défaut: 20.0) - Niveau de bruit de fond (0-100)

**Fonctionnalités:**
- Génération d'images sonar 2D polaires (bearing × range)
- Simulation de 4 montants verticaux de cage avec intensités élevées
- Bruit de fond ajustable pour réalisme
- Configurable via fichier YAML

## Lancement

```bash
# Sonar mock seul
ros2 run sonar sonar_mock --ros-args --params-file install/sonar/share/sonar/config/sonar_params.yaml

# Avec le pipeline complet
ros2 launch bringup mock_pipeline.launch.py
```

## Configuration

Fichier: `config/sonar_params.yaml`

```yaml
sonar_mock:
  ros__parameters:
    publish_rate: 10.0
    cage_distance: 8.0
    cage_width: 2.0
    noise_level: 20.0
```

## Format des données

Les frames sonar sont publiées au format `docking_msgs/Frame` avec:
- Grille polaire: `bearing_count × range_count`
- Intensités: tableau 1D (flatten de l'image 2D)
- Métadonnées: résolutions, plages min/max, vitesse du son

## TODO

- [ ] Interface avec le vrai sonar Oculus M750d via SDK
- [ ] Support de fichiers de replay (enregistrements réels)
- [ ] Ajout de patterns de simulation plus complexes
