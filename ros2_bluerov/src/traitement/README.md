# Traitement Package

Package ROS2 de traitement d'image pour les données sonar. Applique une pipeline de filtrage en deux étapes (polaire puis cartésien) pour améliorer la détection des structures de la cage.

## Description

- **`traitement_unified_node`** : Nœud unique gérant toute la pipeline de traitement. Reçoit les frames polaires brutes, applique les filtres configurés, et publie les résultats en formats polaire et cartésien.

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/docking/sonar/raw` | `docking_msgs/Frame` | Subscription | Frame sonar polaire brute |
| `/docking/tracking/tracked_object` | `docking_msgs/TrackedObject` | Subscription | Position trackée (pour filtre spatial) |
| `/docking/sonar/polar_filtered` | `docking_msgs/Frame` | Publication | Frame polaire après filtrage |
| `/docking/sonar/cartesian_filtered` | `docking_msgs/FrameCartesian` | Publication | Frame cartésienne après filtrage |

## Détails techniques

> Pour plus d'explications, voir les commentaires dans [traitement_unified_node.py](traitement/traitement_unified_node.py).  
> Pour la configuration des paramètres, voir [traitement_unified_params.yaml](config/traitement_unified_params.yaml).

### Pipeline de filtrage (ordre d'application)

#### 1. Filtres Polaires (sur image brute)

- **Filtre Frost** : Réduction adaptative du speckle radar/sonar. Préserve les contours tout en lissant le bruit.
- **Filtre Médian Polaire** : Supprime le bruit impulsionnel (sel et poivre) avec un noyau configurable.

#### 2. Conversion Polaire → Cartésien

- Transformation géométrique avec interpolation bilinéaire
- Mise en cache des coordonnées pour performances optimales

#### 3. Filtres Cartésiens (sur image convertie)

- **Filtre Médian Cartésien** : Deuxième passe de débruitage après conversion.
- **Filtre CLAHE** : Amélioration de contraste adaptative locale. Fait ressortir les structures métalliques.
- **Filtre Seuil** : Met à zéro les pixels sous un seuil d'intensité. Élimine les faibles échos.
- **Filtre Morphologique (Closing)** : Dilatation + Érosion. Comble les discontinuités des barres de cage.
- **Filtre Flip** : Retournement horizontal/vertical optionnel pour corriger l'orientation (par défaut désactivé).
- **Filtre Spatial Gaussien** : Pondération gaussienne centrée sur la position trackée (actif si tracking disponible).
- **Filtre Percentile** : Binarisation gardant uniquement les X% pixels les plus intenses.
- **Filtre Opening-Closing** : Opening (supprime bruit) puis Closing (solidifie structures).
