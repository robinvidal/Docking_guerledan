# Sonar Package

Package pour l'interface avec le sonar Oculus M750d et simulation de données sonar.

## Description

Ce package gère l'acquisition des données sonar pour la détection de la cage d'amarrage. Il fournit à la fois une interface pour le sonar réel et un mock pour le développement sans matériel.

Le simulateur sonar permet de tester le système complet avec une cage virtuelle qui réagit aux commandes de vitesse du ROV (physique inverse).

## Nœuds

### `sonar_mock`

Simule un sonar Oculus M750d en générant des frames synthétiques avec une cage virtuelle qui réagit aux commandes du ROV.

**Topics souscrits:**
- `/bluerov/cmd_vel` (`geometry_msgs/Twist`) - Commandes de vitesse du ROV

**Topics publiés:**
- `/docking/sonar/raw` (`docking_msgs/Frame`) - Frames sonar synthétiques

**Paramètres:**
- Voir `config/sonar_params.yaml` pour la liste complète.

**Fonctionnalités:**
- Génération d'images sonar 2D polaires (bearing × range)
- Simulation de 2 montants verticaux de cage avec intensités élevées
- **Physique inverse** : La cage est fixe dans le monde, le ROV bouge
  - Si ROV avance (linear.x > 0), la cage se rapproche (y diminue)
  - Si ROV va à droite (linear.y > 0), la cage glisse à gauche (x diminue)
  - Si ROV tourne à gauche (angular.z > 0), la cage pivote à droite
- Bruit de fond réaliste (gaussien + speckle)
- Filtrage pré-appliqué (médian, gaussien, compensation de portée, contraste)
- Mise à jour physique à 50 Hz pour mouvement fluide


## Format des données

Les frames sonar sont publiées au format `docking_msgs/Frame` avec:
- Grille polaire: `bearing_count × range_count`
- Intensités: tableau 1D (flatten de l'image 2D, row-major)
- Métadonnées: résolutions, plages min/max, vitesse du son
