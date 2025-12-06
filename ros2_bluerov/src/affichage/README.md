# Affichage Package

Package de visualisation en temps réel pour le système de docking.

## Description

Ce package fournira une interface de visualisation en temps réel pour:
- Images sonar brutes et filtrées
- Overlay des détections (bords de cage)
- Pose estimée du ROV par rapport à la cage
- État de la mission
- Commandes de contrôle

## Fonctionnalités prévues

### Visualisation sonar
# Affichage

Interface PyQt5 qui se connecte aux topics ROS2 du ROV/sonar, affiche les vues (brut, filtré, comparaison) et l'état, et permet de modifier dynamiquement les paramètres ROS.

## Usage
- Se connecte aux topics sonar/pose/état pour afficher brut, filtré, comparaison, état mission et graphes.
- Permet d'ajuster les paramètres ROS en direct via l'UI.

## Lancement
```bash
cd ~/Desktop/Docking_guerledan/ros2_bluerov
source install/setup.bash
ros2 run affichage sonar_viewer
```

## Dépendances
- PyQt5
- pyqtgraph
- docking_msgs

