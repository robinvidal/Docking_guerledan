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
- Affichage polaire des images sonar
- Comparaison brut vs filtré (split-screen)
- Overlay des bords détectés
- Colormap configurable

### Interface opérateur
- Sélection manuelle de la région d'intérêt (ROI)
- Initialisation du tracking par clic
- Boutons start/stop/abort mission
- Indicateurs d'état visuels

### Monitoring
- Graphes temps réel:
  - Position (x, y) vs temps
  - Orientation (yaw) vs temps
  - Erreurs de tracking
  - Commandes de contrôle
- Historique de trajectoire
- Métriques de performance

## Technologies envisagées

### Option 1: RViz2
- Plugin custom pour affichage sonar polaire
- Markers pour bords et pose
- Interface standard ROS2

### Option 2: PyQt5 / RQt
- Interface graphique dédiée
- Contrôle total sur l'affichage
- Intégration rqt_gui

### Option 3: Foxglove Studio
- Interface web moderne
- Pas de développement custom
- Support natif types ROS2

## Topics souscrits (prévus)

- `/docking/sonar/raw` - Images sonar brutes
- `/docking/sonar/filtered` - Images filtrées
- `/docking/tracking/borders` - Bords détectés
- `/docking/localisation/pose` - Pose estimée
- `/docking/mission/state` - État mission
- `/cmd_vel` - Commandes de contrôle

## Topics publiés (prévus)

- `/docking/tracking/roi` - Région d'intérêt sélectionnée
- `/docking/mission/start` - Commande démarrage
- `/docking/mission/abort` - Commande annulation

## Architecture prévue

```
┌─────────────────────────────────────────────┐
│          Interface Graphique                │
│                                             │
│  ┌──────────────┐    ┌──────────────────┐  │
│  │ Sonar View   │    │  Mission Status  │  │
│  │  (Polaire)   │    │  IDLE / APPROACH │  │
│  │              │    │  Progress: 45%   │  │
│  └──────────────┘    └──────────────────┘  │
│                                             │
│  ┌──────────────┐    ┌──────────────────┐  │
│  │ Pose Plot    │    │  Control Panel   │  │
│  │  x, y, yaw   │    │  [Start] [Abort] │  │
│  │  vs time     │    │  Manual / Auto   │  │
│  └──────────────┘    └──────────────────┘  │
│                                             │
│  ┌──────────────────────────────────────┐  │
│  │      Trajectory Viewer (2D)          │  │
│  │      ROV path + Cage position        │  │
│  └──────────────────────────────────────┘  │
└─────────────────────────────────────────────┘
```

## ✅ Nœuds

### `sonar_viewer`

Interface graphique PyQt5 pour visualisation temps réel des données de docking.

**Topics souscrits:**
- `/docking/sonar/raw` (`docking_msgs/Frame`) - Images sonar brutes
- `/docking/sonar/filtered` (`docking_msgs/Frame`) - Images filtrées
- `/docking/tracking/borders` (`docking_msgs/Borders`) - Bords détectés
- `/docking/localisation/pose` (`docking_msgs/PoseRelative`) - Pose estimée
- `/docking/mission/state` (`docking_msgs/State`) - État mission

**Topics publiés:**
- `/docking/mission/abort` (`std_msgs/Bool`) - Commande d'abort

## Fonctionnalités

### 4 Onglets de visualisation

1. **📡 Sonar Brut**
   - Vue cartésienne 2D (top-down) des données sonar
   - Chaque point représente un écho avec intensité en couleur (colormap viridis)
   - Overlay des bords détectés (gros points rouges)
   - Marqueur ROV à l'origine (triangle vert)
   - Axes: X (latéral) et Y (frontal) en mètres

2. **🔍 Sonar Filtré**
   - Même vue cartésienne après filtrage
   - Permet de voir l'amélioration du filtrage
   - Overlay des bords détectés

3. **⚖️ Comparaison**
   - Vues côte à côte: brut vs filtré
   - Permet d'évaluer l'efficacité du filtrage en temps réel

4. **📊 Graphes Pose**
   - Position (x, y) vs temps
   - Orientation (yaw) vs temps
   - Historique de 200 points

### Panneau de contrôle

- **État Mission** - Affichage état actuel avec code couleur
- **Pose** - Position et orientation en temps réel
- **Confiance** - Niveau de confiance de la détection
- **Bouton ABORT** - Arrêt d'urgence de la mission

## Installation des dépendances

```bash
# PyQt5 et pyqtgraph
pip install PyQt5 pyqtgraph
```

## Lancement

```bash
# Avec le pipeline complet
cd ~/Desktop/Docking_guerledan/ros2_bluerov
source install/setup.bash
ros2 run affichage sonar_viewer

# Ou en parallèle du pipeline
# Terminal 1
ros2 launch bringup mock_pipeline.launch.py

# Terminal 2
ros2 run affichage sonar_viewer
```

## Captures d'écran

L'interface affiche en temps réel:
- **Vue cartésienne 2D** : Les données sonar converties de polaire à cartésien
- **Nuage de points colorés** : Chaque point = écho sonar, couleur = intensité
- **Bords de cage** : Overlay en rouge (4 montants détectés)
- **Position ROV** : Triangle vert à l'origine (0, 0)
- **Axe de visée** : Ligne pointillée centrale
- État mission avec code couleur
- Graphes de pose

## Utilisation

### Navigation
- Utilisez les onglets pour changer de vue
- Clic molette pour zoomer/dézoomer dans les images
- Clic droit pour réinitialiser le zoom

### Bouton ABORT
- Clic sur "🛑 ABORT" pour envoyer un signal d'arrêt d'urgence
- La mission passera en état ABORT
- Le contrôle sera désactivé

### Codes couleur des états
- 🟦 **IDLE** - Gris (inactif)
- 🟧 **LOCK_ON** - Orange (acquisition)
- 🟦 **APPROACH** - Bleu clair (approche)
- 🟧 **DOCKING** - Orange foncé (amarrage)
- 🟢 **DOCKED** - Vert (amarré)
- 🟡 **RECOVERY** - Jaune (récupération)
- 🔴 **ABORT** - Rouge (annulé)

## Architecture technique

### Thread-safe ROS ↔ Qt

L'application utilise des **signaux Qt** pour communication thread-safe:
```python
ROSSignals(QObject):
    new_raw_frame = pyqtSignal(object)
    new_filtered_frame = pyqtSignal(object)
    new_borders = pyqtSignal(object)
    new_pose = pyqtSignal(object)
    new_state = pyqtSignal(object)
```

Le nœud ROS tourne dans le thread Qt via un `QTimer` appelant `spin_once()`.

### Vue cartésienne 2D

Les données sonar reçues en coordonnées polaires (r, θ) sont converties en coordonnées cartésiennes (x, y):
- **x = r × sin(θ)** : Position latérale (gauche/droite)
- **y = r × cos(θ)** : Position frontale (devant)

Chaque point est coloré selon son intensité (viridis colormap). Les points faibles (intensité < 30) sont filtrés pour clarté.

### PyQtGraph pour performance

`pyqtgraph` est utilisé pour:
- Affichage rapide de nuages de points (GPU-accelerated)
- Graphes temps réel avec historique
- Overlay de détections (ScatterPlot)

## Performance

- **Fréquence d'affichage:** ~30 FPS (Qt refresh)
- **Conversion polaire→cartésien:** Sous-échantillonnage ×2 pour fluidité
- **Points affichés:** ~15k-30k par frame (après filtrage intensité < 30)
- **Latence:** <50ms entre réception topic et affichage
- **Mémoire:** ~150MB (inclut historique graphes)

## Limitations actuelles

- ❌ Pas de sauvegarde d'images
- ❌ Pas de recording vidéo
- ❌ Pas de sélection ROI manuelle
- ❌ Pas d'export de données

## Extensions futures

### Court terme
- [ ] Sauvegarde snapshots (bouton capture)
- [ ] Ajustement colormap dynamique
- [ ] Affichage métadonnées frame (gain, range, etc.)

### Moyen terme
- [ ] Sélection ROI par clic (pour tracking manuel)
- [ ] Recording vidéo au format MP4
- [ ] Export CSV des données pose
- [ ] Vue 2D cartésienne (top-down)

### Long terme
- [ ] Reconstruction 3D de la cage
- [ ] Replay de missions enregistrées
- [ ] Multi-fenêtres (plusieurs sonars)
- [ ] Interface web (browser-based)

## Dépannage

### Erreur "No module named 'PyQt5'"
```bash
pip install PyQt5 pyqtgraph
```

### Erreur "No module named 'docking_msgs'"
```bash
cd ~/Desktop/Docking_guerledan/ros2_bluerov
colcon build --packages-select docking_msgs affichage
source install/setup.bash
```

### Interface ne se lance pas
```bash
# Vérifier que DISPLAY est configuré
echo $DISPLAY

# Si vide, configurer
export DISPLAY=:0
```

### Images ne s'affichent pas
- Vérifier que le sonar publie: `ros2 topic hz /docking/sonar/raw`
- Vérifier format des messages: `ros2 topic echo /docking/sonar/raw --once`

## TODO

- [x] Interface de base PyQt5
- [x] Affichage images sonar en vue cartésienne 2D
- [x] Conversion polaire → cartésien temps réel
- [x] Overlay bords détectés
- [x] Graphes de pose
- [x] Bouton abort
- [ ] Sélection ROI manuelle
- [ ] Sauvegarde images
- [ ] Recording vidéo
- [ ] Zoom/pan interactif amélioré
- [ ] Affichage trajectoire ROV (historique)

## Roadmap

### Phase 1: Monitoring basique
- [ ] Affichage images sonar (matplotlib ou opencv)
- [ ] Overlay détections
- [ ] Display état mission en texte

### Phase 2: Interface opérateur
- [ ] Boutons start/stop/abort
- [ ] Sélection ROI par clic
- [ ] Indicateurs visuels (LED virtuelles)

### Phase 3: Visualisation avancée
- [ ] Graphes temps réel (position, commandes)
- [ ] Replay de missions enregistrées
- [ ] Métriques et statistiques

## Usage temporaire (sans affichage)

En attendant l'implémentation:

```bash
# Terminal 1: Lancer le pipeline
ros2 launch bringup mock_pipeline.launch.py

# Terminal 2: Observer l'état
ros2 topic echo /docking/mission/state

# Terminal 3: Observer la pose
ros2 topic echo /docking/localisation/pose

# PlotJuggler pour visualisation graphique
ros2 run plotjuggler plotjuggler
```

## Contribution

Toute contribution est bienvenue ! Choix technologiques à discuter:
- RViz2 plugins vs Interface standalone
- Qt/PyQt vs Web (Foxglove)
- 2D vs 3D visualization

## Références

- [RViz2 Plugin Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-Main.html)
- [RQt Plugin Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-RQt-Plugin.html)
- [Foxglove Studio](https://foxglove.dev/)
- [PlotJuggler](https://github.com/facontidavide/PlotJuggler)
