# Tracking CSRT avec Sélection Manuelle

## Description

Ce système permet de tracker la cage dans les images sonar en utilisant le tracker CSRT d'OpenCV avec une sélection manuelle de la bounding box initiale.

## Composants

### 1. `bbox_selector_node`
- Affiche l'image sonar filtrée dans une fenêtre OpenCV
- Permet de dessiner un rectangle avec la souris
- Publie la bounding box sélectionnée sur `/docking/sonar/bbox_selection`

### 2. `csrt_tracker_node`
- Reçoit la bounding box initiale
- Track l'objet frame par frame avec le tracker CSRT
- Publie les résultats sur `/docking/tracking/tracked_object`

## Utilisation

### 1. Compiler les packages

```bash
cd ~/Documents/Docking/src/ros2_bluerov
colcon build --packages-select docking_msgs tracking bringup
source install/setup.bash
```

### 2. Lancer le pipeline complet

**Option A: Pipeline utilisateur (avec mock):**
```bash
ros2 launch bringup user_pipeline.launch.py
```

**Option B: Pipeline complet (avec sonar réel):**
```bash
ros2 launch bringup complete_pipeline.launch.py
```

> **Note:** Par défaut, les launch files utilisent le tracker CSRT avec sélection manuelle.
> Pour revenir au blob tracker, décommentez `blob_tracker` et commentez `csrt_tracker` + `bbox_selector` dans le launch file.

### 3. Utiliser l'interface

1. **Dessiner la bounding box:**
   - Cliquez et maintenez le bouton gauche de la souris
   - Déplacez pour dessiner un rectangle autour de la cage
   - Relâchez le bouton

2. **Valider la sélection:**
   - Appuyez sur **ESPACE** pour confirmer et démarrer le tracking

3. **Quitter:**
   - Appuyez sur **Q** ou **ESC**

### 4. Lancer les nœuds séparément (optionnel)

```bash
# Terminal 1: Sélecteur de bbox
ros2 run tracking bbox_selector_node

# Terminal 2: Tracker CSRT
ros2 run tracking csrt_tracker_node --ros-args -p selection_mode:=manual
```

## Messages ROS

### Input
- `/docking/sonar/cartesian_filtered` ([FrameCartesian](../docking_msgs/msg/FrameCartesian.msg))
  - Images sonar cartésiennes filtrées

### Output
- `/docking/sonar/bbox_selection` ([BBoxSelection](../docking_msgs/msg/BBoxSelection.msg))
  - Bounding box sélectionnée manuellement
  
- `/docking/tracking/tracked_object` ([TrackedObject](../docking_msgs/msg/TrackedObject.msg))
  - Position et dimensions de l'objet tracké

## Paramètres

### `bbox_selector_node`
- `window_name` (string): Nom de la fenêtre d'affichage
- `display_scale` (float): Facteur d'échelle d'affichage (1.0 = taille réelle)

### `csrt_tracker_node`
- `selection_mode` (string): `'manual'` pour sélection manuelle, `'auto'` pour clic + dimensions
- `enable_tracking` (bool): Active/désactive le tracking
- Nombreux paramètres CSRT pour l'optimisation (voir [csrt_tracker_node.py](tracking/csrt_tracker_node.py))

## Modes de sélection

### Mode Manuel (recommandé)
```yaml
selection_mode: 'manual'
```
- Dessinez la bbox exactement autour de la cage
- Plus précis et flexible

### Mode Auto
```yaml
selection_mode: 'auto'
cage_width: 0.9    # Largeur de la cage en mètres
cage_height: 0.5   # Hauteur de la cage en mètres
```
- Cliquez au centre de la cage
- Bbox automatiquement dimensionnée
- Nécessite une bonne connaissance des dimensions réelles

## Dépannage

### "OpenCV non disponible"
```bash
pip install opencv-python opencv-contrib-python
```

### Le tracker perd la cible
- Redessinez une nouvelle bbox
- Ajustez les paramètres CSRT (notamment `psr_threshold`, `filter_lr`)
- Vérifiez la qualité de l'image filtrée

### La fenêtre ne s'affiche pas
- Vérifiez que vous avez un serveur X actif
- Si SSH: utilisez `ssh -X` ou configurez X11 forwarding

## Architecture

```
[Sonar] → [Filtres] → [FrameCartesian]
                            ↓
                     [bbox_selector_node] ← (User draws bbox)
                            ↓
                     [BBoxSelection]
                            ↓
                     [csrt_tracker_node]
                            ↓
                     [TrackedObject] → [Control/Mission]
```
