# Bringup

Fichiers de lancement pour le pipeline de docking BlueROV.

## Commandes de lancement

### Replay rosbag + pipeline complet
```bash
ros2 launch bringup rosbag_pipeline.launch.py
```
Avec un rosbag spécifique :
```bash
ros2 launch bringup rosbag_pipeline.launch.py bag_path:=rosbag/guerledan_02-02-sans-pvc
```

### Replay rosbag seul (visualisation)
```bash
ros2 launch bringup replay_mission.launch.py
```
```bash
ros2 launch bringup replay_mission.launch.py bag_path:=rosbag/06-02-missionthomas1
```

### Sonar réel + traitement + affichage
```bash
ros2 launch bringup sonar_pipeline.launch.py
```

### Pipeline complet terrain (sonar + PX4 + contrôle)
```bash
ros2 launch bringup complete_pipeline.launch.py
```

### Test avec sonar mock
```bash
ros2 launch bringup user_pipeline.launch.py
```

## Commandes utiles

### Monitoring topics
```bash
ros2 topic list | grep docking
ros2 topic hz /docking/sonar/raw
ros2 topic echo /docking/tracking/tracked_object
```

### Enregistrer un rosbag (sonar + traitement + tracking + cap)
```bash
ros2 bag record -o nom_du_rosbag $(ros2 topic list | grep '^/docking/') /inky/mavros/global_position/compass_hdg
```

### Jouer un rosbag manuellement
```bash
ros2 bag play rosbag/guerledan_02-02-sans-pvc --loop
```

### Rebuild rapide
```bash
colcon build --packages-select bringup && source install/setup.bash
```
