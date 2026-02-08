# Messages ROS2 personnalisés

Messages ROS2 pour le pipeline de docking autonome BlueROV.

## Messages

### Frame
Données brutes du sonar Oculus M750d (polaire).
```
std_msgs/Header header
uint32 range_count              # Nombre de bins en distance
uint32 bearing_count            # Nombre de faisceaux angulaires
float32 range_resolution        # Résolution en distance (m)
float32 bearing_resolution      # Résolution angulaire (rad)
float32 min_range, max_range    # Portée min/max (m)
uint8[] intensities             # Données (range_count × bearing_count)
float32 sound_speed             # Vitesse du son (m/s)
uint8 gain                      # Gain (0-100)
```
```python
msg.range_count, msg.bearing_count = 512, 256
msg.range_resolution = 0.03  # 3cm
msg.intensities = ping_data.flatten().tolist()
```

### FrameCartesian
Image sonar projetée en coordonnées cartésiennes.
```
std_msgs/Header header
uint32 width, height            # Dimensions image (pixels)
float32 resolution              # Mètres par pixel
float32 min_range, max_range    # Portée min/max (m)
uint32 origin_x, origin_y       # Position ROV dans l'image (pixels)
uint8[] intensities             # Données (width × height)
float32 sound_speed             # Vitesse du son (m/s)
uint8 gain                      # Gain (0-100)
float32 total_angle             # Angle total couvert (rad)
```
```python
msg.width, msg.height = 512, 256
msg.resolution = 0.02  # 2cm/pixel
msg.origin_x, msg.origin_y = 256, 0
msg.intensities = image.flatten().tolist()
```

### DetectedLines
Lignes détectées par transformée de Hough.
```
std_msgs/Header header
bool is_valid                   # Détection valide
uint32 num_lines                # Nombre de lignes
float32[] rhos                  # Distance perpendiculaire à l'origine (m)
float32[] thetas                # Angle de la normale (rad)
float32[] x1_points, y1_points  # Premier point de chaque ligne (m)
float32[] x2_points, y2_points  # Second point de chaque ligne (m)
float32[] confidences           # Scores de confiance
```
```python
msg.is_valid = True
msg.num_lines = 3
msg.rhos = [1.2, 0.8, 1.5]
msg.thetas = [0.1, -0.2, 0.0]
msg.confidences = [0.9, 0.85, 0.7]
```

### TrackedObject
Résultat du tracking CSRT.
```
std_msgs/Header header
bool is_tracking                # Tracker actif
bool is_initialized             # Tracker initialisé
float32 center_x, center_y      # Position centre (m)
float32 range, bearing          # Position polaire (m, rad)
float32 angle                   # Orientation bbox (rad)
float32 width, height           # Dimensions (m)
int32 bbox_x, bbox_y            # Coin supérieur gauche (pixels)
int32 bbox_width, bbox_height   # Dimensions (pixels)
float32 entry_p1_x, entry_p1_y  # Point 1 entrée cage (m)
float32 entry_p1_range, entry_p1_bearing
float32 entry_p2_x, entry_p2_y  # Point 2 entrée cage (m)
float32 entry_p2_range, entry_p2_bearing
float32 confidence              # Confiance (0.0-1.0)
```
```python
msg.is_tracking = True
msg.center_x, msg.center_y = 2.5, 1.0
msg.width, msg.height = 0.9, 0.5
msg.confidence = 0.95
```

### BBoxSelection
Sélection manuelle de zone à tracker.
```
std_msgs/Header header
int32 x, y                      # Coin supérieur gauche (pixels)
int32 width, height             # Dimensions (pixels)
bool is_valid                   # Sélection valide
```
```python
msg.x, msg.y = 100, 150
msg.width, msg.height = 80, 60
msg.is_valid = True
```

### ClickPosition
Position d'un clic souris sur le sonar.
```
std_msgs/Header header
float32 x, y                    # Position en mètres
int32 pixel_x, pixel_y          # Position en pixels
bool is_valid                   # Clic dans la zone sonar
```
```python
msg.x, msg.y = 1.5, 2.0
msg.pixel_x, msg.pixel_y = 256, 100
msg.is_valid = True
```

### RotatedBBoxInit
Initialisation d'une bounding box orientée (4 coins).
```
std_msgs/Header header
float32 p1_x, p1_y              # Coin 1 (m)
float32 p2_x, p2_y              # Coin 2 (m)
float32 p3_x, p3_y              # Coin 3 (m)
float32 p4_x, p4_y              # Coin 4 (m, auto-calculé si 0,0)
bool is_valid
```
```python
msg.p1_x, msg.p1_y = 1.0, 2.0
msg.p2_x, msg.p2_y = 1.9, 2.0
msg.p3_x, msg.p3_y = 1.9, 2.5
msg.p4_x, msg.p4_y = 1.0, 2.5
msg.is_valid = True
```
