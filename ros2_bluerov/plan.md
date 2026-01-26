# Plan d'Action : Détection de Cage avec Filtre Spatial et Transformée de Hough

## 🎯 Objectif Global
Ajouter un système de détection robuste de la cage en combinant :
1. **Filtre spatial gaussien** centré sur la position trackée pour isoler la cage
2. **Transformée de Hough** pour détecter les lignes de la cage
3. **Algorithme de détection de rectangle** pour identifier la forme de la cage
4. **Affichage en temps réel** du rectangle détecté

---

## 📐 Architecture Actuelle

### Flux de données existant
```
sonar_node/sonar_mock 
  → /docking/sonar/raw (Frame)
    → traitement_polar_node
      → /docking/sonar/polar_filtered (Frame)
        → traitement_cartesian_node
          → /docking/sonar/cartesian_filtered (FrameCartesian)
            → csrt_tracker_node
              → /docking/tracking/tracked_object (TrackedObject)
            → hough_lines_node
              → /docking/tracking/detected_lines (DetectedLines)
            → sonar_viewer (affichage)
```

### Messages existants
- **TrackedObject** : position (center_x, center_y en m), dimensions (width, height), confiance
- **DetectedLines** : lignes détectées (rhos, thetas, points, confidences)
- **FrameCartesian** : image cartésienne + métadonnées (resolution, max_range, origin_x/y)

---

## 📋 Plan Détaillé par Étapes

### ✅ **ÉTAPE 0 : Préparation et Vérification de l'Existant**
**Objectif** : S'assurer que le pipeline actuel fonctionne correctement

#### 0.1 - Tester le pipeline complet
- [x] Lancer `ros2 launch bringup user_pipeline.launch.py` (mock)
- [x] Vérifier que le sonar_viewer s'affiche correctement
- [x] Vérifier que le tracker publie sur `/docking/tracking/tracked_object`
- [x] Vérifier que hough_lines publie sur `/docking/tracking/detected_lines`

#### 0.2 - Visualiser les topics actuels
```bash
ros2 topic echo /docking/tracking/tracked_object
ros2 topic echo /docking/tracking/detected_lines
```

**Critères de succès** :
- ✓ Sonar_viewer affiche l'image cartésienne
- ✓ Les topics publient des données valides
- ✓ Pas d'erreurs dans les logs

---

### ✅ **ÉTAPE 1 : Ajout du Filtre Spatial Gaussien dans traitement_cartesian_node**
**Objectif** : Filtrer l'image pour ne garder que la zone autour de la cage trackée

#### 1.1 - Créer un nouveau message pour le filtre spatial (OPTIONNEL)
Si nécessaire, créer `CageFilter.msg` dans `docking_msgs/msg/` :
```msg
# CageFilter.msg - Paramètres du filtre spatial
float32 center_x      # Centre du filtre (m)
float32 center_y      # Centre du filtre (m)
float32 sigma         # Écart-type du gaussien (m)
float32 radius        # Rayon d'effet (m)
bool is_active        # Filtre actif ou non
```

**DÉCISION** : On va directement s'abonner à `TrackedObject` plutôt que créer un nouveau message.

#### 1.2 - Modifier `traitement_cartesian_node.py`

**Modifications à apporter** :

```python
# Nouveaux paramètres
self.declare_parameter('enable_spatial_filter', True)
self.declare_parameter('spatial_filter_radius', 2.0)  # Rayon en mètres
self.declare_parameter('spatial_filter_sigma', 0.8)   # Écart-type gaussien (m)
self.declare_parameter('spatial_filter_smoothness', 0.3)  # Transition progressive

# Nouvelle subscription
self.tracked_object_sub = self.create_subscription(
    TrackedObject,
    '/docking/tracking/tracked_object',
    self.tracked_object_callback,
    10
)

# Variable d'état
self.last_tracked_position = None  # (center_x, center_y) en mètres
```

**Nouvelle méthode** :
```python
def apply_spatial_gaussian_filter(self, img: np.ndarray, frame_msg: FrameCartesian) -> np.ndarray:
    """
    Applique un filtre gaussien spatial centré sur la position trackée.
    Atténue progressivement les intensités au-delà du rayon spécifié.
    """
    if not self.get_parameter('enable_spatial_filter').value:
        return img
    
    if self.last_tracked_position is None:
        return img  # Pas de position trackée, pas de filtrage
    
    center_x_m, center_y_m = self.last_tracked_position
    radius_m = float(self.get_parameter('spatial_filter_radius').value)
    sigma_m = float(self.get_parameter('spatial_filter_sigma').value)
    
    # Créer une grille de coordonnées en mètres
    height, width = img.shape
    resolution = frame_msg.resolution
    
    # Coordonnées de chaque pixel en mètres (relatif au ROV)
    # x_m[i,j] = (j - origin_x) * resolution
    # y_m[i,j] = (height - i) * resolution
    
    j_indices = np.arange(width)
    i_indices = np.arange(height)
    j_grid, i_grid = np.meshgrid(j_indices, i_indices)
    
    x_grid_m = (j_grid - frame_msg.origin_x) * resolution
    y_grid_m = (height - i_grid) * resolution
    
    # Distance au centre tracké
    dist_m = np.sqrt((x_grid_m - center_x_m)**2 + (y_grid_m - center_y_m)**2)
    
    # Masque gaussien (1.0 au centre, décroît progressivement)
    # Plus sigma est petit, plus la transition est abrupte
    mask = np.exp(-(dist_m**2) / (2 * sigma_m**2))
    
    # Appliquer le masque (multiplication élément par élément)
    filtered_img = (img.astype(np.float32) * mask).astype(np.uint8)
    
    return filtered_img
```

**Modification de `frame_callback`** :
Appliquer le filtre spatial **après** la conversion cartésienne mais **avant** les autres filtres.

#### 1.3 - Tester le filtre spatial

**Test** :
```bash
# Terminal 1
ros2 launch bringup user_pipeline.launch.py

# Terminal 2 - Vérifier que le filtre s'applique
ros2 param set /traitement_cartesian_node enable_spatial_filter true
ros2 param set /traitement_cartesian_node spatial_filter_radius 2.5

# Terminal 3 - Initialiser le tracker
# Cliquer dans sonar_viewer avec Ctrl+Clic sur la cage
```

**Critères de succès** :
- ✓ Quand le tracker est actif, seule la zone autour de la cage reste visible
- ✓ La transition est progressive (pas de bord dur)
- ✓ L'intensité décroît graduellement avec la distance

#### 1.4 - Ajustement des paramètres
Tester différentes valeurs pour trouver le bon compromis :
- `spatial_filter_radius` : 1.5m, 2.0m, 2.5m
- `spatial_filter_sigma` : 0.5m, 0.8m, 1.0m

---

### ✅ **ÉTAPE 2 : Amélioration de la Détection Hough**
**Objectif** : Optimiser la détection de lignes pour la cage

#### 2.1 - Améliorer le preprocessing dans `traitement_cartesian_node`

Avant Hough, l'image doit être binaire ou avec des contours nets.

**Ajouter dans `traitement_cartesian_node.py`** :
```python
# Nouveau paramètre
self.declare_parameter('cart_enable_morphology', True)
self.declare_parameter('cart_morphology_kernel_size', 3)
self.declare_parameter('cart_morphology_iterations', 2)

def apply_morphology_operations(self, img: np.ndarray) -> np.ndarray:
    """Applique des opérations morphologiques pour nettoyer l'image."""
    if not self.get_parameter('cart_enable_morphology').value:
        return img
    
    if cv2 is None:
        return img
    
    kernel_size = int(self.get_parameter('cart_morphology_kernel_size').value)
    iterations = int(self.get_parameter('cart_morphology_iterations').value)
    
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    
    # Closing : fermer les petits trous
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel, iterations=iterations)
    
    # Opening : supprimer les petits bruits
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel, iterations=1)
    
    return img
```

#### 2.2 - Tester la détection Hough améliorée

```bash
# Activer Canny + Morphologie
ros2 param set /traitement_cartesian_node cart_enable_canny true
ros2 param set /traitement_cartesian_node cart_enable_morphology true
ros2 param set /traitement_cartesian_node cart_canny_threshold1 50
ros2 param set /traitement_cartesian_node cart_canny_threshold2 150

# Ajuster Hough
ros2 param set /hough_lines_node threshold 30
ros2 param set /hough_lines_node min_line_length 50
ros2 param set /hough_lines_node num_lines 10
```

**Critères de succès** :
- ✓ Hough détecte au moins 4 lignes (2 verticales pour les poteaux, 2 horizontales pour le haut/bas)
- ✓ Les lignes sont stables d'une frame à l'autre
- ✓ Peu de fausses détections

---

### ✅ **ÉTAPE 3 : Création d'un Nœud de Détection de Rectangle**
**Objectif** : Combiner les lignes Hough pour détecter un rectangle

#### 3.1 - Créer un nouveau message `DetectedRectangle.msg`

**Fichier** : `ros2_bluerov/src/docking_msgs/msg/DetectedRectangle.msg`
```msg
# DetectedRectangle.msg - Rectangle détecté représentant la cage

std_msgs/Header header

# Validité de la détection
bool is_valid

# Position du centre (en mètres, repère ROV)
float32 center_x
float32 center_y

# Dimensions (en mètres)
float32 width
float32 height

# Orientation (angle en radians, 0 = vertical)
float32 angle

# Coordonnées des 4 coins (en mètres, dans le sens horaire depuis coin supérieur gauche)
float32[] corners_x  # 4 éléments
float32[] corners_y  # 4 éléments

# Confiance de la détection (0.0 - 1.0)
float32 confidence

# Lignes utilisées pour la détection (indices dans DetectedLines)
int32[] line_indices
```

#### 3.2 - Créer le nœud `rectangle_detector_node.py`

**Fichier** : `ros2_bluerov/src/tracking/tracking/rectangle_detector_node.py`

**Structure** :
```python
"""
Nœud de détection de rectangle à partir des lignes Hough.
Combine les lignes détectées pour identifier un rectangle correspondant à la cage.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from docking_msgs.msg import DetectedLines, DetectedRectangle, TrackedObject


class RectangleDetectorNode(Node):
    """Détecte un rectangle (cage) à partir des lignes Hough."""
    
    def __init__(self):
        super().__init__('rectangle_detector_node')
        
        # Paramètres
        self.declare_parameter('enable_detection', True)
        self.declare_parameter('expected_cage_width', 1.0)  # m
        self.declare_parameter('expected_cage_height', 1.5)  # m
        self.declare_parameter('width_tolerance', 0.3)  # ±30%
        self.declare_parameter('height_tolerance', 0.3)
        self.declare_parameter('angle_tolerance', 15.0)  # degrés
        self.declare_parameter('min_lines_required', 4)
        self.declare_parameter('use_tracked_position', True)  # Utiliser la position du tracker
        
        # Subscriptions
        self.lines_sub = self.create_subscription(
            DetectedLines,
            '/docking/tracking/detected_lines',
            self.lines_callback,
            10
        )
        
        self.tracked_sub = self.create_subscription(
            TrackedObject,
            '/docking/tracking/tracked_object',
            self.tracked_callback,
            10
        )
        
        # Publisher
        self.rectangle_pub = self.create_publisher(
            DetectedRectangle,
            '/docking/tracking/detected_rectangle',
            10
        )
        
        # État
        self.last_tracked_position = None
        self.last_lines = None
        
        self.get_logger().info('Rectangle Detector node démarré')
    
    def tracked_callback(self, msg: TrackedObject):
        """Mémorise la dernière position trackée."""
        if msg.is_tracking:
            self.last_tracked_position = (msg.center_x, msg.center_y)
    
    def lines_callback(self, msg: DetectedLines):
        """Détecte un rectangle à partir des lignes."""
        if not self.get_parameter('enable_detection').value:
            return
        
        if not msg.is_valid or msg.num_lines < self.get_parameter('min_lines_required').value:
            self.publish_invalid_rectangle(msg.header)
            return
        
        # Extraire les lignes
        lines = []
        for i in range(msg.num_lines):
            line = {
                'rho': msg.rhos[i],
                'theta': msg.thetas[i],
                'x1': msg.x1_points[i],
                'y1': msg.y1_points[i],
                'x2': msg.x2_points[i],
                'y2': msg.y2_points[i],
                'confidence': msg.confidences[i] if msg.confidences else 1.0,
                'index': i
            }
            lines.append(line)
        
        # Détecter le rectangle
        rectangle = self.detect_rectangle(lines, msg.header)
        
        # Publier
        self.rectangle_pub.publish(rectangle)
    
    def detect_rectangle(self, lines, header) -> DetectedRectangle:
        """
        Algorithme de détection de rectangle :
        1. Classifier les lignes en verticales/horizontales
        2. Trouver 2 lignes verticales (poteaux) + 2 horizontales (haut/bas)
        3. Calculer les intersections pour obtenir les 4 coins
        4. Vérifier que les dimensions correspondent à la cage
        """
        # À implémenter (voir détails ci-dessous)
        pass
    
    def classify_lines(self, lines):
        """Sépare les lignes en verticales et horizontales."""
        vertical_lines = []
        horizontal_lines = []
        
        angle_tol = np.deg2rad(self.get_parameter('angle_tolerance').value)
        
        for line in lines:
            theta = line['theta']
            
            # Vertical : theta proche de 0 ou π
            if abs(theta) < angle_tol or abs(theta - np.pi) < angle_tol:
                vertical_lines.append(line)
            # Horizontal : theta proche de π/2
            elif abs(theta - np.pi/2) < angle_tol:
                horizontal_lines.append(line)
        
        return vertical_lines, horizontal_lines
    
    def find_best_rectangle(self, vertical_lines, horizontal_lines):
        """Trouve la meilleure combinaison de 4 lignes formant un rectangle."""
        # Parcourir toutes les combinaisons possibles
        # Pour chaque paire de verticales et paire d'horizontales :
        #   - Calculer les 4 intersections
        #   - Mesurer largeur et hauteur
        #   - Comparer aux dimensions attendues
        #   - Calculer un score
        # Retourner la meilleure combinaison
        pass
```

**Algorithme détaillé de détection** :

```python
def find_best_rectangle(self, vertical_lines, horizontal_lines):
    """Trouve la meilleure combinaison de lignes formant un rectangle."""
    if len(vertical_lines) < 2 or len(horizontal_lines) < 2:
        return None
    
    expected_width = float(self.get_parameter('expected_cage_width').value)
    expected_height = float(self.get_parameter('expected_cage_height').value)
    width_tol = float(self.get_parameter('width_tolerance').value)
    height_tol = float(self.get_parameter('height_tolerance').value)
    
    best_score = -1
    best_rectangle = None
    
    # Essayer toutes les combinaisons de 2 verticales et 2 horizontales
    for i in range(len(vertical_lines)):
        for j in range(i+1, len(vertical_lines)):
            v1, v2 = vertical_lines[i], vertical_lines[j]
            
            for k in range(len(horizontal_lines)):
                for l in range(k+1, len(horizontal_lines)):
                    h1, h2 = horizontal_lines[k], horizontal_lines[l]
                    
                    # Calculer les 4 intersections
                    corners = self.compute_corners(v1, v2, h1, h2)
                    if corners is None:
                        continue
                    
                    # Calculer dimensions
                    width = abs(corners[1][0] - corners[0][0])
                    height = abs(corners[0][1] - corners[3][1])
                    
                    # Vérifier que les dimensions correspondent
                    width_error = abs(width - expected_width) / expected_width
                    height_error = abs(height - expected_height) / expected_height
                    
                    if width_error > width_tol or height_error > height_tol:
                        continue  # Dimensions trop différentes
                    
                    # Calculer un score (basé sur erreur + confiances des lignes)
                    dimension_score = 1.0 - (width_error + height_error) / 2.0
                    confidence_score = (v1['confidence'] + v2['confidence'] + 
                                      h1['confidence'] + h2['confidence']) / 4.0
                    
                    score = 0.6 * dimension_score + 0.4 * confidence_score
                    
                    # Si on utilise la position trackée, privilégier les rectangles proches
                    if self.get_parameter('use_tracked_position').value and self.last_tracked_position:
                        center_x = sum(c[0] for c in corners) / 4
                        center_y = sum(c[1] for c in corners) / 4
                        dist_to_tracked = np.sqrt(
                            (center_x - self.last_tracked_position[0])**2 +
                            (center_y - self.last_tracked_position[1])**2
                        )
                        proximity_score = np.exp(-dist_to_tracked / 1.0)
                        score = 0.5 * score + 0.5 * proximity_score
                    
                    if score > best_score:
                        best_score = score
                        best_rectangle = {
                            'corners': corners,
                            'width': width,
                            'height': height,
                            'lines': [v1, v2, h1, h2],
                            'score': score
                        }
    
    return best_rectangle

def compute_corners(self, v1, v2, h1, h2):
    """Calcule les 4 coins formés par 2 lignes verticales et 2 horizontales."""
    # Intersection de 2 lignes définies par (rho, theta)
    corners = []
    
    for v_line in [v1, v2]:
        for h_line in [h1, h2]:
            corner = self.line_intersection(v_line, h_line)
            if corner is not None:
                corners.append(corner)
    
    if len(corners) != 4:
        return None
    
    # Trier les coins dans le sens horaire (top-left, top-right, bottom-right, bottom-left)
    corners = self.sort_corners_clockwise(corners)
    
    return corners

def line_intersection(self, line1, line2):
    """Calcule l'intersection de 2 lignes définies par (rho, theta)."""
    rho1, theta1 = line1['rho'], line1['theta']
    rho2, theta2 = line2['theta'], line2['theta']
    
    # Système d'équations :
    # x * cos(theta1) + y * sin(theta1) = rho1
    # x * cos(theta2) + y * sin(theta2) = rho2
    
    A = np.array([
        [np.cos(theta1), np.sin(theta1)],
        [np.cos(theta2), np.sin(theta2)]
    ])
    b = np.array([rho1, rho2])
    
    try:
        point = np.linalg.solve(A, b)
        return (float(point[0]), float(point[1]))
    except np.linalg.LinAlgError:
        return None  # Lignes parallèles

def sort_corners_clockwise(self, corners):
    """Trie les 4 coins dans le sens horaire."""
    # Calculer le centre
    cx = sum(c[0] for c in corners) / 4
    cy = sum(c[1] for c in corners) / 4
    
    # Calculer l'angle de chaque coin par rapport au centre
    def angle_from_center(corner):
        return np.arctan2(corner[1] - cy, corner[0] - cx)
    
    sorted_corners = sorted(corners, key=angle_from_center)
    
    return sorted_corners
```

#### 3.3 - Intégrer le nouveau nœud

**Modifier** `ros2_bluerov/src/tracking/setup.py` :
```python
entry_points={
    'console_scripts': [
        'blob_tracker_node = tracking.blob_tracker_node:main',
        'csrt_tracker_node = tracking.csrt_tracker_node:main',
        'hough_lines_node = tracking.hough_lines_node:main',
        'rectangle_detector_node = tracking.rectangle_detector_node:main',  # NOUVEAU
    ],
},
```

**Modifier** `CMakeLists.txt` (si nécessaire pour le nouveau message).

#### 3.4 - Compiler et tester

```bash
cd ~/Desktop/Docking_guerledan/ros2_bluerov
colcon build --packages-select docking_msgs tracking
source install/setup.bash

# Lancer le nouveau nœud
ros2 run tracking rectangle_detector_node

# Écouter les rectangles détectés
ros2 topic echo /docking/tracking/detected_rectangle
```

**Critères de succès** :
- ✓ Le nœud démarre sans erreur
- ✓ Il détecte un rectangle quand la cage est visible
- ✓ Les dimensions sont cohérentes avec la cage (±30%)
- ✓ La position correspond à la zone trackée

---

### ✅ **ÉTAPE 4 : Affichage du Rectangle dans sonar_viewer**
**Objectif** : Visualiser le rectangle détecté en temps réel

#### 4.1 - Modifier `ros_node.py` dans affichage

**Fichier** : `ros2_bluerov/src/affichage/affichage/app/core/ros_node.py`

```python
from docking_msgs.msg import DetectedRectangle  # AJOUTER

# Dans __init__
self.rectangle_sub = self.create_subscription(
    DetectedRectangle,
    '/docking/tracking/detected_rectangle',
    self.rectangle_callback,
    10
)

def rectangle_callback(self, msg):
    """Transmet le rectangle détecté au signal Qt."""
    self.signals.new_detected_rectangle.emit(msg)
```

#### 4.2 - Modifier `signals.py`

**Fichier** : `ros2_bluerov/src/affichage/affichage/app/core/signals.py`

```python
new_detected_rectangle = pyqtSignal(object)  # AJOUTER
```

#### 4.3 - Modifier `sonar_display.py` pour dessiner le rectangle

**Fichier** : `ros2_bluerov/src/affichage/affichage/app/widgets/sonar_display.py`

```python
class SonarCartesianWidget(pg.GraphicsLayoutWidget):
    def __init__(self, ...):
        # ... code existant ...
        
        # Nouvel overlay pour le rectangle
        self.rectangle_item = None
    
    def set_detected_rectangle(self, rect_msg):
        """Affiche le rectangle détecté."""
        if not rect_msg.is_valid:
            if self.rectangle_item is not None:
                self.viewbox.removeItem(self.rectangle_item)
                self.rectangle_item = None
            return
        
        # Extraire les coins (en mètres)
        corners_x = rect_msg.corners_x
        corners_y = rect_msg.corners_y
        
        # Convertir en coordonnées de l'image (pixels dans l'affichage pyqtgraph)
        # pyqtgraph utilise un système où x=lateral, y=frontal
        
        # Créer un polygon
        points = []
        for i in range(4):
            x_m = corners_x[i]
            y_m = corners_y[i]
            # Convertir en coordonnées image (dépend de la résolution et origine)
            # À adapter selon votre système de coordonnées
            points.append((x_m, y_m))
        
        # Fermer le polygon
        points.append(points[0])
        
        # Supprimer l'ancien rectangle
        if self.rectangle_item is not None:
            self.viewbox.removeItem(self.rectangle_item)
        
        # Créer le nouveau
        self.rectangle_item = pg.PlotDataItem(
            [p[0] for p in points],
            [p[1] for p in points],
            pen=pg.mkPen('r', width=3),  # Rouge, épaisseur 3
            name='Cage détectée'
        )
        self.viewbox.addItem(self.rectangle_item)
```

#### 4.4 - Connecter le signal dans `main_window.py`

**Fichier** : `ros2_bluerov/src/affichage/affichage/app/main_window.py`

```python
# Dans __init__
self.ros_signals.new_detected_rectangle.connect(
    self.sonar_cartesian_widget.set_detected_rectangle
)
```

#### 4.5 - Tester l'affichage

```bash
ros2 launch bringup user_pipeline.launch.py

# Le rectangle rouge devrait s'afficher sur la cage détectée
```

**Critères de succès** :
- ✓ Un rectangle rouge apparaît autour de la cage
- ✓ Le rectangle suit la cage si elle bouge
- ✓ Le rectangle disparaît si la détection échoue

---

### ✅ **ÉTAPE 5 : Mise à jour des Launch Files**
**Objectif** : Intégrer le nouveau nœud dans les pipelines

#### 5.1 - Modifier `user_pipeline.launch.py`

```python
rectangle_detector = Node(
    package='tracking',
    executable='rectangle_detector_node',
    name='rectangle_detector_node',
    parameters=[{
        'enable_detection': True,
        'expected_cage_width': 1.0,
        'expected_cage_height': 1.5,
        'use_tracked_position': True,
    }],
    output='screen'
)

return LaunchDescription([
    sonar_mock,
    traitement,
    csrt_tracker,
    hough_lines_node,  # Vérifier qu'il est présent
    rectangle_detector,  # NOUVEAU
    sonar_viewer,
    localisation,
])
```

#### 5.2 - Modifier `sonar_pipeline.launch.py` et `complete_pipeline.launch.py`

Ajouter de même le nœud `rectangle_detector`.

---

### ✅ **ÉTAPE 6 : Tests d'Intégration Complets**
**Objectif** : Valider le système complet

#### 6.1 - Test avec mock

```bash
ros2 launch bringup user_pipeline.launch.py

# 1. Vérifier que la cage s'affiche
# 2. Ctrl+Clic sur la cage pour initialiser le tracker
# 3. Attendre que le tracker soit stable
# 4. Vérifier que le filtre spatial s'applique
# 5. Vérifier que Hough détecte des lignes
# 6. Vérifier que le rectangle apparaît
```

#### 6.2 - Test avec rosbag

```bash
ros2 launch bringup rosbag_pipeline.launch.py

# Même séquence de tests
```

#### 6.3 - Test avec sonar réel (si disponible)

```bash
ros2 launch bringup sonar_pipeline.launch.py

# Valider en conditions réelles
```

#### 6.4 - Tests de robustesse

- [ ] Mouvement latéral du ROV
- [ ] Rotation du ROV
- [ ] Approche de la cage
- [ ] Éloignement de la cage
- [ ] Occlusions partielles
- [ ] Bruit sonar

---

### ✅ **ÉTAPE 7 : Tuning et Optimisation**
**Objectif** : Affiner les paramètres pour des performances optimales

#### 7.1 - Paramètres du filtre spatial

```bash
ros2 param set /traitement_cartesian_node spatial_filter_radius 2.0
ros2 param set /traitement_cartesian_node spatial_filter_sigma 0.8
```

Tester : 1.5m, 2.0m, 2.5m pour le rayon.

#### 7.2 - Paramètres Hough

```bash
ros2 param set /hough_lines_node threshold 30
ros2 param set /hough_lines_node min_line_length 40
ros2 param set /hough_lines_node max_line_gap 15
```

#### 7.3 - Paramètres de détection rectangle

```bash
ros2 param set /rectangle_detector_node width_tolerance 0.4
ros2 param set /rectangle_detector_node height_tolerance 0.4
ros2 param set /rectangle_detector_node angle_tolerance 20.0
```

---

### ✅ **ÉTAPE 8 : Documentation et Nettoyage**
**Objectif** : Finaliser et documenter

#### 8.1 - Créer fichiers de configuration YAML

**Fichier** : `ros2_bluerov/src/tracking/config/rectangle_detector_params.yaml`
```yaml
rectangle_detector_node:
  ros__parameters:
    enable_detection: true
    expected_cage_width: 1.0
    expected_cage_height: 1.5
    width_tolerance: 0.3
    height_tolerance: 0.3
    angle_tolerance: 15.0
    min_lines_required: 4
    use_tracked_position: true
```

#### 8.2 - Mettre à jour le README

Documenter :
- Nouveau nœud `rectangle_detector_node`
- Nouveau message `DetectedRectangle`
- Nouveau topic `/docking/tracking/detected_rectangle`
- Paramètres de configuration

#### 8.3 - Ajouter des logs informatifs

Dans chaque nœud, ajouter des logs pour faciliter le debug :
```python
self.get_logger().info(f'Rectangle détecté: {width:.2f}x{height:.2f}m, confiance={confidence:.2f}')
```

---

## 📊 Checklist Finale

### Fonctionnalités
- [ ] Filtre spatial gaussien opérationnel
- [ ] Détection Hough optimisée
- [ ] Détection de rectangle robuste
- [ ] Affichage du rectangle en temps réel
- [ ] Tous les launch files mis à jour

### Tests
- [ ] Test avec sonar mock
- [ ] Test avec rosbag
- [ ] Test avec sonar réel
- [ ] Tests de robustesse (mouvement, rotation, etc.)

### Documentation
- [ ] README mis à jour
- [ ] Fichiers YAML de configuration créés
- [ ] Commentaires dans le code
- [ ] Ce plan archivé pour référence

---

## 🔧 Dépannage Courant

### Problème : Pas de rectangle détecté
- Vérifier que Hough détecte au moins 4 lignes : `ros2 topic echo /docking/tracking/detected_lines`
- Augmenter `num_lines` dans hough_lines_node
- Réduire `threshold` dans hough_lines_node
- Augmenter les tolérances dans rectangle_detector

### Problème : Rectangle instable (saute d'une frame à l'autre)
- Filtrer temporellement les détections (moyenne glissante)
- Augmenter la pondération de `proximity_score`
- Réduire `max_line_gap` dans Hough

### Problème : Filtre spatial ne s'applique pas
- Vérifier que `enable_spatial_filter` est à `true`
- Vérifier que le tracker publie : `ros2 topic echo /docking/tracking/tracked_object`
- Vérifier les logs de `traitement_cartesian_node`

---

## 🎓 Concepts Clés

### Filtre Gaussien Spatial
- **But** : Isoler la zone d'intérêt (cage)
- **Formule** : `mask = exp(-(dist² / (2σ²)))`
- **Effet** : Atténuation progressive (pas binaire)

### Transformée de Hough
- **But** : Détecter des lignes dans une image
- **Entrée** : Image binaire (Canny)
- **Sortie** : Lignes en coordonnées polaires (ρ, θ)

### Détection de Rectangle
- **Méthode** : Intersection de lignes
- **Contraintes** : Dimensions attendues, parallélisme
- **Score** : Combinaison de précision dimensionnelle et confiance

---

**Temps estimé total** : 6-8 heures (avec tests et ajustements)

**Ordre de priorité** :
1. Étape 1 (filtre spatial) - **Critique**
2. Étape 3 (détection rectangle) - **Critique**
3. Étape 4 (affichage) - **Importante**
4. Étapes 2, 5, 6, 7 (optimisation/tests) - **Amélioration continue**
