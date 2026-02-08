# Tracking Package

Package ROS2 pour la détection et le suivi de la cage d'amarrage dans les images sonar. Propose deux approches : détection géométrique (Hough) et tracking visuel (CSRT).

## Description

- **`hough_lines_node`** : Détecte les lignes dans l'image sonar via transformée de Hough, identifie une forme en U (cage), et publie sa pose filtrée.

- **`csrt_tracker_node`** : Tracker visuel OpenCV CSRT. Suit une région d'intérêt (bbox) sélectionnée manuellement via l'interface graphique (`affichage`) ou détectée automatiquement.

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/docking/sonar/cartesian_filtered` | `FrameCartesian` | Subscription | Image sonar cartésienne filtrée (entrée) |
| `/docking/tracking/detected_lines` | `DetectedLines` | Publication | Lignes détectées par Hough |
| `/docking/tracking/cage_pose` | `PoseStamped` | Publication | Pose de la cage (Hough) |
| `/docking/tracking/tracked_object` | `TrackedObject` | Publication | Position trackée (CSRT) |
| `/docking/sonar/bbox_selection` | `BBoxSelection` | Subscription | Bbox sélectionnée via interface `affichage` |
| `/docking/tracking/trigger_auto_detect` | `Bool` | Subscription | Déclenche la recherche auto |
| `/mavros/global_position/compass_hdg` | `Float32` | Subscription | Cap robot pour correction |

## Détails techniques

> Pour plus d'explications, voir les commentaires dans le code source.  
> Pour la configuration des paramètres, voir [tracking_params.yaml](config/tracking_params.yaml).

### Sélection manuelle de la bbox

La sélection manuelle de la bounding box s'effectue via l'interface graphique du package `affichage` :
1. Lancez le viewer : `ros2 run affichage main`
2. Dessinez un rectangle avec la souris autour de la cage sur l'image cartésienne
3. La bbox est automatiquement publiée sur `/docking/sonar/bbox_selection`
4. Le `csrt_tracker_node` reçoit cette bbox et initialise le tracking

### Détection de la forme en U (cage)

L'algorithme `detect_u_shape` cherche une cage parmi les lignes détectées :

1. **Triplet de lignes** : Teste toutes les combinaisons de 3 lignes (base + 2 bras)
2. **Perpendicularité** : Vérifie que les bras sont perpendiculaires à la base (tolérance `perp_tol`)
3. **Écartement** : Vérifie que la distance entre les milieux des bras ≈ `cage_width` (tolérance `dist_tol`)
4. **Connexion** : Vérifie que les extrémités des bras sont proches de la base (tolérance `conn_tol`)
5. **Score** : Garde le triplet avec la plus grande somme de longueurs
6. **Pose** : Calcule le barycentre des 6 points et l'angle vers le centre de la base

### Fusion des lignes similaires

L'algorithme `merge_similar_lines` évite les doublons :

1. **Tri** : Les lignes sont triées par longueur décroissante (priorité aux grandes)
2. **Clustering** : Pour chaque ligne, on fusionne celles ayant :
   - `|Δρ| < merge_rho_tolerance` (distance à l'origine similaire)
   - `|Δθ| < merge_theta_tolerance` (orientation similaire)
3. **Moyenne** : Le cluster fusionné a pour extrémités la moyenne des points

### Filtrage anti-saut (lissage temporel)

Évite les sauts brusques de position/orientation :

1. **Fenêtre glissante** : Historique des N dernières détections
2. **Détection outlier** : Si `|Δangle| > outlier_threshold_deg`, la mesure est rejetée
3. **Force-accept** : Après `max_consecutive_outliers` rejets, on accepte quand même (évite blocage)
4. **Moyenne vectorielle** : L'angle moyen est calculé via `atan2(Σsin, Σcos)` pour gérer le wrap-around ±π
