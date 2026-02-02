# DetectedLines.msg - Lignes détectées par transformée de Hough
# Représente les n meilleures lignes dans l'espace cartésien

std_msgs/Header header

# Validité de la détection
bool is_valid

# Nombre de lignes détectées
uint32 num_lines

# Paramètres des lignes en coordonnées polaires (rho, theta)
# rho: distance perpendiculaire de la ligne à l'origine (m)
# theta: angle de la normale à la ligne (rad)
float32[] rhos
float32[] thetas

# Points d'intersection avec les bordures de l'image (pour affichage)
# Chaque ligne a 2 points: (x1, y1) et (x2, y2) en mètres
float32[] x1_points
float32[] y1_points
float32[] x2_points
float32[] y2_points

# Scores de confiance (votes normalisés)
float32[] confidences
