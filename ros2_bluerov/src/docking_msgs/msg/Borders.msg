# Borders.msg - Bords de la cage détectés par tracking
# Représente les 2 montants verticaux visibles de la cage en coordonnées polaires

std_msgs/Header header

# Détection des 2 bords (montants verticaux) visibles
# Chaque bord: distance (m) et angle (rad) depuis le sonar
float32[] ranges      # Taille: 2 (un par montant)
float32[] bearings    # Taille: 2 (un par montant)

# Qualité de détection
float32[] confidences # Confiance par bord (0.0-1.0), taille 2
bool is_valid         # True si les 2 bords sont détectés

# Métadonnées géométrie cage
float32 cage_width    # Largeur cage estimée (m)
float32 cage_depth    # Distance moyenne à la cage (m)
