# Borders.msg - Bords de la cage détectés par tracking
# Représente les 4 montants verticaux de la cage en coordonnées polaires

std_msgs/Header header

# Détection des 4 bords (montants verticaux)
# Chaque bord: distance (m) et angle (rad) depuis le sonar
float32[] ranges      # Taille: 4 (un par montant)
float32[] bearings    # Taille: 4 (un par montant)

# Qualité de détection
float32[] confidences # Confiance par bord (0.0-1.0)
bool is_valid         # True si les 4 bords sont détectés

# Métadonnées géométrie cage
float32 cage_width    # Largeur cage détectée (m)
float32 cage_depth    # Profondeur cage détectée (m)
