# RotatedBBoxInit.msg - Initialisation d'une bounding box orientée
# Définit un rectangle orienté à partir de 4 coins dans l'espace sonar

std_msgs/Header header

# Quatre coins définissant le rectangle (en mètres dans le repère sonar)
# Ordre: sens horaire en partant du premier coin
# Point 1: Premier coin (ex: coin supérieur gauche)
float32 p1_x
float32 p1_y

# Point 2: Deuxième coin adjacent au premier (ex: coin supérieur droit)
float32 p2_x
float32 p2_y

# Point 3: Troisième coin (ex: coin inférieur droit)
float32 p3_x
float32 p3_y

# Point 4: Quatrième coin (ex: coin inférieur gauche)
# Si non fourni (0, 0), calculé automatiquement: P4 = P2 + P3 - P1
float32 p4_x
float32 p4_y

bool is_valid  # True si les points sont valides
