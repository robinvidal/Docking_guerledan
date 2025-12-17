# FrameCartesian.msg - Données sonar en projection cartésienne
# Représente un ping sonar converti en grille cartésienne 2D

std_msgs/Header header

# Dimensions de l'image cartésienne
uint32 width           # Largeur de l'image (pixels)
uint32 height          # Hauteur de l'image (pixels)

# Résolution spatiale
float32 resolution     # Résolution en mètres par pixel
float32 min_range      # Distance minimale (m)
float32 max_range      # Distance maximale (m)

# Position du ROV dans l'image (origine)
uint32 origin_x        # Position X du ROV (pixels)
uint32 origin_y        # Position Y du ROV (pixels)

# Données d'intensité (width * height)
# Tableau 2D aplati: intensité[y][x]
uint8[] intensities

# Métadonnées du sonar source
float32 sound_speed    # Vitesse du son utilisée (m/s)
uint8 gain             # Gain appliqué (0-100)
float32 total_angle    # Angle total couvert par le sonar (rad)
