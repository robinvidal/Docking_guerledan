# Frame.msg - Données brutes du sonar Oculus M750d
# Représente un ping sonar complet

std_msgs/Header header

# Paramètres de tir sonar
uint32 range_count          # Nombre de bins en distance
uint32 bearing_count        # Nombre de faisceaux angulaires
float32 range_resolution    # Résolution en distance (m)
float32 bearing_resolution  # Résolution angulaire (rad)
float32 min_range           # Distance minimale (m)
float32 max_range           # Distance maximale (m)

# Données d'intensité (range_count * bearing_count)
# Tableau 2D aplati: intensité[bearing][range]
uint8[] intensities

# Métadonnées
float32 sound_speed         # Vitesse du son utilisée (m/s)
uint8 gain                  # Gain appliqué (0-100)
