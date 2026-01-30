# TrackedObject.msg - Résultat du tracking d'un objet
# Position et bounding box de l'objet suivi

std_msgs/Header header

# État du tracking
bool is_tracking          # True si le tracker suit activement
bool is_initialized       # True si le tracker a été initialisé

# Position du centre de la bounding box (en mètres)
float32 center_x          # Position X du centre (m)
float32 center_y          # Position Y du centre (m)

# Position polaire par rapport au sonar
float32 range             # Distance au centre (m)
float32 bearing           # Angle par rapport à l'axe du sonar (radians)

# Orientation de la bounding box (pour tracking rotatif)
float32 angle             # Angle de rotation de la bbox (radians, 0 = horizontal)

# Dimensions de la bounding box (en mètres)
float32 width             # Largeur (m)
float32 height            # Hauteur (m)

# Bounding box en pixels (pour debug)
int32 bbox_x              # Coin supérieur gauche X (pixels)
int32 bbox_y              # Coin supérieur gauche Y (pixels)
int32 bbox_width          # Largeur (pixels)
int32 bbox_height         # Hauteur (pixels)

# Points d'entrée (2 premiers points cliqués - côté entrée de la cage)
float32 entry_p1_x        # Point 1 de l'entrée - coordonnée X (m)
float32 entry_p1_y        # Point 1 de l'entrée - coordonnée Y (m)
float32 entry_p1_range    # Point 1 de l'entrée - distance (m)
float32 entry_p1_bearing  # Point 1 de l'entrée - angle (radians)
float32 entry_p2_x        # Point 2 de l'entrée - coordonnée X (m)
float32 entry_p2_y        # Point 2 de l'entrée - coordonnée Y (m)
float32 entry_p2_range    # Point 2 de l'entrée - distance (m)
float32 entry_p2_bearing  # Point 2 de l'entrée - angle (radians)

# Confiance du tracking (0.0 - 1.0)
float32 confidence
