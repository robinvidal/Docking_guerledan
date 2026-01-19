# TrackedObject.msg - Résultat du tracking d'un objet
# Position et bounding box de l'objet suivi

std_msgs/Header header

# État du tracking
bool is_tracking          # True si le tracker suit activement
bool is_initialized       # True si le tracker a été initialisé

# Position du centre de la bounding box (en mètres)
float32 center_x          # Position X du centre (m)
float32 center_y          # Position Y du centre (m)

# Dimensions de la bounding box (en mètres)
float32 width             # Largeur (m)
float32 height            # Hauteur (m)

# Bounding box en pixels (pour debug)
int32 bbox_x              # Coin supérieur gauche X (pixels)
int32 bbox_y              # Coin supérieur gauche Y (pixels)
int32 bbox_width          # Largeur (pixels)
int32 bbox_height         # Hauteur (pixels)

# Confiance du tracking (0.0 - 1.0)
float32 confidence
