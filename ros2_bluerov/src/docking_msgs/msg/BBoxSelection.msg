# BBoxSelection.msg - Sélection manuelle de bounding box
# Coordonnées en pixels de la bounding box dessinée par l'utilisateur

std_msgs/Header header

# Bounding box en pixels (coin supérieur gauche + dimensions)
int32 x
int32 y
int32 width
int32 height

# Validité de la sélection
bool is_valid
