# ClickPosition.msg - Position d'un clic souris sur le sonar
# Coordonnées en mètres dans le repère cartésien du sonar

std_msgs/Header header

# Position du clic en mètres
float32 x        # Position latérale (m), positif = droite
float32 y        # Position frontale (m), positif = devant

# Position en pixels (pour debug)
int32 pixel_x
int32 pixel_y

# Validité du clic (dans la zone du sonar)
bool is_valid
