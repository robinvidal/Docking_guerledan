# PoseRelative.msg - Pose du ROV par rapport à la cage
# Calculée à partir des bords détectés

std_msgs/Header header

# Position relative (repère cage, origine = centre cage)
float32 x             # Décalage latéral (m, + = droite)
float32 y             # Distance frontale (m, + = devant)
float32 z             # Décalage vertical (m, + = haut)

# Orientation relative
float32 yaw           # Lacet (rad, 0 = aligné avec cage)
float32 pitch         # Tangage (rad)
float32 roll          # Roulis (rad)

# Qualité d'estimation
float32 confidence    # Confiance globale (0.0-1.0)
bool is_valid         # True si estimation fiable

# Covariance (6x6 aplati, ordre: x,y,z,roll,pitch,yaw)
float64[36] covariance
