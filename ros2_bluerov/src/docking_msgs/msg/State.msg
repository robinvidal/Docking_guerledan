# State.msg - État de la mission de docking
# Machine d'états du système

std_msgs/Header header

# États possibles
uint8 IDLE = 0           # Inactif, attente commande
uint8 LOCK_ON = 1        # Acquisition cage en cours
uint8 APPROACH = 2       # Approche vers cage
uint8 DOCKING = 3        # Phase finale d'amarrage
uint8 DOCKED = 4         # Amarré avec succès
uint8 RECOVERY = 5       # Récupération après perte tracking
uint8 ABORT = 6          # Mission annulée

uint8 current_state      # État actuel

# Conditions de transition
bool cage_detected       # Cage visible par sonar
bool pose_valid          # Localisation fiable
bool alignment_ok        # ROV aligné pour docking
bool contact_detected    # Contact physique détecté

# Progression
float32 progress         # Progression mission (0.0-1.0)
string status_message    # Message lisible pour opérateur

# Commande externe
bool abort_requested     # True si opérateur demande arrêt
