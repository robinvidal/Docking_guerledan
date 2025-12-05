# docking_msgs

Messages communs pour le système de docking autonome du BlueROV.

## Messages définis

### Frame.msg
Données brutes du sonar Oculus M750d. Contient les intensités du ping sonar complet avec métadonnées (résolution, portée, gain).

**Topic**: `/docking/sonar/raw`

### Borders.msg
Bords de la cage détectés par le module de tracking. Représente les 2 montants verticaux en coordonnées polaires avec confiances.

**Topic**: `/docking/tracking/borders`

### PoseRelative.msg
Pose 6DOF du ROV par rapport au centre de la cage, calculée par le module de localisation. Inclut covariance et validation.

**Topic**: `/docking/localisation/pose`

### State.msg
État de la machine d'états de la mission (IDLE, LOCK_ON, APPROACH, DOCKING, etc.) avec conditions de transition.

**Topic**: `/docking/mission/state`



## Usage (Python)

```python
from docking_msgs.msg import Frame, Borders, PoseRelative, State

# Exemple: créer un message Frame
frame = Frame()
frame.header.stamp = self.get_clock().now().to_msg()
frame.range_count = 512
frame.bearing_count = 256
# ...
```

## Usage (C++)

```cpp
#include "docking_msgs/msg/frame.hpp"
#include "docking_msgs/msg/borders.hpp"
// ...

auto frame = docking_msgs::msg::Frame();
frame.header.stamp = this->now();
// ...
```
