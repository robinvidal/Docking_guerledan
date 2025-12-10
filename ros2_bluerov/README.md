# ROS2 BlueROV Docking Pipeline
## Vue d’ensemble du workspace ROS2

Pipeline complet: Sonar → Filtrage → Détection → Localisation → Contrôle → Mission.  
Les messages et utilitaires communs sont dans `docking_msgs` et `docking_utils`. Les scénarios de lancement (dont `user_pipeline`) sont dans `bringup`.

## 📦 Packages

- 🟥 **Pas commencé**
- 🟧 **Commencé**
- 🟨 **Bien avancé**
- 🟩 **Fonctionnel**
- 🟦 **Terminé**

| Package | Description | Status |
|---------|-------------|--------|
| [sonar](src/sonar/README.md) | Acquisition données sonar (mock + interface Oculus + lecture des fichiers .oculus) | 🟨 |
| [traitement](src/traitement/README.md) | Pipeline de filtrage d'images | 🟩 |
| [tracking](src/tracking/README.md) | Détection bords de cage | 🟩 |
| [localisation](src/localisation/README.md) | Calcul pose relative 6DOF | 🟥 |
| [control](src/control/README.md) | Asservissement PID multi-axes | 🟥 |
| [mission](src/mission/README.md) | Machine d'états de docking | 🟥 |
| [docking_msgs](src/docking_msgs/README.md) | Messages custom (Frame, Borders, Pose, State) | 🟩 |
| [docking_utils](src/docking_utils/README.md) | Bibliothèque utilitaires (filtres, géométrie) | 🟩 |
| [bringup](src/bringup/README.md) | Launch files et configuration | 🟨 |
| [affichage](src/affichage/README.md) | Interface visualisation |  🟩 |

### Détails par package (en bref)

- **sonar**
       - Rôle: génère des frames sonar (mock) et, à terme, interface avec l’Oculus M750d.
       - Topics: publie `/docking/sonar/raw`; en simulation, réagit à `/bluerov/cmd_vel`.
       - Paramètres: `publish_rate`, `range_count`, `bearing_count`, `cage_distance`, `noise_level`, etc.

- **traitement**
       - Rôle: applique du filtrage (médian/gaussien, compensation de portée, contraste) aux frames sonar.
       - Topics: souscrit `/docking/sonar/raw`, publie `/docking/sonar/filtered`.

- **tracking**
       - Rôle: détecte les montants de la cage via projection angulaire et détection de pics; calcule confiance et largeur estimée.
       - Topics: souscrit `/docking/sonar/filtered`, publie `/docking/tracking/borders`.

- **localisation**
       - Rôle: calcule la pose relative (x, y, yaw, voire 6DOF) du ROV vis‑à‑vis du centre de la cage, avec validation/covariance.
       - Topics: souscrit `/docking/tracking/borders`, publie `/docking/localisation/pose`.

- **control**
       - Rôle: asservissement PID (x, y, yaw), limites de vitesses, anti‑windup basique.
       - Topics: souscrit `/docking/localisation/pose` et `/docking/mission/state`, publie `/cmd_vel` (souvent remappé vers `/bluerov/cmd_vel`).

- **mission**
       - Rôle: machine d’états (IDLE → LOCK_ON → APPROACH → DOCKING → DOCKED + RECOVERY/ABORT).
       - Topics: souscrit tracking/localisation, publie `/docking/mission/state`.

- **docking_msgs**
       - Rôle: messages ROS2 spécifiques (Frame, Borders, PoseRelative, State).
       - Build: doit être compilé en premier si problème de génération d’IDL.

- **docking_utils**
       - Rôle: librairie Python (conversions coordonnées, filtres signal, géométrie de cage, TF utils).

- **bringup**
       - Rôle: fichiers de lancement orchestrant des scénarios (mock complet, détection seule, sonar seul, et `user_pipeline`).

- **affichage**
       - Rôle: visualisation temps réel (à compléter). Vous pouvez provisoirement utiliser PlotJuggler ou rqt pour monitorer.

## 🔧 Installation des dépendances Python (Linux, bash)

Assurez-vous d’avoir ROS2 Humble sourcé et Python 3.10 dispo.

```bash
# Aller dans le workspace
cd ~/Desktop/Docking_guerledan/ros2_bluerov

# Dépendances Python via requirements.txt
pip install -r requirements.txt
```

Puis build le workspace:

```bash
cd ~/Desktop/Docking_guerledan/ros2_bluerov
colcon build
source install/setup.bash
```

Astuce: si build cassé sur les messages, build sélectif:

```bash
colcon build --packages-select docking_msgs
colcon build --packages-select docking_utils
colcon build
```

## Option 1: Lancer la simulation complète avec téléop clavier

Le launch `user_pipeline` démarre la pipeline utile en simulation pour un utilisateur (sonar mock + traitement + tracking + localisation + mission + control, avec les bons remaps/params).

1) Ouvrez un terminal (sondé bash) et lancez le pipeline:

```bash
# ! Chemin à adapter selon votre installation !
cd ~/Desktop/Docking_guerledan/ros2_bluerov
source install/setup.bash

ros2 launch bringup user_pipeline.launch.py
```

2) Ouvrez un deuxième terminal pour le téléop clavier (remappé vers le topic du ROV simulé):

```bash
# Installer le paquet téléop si nécessaire (debian package ROS Humble)
sudo apt-get update
sudo apt-get install ros-humble-teleop-twist-keyboard
```


```bash
# Lancer le téléop et remapper vers /bluerov/cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
       --ros-args -r /cmd_vel:=/bluerov/cmd_vel
```

Commandes utiles dans la fenêtre de téléop:
- i / , : avancer / reculer
- J / L : strafe gauche / droite
- j / l : tourner gauche / droite
- k : stop
- w/x : augmenter/diminuer vitesse linéaire
- e/c : augmenter/diminuer vitesse angulaire

3) (Optionnel) Monitoring dans un troisième terminal:

```bash
# État de mission
ros2 topic echo /docking/sonar/raw

# Pose relative estimée
ros2 topic echo /docking/tracking/borders

# Fréquences
ros2 topic hz /docking/sonar/raw
ros2 topic hz /docking/sonar/filtered
```

## Option 2: Lancer avec le vrai sonar Oculus M750d

Le launch `sonar_pipeline` démarre le vrai sonar (sonar + traitement + tracking + affichage). Utile pour tests réels avec le BlueROV et l’Oculus.

1) Ouvrez un terminal (sondé bash) et lancez le pipeline:

```bash
# ! Chemin à adapter selon votre installation !
cd ~/Desktop/Docking_guerledan/ros2_bluerov
source install/setup.bash

ros2 launch bringup sonar_pipeline.launch.py
```

2) (Optionnel) Monitoring dans un troisième terminal:

```bash
# État de mission
ros2 topic echo /docking/sonar/raw

# Pose relative estimée
ros2 topic echo /docking/tracking/borders

# Fréquences
ros2 topic hz /docking/sonar/raw
ros2 topic hz /docking/sonar/filtered
```

## Architecture

```
┌──────────────┐
│  sonar_mock  │  Génère frames synthétiques 256×512 @ ~10Hz
└──────┬───────┘
       │ /docking/sonar/raw
       ▼
┌──────────────────┐
│ traitement_node  │ Médian + Gaussien + Contraste + Compensation
└──────┬───────────┘
       │ /docking/sonar/filtered
       ▼
┌──────────────────┐
│  tracking_node   │ Détection montants (projection angulaire)
└──────┬───────────┘
       │ /docking/tracking/borders
       ▼
┌────────────────────┐
│ localisation_node  │ Calcul (x,y,yaw) + validation géométrique
└──────┬─────────────┘
       │ /docking/localisation/pose
       ▼
┌──────────────┐         ┌──────────────┐
│ mission_node │────────▶│ control_node │ PID (x, y, yaw)
└──────────────┘         └──────┬───────┘
 /docking/mission/state         │ /cmd_vel → (remap) /bluerov/cmd_vel
                                ▼
                        [ BlueROV simulé ]
```

## Notes

- Le launch `user_pipeline` suppose des remaps cohérents vers `/bluerov/cmd_vel` pour la simulation; le téléop doit remapper `/cmd_vel` vers ce topic.
- Si vous modifiez des paramètres, mettez à jour les YAML dans `bringup/config` ou les `config` propres à chaque package, puis relancez le launch.
