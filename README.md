# Docking — projet BlueROV
# Docking — BlueROV Autonomous Docking (consolidated README)

Ce dépôt contient le workspace ROS2 et les éléments de simulation/contrôle pour un projet de docking autonome d'un BlueROV en utilisant un sonar frontal simulé.

Objectifs principaux
- Détecter une cage sous-marine et piloter le BlueROV pour s'y insérer automatiquement.
- Fournir un pipeline modulaire (acquisition → traitement → détection → localisation → contrôle → mission) testable en simulation.

Structure du workspace

```
Docking_guerledan/
├── ros2_bluerov/            # Workspace ROS2 (packages listés ci-dessous)
│   └── src/
│       ├── sonar
│       ├── traitement
│       ├── tracking
│       ├── localisation
│       ├── control
│       ├── mission
│       ├── docking_msgs
│       ├── docking_utils
│       ├── bringup
│       └── affichage
├── Noeuds/                  # Paquets utilitaires/historique (archivé)
├── Simulation/              # Scripts et notebooks de simulation
├── docs/
└── README.md                # (vous êtes ici)
```

Paquets principaux (une phrase chacun)
- `sonar` — Génère et publie des frames sonar synthétiques (mock) et gère l'interface vers un futur sonar réel.
- `traitement` — Pipeline de filtrage et amélioration d'image sonar (TVG, CLAHE, median, gaussian, etc.).
- `tracking` — Détection des bords/montants de la cage et extraction des frontières (/docking/tracking/borders).
- `localisation` — Estime la pose relative (x,y,yaw) de la cage depuis les bords détectés.
- `control` — Asservissements PID pour suivre la consigne de pose (publie sur `/cmd_vel`).
- `mission` — Machine d'états pour orchestrer les phases de docking (IDLE → LOCK_ON → APPROACH → DOCKING → DOCKED).
- `docking_msgs` — Définitions des messages custom utilisés par le pipeline (Frame, Borders, Pose, State).
- `docking_utils` — Fonctions utilitaires communes (filtres, conversions géométriques, helpers).
- `bringup` — Fichiers de lancement et configuration YAML pour démarrer le pipeline en simulation.
- `affichage` — Interface graphique (PyQt5/pyqtgraph) pour visualiser et piloter les paramètres en temps réel.

Archivé
- Les README individuels originaux ont été sauvegardés en `.bak` au même emplacement (ex: `ros2_bluerov/README.md.bak`, `Noeuds/README.md.bak`).

## 🚀 Démarrage en 5 minutes

### 1. Vérifier l'installation ROS2

```bash
# Vérifier version ROS2
echo $ROS_DISTRO  # Doit afficher: humble, iron, ou jazzy

# Si vide, sourcer ROS2
source /opt/ros/humble/setup.bash  # Adapter selon votre distro
```

### 2. Build le workspace

```bash
cd ros2_bluerov
colcon build --symlink-install
```

**Temps estimé**: 1-2 minutes

### 3. Sourcer l'environnement

```bash
source install/setup.bash  # ou setup.zsh
```

### 4. Lancer la simulation

Ouvrez **4 terminaux** et sourcez l'environnement dans chacun :

```bash
cd ~/Desktop/Docking_guerledan/ros2_bluerov
source install/setup.bash
```

#### Terminal 1 : Simulateur Sonar
```bash
ros2 run sonar sonar_mock --ros-args \
	--params-file install/sonar/share/sonar/config/sonar_params.yaml
```
Lance le sonar virtuel avec une cage à la position initiale configurée.

#### Terminal 2 : Visualiseur
```bash
ros2 run affichage sonar_viewer
```
Ouvre l'interface graphique avec :
- 📡 **Sonar Brut** : Vue cartésienne des données sonar
- 🔍 **Sonar Filtré** : Après traitement (si `traitement_node` actif)
- ⚖️ **Comparaison** : Côte à côte brut/filtré
- ⚙️ **Contrôle Traitement** : Réglage des filtres en temps réel

#### Terminal 3 : Contrôle Clavier (Teleop)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
	--ros-args -r /cmd_vel:=/bluerov/cmd_vel
```

**Commandes clavier :**
| Touche | Action | Effet sur sonar |
|--------|--------|-----------------|
| `i` | Avancer | Cage se rapproche |
| `,` | Reculer | Cage s'éloigne |
| `J` (Maj+j) | Strafe gauche | Cage glisse à droite |
| `L` (Maj+l) | Strafe droite | Cage glisse à gauche |
| `j` | Tourner gauche | Cage pivote à droite |
| `l` | Tourner droite | Cage pivote à gauche |
| `k` | Stop | Arrêt |
| `w/x` | ↑/↓ vitesse linéaire | |
| `e/c` | ↑/↓ vitesse angulaire | |

#### Terminal 4 (Optionnel) : Monitoring
```bash
# Vérifier les commandes publiées
ros2 topic echo /bluerov/cmd_vel

# Voir la position de la cage (logs du sonar_mock)
# Les logs affichent périodiquement : "Cage relative: x=..., y=..., θ=..."
```

## ✅ Vérifications rapides

```bash
ros2 topic list
ros2 node list
```

Doit contenir les topics et nœuds principaux (ex: `/docking/sonar/raw`, `/sonar_mock`, `/traitement_node`, `/tracking_node`, `/localisation_node`).

---

Si vous voulez que je pousse aussi une version plus détaillée (avec commandes de debug, exemples de `ros2 topic echo` et `rqt_graph`) ou que j'ajoute des badges/illustrations, dites-moi et je l'ajoute.
