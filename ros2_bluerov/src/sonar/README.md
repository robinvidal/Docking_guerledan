# Sonar Package

Package pour l'interface avec le sonar Oculus M750d et simulation de données sonar.

## Description

Ce package gère l'acquisition des données sonar pour la détection de la cage d'amarrage. Il fournit à la fois une interface pour le sonar réel et un mock pour le développement sans matériel.

Le simulateur sonar permet de tester le système complet avec une cage virtuelle qui réagit aux commandes de vitesse du ROV (physique inverse).

## 🚀 Démarrage Rapide

### Prérequis

```bash
# Installer teleop pour contrôle clavier
sudo apt-get install ros-humble-teleop-twist-keyboard

# Compiler le workspace
cd ~/Desktop/Docking_guerledan/ros2_bluerov
colcon build
```

### Lancement du Système Complet

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
- 🔍 **Sonar Filtré** : Après traitement (si traitement_node actif)
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

## 📊 Workflow Typique

1. **Lancer** les 3 terminaux (sonar_mock, sonar_viewer, teleop)
2. **Observer** la cage initiale dans l'onglet "📡 Sonar Brut" du viewer
3. **Appuyer sur `i`** dans teleop → La cage devrait se rapprocher
4. **Tester** les autres mouvements (strafe avec `J`/`L`, rotation avec `j`/`l`)
5. **Optionnel** : Aller dans l'onglet "⚙️ Contrôle Traitement" pour ajuster les filtres

## Nœuds

### `sonar_mock`

Simule un sonar Oculus M750d en générant des frames synthétiques avec une cage virtuelle qui réagit aux commandes du ROV.

**Topics souscrits:**
- `/bluerov/cmd_vel` (`geometry_msgs/Twist`) - Commandes de vitesse du ROV

**Topics publiés:**
- `/docking/sonar/raw` (`docking_msgs/Frame`) - Frames sonar synthétiques

**Paramètres:**
- `publish_rate` (float, défaut: 1.0) - Fréquence de publication (Hz)
- `range_count` (int, défaut: 512) - Nombre de bins de distance
- `bearing_count` (int, défaut: 256) - Nombre de faisceaux angulaires
- `bearing_angle` (float, défaut: 140.0) - Ouverture angulaire totale (degrés) = ±70°
- `min_range` (float, défaut: 1.0) - Distance minimale (m)
- `max_range` (float, défaut: 20.0) - Distance maximale (m)
- `cage_distance` (float, défaut: 4.0) - Distance initiale de la cage (m)
- `cage_width` (float, défaut: 1.0) - Largeur de la cage (m)
- `noise_level` (float, défaut: 10.0) - Niveau de bruit de fond (écart-type, 0-100)
- `cmd_vel_topic` (string, défaut: '/bluerov/cmd_vel') - Topic des commandes de vitesse
- Filtres : `enable_median`, `median_kernel`, `enable_gaussian`, `gaussian_sigma`, etc.

**Fonctionnalités:**
- Génération d'images sonar 2D polaires (bearing × range)
- Simulation de 4 montants verticaux de cage avec intensités élevées
- **Physique inverse** : La cage est fixe dans le monde, le ROV bouge
  - Si ROV avance (linear.x > 0), la cage se rapproche (y diminue)
  - Si ROV va à droite (linear.y > 0), la cage glisse à gauche (x diminue)
  - Si ROV tourne à gauche (angular.z > 0), la cage pivote à droite
- Bruit de fond réaliste (gaussien + speckle)
- Filtrage pré-appliqué (médian, gaussien, compensation de portée, contraste)
- Mise à jour physique à 50 Hz pour mouvement fluide

## Configuration

Fichier: `config/sonar_params.yaml`

```yaml
sonar_mock:
  ros__parameters:
    publish_rate: 1.0       # Hz (fréquence publication sonar)
    cage_distance: 4.0      # Distance initiale cage (m)
    cage_width: 1.0         # Largeur cage (m)
    noise_level: 10.0       # Bruit (0-100)
    bearing_angle: 140.0    # Ouverture ±70°
    cmd_vel_topic: '/bluerov/cmd_vel'  # Topic commandes
    
    # Filtres appliqués avant publication
    enable_median: true
    median_kernel: 3
    enable_gaussian: true
    gaussian_sigma: 2.0
```

## Format des données

Les frames sonar sont publiées au format `docking_msgs/Frame` avec:
- Grille polaire: `bearing_count × range_count`
- Intensités: tableau 1D (flatten de l'image 2D, row-major)
- Métadonnées: résolutions, plages min/max, vitesse du son

## 🐛 Dépannage

**Problème : La cage ne bouge pas quand j'appuie sur les touches**

✅ **Solution :**
- Vérifier que le topic dans teleop correspond : `--ros-args -r /cmd_vel:=/bluerov/cmd_vel`
- Vérifier que `cmd_vel_topic` dans `sonar_params.yaml` = `/bluerov/cmd_vel`
- Vérifier que des messages arrivent : `ros2 topic echo /bluerov/cmd_vel`
- Regarder les logs du sonar_mock : "Première commande reçue..." devrait apparaître

**Problème : Pas d'affichage dans sonar_viewer**

✅ **Solution :**
- Vérifier que sonar_mock est lancé et publie : `ros2 topic hz /docking/sonar/raw`
- Vérifier les logs du viewer : "raw_callback: mean intensity=..." devrait défiler

## TODO

- [ ] Interface avec le vrai sonar Oculus M750d via SDK
- [ ] Support de fichiers de replay (enregistrements réels)
- [ ] Ajout de patterns de simulation plus complexes (filets, structures)
- [ ] Simulation de courants marins (perturbations position cage)
