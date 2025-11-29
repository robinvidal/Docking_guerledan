# Control Package

Package d'asservissement PID pour le contrôle autonome du ROV pendant le docking.

## Description

Ce package implémente des contrôleurs PID sur 3 degrés de liberté (x, y, yaw) pour maintenir le ROV en position relative par rapport à la cage. Il génère des commandes de vitesse à partir de la pose estimée par la localisation.

## Nœuds

### `control_node`

Contrôleur PID multi-axes avec anti-windup pour l'asservissement en position.

**Topics souscrits:**
- `/docking/localisation/pose` (`docking_msgs/PoseRelative`) - Pose relative mesurée
- `/docking/mission/state` (`docking_msgs/State`) - État de la mission (active/désactive le contrôle)

**Topics publiés:**
- `/cmd_vel` (`geometry_msgs/Twist`) - Commandes de vitesse pour le ROV
  - `linear.x`: vitesse frontale (m/s)
  - `linear.y`: vitesse latérale (m/s)
  - `angular.z`: vitesse de lacet (rad/s)

**Paramètres PID:**

**Axe X (latéral):**
- `pid_x_kp` (float, défaut: 0.5) - Gain proportionnel
- `pid_x_ki` (float, défaut: 0.01) - Gain intégral
- `pid_x_kd` (float, défaut: 0.1) - Gain dérivé

**Axe Y (frontal):**
- `pid_y_kp` (float, défaut: 0.3) - Gain proportionnel
- `pid_y_ki` (float, défaut: 0.005) - Gain intégral
- `pid_y_kd` (float, défaut: 0.05) - Gain dérivé

**Yaw (lacet):**
- `pid_yaw_kp` (float, défaut: 1.0) - Gain proportionnel
- `pid_yaw_ki` (float, défaut: 0.02) - Gain intégral
- `pid_yaw_kd` (float, défaut: 0.2) - Gain dérivé

**Limitations:**
- `max_linear_speed` (float, défaut: 0.5) - Vitesse linéaire max (m/s)
- `max_angular_speed` (float, défaut: 0.5) - Vitesse angulaire max (rad/s)

## Architecture du contrôleur

### Classe `PIDController`

Implémentation d'un PID classique avec:
- **Anti-windup** - Arrêt de l'intégration en cas de saturation
- **Filtrage dérivé** - Calcul sur erreur filtrée
- **Réinitialisation** - Reset lors des changements d'état

### Logique d'activation

Le contrôle est actif uniquement pendant les états:
- `APPROACH` - Approche vers la cage
- `DOCKING` - Phase finale d'amarrage

Le contrôleur se désactive automatiquement dans les autres états et publie des commandes nulles.

## Consignes

Le contrôleur vise à amener le ROV vers la position (0, 0, 0) dans le repère cage:
- **x = 0** - Alignement latéral parfait
- **y = 0** - Contact avec la cage (attention: en pratique, consigne y > 0)
- **yaw = 0** - Orientation frontale

## Lancement

```bash
# Contrôle seul
ros2 run control control_node --ros-args --params-file config/control_params.yaml

# Avec le pipeline complet
ros2 launch bringup mock_pipeline.launch.py
```

## Configuration

Fichier: `config/control_params.yaml`

```yaml
control_node:
  ros__parameters:
    # PID X (latéral)
    pid_x_kp: 0.5
    pid_x_ki: 0.01
    pid_x_kd: 0.1
    
    # PID Y (frontal)
    pid_y_kp: 0.3
    pid_y_ki: 0.005
    pid_y_kd: 0.05
    
    # PID Yaw
    pid_yaw_kp: 1.0
    pid_yaw_ki: 0.02
    pid_yaw_kd: 0.2
    
    # Limitations
    max_linear_speed: 0.5
    max_angular_speed: 0.5
```

## Tuning des gains

**Recommandations:**
- **Kp** - Démarrer avec 0.5-1.0, ajuster selon réactivité
- **Ki** - Faible (0.01-0.05) pour éviter overshoots
- **Kd** - 10-20% de Kp pour amortissement

**Tests:**
1. Kp seul → réponse rapide mais oscillations
2. Ajouter Kd → amortissement
3. Ajouter Ki → élimination erreur statique

## Performance

- Temps de convergence typique: 5-10s depuis 5m
- Précision finale: ±10cm en position, ±5° en orientation
- Stabilité: Pas d'oscillations si bien tuné

## TODO

- [ ] Mode de contrôle avancé (MPC, sliding mode)
- [ ] Adaptation des gains selon distance
- [ ] Limitation de jerk (dérivée de l'accélération)
- [ ] Interface avec commandes bas-niveau BlueROV (thruster mapping)
