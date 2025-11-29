# Mission Package

Package de gestion de la machine d'états pour la mission de docking autonome.

## Description

Ce package orchestre la séquence complète de docking autonome via une machine d'états. Il supervise les transitions entre phases, gère les erreurs et les récupérations, et coordonne l'activation des autres modules (contrôle notamment).

## Nœuds

### `mission_node`

Machine d'états à 7 états gérant le cycle complet de docking.

**Topics souscrits:**
- `/docking/tracking/borders` (`docking_msgs/Borders`) - Détection de cage
- `/docking/localisation/pose` (`docking_msgs/PoseRelative`) - Pose estimée
- `/docking/mission/abort` (`std_msgs/Bool`) - Commande d'annulation

**Topics publiés:**
- `/docking/mission/state` (`docking_msgs/State`) - État actuel de la mission

**Paramètres:**
- `lock_on_timeout` (float, défaut: 10.0) - Timeout acquisition cage (s)
- `approach_distance` (float, défaut: 1.0) - Distance de début de docking (m)
- `docking_distance` (float, défaut: 0.3) - Distance de contact final (m)
- `alignment_threshold` (float, défaut: 0.1) - Seuil d'alignement (rad, ~6°)

## Machine d'états

### Diagramme de transitions

```
        ┌──────┐
        │ IDLE │ ◄─────────────┐
        └───┬──┘                │
            │ cage_detected     │
            ▼                   │
      ┌──────────┐             │
      │ LOCK_ON  │             │
      └─┬────┬───┘             │
        │    └──► RECOVERY ────┤
        │ pose_valid            │
        ▼                       │
    ┌──────────┐                │
    │ APPROACH │ ◄──────────────┤
    └─┬────┬───┘                │
      │    └──► RECOVERY        │
      │ close + aligned         │
      ▼                         │
  ┌──────────┐                  │
  │ DOCKING  │                  │
  └─┬────┬───┘                  │
    │    └──► RECOVERY          │
    │ contact                   │
    ▼                           │
┌────────┐                      │
│ DOCKED │                      │
└────────┘                      │
                                │
[abort] ────────► ABORT ────────┘
```

### États

1. **IDLE** - Attente inactive
   - En attente de détection de cage
   - Contrôle désactivé
   - Transition: cage détectée → LOCK_ON

2. **LOCK_ON** - Acquisition de la cage
   - Tentative d'obtenir une pose valide
   - Timeout: 10s → RECOVERY
   - Transition: pose valide → APPROACH

3. **APPROACH** - Approche vers la cage
   - Contrôle PID actif
   - Vérification alignement et distance
   - Transition: proche + aligné → DOCKING
   - Perte pose → RECOVERY

4. **DOCKING** - Phase finale d'amarrage
   - Contrôle PID actif (consignes strictes)
   - Détection contact imminent
   - Transition: contact (y < 0.1m) → DOCKED
   - Perte pose → RECOVERY

5. **DOCKED** - Amarré avec succès
   - Mission terminée
   - Contrôle désactivé
   - État final

6. **RECOVERY** - Récupération après perte
   - Tentative de réacquisition pose
   - Timeout: 30s → ABORT
   - Transition: pose récupérée → APPROACH

7. **ABORT** - Mission annulée
   - Commande d'abort reçue ou timeout recovery
   - Contrôle désactivé
   - État final d'erreur

## Message de sortie

Le message `State` contient:
- **current_state** - État actuel (constante)
- **cage_detected** - Cage visible par tracking (bool)
- **pose_valid** - Pose estimée valide (bool)
- **alignment_ok** - ROV aligné avec cage (bool)
- **contact_detected** - Contact physique détecté (bool)
- **progress** - Progression mission [0-1]
- **status_message** - Description textuelle état
- **abort_requested** - Abort demandé (bool)

## Lancement

```bash
# Mission seule
ros2 run mission mission_node --ros-args --params-file config/mission_params.yaml

# Avec le pipeline complet
ros2 launch bringup mock_pipeline.launch.py

# Commande d'abort
ros2 topic pub /docking/mission/abort std_msgs/Bool "data: true" --once
```

## Configuration

Fichier: `config/mission_params.yaml`

```yaml
mission_node:
  ros__parameters:
    lock_on_timeout: 10.0
    approach_distance: 1.0
    docking_distance: 0.3
    alignment_threshold: 0.1  # ~6°
```

## Critères de transition

### LOCK_ON → APPROACH
- Pose valide reçue avec confiance suffisante

### APPROACH → DOCKING
- Distance ≤ `docking_distance` (0.3m)
- ET |x| < 0.2m (alignement latéral)
- ET |yaw| < `alignment_threshold` (6°)

### DOCKING → DOCKED
- Distance y < 0.1m (contact imminent)
- TODO: Capteur de force/contact réel

### * → RECOVERY
- Perte de pose valide pendant APPROACH ou DOCKING

### RECOVERY → ABORT
- Timeout 30s sans récupération

## Monitoring

```bash
# Observer l'état en temps réel
ros2 topic echo /docking/mission/state

# Visualiser graphiquement
ros2 run rqt_graph rqt_graph
```

## TODO

- [ ] Interface de démarrage (service ou bouton IHM)
- [ ] Détection de contact physique (IMU, capteur force)
- [ ] Logs détaillés pour post-analyse
- [ ] Métriques de performance (temps de docking, tentatives, etc.)
- [ ] Mode manuel/semi-automatique
- [ ] Gestion de zones interdites (obstacles)
