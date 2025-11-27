# Workspace ROS2 — Projet Docking

Ce workspace ROS2 contient les nœuds nécessaires pour réaliser l'accostage (Docking) automatique d'un BlueROV heavy (8 moteurs) dans une cage équipée d'une signature acoustique renforcée. Le sonar frontal Oculus M750d fournit les données brutes utilisées pour détecter, filtrer, suivre et approcher la cage.

## Objectif global
Transformer un flux sonar brut en commandes de déplacement robustes permettant au robot de :
1. Acquérir la cage (LOCK_ON)
2. S'aligner (orientation) et se centrer (position) (APPROACH)
3. Finaliser l'entrée (DOCKING)
Tout en permettant à l'opérateur de garder la main en mode manuel (IDLE).

## Vue d'ensemble des packages

1. sonar
	- Rôle : acquérir les données brutes du sonar et les publier sur un topic.
	- Entrées : driver Oculus (SDK, UDP ou série selon interface).
	- Sortie : message custom (ex : `sonar_msgs/Frame`) ou `sensor_msgs/Image` si conversion en image 2D polaire.

2. traitement
	- Rôle : filtrer et pré‑traiter (réduction bruit, enhancement des bords de la cage, normalisation intensité).
	- Entrées : topic brut du sonar.
	- Sortie : même format que sonar (brut amélioré) sur un second topic.

3. tracking
	- Rôle : extraire la signature de la cage après sélection initiale (opérateur) et suivre ses bords.
	- Entrées : flux filtré (ou brut), éventuelle ROI sélectionnée par l'opérateur.
	- Sortie : positions des bords (format dépendant : pixels (u,v) si image, sinon (r,theta)).

4. localisation
	- Rôle : convertir les bords de cage en position relative du robot et orientation par rapport au centre.
	- Entrées : liste de bords (r,theta) ou (pixels → transformés en (r,theta)).
	- Sortie : pose relative (x,y,heading) dans un frame défini (ex : `cage_frame`).

5. control
	- Rôle : calculer les commandes vitesse (vx, vy) et rotation (yaw_rate) via PID sur position et orientation.
	- Entrées : pose relative (localisation), état mission (active/inactive), paramètres PID.
	- Sortie : commandes (ex : `geometry_msgs/Twist`).

6. mission
	- Rôle : machine d'états centrale (IDLE → LOCK_ON → APPROACH → DOCKING → DONE/ABORT).
	- Décide quand activer les PID, quand rendre la main à l'opérateur, quand réinitialiser tracking.
	- Entrées : qualité tracking, distance cage, input opérateur.
	- Sortie : état courant + autorisation contrôle automatique.

7. affichage (bonus)
	- Rôle : visualiser en temps réel : sonar brut, sonar filtré, overlay tracking, pose estimée.
	- Peut servir à l'opérateur pour initialiser la zone de tracking.

## Flux de données (pipeline)

Driver Sonar → (sonar) → /sonar/raw → (traitement) → /sonar/filtered → (tracking) → /cage/borders → (localisation) → /cage/pose → (mission + control) → /cmd/vel → BlueROV

Optionnel : (affichage) souscrit à /sonar/raw, /sonar/filtered, /cage/borders, /cage/pose.

## Propositions de messages (à préciser)

- `sonar_msgs/Frame` : header (stamp, frame_id), paramètres (range_max, angle_span), data (array intensités), résolution angulaire.
- `cage_msgs/Borders` : tableau de points (r,theta) ou pixels si image.
- `cage_msgs/PoseRelative` : x (m), y (m), heading (rad), qualité (0–1).
- `mission_msgs/State` : enum (IDLE, LOCK_ON, APPROACH, DOCKING, DONE, ABORT), bool auto_active.

Utiliser `std_msgs/Header` pour synchronisation et faciliter TF.

## Frames et TF

- `base_link` : centre du robot.
- `sonar_link` : origine du sonar (offset en X,Y,Z par rapport à base_link).
- `cage_frame` : centre de la cage (référence pour localisation).

Localisation calcule la transform cage_frame → base_link (ou l'inverse selon convention).

## Machine d'états (mission)

IDLE : manuel, tracking arrêté ou en attente sélection initiale.
LOCK_ON : signature détectée, validation stabilité (N frames successives).
APPROACH : contrôle actif, réduire distance latérale et aligner orientation.
DOCKING : phase finale (réduction vitesse, tolérances serrées).
DONE : cage atteinte (critères position/orientation OK).
ABORT : perte de cible ou commande opérateur (retour IDLE).

## Paramètres clés (à mettre dans YAML)

- sonar.rate_hz, sonar.range_max
- traitement.filtre_type, traitement.contraste_gain
- tracking.stabilisation_frames, tracking.method (image|polar)
- localisation.cage_dimensions (L,W,H), localisation.seuil_validité
- control.pid_x (kp,ki,kd), pid_y, pid_yaw
- mission.lockon_min_frames, mission.abort_timeout

## Exemple de topics (suggestion)

```
/sonar/raw (sonar_msgs/Frame)
/sonar/filtered (sonar_msgs/Frame)
/cage/borders (cage_msgs/Borders)
/cage/pose (cage_msgs/PoseRelative)
/mission/state (mission_msgs/State)
/cmd/vel (geometry_msgs/Twist)
```

## Séparation des responsabilités

- sonar : I/O matériel + conversion format.
- traitement : pure image/signal processing.
- tracking : extraction entités (bords) + suivi ROIs.
- localisation : géométrie + transformation en pose relative.
- control : asservissement (aucune logique d'état global).
- mission : orchestration + conditions d'activation.
- affichage : interface opérateur / debugging, non critique runtime.

## Tests recommandés

1. Unitaires : chaque filtre (traitement), chaque calcul PID (control), conversion (localisation).
2. Intégration : pipeline complet sur enregistrement sonar (rosbag).
3. Simulation : injection frames synthétiques (cage simulée en paramètres connus).
4. Robustesse tracking : perte partielle de bord, bruit élevé, variations luminosité.

## Dépendances (à confirmer)

- rclpy, geometry_msgs, std_msgs, tf2_ros
- numpy, scipy, opencv-python (si traitement image)
- matplotlib (debug), pyyaml
- SDK Oculus M750d (lib propriétaire / wrapper Python)

## Lancement (exemple ROS2)

```
ros2 launch docking bringup.launch.py   # lance sonar + traitement + tracking + localisation + mission + control
ros2 run docking affichage_node         # (optionnel) affichage
```

Fournir un fichier `bringup.launch.py` regroupant les nœuds et paramètres.

## Sécurité / Fail-safe

- Timeout de données sonar → ABORT.
- Perte tracking > N secondes → repasser en IDLE.
- Distance trop proche mais orientation mauvaise → réduire vitesse + réacquisition.

## Améliorations possibles

1. Ajouter un module calibration sonar (gain adaptatif).
2. Implémenter un filtre multi-hypothèses pour tracking (éviter faux positifs).
3. Ajouter un module apprentissage (ML) pour détection cage sur frame brute.
4. Publier TF dynamique cage → base_link pour visualisation RViz.
5. Intégrer un simulateur acoustique (ray casting) pour tests hors ligne.
6. Mettre en place CI (tests + lint + ros2 humble/iron matrix).
7. Ajouter un enregistrement automatique rosbag lors des sessions réelles.

## Retours sur l'architecture proposée

Points forts :
- Découpage clair par fonction (acquisition, traitement, extraction, décision, asservissement).
- Machine d'états isolée (mission) facilitant extension (ajout mode RECOVERY).
- Module affichage séparé (pas de surcharge logique).

Points à surveiller / améliorer :
- tracking & localisation peuvent partager des conversions (risque de duplication) → créer un utilitaire commun.
- Format des messages : clarifier tôt (image vs polaire) pour éviter refactoring tardif.
- Latence : chaque étape ajoute un délai, envisager un traitement inline (traitement + tracking fusionnés) si perf critique.
- Robustesse : prévoir un mécanisme de réacquisition (LOCK_ON ← APPROACH si qualité tracking chute).
- Paramètres PID : nécessitent auto‑tuning ou au moins un script d'aide.

Suggestions :
- Ajouter un package commun `docking_utils` (math, conversions sonar→image, filtrage réutilisable).
- Définir un schéma de nommage des topics (`/docking/...`).
- Documenter les frames TF et les conventions (angle yaw positif, origine cage).

## Prochaines étapes (si validé)

1. Définir messages `.msg` et mettre en place package `docking_msgs`.
2. Créer `bringup.launch.py` + YAML paramètres.
3. Implémenter un prototype minimal : sonar (mock) → traitement (pass‑through) → localisation (fake) → control (print cmd).
4. Ajouter tests unitaires des conversions et PID.

