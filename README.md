# Docking — projet BlueROV

Résumé
------
Docking est un projet dont l'objectif est de faire entrer un BlueROV dans une cage. Le BlueROV utilisé est une version "heavy" équipée de 8 moteurs. Un sonar frontal (Oculus M750d) est monté pour la détection et le guidage.


Description rapide du matériel
------------------
- Véhicule : BlueROV heavy (8 moteurs).
![BlueROV](docs/images/BlueROV.png)
- Capteurs : sonar frontal Oculus M750d.
- Objectif : faire entrer le BlueROV dans une cage de dimensions 70 × 80 × 50 cm (modifiable).
- Cage : fabriquée en profilé aluminium, avec mousse collée sur les bords pour augmenter la signature acoustique.
![Cage](docs/images/Cage.png)

Arborescence liée
-----------------
- [`Noeuds/`](Noeuds/) : code des nœuds (commande, IHM, lancement, tracking).
- [`ros2_bluerov/`](ros2_bluerov/) : dossier contenant l'explication et le code ROS2.
- [`Simulation/`](Simulation/) : dossier expliquant la simulation et contenant les modèles et scénarios.

Équipe
------
École : ENSTA Bretagne

- Élève 1 : Robin VIDAL
- Élève 2 : Clément DUNOT
- Élève 3 : Thomas BOURGEOIS
- Élève 4 : Maxime LEFÈVRE

Encadrant : Christophe VIEL
