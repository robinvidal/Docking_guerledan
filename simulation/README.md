# Théorie du contrôle — Simulation BlueROV

Ce document explique la théorie du contrôle implémentée dans la simulation BlueROV. Il ne décrit pas le code ligne à ligne mais les lois de commande, les conventions de repères et surtout les étapes mathématiques pour calculer le point cible (le point situé à 1 m devant le "cage" formée par deux bouées).

## Hypothèses et capteurs
- Le robot n'a PAS de position globale (pas de GPS) : il n'utilise que des mesures locales.
- Capteurs disponibles (par robot) : pour chaque bouée visible, on connaît le **bearing** relatif (angle) et la **distance** (range).
- Champ de vision (FOV) simulé : ±45° autour de l'axe avant du robot. Si une bouée sort du FOV, elle est considérée invisible.
- Conventions d'angle : la simulation utilise une convention de type navigation pour `self.heading` (0 = Nord). Les fonctions trigonométriques mathématiques attendent 0 = Est. Il faut donc convertir quand on mélange repères.

## Objectif de contrôle
Faire en sorte que le robot rejoigne le point cible défini comme : le point situé à `d = 1.0 m` devant le centre de la ligne qui relie les deux bouées (la "cage").

Les objectifs sont découpés en deux commandes découplées et indépendantes :
1. Contrôle d'orientation (cap) : garder le centre de la cage au centre du champ de vision.
2. Contrôle de déplacement (avant / latéral) : approcher le point cible exprimé dans le repère du robot.

Ces deux boucles tournent en parallèle lorsque les deux bouées sont visibles.

## Machine d'état
- `search` : tourner (rotation) jusqu'à ce que les deux bouées soient visibles.
- `tracking_and_navigation` : appliquer les deux contrôleurs décrits ci-dessous.

Condition d'arrêt (pragmatique) : quand la distance au point cible est inférieure à 0.1 m (10 cm), on considère le robot arrivé et on coupe les commandes.

## Notations et repères
- Observations de bouée i : $(\phi_i, r_i)$ où $\phi_i$ est le bearing relatif (radians) dans le repère du robot et $r_i$ la distance.
- Repère du robot : origine au robot, axe X local pointant vers l'avant du robot, axe Y local vers la droite (ou la gauche selon convention — dans la simulation on utilise X avant, Y latéral).
- Quand on a besoin d'appliquer cos/sin dans le repère monde on convertit l'angle de heading par :

  $$\theta_{math} = \text{heading}_{nav} + \frac{\pi}{2}$$

  (cela aligne 0 = Est pour les fonctions trigonométriques).

## Calcul du point cible — étapes
1. Pour chaque bouée visible (i = 1,2) reconstruire sa position exprimée dans le repère du robot :

   $$x_i = r_i\cos(\phi_i)\\
   y_i = r_i\sin(\phi_i)$$

   Ces coordonnées sont locales au robot (le robot est en $(0,0)$ dans ce repère).

2. Calculer le centre de la cage (milieu du segment entre les deux bouées) :

   $$x_c = \frac{x_1 + x_2}{2},\\
   y_c = \frac{y_1 + y_2}{2}$$

3. Calculer le vecteur allant de la première bouée vers la seconde :

   $$v = (x_2 - x_1,\; y_2 - y_1)$$

   Sa longueur : $\|v\| = \sqrt{(x_2-x_1)^2 + (y_2-y_1)^2}$.

4. Calculer le vecteur perpendiculaire normalisé à la ligne des bouées. L'orientation retenue doit pointer "devant" la cage (on choisit par convention) :

   $$p = \frac{1}{\|v\|}(-v_y,\; v_x)$$

   Si $\|v\| \approx 0$, on choisit un vecteur perpendiculaire par défaut (par ex. $(0,1)$).

5. Le point cible local (dans le repère du robot) est positionné devant le centre de la cage d'une distance $d$ :

   $$T = (x_T, y_T) = (x_c, y_c) + d\cdot p$$

   avec $d = 1.0\,\mathrm{m}$ dans la simulation.

Remarque : toutes ces opérations sont faites dans le **repère du robot**, puisque le robot ne connaît pas sa position globale.

## Lois de commande (lois proportionnelles simples)
Les lois implémentées sont volontairement simples (P-contrôleur). Elles sont découplées :

1. Contrôleur d'orientation (rotation)
   - On cherche à centrer le centre de la cage dans l'axe visuel du robot. Le bearing moyen vers les deux bouées donne une estimation de l'offset angulaire du centre :

     $$\phi_c = \frac{\phi_1 + \phi_2}{2}$$

   - Loi proportionnelle :

     $$\omega = k_{p,\theta}\cdot\phi_c$$

     avec $k_{p,\theta}$ le gain de rotation (positif). Cette commande est exprimée en rad/s.

2. Contrôleur déplacement (avant / latéral)
   - On calcule l'erreur de position locale vers le point cible :

     $$e = T - R = (x_T, y_T) - (0,0) = (x_T, y_T)$$

     (le robot est l'origine du repère local)

   - On applique des lois proportionnelles indépendantes sur chaque axe :

     $$v_{forward} = k_{p,x}\cdot e_x\\
     v_{lateral} = k_{p,y}\cdot e_y$$

     où $v_{forward}$ est la vitesse le long de l'axe avant du robot, et $v_{lateral}$ la vitesse latérale.

   - Les commandes sont limitées à des valeurs maximales pour éviter des mouvements irréalistes.

3. Découplage et raisonnement
   - L'orientation est gérée indépendamment de la translation : l'angle vise à garder la cage au centre du FOV pendant que la translation converge vers le point cible.
   - Ce découplage simplifie le contrôle et convient tant que les vitesses restent petites (petites inclinaisons d'erreur angulaire). Pour des erreurs angulaires très grandes, on préfère réduire temporairement la composante de translation.

## Conversions et application des commandes
- Les vitesses calculées (forward, lateral) sont exprimées dans le repère local du robot. Pour mettre à jour la position globale on convertit en vitesse monde via une rotation d'angle $\theta_{math}=\text{heading}+\pi/2$ :

  $$v_x^{world} = v_{forward}\cos(\theta_{math}) - v_{lateral}\sin(\theta_{math})\\
  v_y^{world} = v_{forward}\sin(\theta_{math}) + v_{lateral}\cos(\theta_{math})$$

- L'orientation est intégrée directement : $\text{heading} += \omega\cdot dt$.

## Cas limites et comportements
- Si une ou les deux bouées ne sont pas visibles :
  - En cas d'une bouée manquante, la simulation bascule en état `search` (rotation) pour retrouver la bouée manquante.
  - Si aucune bouée visible, on continue la recherche (on tourne dans la dernière direction connue).
- Si la distance entre les bouées est extrêmement petite (division par zéro potentielle), on utilise une direction perpendiculaire par défaut.
- Arrêt quand la distance au point cible est < 0.1 m.

## Pourquoi cette architecture ?
- Simplicité : deux contrôleurs P indépendants suffisent pour des trajectoires lisses en simulation et dans un contexte de faible dynamique.
- Robustesse locale : en utilisant uniquement des mesures locales (bearing + range), le robot peut atteindre le point cible sans cartographie ou localisation globale.
- Modulaire : on peut remplacer chaque bloc (ex. contrôleur angulaire par un PID, ou le contrôleur de position par un MPC) sans toucher à l'autre.

---

Si vous voulez que je complète ce README par un schéma mathématique plus poussé (stabilité, choix des gains, diagramme de phase) ou des recommandations sur le réglage des gains pour la simulation réelle, dites-le et j'ajouterai une section dédiée.