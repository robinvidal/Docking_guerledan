# 📐 Documentation des Transformations de Coordonnées - Pipeline Sonar Docking

**Version:** 1.0  
**Date:** 2026-02-03  
**Auteur:** Documentation générée par analyse du code source

## 🎯 Objectif
Ce document décrit **toutes les transformations géométriques** appliquées aux données sonar,
de l'acquisition jusqu'à l'affichage. Il suit les bonnes pratiques d'ingénierie pour assurer
la traçabilité et la cohérence des conventions de signes.

---

## 📊 Schéma Simplifié du Pipeline

```
┌──────────────┐    T1     ┌──────────────────┐    T2     ┌────────────────────┐
│  SDK Oculus  │ ───────▶  │   sonar_node     │ ───────▶  │ traitement_unified │
│  (hardware)  │ transpose │   /sonar/raw     │  polaire  │ /sonar/cartesian   │
│              │  .T.ravel │   (Frame msg)    │  → cart.  │ (FrameCartesian)   │
└──────────────┘           └──────────────────┘           └─────────┬──────────┘
                                                                    │
                                      ┌─────────────────────────────┴─────────┐
                                      │                                       │
                               T4     ▼                                T5     ▼
                           ┌─────────────────┐                    ┌──────────────────┐
                           │ csrt_tracker    │                    │ sonar_cartesian  │
                           │ cv2.flip(img,0) │                    │ np.rot90(k=1)    │
                           │ (TrackedObject) │                    │ (affichage GUI)  │
                           └─────────────────┘                    └──────────────────┘
```

---

## 📊 Schéma du Pipeline de Données

```
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                              PIPELINE DE TRANSFORMATIONS                                │
└─────────────────────────────────────────────────────────────────────────────────────────┘

                                    CONVENTION DE REPÈRE
                                    ━━━━━━━━━━━━━━━━━━━━
                                           +Y (avant)
                                            ↑
                                            │
                                            │
                                   -X ←─────┼─────→ +X
                                   (gauche) │    (droite)
                                            │
                                           ROV
                                    
                                    Bearing > 0 : droite
                                    Bearing < 0 : gauche
                                    Range : distance depuis ROV

┌─────────────────────────────────────────────────────────────────────────────────────────┐
│ ÉTAPE 1: ACQUISITION SONAR (sonar_node.py)                                              │
├─────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                         │
│   SDK Oculus                                                                            │
│   ┌─────────────────┐                                                                   │
│   │ raw_ping_data() │ → Matrice (n_range × n_bearing)                                   │
│   │ bearing_data()  │ → Bearings en centièmes de degrés (SDK convention)                │
│   └─────────────────┘                                                                   │
│                                                                                         │
│   ⚠️ TRANSFORMATION T1: Transpose + Ravel                                               │
│   ┌─────────────────────────────────────────────────────────────────────────┐           │
│   │  intensities = work.T.ravel()                                           │           │
│   │                                                                         │           │
│   │  Avant: work[range][bearing]  →  (n_range, n_bearing)                   │           │
│   │  Après: intensities[bearing][range]  →  (n_bearing × n_range) flat      │           │
│   │                                                                         │           │
│   │  ✅ JUSTIFICATION: Le message Frame attend un format bearing-major      │           │
│   │     pour que intensities[i*range_count + j] = pixel(bearing_i, range_j) │           │
│   └─────────────────────────────────────────────────────────────────────────┘           │
│                                                                                         │
│   📤 OUTPUT: Topic /docking/sonar/raw (Frame)                                           │
│      - intensities: array 1D, format [bearing][range] (bearing-major)                   │
│      - bearing_resolution: radians/pixel (toujours positif)                             │
│      - Les bearings vont de -FOV/2 à +FOV/2 (gauche à droite)                           │
└─────────────────────────────────────────────────────────────────────────────────────────┘
                                          │
                                          ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│ ÉTAPE 2: TRAITEMENT (traitement_unified_node.py)                                        │
├─────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                         │
│   📥 INPUT: /docking/sonar/raw                                                          │
│                                                                                         │
│   RECONSTRUCTION IMAGE POLAIRE:                                                         │
│   ┌─────────────────────────────────────────────────────────────────────────┐           │
│   │  polar_img = intensities.reshape((bearing_count, range_count))          │           │
│   │                                                                         │           │
│   │  Axes:                                                                  │           │
│   │    - Axe 0 (lignes)  = bearing (de -FOV/2 à +FOV/2)                     │           │
│   │    - Axe 1 (colonnes) = range (de min_range à max_range)                │           │
│   └─────────────────────────────────────────────────────────────────────────┘           │
│                                                                                         │
│   FILTRES POLAIRES: Frost → Médian (pas de transformation géométrique)                  │
│                                                                                         │
│   ⚠️ TRANSFORMATION T2: Conversion Polaire → Cartésien                                  │
│   ┌─────────────────────────────────────────────────────────────────────────┐           │
│   │  # Grille cartésienne de sortie                                         │           │
│   │  xs = linspace(-max_r, max_r, out_w)   # X: gauche(-) à droite(+)       │           │
│   │  ys = linspace(0, max_r, out_h)        # Y: ROV(0) à avant(max)         │           │
│   │                                                                         │           │
│   │  # Calcul des coordonnées polaires correspondantes                      │           │
│   │  rr = sqrt(xv² + yv²)                                                   │           │
│   │  th = arctan2(-xv, yv)     ⚠️ INVERSION DE X                            │           │
│   │                                                                         │           │
│   │  ✅ JUSTIFICATION de -xv:                                               │           │
│   │     - arctan2(y, x) standard mesure l'angle depuis +X                   │           │
│   │     - arctan2(x, y) mesure l'angle depuis +Y (convention sonar)         │           │
│   │     - arctan2(-x, y) inverse le sens: bearing+ = droite du ROV          │           │
│   │     - Ceci correspond à la convention maritime/sonar standard           │           │
│   │                                                                         │           │
│   │  # Mapping vers indices image polaire                                   │           │
│   │  i_float = (th + FOV/2) / FOV * (bc - 1)   # index bearing              │           │
│   │  j_float = (rr - min_r) / (max_r - min_r) * (rc - 1)  # index range     │           │
│   └─────────────────────────────────────────────────────────────────────────┘           │
│                                                                                         │
│   IMAGE CARTÉSIENNE RÉSULTANTE:                                                         │
│   ┌─────────────────────────────────────────────────────────────────────────┐           │
│   │  cart_img[row][col] où:                                                 │           │
│   │    - row 0 = Y proche du ROV (y=0)                                      │           │
│   │    - row max = Y loin du ROV (y=max_range)                              │           │
│   │    - col 0 = X gauche (x=-max_range)                                    │           │
│   │    - col max = X droite (x=+max_range)                                  │           │
│   │                                                                         │           │
│   │  origin_x = out_w // 2  (colonne centrale = position X du ROV)          │           │
│   │  origin_y = 0           (ligne 0 = position Y du ROV)                   │           │
│   └─────────────────────────────────────────────────────────────────────────┘           │
│                                                                                         │
│   ⚠️ TRANSFORMATION T3: Flip Configurable (optionnel)                                   │
│   ┌─────────────────────────────────────────────────────────────────────────┐           │
│   │  Paramètres: cart_flip_horizontal, cart_flip_vertical                   │           │
│   │                                                                         │           │
│   │  ⚠️ PAR DÉFAUT: False, False (désactivé)                                │           │
│   │                                                                         │           │
│   │  Si activés:                                                            │           │
│   │    - flip_horizontal: gauche ↔ droite (miroir axe Y)                    │           │
│   │    - flip_vertical: haut ↔ bas (miroir axe X)                           │           │
│   └─────────────────────────────────────────────────────────────────────────┘           │
│                                                                                         │
│   📤 OUTPUT: Topic /docking/sonar/cartesian_filtered (FrameCartesian)                   │
│      - intensities: array 1D, format [row][col] (row-major)                             │
│      - origin_x, origin_y: position du ROV dans l'image                                 │
│      - resolution: mètres/pixel                                                         │
└─────────────────────────────────────────────────────────────────────────────────────────┘
                                          │
                     ┌────────────────────┴────────────────────┐
                     ▼                                         ▼
┌────────────────────────────────────────┐   ┌────────────────────────────────────────────┐
│ ÉTAPE 3A: TRACKING (csrt_tracker_node) │   │ ÉTAPE 3B: AFFICHAGE (sonar_cartesian_      │
├────────────────────────────────────────┤   │                       display.py)          │
│                                        │   ├────────────────────────────────────────────┤
│ 📥 INPUT: /docking/sonar/              │   │                                            │
│           cartesian_filtered           │   │ 📥 INPUT: /docking/sonar/                  │
│                                        │   │           cartesian_filtered               │
│ ⚠️ TRANSFORMATION T4:                  │   │                                            │
│    Flip Vertical pour OpenCV           │   │ ⚠️ TRANSFORMATION T5:                      │
│ ┌────────────────────────────────────┐ │   │    Rotation 90° pour PyQtGraph             │
│ │ img = cv2.flip(img, 0)             │ │   │ ┌────────────────────────────────────────┐ │
│ │                                    │ │   │ │ rgb = np.rot90(rgb, k=1)               │ │
│ │ ✅ JUSTIFICATION:                  │ │   │ │                                        │ │
│ │    OpenCV utilise Y vers le bas    │ │   │ │ ✅ JUSTIFICATION:                      │ │
│ │    Notre convention: Y vers haut   │ │   │ │    PyQtGraph ImageItem attend          │ │
│ │    → Flip nécessaire pour que      │ │   │ │    l'image avec:                       │ │
│ │      le tracker voie l'image       │ │   │ │    - Axe 0 = X (horizontal)            │ │
│ │      dans le bon sens              │ │   │ │    - Axe 1 = Y (vertical)              │ │
│ │                                    │ │   │ │    Notre image:                        │ │
│ │ ⚠️ CONSÉQUENCE:                    │ │   │ │    - Axe 0 = Y (rows)                  │ │
│ │    Toutes les coordonnées Y        │ │   │ │    - Axe 1 = X (cols)                  │ │
│ │    du tracker sont inversées       │ │   │ │    → rot90 aligne les axes             │ │
│ └────────────────────────────────────┘ │   │ └────────────────────────────────────────┘ │
│                                        │   │                                            │
│ COMPENSATION Y:                        │   │ POSITIONNEMENT:                            │
│ ┌────────────────────────────────────┐ │   │ ┌────────────────────────────────────────┐ │
│ │ # À l'initialisation:              │ │   │ │ setRect(-max_r, 0, 2*max_r, max_r)     │ │
│ │ bbox_y_flipped = H - y - h         │ │   │ │                                        │ │
│ │                                    │ │   │ │ L'image est placée dans le repère:     │ │
│ │ # À la publication:                │ │   │ │   X: [-max_r, +max_r]                  │ │
│ │ bbox_y_display = H - y - h         │ │   │ │   Y: [0, max_r]                        │ │
│ │                                    │ │   │ └────────────────────────────────────────┘ │
│ │ # Conversion pixel → mètre:        │ │   │                                            │
│ │ y_m = (H - y_px) * resolution      │ │   │ 📤 OUTPUT: Affichage graphique correct     │
│ └────────────────────────────────────┘ │   │    avec coordonnées en mètres              │
│                                        │   │                                            │
│ 📤 OUTPUT: /docking/tracking/          │   └────────────────────────────────────────────┘
│            tracked_object              │
│    - bbox en coordonnées display       │
│    - center_x, center_y en mètres      │
└────────────────────────────────────────┘
```

---

## 📋 Tableau Récapitulatif des Transformations

| ID | Fichier | Ligne | Transformation | Type | Justification | Réversible |
|----|---------|-------|----------------|------|---------------|------------|
| T1 | sonar_node.py | 194 | `.T.ravel()` | Transpose + Flatten | Format message bearing-major | N/A |
| T2 | traitement_unified_node.py | 167 | `arctan2(-xv, yv)` | Inversion X | Convention bearing sonar | Implicite |
| T3 | traitement_unified_node.py | 295-310 | `cv2.flip()` | Flip H/V configurable | Correction si nécessaire | Oui |
| T4 | csrt_tracker_node.py | 107 + 155 | `cv2.flip(img, 0)` + `center_x_m = -center_x_m` | Flip Vertical + Correction X | Convention Y OpenCV + Compensation T2 | Oui |
| T5 | sonar_cartesian_display.py | 199 | `np.rot90(k=1)` | Rotation 90° | Convention PyQtGraph | Oui |

---

## 🔄 Conventions de Signes

### Convention Sonar Standard (utilisée dans ce projet)

```
                    +Y (avant, range croissant)
                     ↑
                     │
                     │  bearing = 0
                     │
        bearing < 0  │  bearing > 0
        (gauche)     │  (droite)
                     │
      -X ←───────────┼───────────→ +X
                     │
                    ROV
                   (0,0)

    Formules de conversion:
    ━━━━━━━━━━━━━━━━━━━━━━━━
    Polaire → Cartésien:
        x = range × sin(bearing)
        y = range × cos(bearing)
    
    Cartésien → Polaire:
        range = √(x² + y²)
        bearing = arctan2(x, y)    ← Note: arctan2(x,y) pas arctan2(y,x)
```

### Pourquoi `-xv` dans la conversion ?

```python
# Dans traitement_unified_node.py et sonar_display.py:
th = np.arctan2(-xv, yv)
```

**Explication détaillée:**

1. L'image polaire source a les bearings ordonnés de `-FOV/2` à `+FOV/2` (ligne 0 à ligne N)
2. L'image cartésienne a X de `-max_r` à `+max_r` (colonne 0 à colonne M)
3. Pour un point à `x > 0` (droite), on veut un `bearing > 0` (droite)
4. `arctan2(x, y)` donne un angle positif pour `x > 0`
5. Mais l'ordre des bearings dans l'image polaire est **inversé** par rapport à ça
6. Donc on utilise `arctan2(-x, y)` pour compenser

**Alternative propre:** On pourrait inverser l'ordre des bearings dans l'image polaire à la source,
mais cela impacterait tous les traitements en aval.

---

## ⚠️ Points Critiques

### 1. Le Flip Vertical du Tracker (T4)

Le tracker CSRT applique un flip vertical car:
- **Notre convention:** Y=0 en bas (ROV), Y augmente vers le haut (avant)
- **Convention OpenCV:** Y=0 en haut, Y augmente vers le bas

Ce flip crée une **couche d'indirection** qui complique le code:
```python
# Partout dans csrt_tracker_node.py:
bbox_y_flipped = msg.height - bbox_y - bbox_h  # Compensation nécessaire
```

### 2. La Rotation 90° de l'Affichage (T5)

PyQtGraph `ImageItem` interprète les arrays numpy comme:
- Axe 0 → direction X (horizontal)
- Axe 1 → direction Y (vertical)

Notre image cartésienne est:
- Axe 0 → lignes → Y
- Axe 1 → colonnes → X

D'où le `rot90(k=1)` pour aligner les axes.

### 3. La Correction X dans le Tracker (T4)

```python
# csrt_tracker_node.py:155
center_x_m = -center_x_m
```

Cette correction compense l'inversion de l'axe X introduite par la transformation T2 (arctan2(-xv, yv)).
Le tracker inverse le signe de center_x pour publier des coordonnées conformes à la convention sonar:
- X positif = droite du ROV
- X négatif = gauche du ROV

---

## ✅ Recommandations pour Consolidation

### Option 1: Garder l'Architecture Actuelle (Minimal)

L'architecture actuelle fonctionne mais est complexe. Si on la garde:

1. **Documenter** chaque transformation (ce document)
2. **Commenter** systématiquement les compensations dans le code
3. **Tester** avec un pattern asymétrique pour valider gauche/droite

### Option 2: Simplification (Recommandé)

**Principe:** Établir une convention unique dès la source et s'y tenir.

```
CONVENTION PROPOSÉE:
━━━━━━━━━━━━━━━━━━━

1. Images toujours orientées "Y vers le haut" (convention cartographique)
2. Origine en bas à gauche ou au centre en bas
3. X positif vers la droite
4. Y positif vers le haut

Transformations nécessaires uniquement à l'interface avec les bibliothèques
(OpenCV, PyQtGraph) et documentées localement.
```

**Modifications suggérées:**

1. **Supprimer** les paramètres `cart_flip_horizontal/vertical` si non utilisés
2. **Isoler** le flip OpenCV dans une fonction dédiée avec documentation
3. **Utiliser** des constantes nommées pour les orientations

---

## 🧪 Procédure de Validation

### Test Pattern Asymétrique

Pour valider que toutes les transformations sont cohérentes:

1. Créer un pattern de test avec une forme asymétrique (ex: lettre "F")
2. Positionner le pattern en haut à droite du sonar
3. Vérifier que:
   - L'affichage montre le "F" en haut à droite
   - Le tracker détecte le "F" en haut à droite
   - Les coordonnées publiées sont positives en X et Y

### Vérification des Signes

```python
# Test de cohérence des conventions:
# Un objet à droite du ROV (X > 0) doit avoir:
# - bearing > 0
# - center_x > 0 dans TrackedObject
# - Apparaître à droite dans l'affichage
```

---

## 📝 Historique des Modifications

| Date | Auteur | Description |
|------|--------|-------------|
| 2026-02-03 | Initial | Documentation initiale des transformations |
| 2026-02-03 | Mise à jour | Ajout de la correction X dans le tracker (T4) |

---

## 📚 Références

- [Convention sonar maritime](https://en.wikipedia.org/wiki/Bearing_(navigation))
- [OpenCV coordinate system](https://docs.opencv.org/master/d9/d0c/group__calib3d.html)
- [PyQtGraph ImageItem](https://pyqtgraph.readthedocs.io/en/latest/api_reference/graphicsItems/imageitem.html)
- [NumPy array indexing](https://numpy.org/doc/stable/reference/arrays.indexing.html)
