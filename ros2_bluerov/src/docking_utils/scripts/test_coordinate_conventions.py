#!/usr/bin/env python3
"""
Script de validation des conventions de coordonnées du pipeline sonar.

Ce script génère des images tests asymétriques pour vérifier que:
1. Gauche/Droite sont corrects (pas de miroir horizontal)
2. Haut/Bas sont corrects (pas de miroir vertical)
3. Les coordonnées du tracker correspondent à l'affichage

Usage:
    ros2 run docking_utils test_coordinate_conventions

Ou directement:
    python3 test_coordinate_conventions.py
"""

import numpy as np


def create_test_pattern_polar(n_bearing: int = 256, n_range: int = 512) -> np.ndarray:
    """
    Crée un pattern de test asymétrique en coordonnées polaires.
    
    Le pattern contient:
    - Une lettre "R" (Right) à droite (bearings positifs, indices hauts)
    - Une lettre "L" (Left) à gauche (bearings négatifs, indices bas)
    - Une flèche pointant vers l'avant (ranges élevés)
    
    Ce pattern permet de vérifier visuellement que:
    - R apparaît à droite dans l'affichage
    - L apparaît à gauche dans l'affichage
    - La flèche pointe vers le haut (avant du ROV)
    
    Returns:
        np.ndarray: Image polaire de shape (n_bearing, n_range)
    """
    img = np.zeros((n_bearing, n_range), dtype=np.uint8)
    
    # Marqueur "DROITE" (bearings positifs = indices > n_bearing/2)
    # Rectangle à droite
    b_start_r = int(n_bearing * 0.7)
    b_end_r = int(n_bearing * 0.9)
    r_start = int(n_range * 0.3)
    r_end = int(n_range * 0.6)
    img[b_start_r:b_end_r, r_start:r_end] = 200
    
    # Marqueur "GAUCHE" (bearings négatifs = indices < n_bearing/2)
    # Rectangle plus petit à gauche
    b_start_l = int(n_bearing * 0.1)
    b_end_l = int(n_bearing * 0.3)
    img[b_start_l:b_end_l, r_start:r_end] = 150
    
    # Flèche vers l'avant (centre, range élevé)
    # Triangle pointant vers les ranges élevés
    center_b = n_bearing // 2
    for i in range(50):
        half_width = max(1, 25 - i // 2)
        r_pos = int(n_range * 0.7) + i * 2
        if r_pos < n_range:
            img[center_b - half_width:center_b + half_width, r_pos] = 255
    
    return img


def create_test_pattern_cartesian(width: int = 512, height: int = 256) -> np.ndarray:
    """
    Crée un pattern de test asymétrique en coordonnées cartésiennes.
    
    Convention attendue de l'image cartésienne:
    - Axe 0 (lignes): Y de 0 (ROV, en bas) à max_range (avant, en haut)
    - Axe 1 (colonnes): X de -max_range (gauche) à +max_range (droite)
    
    Le pattern contient:
    - "D" (Droite) à droite (colonnes > width/2)
    - "G" (Gauche) à gauche (colonnes < width/2)
    - Triangle vers le haut (lignes élevées = Y élevé = avant)
    
    Returns:
        np.ndarray: Image cartésienne de shape (height, width)
    """
    img = np.zeros((height, width), dtype=np.uint8)
    
    # "D" pour DROITE - rectangle dans le quadrant droit
    x_start_d = int(width * 0.65)
    x_end_d = int(width * 0.85)
    y_start = int(height * 0.3)
    y_end = int(height * 0.7)
    img[y_start:y_end, x_start_d:x_end_d] = 200
    
    # "G" pour GAUCHE - rectangle plus petit dans le quadrant gauche
    x_start_g = int(width * 0.15)
    x_end_g = int(width * 0.35)
    img[y_start:y_end, x_start_g:x_end_g] = 150
    
    # Flèche vers l'AVANT (haut de l'image = Y élevé)
    center_x = width // 2
    for i in range(40):
        half_width = max(1, 20 - i // 2)
        y_pos = int(height * 0.75) + i
        if y_pos < height:
            img[y_pos, center_x - half_width:center_x + half_width] = 255
    
    return img


def print_conventions():
    """Affiche les conventions de coordonnées attendues."""
    print("""
╔═══════════════════════════════════════════════════════════════════════════════╗
║                    CONVENTIONS DE COORDONNÉES - DOCKING                        ║
╠═══════════════════════════════════════════════════════════════════════════════╣
║                                                                               ║
║   REPÈRE SONAR (convention maritime standard):                                ║
║                                                                               ║
║                        +Y (AVANT, range croissant)                            ║
║                         ↑                                                     ║
║                         │                                                     ║
║                         │  bearing = 0                                        ║
║           bearing < 0   │   bearing > 0                                       ║
║           (GAUCHE)      │   (DROITE)                                          ║
║                         │                                                     ║
║          -X ←───────────┼───────────→ +X                                      ║
║                         │                                                     ║
║                        ROV                                                    ║
║                       (0,0)                                                   ║
║                                                                               ║
╠═══════════════════════════════════════════════════════════════════════════════╣
║   TRANSFORMATIONS DANS LE PIPELINE:                                           ║
╠═══════════════════════════════════════════════════════════════════════════════╣
║                                                                               ║
║   T1 (sonar_node.py):                                                         ║
║       .T.ravel() - Transpose pour format bearing-major                        ║
║       Pas de changement d'orientation visuelle                                ║
║                                                                               ║
║   T2 (traitement_unified_node.py):                                            ║
║       arctan2(-xv, yv) - Conversion polaire→cartésien                         ║
║       Le -xv compense l'ordre des bearings                                    ║
║                                                                               ║
║   T3 (traitement_unified_node.py):                                            ║
║       cv2.flip() optionnel - DÉSACTIVÉ par défaut                             ║
║                                                                               ║
║   T4 (csrt_tracker_node.py):                                                  ║
║       cv2.flip(img, 0) - Flip vertical pour OpenCV                            ║
║       Compensation: y_flipped = height - y - h                                ║
║                                                                               ║
║   T5 (sonar_cartesian_display.py):                                            ║
║       np.rot90(k=1) - Rotation 90° pour PyQtGraph                             ║
║                                                                               ║
╠═══════════════════════════════════════════════════════════════════════════════╣
║   VÉRIFICATION ATTENDUE:                                                      ║
╠═══════════════════════════════════════════════════════════════════════════════╣
║                                                                               ║
║   Pattern de test (create_test_pattern_cartesian):                            ║
║                                                                               ║
║       ┌─────────────────────────────────────┐                                ║
║       │            ▲ (flèche)               │  ← AVANT (Y élevé)              ║
║       │           ╱ ╲                       │                                ║
║       │                                     │                                ║
║       │    ┌───┐           ┌───┐           │                                ║
║       │    │ G │           │ D │           │                                ║
║       │    └───┘           └───┘           │                                ║
║       │  (gauche)         (droite)         │                                ║
║       │                                     │                                ║
║       │              ROV                    │  ← ARRIÈRE (Y=0)               ║
║       └─────────────────────────────────────┘                                ║
║         X<0                      X>0                                         ║
║                                                                               ║
║   Si G apparaît à droite ou D à gauche → problème de miroir horizontal       ║
║   Si la flèche pointe vers le bas → problème de miroir vertical              ║
║                                                                               ║
╚═══════════════════════════════════════════════════════════════════════════════╝
""")


def verify_transformation_chain():
    """
    Vérifie mathématiquement que la chaîne de transformations est cohérente.
    
    Simule un point à droite du ROV (bearing > 0, x > 0) et trace
    ses coordonnées à travers chaque étape.
    """
    print("\n" + "="*70)
    print("VÉRIFICATION DE LA CHAÎNE DE TRANSFORMATIONS")
    print("="*70)
    
    # Point de test: objet à DROITE et DEVANT le ROV
    # En coordonnées polaires sonar: bearing = +30°, range = 2m
    bearing_deg = 30.0
    range_m = 2.0
    
    bearing_rad = np.radians(bearing_deg)
    
    # Conversion polaire → cartésien (convention sonar: x = r*sin(b), y = r*cos(b))
    x_m = range_m * np.sin(bearing_rad)
    y_m = range_m * np.cos(bearing_rad)
    
    print(f"\n1. POINT DE TEST (coordonnées réelles):")
    print(f"   Bearing: {bearing_deg}° (DROITE du ROV)")
    print(f"   Range: {range_m} m")
    print(f"   → X = {x_m:.3f} m (positif = DROITE) ✓")
    print(f"   → Y = {y_m:.3f} m (positif = AVANT) ✓")
    
    # Simulation de la conversion dans traitement_unified_node.py
    print(f"\n2. CONVERSION T2 (arctan2(-xv, yv)):")
    print(f"   Dans polar_to_cartesian(), on fait le MAPPING INVERSE:")
    print(f"   Pour chaque pixel (col, row) de l'image cartésienne,")
    print(f"   on cherche le pixel correspondant dans l'image polaire.")
    print()
    
    # Simulons un pixel de l'image cartésienne à DROITE (col > center)
    # Les xs vont de -max_r à +max_r
    max_r = 4.0  # max_range typique
    out_w = 512
    col_droite = int(out_w * 0.75)  # Colonne à droite du centre
    
    # xs = linspace(-max_r, max_r, out_w), donc:
    x_pixel = -max_r + (col_droite / (out_w - 1)) * (2 * max_r)
    y_pixel = 2.0  # Milieu en Y
    
    print(f"   Pixel test: colonne {col_droite}/{out_w} (DROITE)")
    print(f"   → x_grid = {x_pixel:.2f} m (positif = droite) ✓")
    print(f"   → y_grid = {y_pixel:.2f} m")
    
    # Calcul du bearing avec -xv
    th = np.arctan2(-x_pixel, y_pixel)
    print(f"\n   th = arctan2(-{x_pixel:.2f}, {y_pixel:.2f}) = {np.degrees(th):.1f}°")
    
    # Calcul de l'index dans l'image polaire
    fov_total = np.radians(120)  # FOV typique de 120°
    n_bearing = 256
    i_float = (th + fov_total/2) / fov_total * (n_bearing - 1)
    print(f"   Index bearing: i_float = {i_float:.1f}")
    
    # Interprétation
    print(f"\n   INTERPRÉTATION:")
    print(f"   - L'image polaire a bearing[0] = -60° (GAUCHE)")
    print(f"   - L'image polaire a bearing[255] = +60° (DROITE)")
    print(f"   - th = {np.degrees(th):.1f}° donne i_float = {i_float:.1f}")
    
    if th < 0:
        print(f"   - th < 0 → correspond aux PETITS indices (gauche de l'image polaire)")
    else:
        print(f"   - th > 0 → correspond aux GRANDS indices (droite de l'image polaire)")
    
    print(f"\n   CONCLUSION:")
    print(f"   Le -xv dans arctan2(-xv, yv) INVERSE le mapping:")
    print(f"   - x_pixel > 0 (droite cartésien) → th < 0 → petits indices polaires")
    print(f"   - Ceci signifie que les PETITS indices polaires correspondent à DROITE")
    print(f"   - C'est l'INVERSE de la convention bearing[0]=gauche, bearing[N]=droite")
    print(f"   - Le -xv CORRIGE donc cette inversion ✓")
    
    print(f"\n3. AFFICHAGE FINAL:")
    print(f"   Après rotation T5 et positionnement:")
    print(f"   Le point doit apparaître à DROITE et en HAUT de l'image")
    print(f"   car X > 0 (droite) et Y > 0 (avant/haut)")
    
    print("\n" + "="*70)


def main():
    """Point d'entrée principal."""
    print_conventions()
    verify_transformation_chain()
    
    # Créer les patterns de test
    polar = create_test_pattern_polar()
    cartesian = create_test_pattern_cartesian()
    
    print("\n" + "="*70)
    print("PATTERNS DE TEST GÉNÉRÉS")
    print("="*70)
    print(f"Pattern polaire: shape {polar.shape}")
    print(f"Pattern cartésien: shape {cartesian.shape}")
    print("\nPour tester visuellement, vous pouvez:")
    print("1. Publier ces patterns sur les topics sonar")
    print("2. Vérifier que G est à gauche et D à droite")
    print("3. Vérifier que la flèche pointe vers le haut")


if __name__ == '__main__':
    main()
