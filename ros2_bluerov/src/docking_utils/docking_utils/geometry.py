"""
Module de validation et calculs géométriques pour la cage.

Note: La détection publie 2 bords (montants verticaux visibles). Les fonctions
ci-dessous fournissent des utilitaires basés sur 2 bords, avec largeur attendue
pour compléter l'estimation de centre et d'orientation.
"""

import numpy as np
from typing import Tuple, List, Optional


# Dimensions nominales de la cage (à ajuster selon cage réelle)
CAGE_WIDTH = 2.0   # Largeur (m)
CAGE_DEPTH = 2.0   # Profondeur (m)
CAGE_HEIGHT = 2.0  # Hauteur (m)

# Tolérances de validation
WIDTH_TOLERANCE = 0.3  # ±30cm
DEPTH_TOLERANCE = 0.3
ANGLE_TOLERANCE = np.deg2rad(15)  # ±15°


def validate_cage_geometry(ranges: np.ndarray, bearings: np.ndarray,
                           expected_width: float = CAGE_WIDTH,
                           expected_depth: float = CAGE_DEPTH) -> Tuple[bool, str]:
    """
    Valide grossièrement la cohérence de 2 bords visibles avec une cage.

    Args:
        ranges: Distances des 2 bords (m)
        bearings: Angles des 2 bords (rad)
        expected_width: Largeur attendue de la cage (m)
        expected_depth: Profondeur attendue de la cage (m)

    Returns:
        (is_valid, message): True si cohérent, sinon warning

    Notes:
        Avec seulement 2 bords, on vérifie l'écartement angulaire raisonnable
        et une profondeur plausible. La largeur exacte nécessite hypothèses.
    """
    if len(ranges) != 2 or len(bearings) != 2:
        return False, "Exactement 2 bords requis"

    from docking_utils.conversions import polar_to_cartesian
    points = np.array([polar_to_cartesian(r, b) for r, b in zip(ranges, bearings)])

    # Profondeur moyenne
    depth_measured = float(np.mean(points[:, 1]))
    if depth_measured < 1.0 or depth_measured > 20.0:
        return False, f"Profondeur hors limites: {depth_measured:.2f}m"

    # Écart latéral (distance x entre deux montants)
    lateral_gap = abs(points[1, 0] - points[0, 0])
    if lateral_gap < 0.2:
        return False, "Montants trop proches (écart latéral < 0.2m)"

    # Écart angulaire raisonnable
    angular_gap = abs(bearings[1] - bearings[0])
    if angular_gap < np.deg2rad(1.0):
        return False, "Montants quasi confondus (écart angulaire < 1°)"

    return True, "Géométrie cohérente avec 2 bords"


def compute_cage_center_from_2_borders(ranges: np.ndarray, bearings: np.ndarray,
                                       expected_width: float = CAGE_WIDTH) -> Tuple[float, float]:
    """
    Estime le centre de la cage à partir de 2 bords en supposant symétrie.

    Args:
        ranges: Distances des 2 bords (m)
        bearings: Angles des 2 bords (rad)
        expected_width: Largeur attendue de la cage (m)

    Returns:
        (x_center, y_center) en mètres
    """
    from docking_utils.conversions import polar_to_cartesian
    points = np.array([polar_to_cartesian(r, b) for r, b in zip(ranges, bearings)])

    # Centre latéral = milieu entre les deux montants visibles
    x_center = float(np.mean(points[:, 0]))

    # Profondeur: moyenne des deux bords (approx du plan de façade)
    y_center = float(np.mean(points[:, 1]))

    return x_center, y_center


def compute_cage_orientation_from_2_borders(ranges: np.ndarray, bearings: np.ndarray) -> float:
    """
    Estime l'orientation (yaw) de la cage à partir de 2 bords.

    Approche: vecteur entre les deux montants visibles donne la direction
    latérale de la cage. L'orientation de la façade est perpendiculaire.

    Args:
        ranges: Distances des 2 bords (m)
        bearings: Angles des 2 bords (rad)

    Returns:
        yaw (rad), 0 = façade perpendiculaire à l'axe du ROV (alignée)
    """
    from docking_utils.conversions import polar_to_cartesian, normalize_angle
    points = np.array([polar_to_cartesian(r, b) for r, b in zip(ranges, bearings)])

    # Vecteur entre montants (x,y)
    dx = points[1, 0] - points[0, 0]
    dy = points[1, 1] - points[0, 1]

    # Direction latérale (angle du segment)
    segment_angle = np.arctan2(dy, dx)

    # Façade ≈ perpendiculaire au segment (ajouter ±90°)
    facade_angle = segment_angle + np.pi / 2.0

    return normalize_angle(facade_angle)


def check_collision_risk(x: float, y: float, cage_width: float = CAGE_WIDTH,
                         safety_margin: float = 0.5) -> Tuple[bool, str]:
    """
    Vérifie si le ROV risque une collision avec la cage.
    
    Args:
        x: Position latérale ROV (m)
        y: Distance frontale ROV (m)
        cage_width: Largeur de la cage (m)
        safety_margin: Marge de sécurité (m)
    
    Returns:
        (is_safe, warning): False si risque collision, message d'alerte
    
    Examples:
        >>> safe, msg = check_collision_risk(0.0, 5.0)  # Centré, loin
        >>> assert safe
        >>> safe, msg = check_collision_risk(1.5, 0.5, cage_width=2.0)  # Trop latéral
        >>> assert not safe
    """
    # Vérification dépassement latéral
    max_lateral = (cage_width / 2.0) - safety_margin
    
    if abs(x) > max_lateral:
        return False, f"Dépassement latéral: {x:.2f}m (max: ±{max_lateral:.2f}m)"
    
    # Vérification distance frontale minimale
    if y < safety_margin:
        return False, f"Trop proche: {y:.2f}m (min: {safety_margin:.2f}m)"
    
    return True, "Aucun risque détecté"


def estimate_approach_trajectory(x_current: float, y_current: float,
                                 x_target: float = 0.0, y_target: float = 0.5) -> List[Tuple[float, float]]:
    """
    Génère une trajectoire d'approche simple (ligne droite avec waypoints).
    
    Args:
        x_current: Position latérale actuelle (m)
        y_current: Distance frontale actuelle (m)
        x_target: Position latérale cible (m, 0 = centre)
        y_target: Distance frontale cible (m, 0.5 = proche pour docking)
    
    Returns:
        Liste de waypoints [(x, y), ...]
    
    Examples:
        >>> waypoints = estimate_approach_trajectory(2.0, 10.0)
        >>> assert len(waypoints) > 2
        >>> assert waypoints[-1] == (0.0, 0.5)
    """
    # Nombre de waypoints basé sur distance
    distance = np.sqrt((x_target - x_current)**2 + (y_target - y_current)**2)
    num_waypoints = max(3, int(distance / 1.0))  # 1 waypoint par mètre
    
    # Interpolation linéaire
    x_waypoints = np.linspace(x_current, x_target, num_waypoints)
    y_waypoints = np.linspace(y_current, y_target, num_waypoints)
    
    return list(zip(x_waypoints, y_waypoints))
