"""
Module de validation et calculs géométriques pour la cage.
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
    Valide que les 4 bords détectés correspondent à une cage rectangulaire.
    
    Args:
        ranges: Distances des 4 bords (m)
        bearings: Angles des 4 bords (rad)
        expected_width: Largeur attendue de la cage (m)
        expected_depth: Profondeur attendue de la cage (m)
    
    Returns:
        (is_valid, message): True si géométrie cohérente, message d'erreur sinon
    
    Examples:
        >>> # Cage idéale à 5m, centrée
        >>> ranges = np.array([5.0, 5.0, 5.0, 5.0])
        >>> bearings = np.array([-0.2, -0.1, 0.1, 0.2])
        >>> valid, msg = validate_cage_geometry(ranges, bearings)
    """
    if len(ranges) != 4 or len(bearings) != 4:
        return False, "Exactement 4 bords requis"
    
    # Conversion en cartésien
    from docking_utils.conversions import polar_to_cartesian
    points = [polar_to_cartesian(r, b) for r, b in zip(ranges, bearings)]
    points = np.array(points)  # Shape: (4, 2)
    
    # Tri des points (gauche->droite)
    sorted_idx = np.argsort(points[:, 0])
    points_sorted = points[sorted_idx]
    
    # Les 2 points gauches = montants gauches, 2 droits = montants droits
    left_points = points_sorted[:2]
    right_points = points_sorted[2:]
    
    # Calcul largeur (distance horizontale entre colonnes gauche et droite)
    width_measured = np.mean(right_points[:, 0]) - np.mean(left_points[:, 0])
    
    # Calcul profondeur (distance verticale moyenne)
    depth_measured = np.mean(points[:, 1])
    
    # Validation largeur
    if abs(width_measured - expected_width) > WIDTH_TOLERANCE:
        return False, f"Largeur invalide: {width_measured:.2f}m (attendu: {expected_width:.2f}m)"
    
    # Validation profondeur (ordre de grandeur)
    if depth_measured < 1.0 or depth_measured > 20.0:
        return False, f"Profondeur hors limites: {depth_measured:.2f}m"
    
    # Validation parallélisme (colonnes gauche et droite doivent être à même y)
    left_y_diff = abs(left_points[0, 1] - left_points[1, 1])
    right_y_diff = abs(right_points[0, 1] - right_points[1, 1])
    
    if left_y_diff > DEPTH_TOLERANCE or right_y_diff > DEPTH_TOLERANCE:
        return False, "Bords non parallèles (cage non rectangulaire)"
    
    return True, "Géométrie cage valide"


def compute_cage_center(ranges: np.ndarray, bearings: np.ndarray) -> Tuple[float, float]:
    """
    Calcule la position du centre de la cage.
    
    Args:
        ranges: Distances des 4 bords (m)
        bearings: Angles des 4 bords (rad)
    
    Returns:
        (x_center, y_center) en mètres
    
    Examples:
        >>> ranges = np.array([5.0, 5.0, 5.0, 5.0])
        >>> bearings = np.deg2rad(np.array([-10, -5, 5, 10]))
        >>> x_c, y_c = compute_cage_center(ranges, bearings)
        >>> assert abs(x_c) < 0.5  # Centré latéralement
    """
    from docking_utils.conversions import polar_to_cartesian
    
    # Conversion cartésienne
    points = np.array([polar_to_cartesian(r, b) for r, b in zip(ranges, bearings)])
    
    # Centre = moyenne des 4 points
    x_center = np.mean(points[:, 0])
    y_center = np.mean(points[:, 1])
    
    return x_center, y_center


def compute_cage_orientation(ranges: np.ndarray, bearings: np.ndarray) -> float:
    """
    Calcule l'orientation de la cage (yaw) par rapport au ROV.
    
    Args:
        ranges: Distances des 4 bords (m)
        bearings: Angles des 4 bords (rad)
    
    Returns:
        Angle de lacet de la cage (rad), 0 = cage alignée frontalement
    
    Examples:
        >>> # Cage parfaitement frontale
        >>> ranges = np.array([5.0, 5.0, 5.0, 5.0])
        >>> bearings = np.deg2rad(np.array([-10, -5, 5, 10]))
        >>> yaw = compute_cage_orientation(ranges, bearings)
        >>> assert abs(yaw) < np.deg2rad(5)
    """
    from docking_utils.conversions import polar_to_cartesian
    
    # Conversion cartésienne
    points = np.array([polar_to_cartesian(r, b) for r, b in zip(ranges, bearings)])
    
    # Tri gauche->droite
    sorted_idx = np.argsort(points[:, 0])
    points_sorted = points[sorted_idx]
    
    # Régression linéaire sur bords gauches et droits
    left_points = points_sorted[:2]
    right_points = points_sorted[2:]
    
    # Angle moyen des colonnes (doit être ~90° si cage frontale)
    left_angle = np.arctan2(left_points[1, 1] - left_points[0, 1],
                            left_points[1, 0] - left_points[0, 0])
    right_angle = np.arctan2(right_points[1, 1] - right_points[0, 1],
                             right_points[1, 0] - right_points[0, 0])
    
    # Déviation par rapport à verticale (90°)
    mean_angle = (left_angle + right_angle) / 2.0
    yaw = mean_angle - np.pi / 2.0  # Déviation par rapport à frontal
    
    from docking_utils.conversions import normalize_angle
    return normalize_angle(yaw)


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
