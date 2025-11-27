"""
Module de conversions de coordonnées et transformations géométriques.
"""

import numpy as np
from typing import Tuple


def polar_to_cartesian(range_m: float, bearing_rad: float) -> Tuple[float, float]:
    """
    Convertit coordonnées polaires en cartésiennes (2D).
    
    Args:
        range_m: Distance en mètres
        bearing_rad: Angle en radians (0 = devant, + = gauche)
    
    Returns:
        (x, y) en mètres (x = latéral, y = frontal)
    
    Examples:
        >>> x, y = polar_to_cartesian(10.0, 0.0)
        >>> assert abs(x) < 1e-6 and abs(y - 10.0) < 1e-6
    """
    x = range_m * np.sin(bearing_rad)
    y = range_m * np.cos(bearing_rad)
    return x, y


def cartesian_to_polar(x: float, y: float) -> Tuple[float, float]:
    """
    Convertit coordonnées cartésiennes en polaires (2D).
    
    Args:
        x: Décalage latéral en mètres (+ = droite)
        y: Distance frontale en mètres (+ = devant)
    
    Returns:
        (range, bearing) - distance (m), angle (rad)
    
    Examples:
        >>> r, theta = cartesian_to_polar(0.0, 10.0)
        >>> assert abs(r - 10.0) < 1e-6 and abs(theta) < 1e-6
    """
    range_m = np.sqrt(x**2 + y**2)
    bearing_rad = np.arctan2(x, y)
    return range_m, bearing_rad


def normalize_angle(angle_rad: float) -> float:
    """
    Normalise un angle dans [-π, π].
    
    Args:
        angle_rad: Angle en radians
    
    Returns:
        Angle normalisé dans [-π, π]
    
    Examples:
        >>> assert abs(normalize_angle(3.5 * np.pi) - (-0.5 * np.pi)) < 1e-6
    """
    return np.arctan2(np.sin(angle_rad), np.cos(angle_rad))


def sonar_to_body_frame(range_m: float, bearing_rad: float, 
                        sonar_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> Tuple[float, float, float]:
    """
    Transforme point du repère sonar vers repère corps ROV.
    
    Args:
        range_m: Distance mesurée par sonar (m)
        bearing_rad: Angle mesuré par sonar (rad)
        sonar_offset: Offset (x,y,z) du sonar dans repère corps (m)
    
    Returns:
        (x, y, z) dans repère corps ROV
    
    Examples:
        >>> x, y, z = sonar_to_body_frame(5.0, 0.0)
        >>> assert abs(y - 5.0) < 1e-6
    """
    # Conversion polaire->cartésien dans repère sonar
    x_sonar, y_sonar = polar_to_cartesian(range_m, bearing_rad)
    
    # Translation vers repère corps
    x_body = x_sonar + sonar_offset[0]
    y_body = y_sonar + sonar_offset[1]
    z_body = sonar_offset[2]  # Pas d'info z depuis sonar 2D
    
    return x_body, y_body, z_body


def interpolate_sonar_data(ranges: np.ndarray, bearings: np.ndarray, 
                           intensities: np.ndarray, 
                           target_bearing: float) -> float:
    """
    Interpole linéairement intensité sonar à un angle donné.
    
    Args:
        ranges: Array des distances (non utilisé pour interpolation angulaire)
        bearings: Array des angles de faisceaux (rad), triés
        intensities: Array 2D [bearing, range] des intensités
        target_bearing: Angle cible pour interpolation (rad)
    
    Returns:
        Intensité interpolée (moyenne sur ranges)
    
    Examples:
        >>> bearings = np.linspace(-np.pi/2, np.pi/2, 10)
        >>> intensities = np.random.rand(10, 100)
        >>> val = interpolate_sonar_data(None, bearings, intensities, 0.0)
        >>> assert 0.0 <= val <= 255.0
    """
    # Trouve indices encadrants
    idx = np.searchsorted(bearings, target_bearing)
    
    if idx == 0:
        return np.mean(intensities[0, :])
    elif idx >= len(bearings):
        return np.mean(intensities[-1, :])
    
    # Interpolation linéaire
    bearing_low = bearings[idx - 1]
    bearing_high = bearings[idx]
    alpha = (target_bearing - bearing_low) / (bearing_high - bearing_low)
    
    intensity_low = np.mean(intensities[idx - 1, :])
    intensity_high = np.mean(intensities[idx, :])
    
    return intensity_low * (1 - alpha) + intensity_high * alpha
