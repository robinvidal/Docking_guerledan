"""
Module de filtres de signal pour traitement sonar.
"""

import numpy as np
from scipy import ndimage, signal
from typing import Tuple


def median_filter(image: np.ndarray, kernel_size: int = 3) -> np.ndarray:
    """
    Applique un filtre médian pour réduire le bruit impulsionnel.
    
    Args:
        image: Image sonar 2D [bearing, range]
        kernel_size: Taille du noyau (impair)
    
    Returns:
        Image filtrée
    
    Examples:
        >>> img = np.random.randint(0, 255, (100, 200), dtype=np.uint8)
        >>> filtered = median_filter(img, 5)
        >>> assert filtered.shape == img.shape
    """
    return ndimage.median_filter(image, size=kernel_size)


def gaussian_filter(image: np.ndarray, sigma: float = 1.0) -> np.ndarray:
    """
    Applique un filtre gaussien pour lissage.
    
    Args:
        image: Image sonar 2D
        sigma: Écart-type du noyau gaussien
    
    Returns:
        Image lissée
    
    Examples:
        >>> img = np.random.rand(100, 200)
        >>> smoothed = gaussian_filter(img, 2.0)
        >>> assert smoothed.shape == img.shape
    """
    return ndimage.gaussian_filter(image, sigma=sigma)


def morphological_opening(image: np.ndarray, kernel_size: int = 3) -> np.ndarray:
    """
    Opération morphologique d'ouverture (érosion + dilatation).
    Élimine petits objets bruités.
    
    Args:
        image: Image binaire ou niveaux de gris
        kernel_size: Taille de l'élément structurant
    
    Returns:
        Image après ouverture
    
    Examples:
        >>> img = np.random.randint(0, 2, (100, 200), dtype=np.uint8)
        >>> opened = morphological_opening(img, 5)
        >>> assert opened.shape == img.shape
    """
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    eroded = ndimage.binary_erosion(image, structure=kernel)
    opened = ndimage.binary_dilation(eroded, structure=kernel)
    return opened.astype(image.dtype)


def morphological_closing(image: np.ndarray, kernel_size: int = 3) -> np.ndarray:
    """
    Opération morphologique de fermeture (dilatation + érosion).
    Remplit petits trous.
    
    Args:
        image: Image binaire ou niveaux de gris
        kernel_size: Taille de l'élément structurant
    
    Returns:
        Image après fermeture
    """
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    dilated = ndimage.binary_dilation(image, structure=kernel)
    closed = ndimage.binary_erosion(dilated, structure=kernel)
    return closed.astype(image.dtype)


def adaptive_threshold(image: np.ndarray, block_size: int = 11, c: float = 2.0) -> np.ndarray:
    """
    Seuillage adaptatif local pour gérer variations d'intensité.
    
    Args:
        image: Image niveaux de gris
        block_size: Taille du voisinage local (impair)
        c: Constante soustraite à la moyenne locale
    
    Returns:
        Image binaire (0 ou 255)
    
    Examples:
        >>> img = np.random.randint(0, 255, (100, 200), dtype=np.uint8)
        >>> binary = adaptive_threshold(img, 15, 5.0)
        >>> assert set(np.unique(binary)).issubset({0, 255})
    """
    # Moyenne locale
    local_mean = ndimage.uniform_filter(image.astype(float), size=block_size)
    
    # Seuillage
    threshold = local_mean - c
    binary = (image > threshold).astype(np.uint8) * 255
    
    return binary


def wiener_filter(image: np.ndarray, noise_variance: float = None) -> np.ndarray:
    """
    Filtre de Wiener pour débruitage avec préservation contours.
    
    Args:
        image: Image sonar 2D
        noise_variance: Variance du bruit (auto-estimée si None)
    
    Returns:
        Image débruitée
    
    Examples:
        >>> img = np.random.rand(100, 200)
        >>> denoised = wiener_filter(img)
        >>> assert denoised.shape == img.shape
    """
    if noise_variance is None:
        # Estimation noise via MAD (Median Absolute Deviation)
        noise_variance = np.median(np.abs(image - np.median(image))) / 0.6745
    
    # Application filtre Wiener (version simplifiée)
    filtered = signal.wiener(image, mysize=5, noise=noise_variance)
    
    return filtered


def contrast_enhancement(image: np.ndarray, clip_limit: float = 2.0) -> np.ndarray:
    """
    Amélioration du contraste par étirement d'histogramme.
    
    Args:
        image: Image niveaux de gris [0-255]
        clip_limit: Limite d'écrêtage pour éviter sur-amplification
    
    Returns:
        Image avec contraste amélioré
    
    Examples:
        >>> img = np.random.randint(50, 150, (100, 200), dtype=np.uint8)
        >>> enhanced = contrast_enhancement(img)
        >>> assert enhanced.min() < img.min() or enhanced.max() > img.max()
    """
    # Clipping des valeurs extrêmes
    p_low, p_high = np.percentile(image, [clip_limit, 100 - clip_limit])
    
    # Étirement
    clipped = np.clip(image, p_low, p_high)
    normalized = (clipped - p_low) / (p_high - p_low) * 255
    
    return normalized.astype(np.uint8)


def range_compensation(image: np.ndarray, ranges: np.ndarray, 
                       alpha: float = 0.0001) -> np.ndarray:
    """
    Compense l'atténuation en fonction de la distance (spreading loss).
    
    Args:
        image: Image sonar 2D [bearing, range]
        ranges: Array des distances correspondant aux bins (m)
        alpha: Coefficient d'atténuation volumique (m^-1)
    
    Returns:
        Image compensée
    
    Examples:
        >>> img = np.random.rand(100, 200)
        >>> ranges = np.linspace(1, 50, 200)
        >>> compensated = range_compensation(img, ranges, 0.0001)
        >>> assert compensated.shape == img.shape
    """
    # Calcul du gain de compensation (spreading + atténuation)
    # TL = 20*log10(r) + alpha*r
    gain = ranges[:, np.newaxis].T * np.exp(alpha * ranges[:, np.newaxis].T)
    
    # Application (broadcasting sur bearings)
    compensated = image * gain
    
    # Normalisation
    compensated = np.clip(compensated, 0, 255)
    
    return compensated.astype(image.dtype)
