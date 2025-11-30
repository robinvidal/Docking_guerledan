"""
Module de filtres de signal pour traitement sonar.
"""

import numpy as np
import cv2
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
    # OpenCV requires odd kernel size and 8-bit or 3-channel images
    k = max(1, int(kernel_size) | 1)
    img = image
    if img.dtype != np.uint8:
        img = np.clip(img, 0, 255).astype(np.uint8)
    return cv2.medianBlur(img, k)


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
    # OpenCV GaussianBlur uses kernel size derived from sigma
    # Choose kernel size as next odd integer of 6*sigma+1
    ksize = max(1, int(6 * sigma + 1))
    if ksize % 2 == 0:
        ksize += 1
    img = image
    if img.dtype != np.uint8:
        img = np.clip(img, 0, 255).astype(np.uint8)
    return cv2.GaussianBlur(img, (ksize, ksize), sigmaX=sigma)


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
    k = max(1, int(kernel_size))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    img = image
    if img.dtype != np.uint8:
        img = np.clip(img, 0, 255).astype(np.uint8)
    opened = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    return opened


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
    k = max(1, int(kernel_size))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    img = image
    if img.dtype != np.uint8:
        img = np.clip(img, 0, 255).astype(np.uint8)
    closed = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    return closed


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
    # OpenCV adaptiveThreshold requires 8-bit single-channel image
    bs = max(3, int(block_size) | 1)
    img = image
    if img.dtype != np.uint8:
        img = np.clip(img, 0, 255).astype(np.uint8)
    binary = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                   cv2.THRESH_BINARY, bs, c)
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
    # OpenCV does not provide a direct Wiener filter; approximate with
    # Non-local means denoising which preserves edges and is performant.
    img = image
    if img.dtype != np.uint8:
        img = np.clip(img, 0, 255).astype(np.uint8)
    # h parameter controls filter strength; map noise_variance to h
    if noise_variance is None:
        h = 10
    else:
        h = float(max(1.0, noise_variance * 10.0))
    # fastNlMeansDenoising expects 8-bit single-channel
    denoised = cv2.fastNlMeansDenoising(img, None, h, 7, 21)
    return denoised


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
    img = image
    if img.dtype != np.uint8:
        img = np.clip(img, 0, 255).astype(np.uint8)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
    enhanced = clahe.apply(img)
    return enhanced


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
    compensated = image * gain
    compensated = np.clip(compensated, 0, 255)
    return compensated.astype(image.dtype)
