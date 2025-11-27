"""
Tests unitaires pour le module filters.
"""

import pytest
import numpy as np
from docking_utils.filters import (
    median_filter,
    gaussian_filter,
    morphological_opening,
    morphological_closing,
    adaptive_threshold,
    contrast_enhancement,
    range_compensation
)


class TestMedianFilter:
    """Tests du filtre médian."""
    
    def test_median_filter_shape(self):
        """Vérifie que la forme est préservée."""
        img = np.random.randint(0, 255, (100, 200), dtype=np.uint8)
        filtered = median_filter(img, 5)
        assert filtered.shape == img.shape
    
    def test_median_filter_removes_impulse_noise(self):
        """Vérifie réduction du bruit impulsionnel."""
        # Image uniforme avec pixels isolés
        img = np.ones((50, 50), dtype=np.uint8) * 100
        img[25, 25] = 255  # Pixel bruité
        
        filtered = median_filter(img, 3)
        # Le pixel bruité doit être atténué
        assert filtered[25, 25] < 150


class TestGaussianFilter:
    """Tests du filtre gaussien."""
    
    def test_gaussian_filter_shape(self):
        """Vérifie préservation de forme."""
        img = np.random.rand(100, 200)
        smoothed = gaussian_filter(img, 2.0)
        assert smoothed.shape == img.shape
    
    def test_gaussian_filter_smoothes(self):
        """Vérifie que le filtre lisse bien."""
        # Image avec gradient brutal
        img = np.zeros((50, 50))
        img[:, 25:] = 1.0
        
        smoothed = gaussian_filter(img, 3.0)
        # Zone de transition doit être lissée
        assert 0.0 < smoothed[25, 25] < 1.0


class TestMorphologicalOperations:
    """Tests des opérations morphologiques."""
    
    def test_opening_removes_small_objects(self):
        """Ouverture élimine petits objets."""
        img = np.zeros((100, 100), dtype=np.uint8)
        img[50, 50] = 1  # Petit objet isolé
        img[20:40, 20:40] = 1  # Grand objet
        
        opened = morphological_opening(img, 5)
        # Petit objet doit disparaître
        assert opened[50, 50] == 0
        # Grand objet préservé (partiellement)
        assert np.sum(opened[20:40, 20:40]) > 0
    
    def test_closing_fills_holes(self):
        """Fermeture remplit petits trous."""
        img = np.ones((100, 100), dtype=np.uint8)
        img[50, 50] = 0  # Petit trou
        
        closed = morphological_closing(img, 5)
        # Trou doit être rempli
        assert closed[50, 50] == 1


class TestAdaptiveThreshold:
    """Tests du seuillage adaptatif."""
    
    def test_adaptive_threshold_output_binary(self):
        """Sortie doit être binaire (0 ou 255)."""
        img = np.random.randint(0, 255, (100, 200), dtype=np.uint8)
        binary = adaptive_threshold(img, 15, 5.0)
        
        unique_values = np.unique(binary)
        assert set(unique_values).issubset({0, 255})
    
    def test_adaptive_threshold_handles_gradients(self):
        """Gère les gradients d'illumination."""
        # Image avec gradient
        img = np.linspace(50, 200, 10000).reshape(100, 100).astype(np.uint8)
        binary = adaptive_threshold(img, 11, 2.0)
        
        # Doit avoir des zones noires et blanches
        assert 0 in binary
        assert 255 in binary


class TestContrastEnhancement:
    """Tests de l'amélioration de contraste."""
    
    def test_contrast_enhancement_expands_range(self):
        """Améliore la plage dynamique."""
        # Image à faible contraste
        img = np.random.randint(100, 150, (100, 200), dtype=np.uint8)
        enhanced = contrast_enhancement(img, 2.0)
        
        # La plage doit être élargie
        assert enhanced.min() < img.min() or enhanced.max() > img.max()
    
    def test_contrast_enhancement_preserves_shape(self):
        """Préserve la forme."""
        img = np.random.randint(50, 200, (100, 200), dtype=np.uint8)
        enhanced = contrast_enhancement(img)
        assert enhanced.shape == img.shape


class TestRangeCompensation:
    """Tests de la compensation de portée."""
    
    def test_range_compensation_shape(self):
        """Préserve la forme."""
        img = np.random.rand(100, 200)
        ranges = np.linspace(1, 50, 200)
        compensated = range_compensation(img, ranges, 0.0001)
        assert compensated.shape == img.shape
    
    def test_range_compensation_amplifies_far_ranges(self):
        """Amplifie les distances lointaines."""
        # Image uniforme
        img = np.ones((10, 100)) * 50.0
        ranges = np.linspace(1, 50, 100)
        
        compensated = range_compensation(img, ranges, 0.001)
        
        # Les bins lointains doivent être amplifiés
        assert np.mean(compensated[:, -10:]) > np.mean(compensated[:, :10])


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
