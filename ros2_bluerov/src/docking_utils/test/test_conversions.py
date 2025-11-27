"""
Tests unitaires pour le module conversions.
"""

import pytest
import numpy as np
from docking_utils.conversions import (
    polar_to_cartesian,
    cartesian_to_polar,
    normalize_angle,
    sonar_to_body_frame,
    interpolate_sonar_data
)


class TestPolarCartesian:
    """Tests de conversion polaire/cartésien."""
    
    def test_polar_to_cartesian_zero_angle(self):
        """Test conversion avec angle nul (devant)."""
        x, y = polar_to_cartesian(10.0, 0.0)
        assert abs(x) < 1e-6
        assert abs(y - 10.0) < 1e-6
    
    def test_polar_to_cartesian_90deg(self):
        """Test conversion avec angle 90° (gauche)."""
        x, y = polar_to_cartesian(5.0, np.pi / 2)
        assert abs(x - 5.0) < 1e-6
        assert abs(y) < 1e-6
    
    def test_cartesian_to_polar_zero_angle(self):
        """Test conversion inverse avec angle nul."""
        r, theta = cartesian_to_polar(0.0, 10.0)
        assert abs(r - 10.0) < 1e-6
        assert abs(theta) < 1e-6
    
    def test_cartesian_to_polar_negative_x(self):
        """Test avec x négatif (droite)."""
        r, theta = cartesian_to_polar(-5.0, 5.0)
        assert abs(r - np.sqrt(50)) < 1e-6
        assert theta < 0  # Angle négatif pour x négatif
    
    def test_roundtrip_conversion(self):
        """Test aller-retour polaire->cartésien->polaire."""
        r_orig, theta_orig = 7.5, np.deg2rad(30)
        x, y = polar_to_cartesian(r_orig, theta_orig)
        r_back, theta_back = cartesian_to_polar(x, y)
        
        assert abs(r_back - r_orig) < 1e-6
        assert abs(theta_back - theta_orig) < 1e-6


class TestAngleNormalization:
    """Tests de normalisation d'angles."""
    
    def test_normalize_small_angle(self):
        """Petit angle déjà dans [-π, π]."""
        assert abs(normalize_angle(0.5) - 0.5) < 1e-6
    
    def test_normalize_large_positive(self):
        """Angle > π."""
        normalized = normalize_angle(3.5 * np.pi)
        assert -np.pi <= normalized <= np.pi
        assert abs(normalized - (-0.5 * np.pi)) < 1e-6
    
    def test_normalize_large_negative(self):
        """Angle < -π."""
        normalized = normalize_angle(-3.5 * np.pi)
        assert -np.pi <= normalized <= np.pi
        assert abs(normalized - (0.5 * np.pi)) < 1e-6
    
    def test_normalize_two_pi(self):
        """Angle = 2π doit donner 0."""
        assert abs(normalize_angle(2 * np.pi)) < 1e-6


class TestSonarToBodyFrame:
    """Tests de transformation repère sonar->corps."""
    
    def test_no_offset(self):
        """Sans offset, repères confondus."""
        x, y, z = sonar_to_body_frame(5.0, 0.0, (0.0, 0.0, 0.0))
        assert abs(y - 5.0) < 1e-6
        assert abs(x) < 1e-6
        assert abs(z) < 1e-6
    
    def test_with_x_offset(self):
        """Avec offset latéral."""
        x, y, z = sonar_to_body_frame(5.0, 0.0, (1.0, 0.0, 0.0))
        assert abs(x - 1.0) < 1e-6
        assert abs(y - 5.0) < 1e-6
    
    def test_with_y_offset(self):
        """Avec offset frontal."""
        x, y, z = sonar_to_body_frame(5.0, 0.0, (0.0, 2.0, 0.0))
        assert abs(y - 7.0) < 1e-6
    
    def test_with_z_offset(self):
        """Avec offset vertical."""
        x, y, z = sonar_to_body_frame(5.0, 0.0, (0.0, 0.0, -0.5))
        assert abs(z - (-0.5)) < 1e-6


class TestInterpolateSonarData:
    """Tests d'interpolation de données sonar."""
    
    def test_interpolate_exact_bearing(self):
        """Interpolation sur un faisceau existant."""
        bearings = np.linspace(-np.pi / 2, np.pi / 2, 10)
        intensities = np.ones((10, 100)) * 100
        
        val = interpolate_sonar_data(None, bearings, intensities, bearings[5])
        assert abs(val - 100.0) < 1e-6
    
    def test_interpolate_between_bearings(self):
        """Interpolation entre deux faisceaux."""
        bearings = np.array([0.0, 0.1])
        intensities = np.array([[50.0] * 100, [100.0] * 100])
        
        # Au milieu -> moyenne
        val = interpolate_sonar_data(None, bearings, intensities, 0.05)
        assert 70.0 < val < 80.0  # Environ 75
    
    def test_interpolate_out_of_bounds_low(self):
        """Angle plus petit que minimum."""
        bearings = np.linspace(0.0, 1.0, 10)
        intensities = np.ones((10, 100)) * 50
        
        val = interpolate_sonar_data(None, bearings, intensities, -0.5)
        assert abs(val - 50.0) < 1e-6  # Utilise premier faisceau
    
    def test_interpolate_out_of_bounds_high(self):
        """Angle plus grand que maximum."""
        bearings = np.linspace(0.0, 1.0, 10)
        intensities = np.ones((10, 100)) * 75
        
        val = interpolate_sonar_data(None, bearings, intensities, 2.0)
        assert abs(val - 75.0) < 1e-6  # Utilise dernier faisceau


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
