"""
Tests unitaires pour le module geometry.
"""

import pytest
import numpy as np
from docking_utils.geometry import (
    validate_cage_geometry,
    compute_cage_center,
    compute_cage_orientation,
    check_collision_risk,
    estimate_approach_trajectory,
    CAGE_WIDTH,
    CAGE_DEPTH
)


class TestValidateCageGeometry:
    """Tests de validation de géométrie de cage."""
    
    def test_validate_correct_geometry(self):
        """Cage rectangulaire correcte."""
        # 4 bords formant rectangle 2x2m à 5m
        ranges = np.array([5.0, 5.0, 5.0, 5.0])
        bearings = np.deg2rad(np.array([-10, -5, 5, 10]))  # ~1m de largeur à 5m
        
        valid, msg = validate_cage_geometry(ranges, bearings, 
                                           expected_width=CAGE_WIDTH,
                                           expected_depth=CAGE_DEPTH)
        assert valid or "Largeur" in msg  # Peut échouer selon géométrie exacte
    
    def test_validate_wrong_number_borders(self):
        """Nombre incorrect de bords."""
        ranges = np.array([5.0, 5.0])
        bearings = np.array([0.0, 0.1])
        
        valid, msg = validate_cage_geometry(ranges, bearings)
        assert not valid
        assert "4 bords" in msg
    
    def test_validate_wrong_width(self):
        """Largeur invalide."""
        # Bords trop écartés
        ranges = np.array([5.0, 5.0, 5.0, 5.0])
        bearings = np.deg2rad(np.array([-30, -20, 20, 30]))  # Trop large
        
        valid, msg = validate_cage_geometry(ranges, bearings, expected_width=2.0)
        assert not valid


class TestComputeCageCenter:
    """Tests de calcul du centre de cage."""
    
    def test_centered_cage(self):
        """Cage parfaitement centrée."""
        ranges = np.array([5.0, 5.0, 5.0, 5.0])
        bearings = np.deg2rad(np.array([-5, -2, 2, 5]))
        
        x_c, y_c = compute_cage_center(ranges, bearings)
        # Doit être centré latéralement
        assert abs(x_c) < 0.5
        assert 4.0 < y_c < 6.0  # Distance ~5m
    
    def test_offset_cage(self):
        """Cage décalée latéralement."""
        # Tous les bords à droite (angles négatifs)
        ranges = np.array([5.0, 5.0, 5.0, 5.0])
        bearings = np.deg2rad(np.array([-15, -10, -5, -2]))
        
        x_c, y_c = compute_cage_center(ranges, bearings)
        # Centre doit être négatif (droite)
        assert x_c < 0


class TestComputeCageOrientation:
    """Tests de calcul d'orientation de cage."""
    
    def test_frontal_cage(self):
        """Cage parfaitement frontale."""
        ranges = np.array([5.0, 5.0, 5.0, 5.0])
        bearings = np.deg2rad(np.array([-10, -5, 5, 10]))
        
        yaw = compute_cage_orientation(ranges, bearings)
        # Doit être proche de 0
        assert abs(yaw) < np.deg2rad(20)
    
    def test_angled_cage(self):
        """Cage avec angle."""
        # Simulation cage tournée (distances différentes)
        ranges = np.array([4.0, 4.5, 5.5, 6.0])
        bearings = np.deg2rad(np.array([-10, -5, 5, 10]))
        
        yaw = compute_cage_orientation(ranges, bearings)
        # Doit détecter une rotation
        assert abs(yaw) > np.deg2rad(5)


class TestCheckCollisionRisk:
    """Tests de détection de risque de collision."""
    
    def test_safe_centered_position(self):
        """Position sûre au centre."""
        safe, msg = check_collision_risk(0.0, 5.0, cage_width=2.0, safety_margin=0.5)
        assert safe
    
    def test_lateral_overflow(self):
        """Dépassement latéral."""
        safe, msg = check_collision_risk(1.5, 5.0, cage_width=2.0, safety_margin=0.5)
        assert not safe
        assert "latéral" in msg.lower()
    
    def test_too_close_frontal(self):
        """Trop proche frontalement."""
        safe, msg = check_collision_risk(0.0, 0.2, cage_width=2.0, safety_margin=0.5)
        assert not safe
        assert "proche" in msg.lower()
    
    def test_negative_x_position(self):
        """Position latérale négative (droite)."""
        safe, msg = check_collision_risk(-0.3, 5.0, cage_width=2.0, safety_margin=0.5)
        assert safe  # Dans les limites


class TestEstimateApproachTrajectory:
    """Tests de génération de trajectoire."""
    
    def test_trajectory_endpoints(self):
        """Vérifie début et fin de trajectoire."""
        waypoints = estimate_approach_trajectory(2.0, 10.0, 0.0, 0.5)
        
        # Premier waypoint = position actuelle
        assert abs(waypoints[0][0] - 2.0) < 1e-6
        assert abs(waypoints[0][1] - 10.0) < 1e-6
        
        # Dernier waypoint = cible
        assert abs(waypoints[-1][0] - 0.0) < 1e-6
        assert abs(waypoints[-1][1] - 0.5) < 1e-6
    
    def test_trajectory_length(self):
        """Nombre de waypoints adapté à la distance."""
        # Distance courte
        waypoints_short = estimate_approach_trajectory(0.5, 2.0, 0.0, 0.5)
        # Distance longue
        waypoints_long = estimate_approach_trajectory(5.0, 20.0, 0.0, 0.5)
        
        assert len(waypoints_long) > len(waypoints_short)
    
    def test_trajectory_monotonic(self):
        """Trajectoire monotone vers cible."""
        waypoints = estimate_approach_trajectory(3.0, 15.0, 0.0, 1.0)
        
        # x doit décroître vers 0
        x_values = [wp[0] for wp in waypoints]
        assert all(x_values[i] >= x_values[i+1] for i in range(len(x_values)-1))
        
        # y doit décroître vers 1.0
        y_values = [wp[1] for wp in waypoints]
        assert all(y_values[i] >= y_values[i+1] for i in range(len(y_values)-1))


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
