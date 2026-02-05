"""
Contrôleurs pour les missions de docking BlueROV
- SinglePointController: Mission simple pour rejoindre un point unique
- CageFollowController: Mission complète pour rejoindre une cage (2 poteaux)
"""

import numpy as np
import scipy as sp
import math


class SinglePointController:
    """
    Contrôleur simple pour rejoindre un point unique.
    
    Utilise deux contrôleurs découplés:
    - Heading: Oriente le robot vers la cible (bearing = 0)
    - Forward: Avance vers la cible
    - Lateral: Correction latérale pour approche directe
    
    Convention:
    - bearing: 0 = droit devant, positif = à droite, négatif = à gauche
    - range: distance en mètres
    """
    
    def __init__(self, stop_distance=1.0):
        """
        Args:
            stop_distance: Distance d'arrêt à la cible (m)
        """
        self.stop_distance = stop_distance
        
        # État du contrôleur
        self.state = "idle"  # "idle", "tracking", "arrived"
        
        # Gains des contrôleurs (à ajuster selon tests réels)
        # Limites de vitesse (m/s et rad/s)
        self.max_forward_speed = 0.5 # 0.5
        self.max_lateral_speed = 0.5 # 0.5
        self.max_angular_speed = 0.5 # 0.5
        
    def control_step(self, bearing, range_m, visible):
        """
        Calcule les commandes de vitesse pour rejoindre le point cible.
        
        Args:
            bearing: Angle vers la cible (rad), 0 = devant, + = droite
            range_m: Distance à la cible (m)
            visible: True si la cible est visible
            
        Returns:
            tuple: (forward_speed, lateral_speed, angular_speed)
                   Vitesses en m/s et rad/s
        """
        # Cible non visible: ne rien faire
        if not visible:
            self.state = "idle"
            return 0.0, 0.0, 0.0
        
        # Vérifier si on est arrivé
        if range_m < self.stop_distance:
            self.state = "arrived"
            return 0.0, 0.0, 0.0
        
        # Mode tracking actif
        self.state = "tracking"
        
        # ========== CONTRÔLEUR HEADING ==========
        # Objectif: bearing = 0 (cible droit devant)
        # Commande proportionnelle sur le bearing
        angular_speed = self.kp_heading * bearing
        
        # ========== CONTRÔLEUR FORWARD ==========
        # Distance en X (devant le robot) = range * math.cos(bearing)
        error_x = range_m * math.cos(bearing) - self.stop_distance
        forward_speed = self.kp_forward * error_x
        
        # ========== CONTRÔLEUR LATERAL ==========
        # Distance en Y (à droite du robot) = range * sin(bearing)
        error_y = range_m * math.sin(bearing)
        lateral_speed = self.kp_lateral * error_y
        
        # Limitation des vitesses
        forward_speed = np.clip(forward_speed, -self.max_forward_speed, self.max_forward_speed)
        lateral_speed = np.clip(lateral_speed, -self.max_lateral_speed, self.max_lateral_speed)
        angular_speed = np.clip(angular_speed, -self.max_angular_speed, self.max_angular_speed)
        
        return forward_speed, lateral_speed, angular_speed


class OrientedApproachController:
    """
    Contrôleur pour approcher une cible avec un angle d'orientation spécifique.
    
    Phases:
    1. "approaching": Avance vers la cible (comme SinglePointController) jusqu'à orbit_distance
    2. "orbiting": Orbite autour de la cible en gardant bearing=0, jusqu'à être aligné (±5°)
    3. "final_approach": Avance vers la cible jusqu'à stop_distance
    4. "arrived": Mission terminée
    
    Convention:
    - bearing: 0 = droit devant, positif = à droite, négatif = à gauche
    - range: distance en mètres
    - cage_angle: orientation de la cage (repère monde), normale au fond de la cage
    - approach_angle_error: différence entre position angulaire actuelle et cible autour de la cage
    """
    
    def __init__(self, stop_distance=0.5, orbit_distance=1.5, angle_tolerance_deg=5.0):
        """
        Args:
            stop_distance: Distance d'arrêt finale à la cible (m)
            orbit_distance: Distance à laquelle on commence à orbiter (m)
            angle_tolerance_deg: Tolérance angulaire pour considérer l'alignement OK (degrés)
        """
        self.stop_distance = stop_distance
        self.orbit_distance = orbit_distance
        self.angle_tolerance = math.radians(angle_tolerance_deg)
        
        # État du contrôleur
        self.state = "idle"  # "idle", "approaching", "orbiting", "final_approach", "arrived"
        
        # Gains pour l'approche (identiques à SinglePointController)
        self.kp_heading = 0.5     # Gain pour le cap (rad/s par rad d'erreur)
        self.kp_forward = 0.8     # Gain pour l'avance (m/s par m d'erreur)
        self.kp_lateral = 0.5     # Gain pour le latéral (m/s par m d'erreur)
        
        # Gains pour l'orbite (proportionnel)
        self.kp_orbit_heading = 2.0   # Gain P pour garder bearing = 0 pendant l'orbite
        self.kp_orbit_distance = 1.0  # Gain P pour maintenir la distance d'orbite
        self.orbit_speed = 0.8        # Vitesse latérale d'orbite (m/s)
        
        # Gains dérivés pour l'orbite
        self.kd_orbit_heading = 0.3   # Gain D pour le cap pendant l'orbite
        self.kd_orbit_distance = 0.2  # Gain D pour la distance d'orbite
        
        # État précédent pour le calcul dérivé (orbite)
        self._orbit_bearing_prev = 0.0
        self._orbit_distance_error_prev = 0.0
        self._orbit_dt = 0.1  # Période de contrôle estimée (s)
        
        # Limites de vitesse (m/s et rad/s)
        self.max_forward_speed = 0.5
        self.max_lateral_speed = 0.5
        self.max_angular_speed = 0.5
        
    def control_step(self, bearing, range_m, approach_angle_error, visible):
        """
        Calcule les commandes de vitesse pour approcher la cible avec le bon angle.
        
        Args:
            bearing: Angle vers la cible (rad), 0 = devant, + = droite
            range_m: Distance à la cible (m)
            approach_angle_error: Erreur angulaire d'approche (rad)
                                  = cage_angle - angle_position_robot_autour_cible
                                  Positif = robot doit orbiter dans le sens horaire (latéral +)
            visible: True si la cible est visible
            
        Returns:
            tuple: (forward_speed, lateral_speed, angular_speed)
                   Vitesses en m/s et rad/s
        """
        # Cible non visible: ne rien faire
        if not visible:
            self.state = "idle"
            return 0.0, 0.0, 0.0
        
        # Vérifier si on est arrivé (distance finale)
        if range_m < self.stop_distance:
            self.state = "arrived"
            return 0.0, 0.0, 0.0
        
        # Normaliser l'erreur d'angle entre -pi et pi
        approach_angle_error = math.atan2(math.sin(approach_angle_error), 
                                          math.cos(approach_angle_error))
        
        # ========== MACHINE À ÉTATS ==========
        
        # Marge pour la transition approche → orbite (évite les oscillations)
        orbit_margin = 0.2  # 20cm de marge
        
        # Phase 1: Approche initiale jusqu'à orbit_distance + marge
        if range_m > self.orbit_distance + orbit_margin:
            self.state = "Approaching"
            return self._approach_control(bearing, range_m, self.orbit_distance)
        
        if abs(approach_angle_error) > ( -self.angle_tolerance + np.pi):
            # Phase 3: Approche finale (bien aligné)
            self.state = "Final_approach"
            return self._approach_control(bearing, range_m, self.stop_distance)
        else:
            print("Approach angle error (deg):", np.degrees(approach_angle_error))
            self.state = "Orbiting"
            return self._orbit_control(bearing, range_m, approach_angle_error)
    
    def _approach_control(self, bearing, range_m, target_distance):
        """
        Contrôle d'approche vers une distance cible.
        
        Args:
            bearing: Angle vers la cible (rad), 0 = devant, + = droite
            range_m: Distance actuelle à la cible (m)
            target_distance: Distance cible à atteindre (m)
        """
        # Heading: bearing = 0
        angular_speed = self.kp_heading * bearing
        
        # Forward: avancer vers target_distance
        error_x = range_m * math.cos(bearing) - target_distance
        forward_speed = self.kp_forward * error_x
        
        # Lateral: correction latérale
        error_y = range_m * math.sin(bearing)
        lateral_speed = self.kp_lateral * error_y
        
        # Limitation
        forward_speed = np.clip(forward_speed, -self.max_forward_speed, self.max_forward_speed)
        lateral_speed = np.clip(lateral_speed, -self.max_lateral_speed, self.max_lateral_speed)
        angular_speed = np.clip(angular_speed, -self.max_angular_speed, self.max_angular_speed)

        print(f"forward_speed: {forward_speed:.2f}, lateral_speed: {lateral_speed:.2f}, "
              f"angular_speed: {angular_speed:.2f}")
        
        return forward_speed, lateral_speed, angular_speed
    
    def _orbit_control(self, bearing, range_m, approach_angle_error):
        """
        Contrôle d'orbite autour de la cible (PD pour forward et angular).
        - Garder bearing = 0 (regarder la cible)
        - Maintenir distance = orbit_distance
        - Se déplacer latéralement pour réduire approach_angle_error
        """
        print("Orbiting...")
        dt = self._orbit_dt
        
        # ========== HEADING (PD): garder bearing = 0 ==========
        # Terme proportionnel
        p_angular = self.kp_orbit_heading * bearing
        # Terme dérivé
        bearing_derivative = (bearing - self._orbit_bearing_prev) / dt if dt > 0 else 0.0
        d_angular = self.kd_orbit_heading * bearing_derivative
        # Mise à jour de l'état précédent
        self._orbit_bearing_prev = bearing
        # Commande PD
        angular_speed = p_angular + d_angular
        
        # ========== FORWARD (PD): maintenir distance d'orbite ==========
        distance_error = range_m - self.orbit_distance
        # Terme proportionnel
        p_forward = self.kp_orbit_distance * distance_error
        # Terme dérivé
        distance_derivative = (distance_error - self._orbit_distance_error_prev) / dt if dt > 0 else 0.0
        d_forward = self.kd_orbit_distance * distance_derivative
        # Mise à jour de l'état précédent
        self._orbit_distance_error_prev = distance_error
        # Commande PD
        forward_speed = p_forward + d_forward

        # ========== LATERAL (P): orbiter pour minimiser l'erreur d'angle ==========
        # approach_angle_error > 0 → orbiter vers la droite (lateral > 0)
        # approach_angle_error < 0 → orbiter vers la gauche (lateral < 0)
        orbit_direction = 1.0 if approach_angle_error > 0 else -1.0
        lateral_speed = orbit_direction * self.orbit_speed

        # Limitation
        forward_speed = np.clip(forward_speed, -self.max_forward_speed, self.max_forward_speed)
        lateral_speed = np.clip(lateral_speed, -self.max_lateral_speed, self.max_lateral_speed)
        angular_speed = np.clip(angular_speed, -self.max_angular_speed, self.max_angular_speed)
        print(f"  forward_speed: {forward_speed:.2f} (P:{p_forward:.2f} D:{d_forward:.2f}), "
              f"lateral_speed: {lateral_speed:.2f}, "
              f"angular_speed: {angular_speed:.2f} (P:{p_angular:.2f} D:{d_angular:.2f})"
              f" approach_angle_error (deg): {math.degrees(approach_angle_error):.2f}")
        
        return forward_speed, lateral_speed, angular_speed