"""
Contrôleurs pour les missions de docking BlueROV
- SinglePointController: Mission simple pour rejoindre un point unique
- CageFollowController: Mission complète pour rejoindre une cage (2 poteaux)
"""

import numpy as np
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
    
    def __init__(self, stop_distance=0.5):
        """
        Args:
            stop_distance: Distance d'arrêt à la cible (m)
        """
        self.stop_distance = stop_distance
        
        # État du contrôleur
        self.state = "idle"  # "idle", "tracking", "arrived"
        
        # Gains des contrôleurs (à ajuster selon tests réels)
        self.kp_heading = 1.5     # Gain pour le cap (rad/s par rad d'erreur)
        self.kp_forward = 0.8     # Gain pour l'avance (m/s par m d'erreur)
        self.kp_lateral = 0.5     # Gain pour le latéral (m/s par m d'erreur)
        
        # Limites de vitesse (m/s et rad/s)
        self.max_forward_speed = 0.7
        self.max_lateral_speed = 0.4
        self.max_angular_speed = 0.7
        
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


class CageFollowController:
    """
    Contrôleur simplifié avec 2 états et 2 contrôleurs indépendants
    
    États:
    - SEARCH: Rotation pour trouver les deux bouées
    - TRACKING_AND_NAVIGATION: Suivi de cage + navigation vers cible
    
    Contrôleurs indépendants:
    - Heading Controller: Oriente le robot pour centrer les bouées dans le FOV
    - Movement Controller: Navigue vers la cible (1m devant la cage)
    """
    
    def __init__(self, target_distance_ahead=1.0):
        """
        Args:
            target_distance_ahead: Distance cible devant les bouées (m)
        """
        self.target_distance_ahead = target_distance_ahead
        
        # États du contrôleur (seulement 2)
        self.state = "search"  # "search" ou "tracking_and_navigation"
        self.search_direction = 1  # 1 pour droite, -1 pour gauche
        
        # Gains pour les 2 contrôleurs indépendants
        self.kp_heading = 1.2     # Contrôleur cap : centrer les bouées
        self.kp_forward = 1.0     # Contrôleur déplacement : avant/arrière
        self.kp_lateral = 1.0     # Contrôleur déplacement : latéral
        
        # Paramètres de recherche
        self.search_angular_speed = 0.5  # Vitesse de rotation en mode recherche (rad/s)
        
        # Distance d'arrêt à la cible
        self.stop_distance = 0.5  # 50cm
        
    def both_buoys_visible(self, obs1, obs2):
        """
        Vérifie si les deux bouées sont visibles
        
        Args:
            obs1: (bearing, distance, visible) pour bouée 1
            obs2: (bearing, distance, visible) pour bouée 2
            
        Returns:
            bool: True si les deux bouées sont visibles
        """
        return obs1[2] and obs2[2]  # obs[2] = visible
    
    def heading_controller(self, obs1, obs2):
        """
        CONTRÔLEUR 1 (CAP): Centre le robot entre les deux bouées
        Objectif: Maintenir les bouées au centre du champ de vision
        
        Args:
            obs1: (bearing, distance, visible) pour bouée 1
            obs2: (bearing, distance, visible) pour bouée 2
            
        Returns:
            angular_speed: commande de vitesse angulaire (rad/s)
        """
        bearing1, _, _ = obs1
        bearing2, _, _ = obs2
        
        # Bearing vers le centre des deux bouées
        center_bearing = (bearing1 + bearing2) / 2
        
        # Contrôle proportionnel : tourner pour centrer
        angular_speed = self.kp_heading * center_bearing
        
        return angular_speed
    
    def movement_controller(self, obs1, obs2):
        """
        CONTRÔLEUR 2 (DÉPLACEMENT): Navigue vers la cible (1m devant les bouées)
        Utilise le contrôle d'erreur : error = target_position - robot_position
        Dans le repère local du robot (robot à l'origine)
        
        Args:
            obs1: (bearing, distance, visible) pour bouée 1
            obs2: (bearing, distance, visible) pour bouée 2
            
        Returns:
            (forward_speed, lateral_speed): commandes de vitesse (m/s)
        """
        bearing1, dist1, _ = obs1
        bearing2, dist2, _ = obs2
        
        # Position des bouées dans le repère du robot
        x1 = dist1 * math.cos(bearing1)
        y1 = dist1 * math.sin(bearing1)
        x2 = dist2 * math.cos(bearing2)
        y2 = dist2 * math.sin(bearing2)
        
        # Centre de la cage
        cage_center_x = (x1 + x2) / 2
        cage_center_y = (y1 + y2) / 2
        
        # Vecteur perpendiculaire pour trouver la cible (1m devant)
        bouee_vector_x = x2 - x1
        bouee_vector_y = y2 - y1
        bouee_length = math.sqrt(bouee_vector_x**2 + bouee_vector_y**2)
        
        if bouee_length > 0:
            perp_x = bouee_vector_y / bouee_length
            perp_y = -bouee_vector_x / bouee_length
        else:
            perp_x, perp_y = 0, 1
        
        # Position cible dans le repère du robot
        target_x = cage_center_x + self.target_distance_ahead * perp_x
        target_y = cage_center_y + self.target_distance_ahead * perp_y
        
        # CONTRÔLE D'ERREUR : error = target_position - robot_position
        # Robot position = (0, 0) dans son propre repère
        # Donc error = target_position - (0, 0) = target_position
        error_x = target_x  # Erreur en X (avant/arrière)
        error_y = target_y  # Erreur en Y (gauche/droite)

        # Contrôles proportionnels sur l'erreur
        forward_speed = self.kp_forward * error_x
        lateral_speed = self.kp_lateral * error_y

        return forward_speed, lateral_speed
    
    def calculate_distance_to_target(self, obs1, obs2):
        """
        Calcule la distance entre le robot et la cible
        
        Args:
            obs1: (bearing, distance, visible) pour bouée 1
            obs2: (bearing, distance, visible) pour bouée 2
            
        Returns:
            float: distance à la cible (m)
        """
        bearing1, dist1, _ = obs1
        bearing2, dist2, _ = obs2
        
        # Position des bouées dans le repère du robot
        x1 = dist1 * math.cos(bearing1)
        y1 = dist1 * math.sin(bearing1)
        x2 = dist2 * math.cos(bearing2)
        y2 = dist2 * math.sin(bearing2)
        
        # Centre de la cage
        cage_center_x = (x1 + x2) / 2
        cage_center_y = (y1 + y2) / 2
        
        # Vecteur perpendiculaire
        bouee_vector_x = x2 - x1
        bouee_vector_y = y2 - y1
        bouee_length = math.sqrt(bouee_vector_x**2 + bouee_vector_y**2)
        
        if bouee_length > 0:
            perp_x = bouee_vector_y / bouee_length
            perp_y = -bouee_vector_x / bouee_length
        else:
            perp_x, perp_y = 0, 1
        
        # Position cible
        target_x = cage_center_x + self.target_distance_ahead * perp_x
        target_y = cage_center_y + self.target_distance_ahead * perp_y
        
        # Distance euclidienne
        distance = math.sqrt(target_x**2 + target_y**2)
        
        return distance
    
    def control_step(self, obs1, obs2):
        """
        Calcule les commandes de contrôle avec 2 états et 2 contrôleurs indépendants
        Logique identique à bluerov_simulation.py
        
        Args:
            obs1: (bearing, distance, visible) pour bouée 1
            obs2: (bearing, distance, visible) pour bouée 2
            
        Returns:
            (forward_speed, lateral_speed, angular_speed): commandes de vitesse
            forward_speed: vitesse avant/arrière (m/s)
            lateral_speed: vitesse latérale (m/s)
            angular_speed: vitesse angulaire (rad/s)
        """
        both_visible = self.both_buoys_visible(obs1, obs2)
        
        if not both_visible:
            # ========== ÉTAT 1: IDLE ==========
            # Cage non visible: ne rien faire (on suppose qu'elle sera visible au départ)
            self.state = "idle"
            
            # Ne pas tourner, attendre que la cage soit visible
            return 0.0, 0.0, 0.0
        
        else:
            # ========== ÉTAT 2: TRACKING_AND_NAVIGATION ==========
            # Utiliser les 2 contrôleurs indépendants
            self.state = "tracking_and_navigation"
            
            # Vérifier si on est arrivé à la cible (arrêt à 10cm)
            distance_to_target = self.calculate_distance_to_target(obs1, obs2)
            
            # Arrêt si très proche de la cible
            if distance_to_target < self.stop_distance:
                return 0.0, 0.0, 0.0
            
            # CONTRÔLEUR 1: CAP (indépendant)
            angular_speed = self.heading_controller(obs1, obs2)
            
            # CONTRÔLEUR 2: DÉPLACEMENT (indépendant)
            forward_speed, lateral_speed = self.movement_controller(obs1, obs2)
            
            # Limitation des vitesses (sécurité)
            forward_speed = np.clip(forward_speed, -0.5, 0.5)
            lateral_speed = np.clip(lateral_speed, -0.3, 0.3)
            angular_speed = np.clip(angular_speed, -0.5, 0.5)
            
            return forward_speed, lateral_speed, angular_speed
