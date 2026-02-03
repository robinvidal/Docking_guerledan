"""
Simulation BlueROV - Navigation entre deux bouées (4 robots en temps réel)
Utilise uniquement les données locales du robot (cap et distance des bouées)
Objectif: Rejoindre un point centré entre les deux bouées, à 1m devant elles
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import math
import time
from plot_bluerov import plot_bluerov

class BlueROV:
    """Classe représentant le BlueROV avec ses capacités de mouvement et de détection"""
    
    def __init__(self, x=0, y=0, heading=0):
        """
        Initialise le robot
        Args:
            x, y: position initiale (m)
            heading: cap initial en radians (0 = vers le nord/haut)
        """
        self.x = x
        self.y = y
        self.heading = heading  # radians, 0 = nord, pi/2 = est
        
        # Paramètres de contrôle
        self.max_speed_x = 0.25  # m/s
        self.max_speed_y = 0.5  # m/s

        self.max_angular_speed = 0.5  # rad/s
        
        # Historique pour le tracé
        self.trajectory_x = [x]
        self.trajectory_y = [y]
        
    def get_buoy_observations(self, buoy1_pos, buoy2_pos):
        """
        Simule les observations du robot avec un champ de vision limité à ±45°
        Dans le repère local du robot (0° = devant lui)
        
        Returns:
            tuple: ((bearing1, distance1, visible1), (bearing2, distance2, visible2))
                   bearing en radians [-pi, pi], 0 = devant le robot
                   visible = True si la bouée est dans le champ de vision
        """
        observations = []
        fov_half_angle = np.pi / 4  # 45° de chaque côté
        
        for buoy_pos in [buoy1_pos, buoy2_pos]:
            # Vecteur du robot vers la bouée
            dx = buoy_pos[0] - self.x
            dy = buoy_pos[1] - self.y
            
            # Distance
            distance = np.sqrt(dx**2 + dy**2)
            
            # Angle absolu vers la bouée (mathématique: 0° = Est, 90° = Nord)
            angle_to_buoy = math.atan2(dy, dx)
            
            # Conversion vers convention navigation (0° = Nord, 90° = Est)
            # En navigation: Nord = π/2, Est = 0, Sud = -π/2, Ouest = π
            nav_angle_to_buoy = angle_to_buoy - np.pi/2
            nav_angle_to_buoy = math.atan2(math.sin(nav_angle_to_buoy), math.cos(nav_angle_to_buoy))
            
            # Bearing relatif par rapport au cap du robot
            # bearing = 0 signifie devant le robot
            bearing = nav_angle_to_buoy - self.heading
            
            # Normaliser l'angle entre -pi et pi
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            
            # Vérifier si la bouée est dans le champ de vision (±45°)
            visible = abs(bearing) <= fov_half_angle
            
            observations.append((bearing, distance, visible))
            
        return observations
    
    def move(self, forward_speed, lateral_speed, angular_speed, dt):
        """
        Déplace le robot selon les commandes de vitesse en 3 axes
        Args:
            forward_speed: vitesse avant/arrière dans le repère du robot (m/s)
            lateral_speed: vitesse latérale dans le repère du robot (m/s) 
            angular_speed: vitesse angulaire (rad/s)
            dt: pas de temps (s)
        """
        # Limiter les vitesses
        forward_speed = np.clip(forward_speed, -self.max_speed_x, self.max_speed_x)
        lateral_speed = np.clip(lateral_speed, -self.max_speed_y, self.max_speed_y)
        angular_speed = np.clip(angular_speed, -self.max_angular_speed, self.max_angular_speed)
        
        # Conversion des vitesses du repère robot vers le repère monde
        # Remarque : self.heading est exprimé avec la convention "navigation"
        # (0 = Nord). Pour utiliser cos/sin (qui attendent 0 = Est),
        # on convertit vers l'angle mathématique : theta_math = heading + pi/2
        theta = self.heading + math.pi/2
        # Forward = axe X du robot (devant), Lateral = axe Y du robot (gauche)
        world_vx = forward_speed * math.cos(theta) - lateral_speed * math.sin(theta)
        world_vy = forward_speed * math.sin(theta) + lateral_speed * math.cos(theta)
        
        # Mise à jour de la position
        self.x += world_vx * dt
        self.y += world_vy * dt
        
        # Mise à jour du cap
        self.heading += angular_speed * dt
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))
        
        # Sauvegarder la trajectoire
        self.trajectory_x.append(self.x)
        self.trajectory_y.append(self.y)

class Controller:
    """Contrôleur simplifié avec 2 états et 2 contrôleurs indépendants"""
    
    def __init__(self):
        self.target_distance_ahead = 1.0  # Distance cible devant les bouées (m)
        
        # États du contrôleur (seulement 2)
        self.state = "search"  # "search" ou "tracking_and_navigation"
        self.search_direction = 1  # 1 pour droite, -1 pour gauche
        
        # Gains pour les 2 contrôleurs indépendants
        self.kp_heading = 1.2     # Contrôleur cap : centrer les bouées
        self.kp_forward = 0.5      # Contrôleur déplacement : avant/arrière
        self.kp_lateral = 1    # Contrôleur déplacement : latéral
        
        # Paramètres de recherche
        self.search_angular_speed = 0.5  # Vitesse de rotation en mode recherche
        
    def both_buoys_visible(self, obs1, obs2):
        """Vérifie si les deux bouées sont visibles"""
        return obs1[2] and obs2[2]  # obs[2] = visible
    
    def heading_controller(self, obs1, obs2):
        """
        Contrôleur CAP : Centre le robot entre les deux bouées
        
        Returns:
            angular_speed: commande de vitesse angulaire pour centrer les bouées
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
        Contrôleur DÉPLACEMENT : Navigue vers la cible (1m devant les bouées)
        Utilise le contrôle d'erreur : error = target_position - robot_position
        Dans le repère local du robot (robot à l'origine)
        
        Returns:
            (forward_speed, lateral_speed): commandes de déplacement
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
        error_x = target_x - 0  # Erreur en X (avant/arrière)
        error_y = target_y - 0  # Erreur en Y (gauche/droite)

        # Contrôles proportionnels sur l'erreur
        # Associer correctement les gains aux axes :
        # error_x -> forward, error_y -> lateral
        forward_speed = self.kp_forward * error_x
        lateral_speed = self.kp_lateral * error_y

        return forward_speed, lateral_speed
    
    def control_step(self, robot, obs1, obs2):
        """
        Contrôle simplifié avec 2 états et 2 contrôleurs indépendants
        
        Returns:
            (forward_speed, lateral_speed, angular_speed): commandes de vitesse
        """
        both_visible = self.both_buoys_visible(obs1, obs2)
        
        if not both_visible:
            # ÉTAT 1: SEARCH - Tourner jusqu'à voir les deux bouées
            self.state = "search"
            
            # Déterminer la direction de recherche
            if obs1[2] and not obs2[2]:  # Seule bouée 1 visible
                angular_speed = -self.search_angular_speed  # Tourner vers la droite
            elif obs2[2] and not obs1[2]:  # Seule bouée 2 visible
                angular_speed = self.search_angular_speed   # Tourner vers la gauche
            else:  # Aucune bouée visible
                angular_speed = self.search_direction * self.search_angular_speed
            
            return 0.0, 0.0, angular_speed
        
        else:
            # ÉTAT 2: TRACKING_AND_NAVIGATION - Utiliser les 2 contrôleurs indépendants
            self.state = "tracking_and_navigation"
            
            # Vérifier si on est arrivé à la cible (arrêt à 10cm)
            bearing1, dist1, _ = obs1
            bearing2, dist2, _ = obs2
            
            # Calculer la distance à la cible pour l'arrêt
            x1 = dist1 * math.cos(bearing1)
            y1 = dist1 * math.sin(bearing1)
            x2 = dist2 * math.cos(bearing2)
            y2 = dist2 * math.sin(bearing2)
            
            cage_center_x = (x1 + x2) / 2
            cage_center_y = (y1 + y2) / 2
            
            bouee_vector_x = x2 - x1
            bouee_vector_y = y2 - y1
            bouee_length = math.sqrt(bouee_vector_x**2 + bouee_vector_y**2)
            
            if bouee_length > 0:
                perp_x = bouee_vector_y / bouee_length
                perp_y = -bouee_vector_x / bouee_length
            else:
                perp_x, perp_y = 0, 1
            
            target_x = cage_center_x + self.target_distance_ahead * perp_x
            target_y = cage_center_y + self.target_distance_ahead * perp_y
            distance_to_target = math.sqrt(target_x**2 + target_y**2)
            
            # Arrêt si très proche de la cible (10cm)
            if distance_to_target < 0.1:
                return 0.0, 0.0, 0.0
            
            # CONTRÔLEUR 1: CAP (indépendant)
            angular_speed = self.heading_controller(obs1, obs2)
            
            # CONTRÔLEUR 2: DÉPLACEMENT (indépendant)
            forward_speed, lateral_speed = self.movement_controller(obs1, obs2)
            
            # Limitation des vitesses
            forward_speed = np.clip(forward_speed, -0.5, 0.5)
            lateral_speed = np.clip(lateral_speed, -0.3, 0.3)
            angular_speed = np.clip(angular_speed, -0.5, 0.5)
            
            return forward_speed, lateral_speed, angular_speed

class MultiRobotSimulation:
    """Classe principale de simulation avec 4 robots"""
    
    def __init__(self):
        # Initialisation des bouées (positions fixes dans le monde)
        self.buoy1_pos = np.array([-0.4, 5.0])  # Bouée gauche
        self.buoy2_pos = np.array([0.4, 5.0])   # Bouée droite (80cm d'écart)
        
        # Point cible calculé
        mid_x = (self.buoy1_pos[0] + self.buoy2_pos[0]) / 2
        mid_y = (self.buoy1_pos[1] + self.buoy2_pos[1]) / 2
        self.target_pos = np.array([mid_x, mid_y - 1.0])  # 1m devant les bouées
        
        # Initialisation de 4 robots à des positions différentes
        start_positions = [
            (-2.5, 0, np.pi/4),      # Robot 1: à gauche, orienté vers NE
            (2.5, 0, -np.pi/4),      # Robot 2: à droite, orienté vers NW (symétrique)
            (0.2, -1.5, np.pi/2),    # Robot 3: légèrement décalé du centre, orienté vers N
            (-1, 2, 0)               # Robot 4: à gauche centre, orienté vers E
        ]
        
        self.robots = []
        self.controllers = []
        self.colors = ['blue', 'red', 'green', 'orange']
        
        for i, (x, y, heading) in enumerate(start_positions):
            self.robots.append(BlueROV(x, y, heading))
            self.controllers.append(Controller())
        
        # Paramètres de simulation
        self.dt = 0.05  # pas de temps plus petit pour temps réel
        self.start_time = time.time()
        
        # Pour sauvegarder les commandes pour l'affichage
        self.last_commands = [(0, 0, 0)] * 4
        
        # Pour l'animation
        self.fig, self.ax = plt.subplots(figsize=(14, 10))
        self.setup_plot()
        # Pause flag contrôlé par la barre espace
        self.paused = False
        
    def setup_plot(self):
        """Configure l'affichage de la simulation"""
        self.ax.set_xlim(-4, 4)
        self.ax.set_ylim(-2, 7)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Simulation BlueROV - 4 Robots Navigation entre bouées (Temps Réel)')
        
        # Bouées et cible
        self.buoy1_circle = plt.Circle(self.buoy1_pos, 0.1, color='darkred', label='Bouée 1')
        self.buoy2_circle = plt.Circle(self.buoy2_pos, 0.1, color='darkred', label='Bouée 2')
        self.target_circle = plt.Circle(self.target_pos, 0.15, color='gold', 
                                      edgecolor='black', linewidth=2, label='Cible')
        
        self.ax.add_patch(self.buoy1_circle)
        self.ax.add_patch(self.buoy2_circle)
        self.ax.add_patch(self.target_circle)
        
        # Ligne entre les bouées pour visualiser la "cage"
        self.ax.plot([self.buoy1_pos[0], self.buoy2_pos[0]], 
                    [self.buoy1_pos[1], self.buoy2_pos[1]], 
                    'k-', linewidth=3, alpha=0.7, label='Cage (80cm)')
        
        # Robots (triangles orientés)
        self.robot_triangles = []
        self.trajectory_lines = []
        self.vision_lines_1 = []
        self.vision_lines_2 = []
        
        for i in range(4):
            # Réserver une liste de patches pour chaque robot (sera rempli dans update_plot)
            self.robot_triangles.append([])
            
            # Ligne de trajectoire
            line, = self.ax.plot([], [], color=self.colors[i], alpha=0.3, linewidth=1)
            self.trajectory_lines.append(line)
            
            # Lignes de vision vers les bouées
            line1, = self.ax.plot([], [], color=self.colors[i], linestyle='--', alpha=0.4)
            line2, = self.ax.plot([], [], color=self.colors[i], linestyle='--', alpha=0.4)
            self.vision_lines_1.append(line1)
            self.vision_lines_2.append(line2)
        
        # Informations textuelles pour chaque robot
        self.info_texts = []
        positions = [(0.02, 0.98), (0.52, 0.98), (0.02, 0.50), (0.52, 0.50)]
        
        for i, (x_pos, y_pos) in enumerate(positions):
            text = self.ax.text(x_pos, y_pos, '', transform=self.ax.transAxes, 
                              verticalalignment='top', fontfamily='monospace', fontsize=8,
                              bbox=dict(boxstyle='round', facecolor=self.colors[i], alpha=0.2))
            self.info_texts.append(text)
        
        self.ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=4)
        
    def update_plot(self):
        """Met à jour l'affichage"""
        current_time = time.time() - self.start_time
        
        for i, robot in enumerate(self.robots):
            # Position du robot : supprimer les patches précédents (s'il y en a)
            for p in list(self.robot_triangles[i]):
                try:
                    p.remove()
                except Exception:
                    pass
            # Dessiner le BlueROV stylisé via plot_bluerov
            # plot_bluerov attend theta mathématique (0 = +x local). Notre heading
            # est en convention navigation (0 = Nord). Convertir : theta_math = heading + pi/2
            theta_math = robot.heading + math.pi/2
            patches_added = plot_bluerov(self.ax, x=robot.x, y=robot.y, theta=theta_math,
                                         size=0.3, color=self.colors[i], alpha=0.9, label=None, linewidth=0.8)
            # Sauvegarder la liste de patches pour suppression au prochain frame
            self.robot_triangles[i] = patches_added
            
            # Trajectoire
            self.trajectory_lines[i].set_data(robot.trajectory_x, robot.trajectory_y)
            
            # Observations avec champ de vision
            obs1, obs2 = robot.get_buoy_observations(self.buoy1_pos, self.buoy2_pos)
            
            # Commandes de contrôle pour affichage
            forward_cmd, lateral_cmd, angular_cmd = 0, 0, 0
            if i < len(self.last_commands):
                forward_cmd, lateral_cmd, angular_cmd = self.last_commands[i]
            
            # Lignes de vision vers les bouées (seulement si visibles)
            if obs1[2]:  # Si bouée 1 visible
                self.vision_lines_1[i].set_data([robot.x, self.buoy1_pos[0]], 
                                               [robot.y, self.buoy1_pos[1]])
            else:
                self.vision_lines_1[i].set_data([], [])
                
            if obs2[2]:  # Si bouée 2 visible
                self.vision_lines_2[i].set_data([robot.x, self.buoy2_pos[0]], 
                                               [robot.y, self.buoy2_pos[1]])
            else:
                self.vision_lines_2[i].set_data([], [])
            
            # Informations pour ce robot
            dist_to_target = np.linalg.norm([robot.x - self.target_pos[0], 
                                           robot.y - self.target_pos[1]])
            
            # Déterminer l'état du robot
            controller_state = self.controllers[i].state
            status_map = {
                "search": "Recherche",
                "tracking_and_navigation": "Track+Nav"
            }
            status = status_map.get(controller_state, controller_state)
            
            # Visibilité des bouées
            b1_vis = "VIS" if obs1[2] else "NON"
            b2_vis = "VIS" if obs2[2] else "NON"
            
            # Calculer les distances X et Y de la cible dans le repère du robot
            target_x, target_y = 0, 0  # Valeurs par défaut
            if obs1[2] and obs2[2]:  # Si les deux bouées sont visibles
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
                
                # Vecteur perpendiculaire pour trouver la cible
                bouee_vector_x = x2 - x1
                bouee_vector_y = y2 - y1
                bouee_length = math.sqrt(bouee_vector_x**2 + bouee_vector_y**2)
                
                if bouee_length > 0:
                    perp_x = bouee_vector_y / bouee_length
                    perp_y = -bouee_vector_x / bouee_length
                else:
                    perp_x, perp_y = 0, 1
                
                # Position cible dans le repère du robot
                target_x = cage_center_x + 1.0 * perp_x  # 1m devant
                target_y = cage_center_y + 1.0 * perp_y
            
            info = f"""Robot {i+1} - t={current_time:.1f}s
Pos: ({robot.x:.1f}, {robot.y:.1f})
Cap: {np.degrees(robot.heading):.0f}°
État: {status}

B1: {b1_vis} {np.degrees(obs1[0]):+.0f}° {obs1[1]:.1f}m
B2: {b2_vis} {np.degrees(obs2[0]):+.0f}° {obs2[1]:.1f}m
Dist cible: {dist_to_target:.2f}m

Cible X: {target_x:+.2f}m (avant/arrière)
Cible Y: {target_y:+.2f}m (gauche/droite)

Cmd: F={forward_cmd:+.2f} L={lateral_cmd:+.2f} ω={np.degrees(angular_cmd):+.0f}°/s"""
            
            self.info_texts[i].set_text(info)
        
        # Titre avec temps global (ajouter indication PAUSE)
        title = 'BlueROV - Tracking cage + Navigation simultanés'
        if getattr(self, 'paused', False):
            title += '   [PAUSED] (appuyez sur espace pour reprendre)'
        self.ax.set_title(title)
        
    def setup_plot(self):
        """Configure l'affichage de la simulation"""
        self.ax.set_xlim(-4, 4)
        self.ax.set_ylim(-2, 7)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('BlueROV - Tracking cage + Navigation cible (contrôle découplé XY)')
        
        # Bouées et cible
        self.buoy1_circle = plt.Circle(self.buoy1_pos, 0.1, color='darkred', label='Bouée 1')
        self.buoy2_circle = plt.Circle(self.buoy2_pos, 0.1, color='darkred', label='Bouée 2')
        self.target_circle = plt.Circle(self.target_pos, 0.15, color='gold', 
                                      edgecolor='black', linewidth=2, label='Cible')
        
        self.ax.add_patch(self.buoy1_circle)
        self.ax.add_patch(self.buoy2_circle)
        self.ax.add_patch(self.target_circle)
        
        # Ligne entre les bouées pour visualiser la "cage"
        self.ax.plot([self.buoy1_pos[0], self.buoy2_pos[0]], 
                    [self.buoy1_pos[1], self.buoy2_pos[1]], 
                    'k-', linewidth=3, alpha=0.7, label='Cage (80cm)')
        
        # Robots (triangles orientés)
        self.robot_triangles = []
        self.trajectory_lines = []
        self.vision_lines_1 = []
        self.vision_lines_2 = []
        
        for i in range(4):
            # Réserver une liste de patches pour chaque robot (sera rempli dans update_plot)
            self.robot_triangles.append([])
            
            # Ligne de trajectoire
            line, = self.ax.plot([], [], color=self.colors[i], alpha=0.3, linewidth=1)
            self.trajectory_lines.append(line)
            
            # Lignes de vision vers les bouées (visibles seulement si dans le champ de vision)
            line1, = self.ax.plot([], [], color=self.colors[i], linestyle='--', alpha=0.6, linewidth=2)
            line2, = self.ax.plot([], [], color=self.colors[i], linestyle='--', alpha=0.6, linewidth=2)
            self.vision_lines_1.append(line1)
            self.vision_lines_2.append(line2)
        
        # Informations textuelles pour chaque robot
        self.info_texts = []
        positions = [(0.02, 0.98), (0.52, 0.98), (0.02, 0.50), (0.52, 0.50)]
        
        for i, (x_pos, y_pos) in enumerate(positions):
            text = self.ax.text(x_pos, y_pos, '', transform=self.ax.transAxes, 
                              verticalalignment='top', fontfamily='monospace', fontsize=8,
                              bbox=dict(boxstyle='round', facecolor=self.colors[i], alpha=0.2))
            self.info_texts.append(text)
        
        self.ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=4)
        
    def step(self):
        """Un pas de simulation pour tous les robots"""
        robots_arrived = 0
        robots_stopped = 0
        
        for i, robot in enumerate(self.robots):
            # Observations du robot avec champ de vision limité
            obs1, obs2 = robot.get_buoy_observations(self.buoy1_pos, self.buoy2_pos)
            
            # Calcul des commandes (utilise seulement les données locales)
            forward_speed, lateral_speed, angular_speed = self.controllers[i].control_step(robot, obs1, obs2)
            
            # Sauvegarder les commandes pour l'affichage
            self.last_commands[i] = (forward_speed, lateral_speed, angular_speed)
            
            # Mouvement du robot
            robot.move(forward_speed, lateral_speed, angular_speed, self.dt)
            
            # Vérifier si le robot est arrivé (distance < 10cm)
            dist_to_target = np.linalg.norm([robot.x - self.target_pos[0], 
                                           robot.y - self.target_pos[1]])
            if dist_to_target < 0.1:  # 10cm
                robots_arrived += 1
            
            # Vérifier si le robot s'est arrêté (vitesse nulle)
            if (abs(forward_speed) < 0.01 and abs(lateral_speed) < 0.01 and 
                abs(angular_speed) < 0.01):
                robots_stopped += 1
        
        # Temps réel
        elapsed_time = time.time() - self.start_time
        
        # Arrêt si tous les robots sont arrivés OU si tous sont arrêtés OU après 2min
        return robots_arrived == 4 or robots_stopped == 4 or elapsed_time > 120
    
    def run_simulation(self):
        """Lance la simulation en temps réel"""
        # Handler clavier pour pause
        def on_key(event):
            if event.key == ' ':  # barre espace
                self.paused = not self.paused

        self.fig.canvas.mpl_connect('key_press_event', on_key)

        def animate_func(frame):
            # Si en pause, on n'avance pas la simulation mais on met à jour l'affichage
            if not getattr(self, 'paused', False):
                finished = self.step()
            else:
                finished = False
            self.update_plot()
            
            if finished:
                elapsed_time = time.time() - self.start_time
                print(f"\nSimulation terminée après {elapsed_time:.1f}s")
                
                # Afficher les distances finales
                for i, robot in enumerate(self.robots):
                    dist = np.linalg.norm([robot.x - self.target_pos[0], 
                                         robot.y - self.target_pos[1]])
                    status = "ARRIVÉ (< 10cm)" if dist < 0.1 else f"distance = {dist:.3f}m"
                    print(f"Robot {i+1}: {status}")
                
                return []
            
            return []
        
        # Animation en temps réel (50ms = 20 FPS)
        anim = FuncAnimation(self.fig, animate_func, interval=50, blit=False)
        plt.show()
        return anim

if __name__ == "__main__":
    # Créer et lancer la simulation avec 4 robots
    sim = MultiRobotSimulation()
    print("Démarrage de la simulation BlueROV avec 4 robots")
    print("CHAMP DE VISION limité à ±45° pour chaque robot")
    print("TRACKING + NAVIGATION simultanés vers la cible")
    print("CONTROLE découplé 3 axes :")
    print("   • Rotation : tracking de la cage (maintenir au centre du FOV)")
    print("   • Forward/Lateral : navigation directe vers la cible")
    print("DONNEES LOCALES seulement (cap et distance des bouées visibles)")
    print("RECHERCHE automatique si bouées hors champ de vision")
    print("ARRET automatique à moins de 10cm de la cible")
    print("\nAppuyez sur Ctrl+C pour arrêter la simulation")
    
    try:
        # Lancer la simulation en temps réel
        anim = sim.run_simulation()
    except KeyboardInterrupt:
        print("\nSimulation arrêtée par l'utilisateur")