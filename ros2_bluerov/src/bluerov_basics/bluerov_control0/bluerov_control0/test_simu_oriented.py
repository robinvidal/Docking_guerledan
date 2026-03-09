"""
Simulation BlueROV - Approche orientée vers une cage
Utilise le OrientedApproachController depuis control.py
Chaque robot doit approcher la cible par un angle spécifique (normale au fond de la cage)

Phases:
1. Approche jusqu'à 1.5m
2. Orbite autour de la cible pour s'aligner avec l'angle de la cage
3. Approche finale jusqu'à stop_distance
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import math
import scipy as sp
import time
from plot_bluerov import plot_bluerov
from control import OrientedApproachController


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
        self.max_speed_x = 0.5  # m/s
        self.max_speed_y = 0.5  # m/s
        self.max_angular_speed = 0.5  # rad/s
        
        # Historique pour le tracé
        self.trajectory_x = [x]
        self.trajectory_y = [y]
        
    def get_target_observation(self, target_pos, cage_angle):
        """
        Calcule le range, bearing et l'erreur d'angle d'approche vers la cible
        
        Args:
            target_pos: Position de la cible (x, y)
            cage_angle: Orientation de la cage (rad, repère monde, normale au fond)
        
        Returns:
            tuple: (bearing, range_m, approach_angle_error, visible)
                   bearing en radians [-pi, pi], 0 = devant le robot
                   range_m en mètres
                   approach_angle_error: erreur entre position angulaire actuelle et cible
                   visible = True si le bearing est dans le FOV (±45°)
        """
        fov_half_angle = np.pi / 4  # 45° de chaque côté
        
        # Vecteur du robot vers la cible
        dx = target_pos[0] - self.x
        dy = target_pos[1] - self.y
        
        # Distance
        range_m = np.sqrt(dx**2 + dy**2)
        
        # Angle absolu vers la cible (mathématique: 0° = Est, 90° = Nord)
        angle_to_target = math.atan2(dy, dx)
        
        # Conversion vers convention navigation (0° = Nord, 90° = Est)
        nav_angle_to_target = angle_to_target - np.pi/2
        nav_angle_to_target = math.atan2(math.sin(nav_angle_to_target), math.cos(nav_angle_to_target))
        
        # Bearing relatif par rapport au cap du robot
        bearing = nav_angle_to_target - self.heading
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        
        # ========== CALCUL DE L'ERREUR D'ANGLE D'APPROCHE ==========
        # Position angulaire actuelle du robot autour de la cible
        # = depuis quelle direction le robot voit la cible
        # Vecteur cible -> robot
        dx_from_target = self.x - target_pos[0]
        dy_from_target = self.y - target_pos[1]
        
        # Angle de la position du robot par rapport à la cible (repère monde, convention math)
        robot_angle_around_target = math.atan2(dy_from_target, dx_from_target)
        
        # L'angle d'approche cible = cage_angle (le robot doit arriver FACE à la cage)
        # Si cage_angle = 45°, le robot doit arriver depuis 45° + 180° = 225° 
        # pour être face à l'entrée de la cage
        desired_approach_angle = cage_angle 
        
        # Erreur = angle_cible - position_actuelle
        approach_angle_error = desired_approach_angle - robot_angle_around_target
        approach_angle_error = math.atan2(math.sin(approach_angle_error), math.cos(approach_angle_error))
        
        # Vérifier si la cible est dans le champ de vision (±45°)
        visible = abs(bearing) <= fov_half_angle
        
        return bearing, range_m, approach_angle_error, visible
    
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
        theta = self.heading + math.pi/2
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
    """Contrôleur utilisant OrientedApproachController"""
    
    def __init__(self):
        """Initialise le contrôleur"""
        self.oriented_controller = OrientedApproachController(
            stop_distance=0.5,
            orbit_distance=1.5,
            angle_tolerance_deg=5.0
        )
        self.state = self.oriented_controller.state
    
    def control_step(self, bearing, range_m, approach_angle_error, visible):
        """
        Calcule les commandes de vitesse pour approcher avec le bon angle
        
        Args:
            bearing: angle vers la cible (rad), 0 = devant
            range_m: distance à la cible (m)
            approach_angle_error: erreur d'angle d'approche (rad)
            visible: True si cible visible
            
        Returns:
            (forward_speed, lateral_speed, angular_speed): vitesses en m/s et rad/s
        """
        forward_speed, lateral_speed, angular_speed = self.oriented_controller.control_step(
            bearing, range_m, approach_angle_error, visible
        )
        self.state = self.oriented_controller.state
        return forward_speed, lateral_speed, angular_speed


class MultiRobotSimulation:
    """Classe principale de simulation avec 4 robots approchant une cible orientée"""
    
    def __init__(self):
        # Cible unique avec angle d'orientation
        self.target_pos = np.array([0.0, 5.0])  # Cible à 5m devant
        self.cage_angle = np.pi / 4  # 45° - La cage "regarde" vers le Nord-Est
        # Le robot doit donc arriver depuis le Nord-Est (heading = 45° + 180° = 225°)
        
        # Initialisation de 4 robots à des positions différentes
        start_positions = [
            (-3.0, 0, 0),             # Robot 1: à gauche, orienté vers Nord
            (3.0, 0, 0),              # Robot 2: à droite, orienté vers Nord
            (0.0, -2.0, np.pi/4),     # Robot 3: en bas, orienté vers NE
            (-2.0, 3.0, -np.pi/2)     # Robot 4: en haut à gauche, orienté vers Est
        ]
        
        self.robots = []
        self.controllers = []
        self.colors = ['blue', 'red', 'green', 'orange']
        
        for i, (x, y, heading) in enumerate(start_positions):
            self.robots.append(BlueROV(x, y, heading))
            self.controllers.append(Controller())
        
        # Paramètres de simulation
        self.dt = 0.05  # pas de temps (20 Hz)
        self.start_time = time.time()
        
        # Pour sauvegarder les commandes et observations
        self.last_commands = [(0.0, 0.0, 0.0)] * 4
        self.last_observations = [(0, 0, 0, False)] * 4
        
        # Pour l'animation
        self.fig, self.ax = plt.subplots(figsize=(14, 10))
        self.setup_plot()
        self.paused = False
        
    def setup_plot(self):
        """Configure l'affichage de la simulation"""
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-3, 8)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Simulation BlueROV - Oriented Approach (OrientedApproachController)')
        
        # Cible unique
        self.target_circle = plt.Circle(self.target_pos, 0.2, color='gold', 
                                        edgecolor='black', linewidth=2, label='Cible')
        self.ax.add_patch(self.target_circle)
        
        # Cercle d'orbite (1.5m)
        self.orbit_circle = plt.Circle(self.target_pos, 1.5, color='gray', 
                                       fill=False, linestyle='--', alpha=0.5, label='Orbite (1.5m)')
        self.ax.add_patch(self.orbit_circle)
        
        # Flèche d'orientation de la cage
        arrow_length = 1.0
        arrow_dx = arrow_length * math.cos(self.cage_angle)
        arrow_dy = arrow_length * math.sin(self.cage_angle)
        self.cage_arrow = self.ax.arrow(self.target_pos[0], self.target_pos[1],
                                        arrow_dx, arrow_dy,
                                        head_width=0.15, head_length=0.1,
                                        fc='purple', ec='purple', linewidth=2,
                                        label=f'Angle cage ({np.degrees(self.cage_angle):.0f}°)')
        
        # Point d'arrivée idéal (où le robot doit arriver)
        arrival_distance = 0.5  # stop_distance
        arrival_x = self.target_pos[0] + arrival_distance * math.cos(self.cage_angle)
        arrival_y = self.target_pos[1] + arrival_distance * math.sin(self.cage_angle)
        self.arrival_point = plt.Circle((arrival_x, arrival_y), 0.1, color='purple', 
                                        alpha=0.5, label='Point arrivée')
        self.ax.add_patch(self.arrival_point)
        
        # Robots (triangles orientés)
        self.robot_triangles = []
        self.trajectory_lines = []
        self.vision_lines = []
        
        for i in range(4):
            self.robot_triangles.append([])
            
            # Ligne de trajectoire
            line, = self.ax.plot([], [], color=self.colors[i], alpha=0.3, linewidth=1)
            self.trajectory_lines.append(line)
            
            # Ligne de vision vers la cible
            line_vision, = self.ax.plot([], [], color=self.colors[i], linestyle='--', alpha=0.6, linewidth=2)
            self.vision_lines.append(line_vision)
        
        # Informations textuelles pour chaque robot
        self.info_texts = []
        positions = [(0.02, 0.98), (0.68, 0.98), (0.02, 0.48), (0.68, 0.48)]
        
        for i, (x_pos, y_pos) in enumerate(positions):
            text = self.ax.text(x_pos, y_pos, '', transform=self.ax.transAxes, 
                              verticalalignment='top', fontfamily='monospace', fontsize=7,
                              bbox=dict(boxstyle='round', facecolor=self.colors[i], alpha=0.2))
            self.info_texts.append(text)
        
        self.ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=4)
        
    def update_plot(self):
        """Met à jour l'affichage"""
        current_time = time.time() - self.start_time
        
        for i, robot in enumerate(self.robots):
            # Position du robot
            for p in list(self.robot_triangles[i]):
                try:
                    p.remove()
                except Exception:
                    pass
            
            theta_math = robot.heading + math.pi/2
            patches_added = plot_bluerov(self.ax, x=robot.x, y=robot.y, theta=theta_math,
                                         size=0.3, color=self.colors[i], alpha=0.9, label=None, linewidth=0.8)
            self.robot_triangles[i] = patches_added
            
            # Trajectoire
            self.trajectory_lines[i].set_data(robot.trajectory_x, robot.trajectory_y)
            
            # Observation vers la cible
            bearing, range_m, approach_error, visible = self.last_observations[i]
            
            # Ligne de vision vers la cible
            if visible:
                self.vision_lines[i].set_data([robot.x, self.target_pos[0]], 
                                             [robot.y, self.target_pos[1]])
            else:
                self.vision_lines[i].set_data([], [])
            
            # Commandes pour affichage
            fwd_speed, lat_speed, ang_speed = self.last_commands[i]
            
            # Distance à la cible
            dist_to_target = np.linalg.norm([robot.x - self.target_pos[0], 
                                           robot.y - self.target_pos[1]])
            
            # État du contrôleur
            controller_state = self.controllers[i].state
            state_name = controller_state.upper()
            vis_status = "VIS" if visible else "NON"
            
            # Affichage
            info = f"""Robot {i+1} - t={current_time:.1f}s
Pos: ({robot.x:.2f}, {robot.y:.2f})m
Cap: {np.degrees(robot.heading):.0f}°
État: [{state_name}]

Target: {vis_status} brg={np.degrees(bearing):+.0f}° rng={range_m:.2f}m
Angle err: {np.degrees(approach_error):+.0f}°
Dist: {dist_to_target:.2f}m

[CMD] Fwd={fwd_speed:.2f} Lat={lat_speed:.2f} Yaw={ang_speed:.2f}"""
            
            self.info_texts[i].set_text(info)
        
        # Titre avec indication PAUSE
        title = f'BlueROV Oriented Approach - Cage angle: {np.degrees(self.cage_angle):.0f}°'
        if getattr(self, 'paused', False):
            title += '   [PAUSED]'
        self.ax.set_title(title)
        
    def step(self):
        """Un pas de simulation pour tous les robots"""
        robots_arrived = 0
        
        for i, robot in enumerate(self.robots):
            # Observation vers la cible (range, bearing, approach_angle_error)
            bearing, range_m, approach_error, visible = robot.get_target_observation(
                self.target_pos, self.cage_angle
            )
            self.last_observations[i] = (bearing, range_m, approach_error, visible)
            
            # Calcul des commandes de vitesse
            forward_speed, lateral_speed, angular_speed = self.controllers[i].control_step(
                bearing, range_m, approach_error, visible
            )
            
            # Sauvegarder les commandes pour l'affichage
            self.last_commands[i] = (forward_speed, lateral_speed, angular_speed)
            
            # Mouvement du robot
            robot.move(forward_speed, lateral_speed, angular_speed, self.dt)
            
            # Vérifier si le robot est arrivé
            dist_to_target = np.linalg.norm([robot.x - self.target_pos[0], 
                                           robot.y - self.target_pos[1]])
            if dist_to_target < 0.5:
                robots_arrived += 1
        
        # Temps réel
        elapsed_time = time.time() - self.start_time
        
        # Arrêt si tous les robots sont arrivés OU après 2min
        return robots_arrived == 4 or elapsed_time > 120
    
    def run_simulation(self):
        """Lance la simulation en temps réel"""
        def on_key(event):
            if event.key == ' ':
                self.paused = not self.paused

        self.fig.canvas.mpl_connect('key_press_event', on_key)

        def animate_func(frame):
            if not getattr(self, 'paused', False):
                finished = self.step()
            else:
                finished = False
            self.update_plot()
            
            if finished:
                elapsed_time = time.time() - self.start_time
                print(f"\nSimulation terminée après {elapsed_time:.1f}s")
                
                # Afficher les résultats finaux
                target_heading = self.cage_angle   # Heading que le robot doit avoir
                target_heading = math.atan2(math.sin(target_heading), math.cos(target_heading))
                
                for i, robot in enumerate(self.robots):
                    dist = np.linalg.norm([robot.x - self.target_pos[0], 
                                         robot.y - self.target_pos[1]])
                    heading_error = robot.heading - target_heading
                    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
                    
                    status = "ARRIVÉ" if dist < 0.5 else f"dist={dist:.2f}m"
                    print(f"Robot {i+1}: {status}, heading_err={np.degrees(heading_error):+.1f}°")
                
                return []
            return []
        
        anim = FuncAnimation(self.fig, animate_func, interval=50, blit=False)
        plt.show()
        return anim


if __name__ == "__main__":
    sim = MultiRobotSimulation()
    print("=" * 70)
    print("Simulation BlueROV - Oriented Approach (OrientedApproachController)")
    print("=" * 70)
    print("FONCTIONNEMENT:")
    print("  • Phase 1 (APPROACHING): Approche jusqu'à 1.5m de la cible")
    print("  • Phase 2 (ORBITING): Orbite autour de la cible pour s'aligner")
    print("  • Phase 3 (FINAL_APPROACH): Approche finale jusqu'à 0.5m")
    print()
    print(f"CAGE:")
    print(f"  • Position: {sim.target_pos}")
    print(f"  • Angle: {np.degrees(sim.cage_angle):.0f}° (flèche violette)")
    print(f"  • Robot doit arriver avec heading: {np.degrees(sim.cage_angle):.0f}°")
    print()
    print("Appuyez sur ESPACE pour mettre en pause")
    print("Appuyez sur Ctrl+C pour arrêter")
    print("=" * 70 + "\n")
    
    try:
        anim = sim.run_simulation()
    except KeyboardInterrupt:
        print("\nSimulation arrêtée par l'utilisateur")
