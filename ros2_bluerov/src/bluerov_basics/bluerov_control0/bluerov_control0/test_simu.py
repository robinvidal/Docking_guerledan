"""
Simulation BlueROV - Version basique
Simulation simple d'un robot BlueROV suivant une cible
avec contrôle proportionnel intégré (sans utiliser control.py)
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
        self.max_speed_x = 0.5  # m/s
        self.max_speed_y = 0.5  # m/s
        self.max_angular_speed = 0.5  # rad/s
        
        # Historique pour le tracé
        self.trajectory_x = [x]
        self.trajectory_y = [y]
        
    def get_target_observation(self, target_pos):
        """
        Calcule le range et bearing vers la cible
        
        Args:
            target_pos: Position de la cible (x, y)
        
        Returns:
            tuple: (bearing, range_m, visible)
                   bearing en radians [-pi, pi], 0 = devant le robot
                   range_m en mètres
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
        
        # Vérifier si la cible est dans le champ de vision (±45°)
        visible = abs(bearing) <= fov_half_angle
        
        return bearing, range_m, visible
    
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


class SimpleController:
    """Contrôleur proportionnel simple intégré"""
    
    def __init__(self, stop_distance=1.0):
        """
        Args:
            stop_distance: Distance d'arrêt à la cible (m)
        """
        self.stop_distance = stop_distance
        
        # État du contrôleur
        self.state = "idle"  # "idle", "tracking", "arrived"
        
        # Gains proportionnels
        self.kp_heading = 0.5
        self.kp_forward = 0.8
        self.kp_lateral = 0.5
        
        # Limites de vitesse
        self.max_forward_speed = 0.5
        self.max_lateral_speed = 0.5
        self.max_angular_speed = 0.5
    
    def control_step(self, bearing, range_m, visible):
        """
        Calcule les commandes de vitesse pour rejoindre la cible
        
        Args:
            bearing: angle vers la cible (rad), 0 = devant
            range_m: distance à la cible (m)
            visible: True si cible visible
            
        Returns:
            (forward_speed, lateral_speed, angular_speed): vitesses en m/s et rad/s
        """
        # Cible non visible
        if not visible:
            self.state = "idle"
            return 0.0, 0.0, 0.0
        
        # Arrivé
        if range_m < self.stop_distance:
            self.state = "arrived"
            return 0.0, 0.0, 0.0
        
        # Tracking
        self.state = "tracking"
        
        # Contrôle proportionnel
        angular_speed = self.kp_heading * bearing
        
        error_x = range_m * math.cos(bearing) - self.stop_distance
        forward_speed = self.kp_forward * error_x
        
        error_y = range_m * math.sin(bearing)
        lateral_speed = self.kp_lateral * error_y
        
        # Limitation
        forward_speed = np.clip(forward_speed, -self.max_forward_speed, self.max_forward_speed)
        lateral_speed = np.clip(lateral_speed, -self.max_lateral_speed, self.max_lateral_speed)
        angular_speed = np.clip(angular_speed, -self.max_angular_speed, self.max_angular_speed)
        
        return forward_speed, lateral_speed, angular_speed


class Simulation:
    """Classe principale de simulation avec un robot"""
    
    def __init__(self):
        # Cible
        self.target_pos = np.array([0.0, 5.0])
        
        # Robot
        self.robot = BlueROV(x=0.0, y=0.0, heading=0)
        self.controller = SimpleController(stop_distance=0.5)
        
        # Paramètres de simulation
        self.dt = 0.05  # 20 Hz
        self.start_time = time.time()
        
        # Dernières commandes/observations
        self.last_commands = (0.0, 0.0, 0.0)
        self.last_observation = (0, 0, False)
        
        # Animation
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.setup_plot()
        self.paused = False
        
    def setup_plot(self):
        """Configure l'affichage"""
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-1, 7)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Simulation BlueROV - Basic')
        
        # Cible
        self.target_circle = plt.Circle(self.target_pos, 0.2, color='gold', 
                                        edgecolor='black', linewidth=2, label='Cible')
        self.ax.add_patch(self.target_circle)
        
        # Cercle stop
        self.stop_circle = plt.Circle(self.target_pos, 0.5, color='green', 
                                      fill=False, linestyle='--', alpha=0.5, label='Stop (0.5m)')
        self.ax.add_patch(self.stop_circle)
        
        # Robot
        self.robot_patches = []
        self.trajectory_line, = self.ax.plot([], [], 'b-', alpha=0.3, linewidth=1)
        self.vision_line, = self.ax.plot([], [], 'b--', alpha=0.6, linewidth=2)
        
        # Info text
        self.info_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes, 
                                      verticalalignment='top', fontfamily='monospace', fontsize=9,
                                      bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))
        
        self.ax.legend(loc='upper right')
        
    def update_plot(self):
        """Met à jour l'affichage"""
        current_time = time.time() - self.start_time
        
        # Robot
        for p in list(self.robot_patches):
            try:
                p.remove()
            except Exception:
                pass
        
        theta_math = self.robot.heading + math.pi/2
        self.robot_patches = plot_bluerov(self.ax, x=self.robot.x, y=self.robot.y, 
                                          theta=theta_math, size=0.3, color='blue', 
                                          alpha=0.9, label=None, linewidth=0.8)
        
        # Trajectoire
        self.trajectory_line.set_data(self.robot.trajectory_x, self.robot.trajectory_y)
        
        # Vision
        bearing, range_m, visible = self.last_observation
        if visible:
            self.vision_line.set_data([self.robot.x, self.target_pos[0]], 
                                     [self.robot.y, self.target_pos[1]])
        else:
            self.vision_line.set_data([], [])
        
        # Commandes
        fwd, lat, yaw = self.last_commands
        dist = np.linalg.norm([self.robot.x - self.target_pos[0], 
                              self.robot.y - self.target_pos[1]])
        
        state = self.controller.state.upper()
        vis = "VIS" if visible else "NON"
        
        info = f"""t={current_time:.1f}s
Pos: ({self.robot.x:.2f}, {self.robot.y:.2f})m
Cap: {np.degrees(self.robot.heading):.0f}°
État: [{state}]

Target: {vis} brg={np.degrees(bearing):+.0f}° rng={range_m:.2f}m
Dist: {dist:.2f}m

[CMD] Fwd={fwd:.2f} Lat={lat:.2f} Yaw={yaw:.2f}"""
        
        self.info_text.set_text(info)
        
        title = 'BlueROV - Basic Simulation'
        if self.paused:
            title += '   [PAUSED]'
        self.ax.set_title(title)
        
    def step(self):
        """Un pas de simulation"""
        bearing, range_m, visible = self.robot.get_target_observation(self.target_pos)
        self.last_observation = (bearing, range_m, visible)
        
        fwd, lat, yaw = self.controller.control_step(bearing, range_m, visible)
        self.last_commands = (fwd, lat, yaw)
        
        self.robot.move(fwd, lat, yaw, self.dt)
        
        dist = np.linalg.norm([self.robot.x - self.target_pos[0], 
                              self.robot.y - self.target_pos[1]])
        elapsed = time.time() - self.start_time
        
        return dist < 0.5 or elapsed > 60
    
    def run_simulation(self):
        """Lance la simulation"""
        def on_key(event):
            if event.key == ' ':
                self.paused = not self.paused

        self.fig.canvas.mpl_connect('key_press_event', on_key)

        def animate_func(frame):
            if not self.paused:
                finished = self.step()
            else:
                finished = False
            self.update_plot()
            
            if finished:
                elapsed = time.time() - self.start_time
                dist = np.linalg.norm([self.robot.x - self.target_pos[0], 
                                      self.robot.y - self.target_pos[1]])
                status = "ARRIVÉ" if dist < 0.5 else f"dist={dist:.2f}m"
                print(f"\nSimulation terminée après {elapsed:.1f}s - {status}")
                return []
            return []
        
        anim = FuncAnimation(self.fig, animate_func, interval=50, blit=False)
        plt.show()
        return anim


if __name__ == "__main__":
    sim = Simulation()
    print("=" * 50)
    print("Simulation BlueROV - Basic")
    print("=" * 50)
    print("Contrôle proportionnel simple intégré")
    print(f"Cible: {sim.target_pos}")
    print("Stop distance: 0.5m")
    print()
    print("ESPACE pour pause, Ctrl+C pour arrêter")
    print("=" * 50 + "\n")
    
    try:
        anim = sim.run_simulation()
    except KeyboardInterrupt:
        print("\nSimulation arrêtée")
