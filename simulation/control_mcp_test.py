"""
Simulation BlueROV - MPC ROBUSTE (Hybride MPC + PID de secours)
Objectif : Ne jamais bloquer, fluidité maximale.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import time
from scipy.optimize import minimize
from plot_bluerov import plot_bluerov

# --- Fonctions Utilitaires ---
def normalize_angle(angle):
    """Ramène un angle entre -pi et pi"""
    return math.atan2(math.sin(angle), math.cos(angle))

class BlueROV:
    """Classe Robot (Inchangée)"""
    def __init__(self, x=0, y=0, heading=0):
        self.x = x
        self.y = y
        self.heading = heading
        self.max_speed = 0.6          # Un peu plus de puissance
        self.max_angular_speed = 1.0  # Rotation plus rapide
        self.trajectory_x = [x]
        self.trajectory_y = [y]
        
    def get_buoy_observations(self, buoy1_pos, buoy2_pos):
        observations = []
        fov_half_angle = np.pi / 2.0  # FOV large pour éviter de perdre la cible
        for buoy_pos in [buoy1_pos, buoy2_pos]:
            dx = buoy_pos[0] - self.x
            dy = buoy_pos[1] - self.y
            distance = np.sqrt(dx**2 + dy**2)
            angle_to_buoy = math.atan2(dy, dx)
            nav_angle = angle_to_buoy - np.pi/2
            nav_angle = math.atan2(math.sin(nav_angle), math.cos(nav_angle))
            bearing = normalize_angle(nav_angle - self.heading)
            visible = abs(bearing) <= fov_half_angle
            observations.append((bearing, distance, visible))
        return observations
    
    def move(self, u, v, r, dt):
        u = np.clip(u, -self.max_speed, self.max_speed)
        v = np.clip(v, -self.max_speed, self.max_speed)
        r = np.clip(r, -self.max_angular_speed, self.max_angular_speed)
        
        theta = self.heading + math.pi/2
        dx = u * math.cos(theta) - v * math.sin(theta)
        dy = u * math.sin(theta) + v * math.cos(theta)
        
        self.x += dx * dt
        self.y += dy * dt
        self.heading = normalize_angle(self.heading + r * dt)
        self.trajectory_x.append(self.x)
        self.trajectory_y.append(self.y)

class RobustMPC:
    """
    MPC avec "Ceinture et Bretelles" :
    Si l'optimisation échoue, on utilise un contrôleur P simple (Fallback).
    """
    def __init__(self, dt=0.1):
        self.dt = dt
        self.horizon = 10     # Réduit à 10 pour être plus rapide et stable
        self.last_u = np.zeros(3)
        self.target_dist_ahead = 1.0
        
        # Poids (Tuning)
        self.W_pos = 10.0
        self.W_head = 5.0
        self.W_energy = 0.1  # Faible pour ne pas "étouffer" le mouvement
        self.W_smooth = 1.0
        
        self.status = "INIT"

    def simple_pid_fallback(self, target_x, target_y, target_psi):
        """
        Un contrôleur proportionnel simple qui sert de roue de secours
        si le MPC échoue.
        """
        # Gains simples
        Kp_pos = 1.0
        Kp_head = 1.5
        
        # Erreur cap
        r = Kp_head * normalize_angle(target_psi) # target_psi est relatif ici (0 = aligné)
        
        # Erreur position (dans le repère robot)
        u = Kp_pos * target_x
        v = Kp_pos * target_y
        
        # Saturation simple
        u = np.clip(u, -0.5, 0.5)
        v = np.clip(v, -0.5, 0.5)
        r = np.clip(r, -0.8, 0.8)
        
        return u, v, r

    def kinematic_model(self, state, cmd):
        x, y, psi = state
        u, v, r = cmd
        x_next = x + (u * np.cos(psi) - v * np.sin(psi)) * self.dt
        y_next = y + (u * np.sin(psi) + v * np.cos(psi)) * self.dt
        psi_next = psi + r * self.dt
        return np.array([x_next, y_next, psi_next])

    def cost_function(self, U_flat, target_pos, target_psi):
        U = U_flat.reshape((self.horizon, 3))
        cost = 0.0
        state = np.array([0.0, 0.0, 0.0]) # Départ robot
        prev = self.last_u
        
        for k in range(self.horizon):
            cmd = U[k]
            state = self.kinematic_model(state, cmd)
            
            # Coûts
            dist_sq = (state[0] - target_pos[0])**2 + (state[1] - target_pos[1])**2
            head_err = (1.0 - np.cos(state[2] - target_psi))
            
            cost += self.W_pos * dist_sq
            cost += self.W_head * head_err
            cost += self.W_energy * np.sum(cmd**2)
            cost += self.W_smooth * np.sum((cmd - prev)**2)
            prev = cmd
            
        return cost

    def control_step(self, robot, obs1, obs2):
        # 1. Sécurité : Si on ne voit rien, on tourne
        if not (obs1[2] and obs2[2]):
            self.status = "SEARCH"
            self.last_u = np.zeros(3)
            # Tourner vers la dernière bouée vue ou à droite par défaut
            if obs1[2]: return 0, 0, -0.5
            return 0, 0, 0.5
            
        # 2. Calcul Cible (Méthode simplifiée "Centroid" + Décalage)
        b1_bearing, b1_dist, _ = obs1
        b2_bearing, b2_dist, _ = obs2
        
        # Conversion Cartésienne Locale
        bx1, by1 = b1_dist * math.cos(b1_bearing), b1_dist * math.sin(b1_bearing)
        bx2, by2 = b2_dist * math.cos(b2_bearing), b2_dist * math.sin(b2_bearing)
        
        # Centre de la cage
        cx, cy = (bx1 + bx2)/2, (by1 + by2)/2
        
        # Angle vers le centre (votre intuition)
        angle_to_center = math.atan2(cy, cx)
        
        # Vecteur Cage (pour orientation)
        vx, vy = bx2 - bx1, by2 - by1
        cage_angle = math.atan2(vy, vx)
        # La normale est à -90° ou +90° du vecteur cage
        normal_angle = cage_angle - np.pi/2
        
        # CORRECTION DU SENS : On veut que la normale pointe vers le robot (ou l'inverse selon convention)
        # Simplification : On veut que le robot s'aligne pour que son cap (0) = normal_angle
        # Donc target_psi = normal_angle
        # On s'assure que c'est l'angle "rentrant"
        if abs(normalize_angle(normal_angle - angle_to_center)) > np.pi/2:
            normal_angle += np.pi # On inverse si ça pointe à l'opposé
        
        target_psi = normalize_angle(normal_angle)
        
        # Position Cible : On projette 1m devant le centre selon cet angle normal
        # Attention: target_psi est l'angle d'orientation DESIRE du robot.
        # Le vecteur direction est (cos(target_psi), sin(target_psi))
        # Comme on est dans le repère local, on veut reculer de 1m par rapport au centre "vers nous"
        # Ou avancer de 1m "derrière" le centre ?
        # L'objectif : "1m devant les bouées".
        tx = cx - self.target_dist_ahead * math.cos(target_psi)
        ty = cy - self.target_dist_ahead * math.sin(target_psi)
        
        # Arrêt
        if math.hypot(tx, ty) < 0.15:
            self.status = "ARRIVED"
            return 0.0, 0.0, 0.0

        self.status = "MPC"

        # 3. Optimisation
        u0 = np.tile(self.last_u, self.horizon) # Warm start avec la dernière commande
        bnds = [(-0.5, 0.5), (-0.5, 0.5), (-1.0, 1.0)] * self.horizon
        
        # Essai MPC
        try:
            res = minimize(self.cost_function, u0, args=([tx, ty], target_psi),
                           method='SLSQP', bounds=bnds, 
                           options={'ftol': 1e-2, 'maxiter': 10, 'disp': False}) # ftol relaché pour vitesse
            
            if res.success:
                opt = res.x.reshape((self.horizon, 3))[0]
                self.last_u = opt
                return opt[0], opt[1], opt[2]
            else:
                raise ValueError("Solver failed")
                
        except Exception:
            # 4. FALLBACK (Sauvetage !)
            # Si le MPC échoue, on utilise le PID simple calculé plus haut
            self.status = "FALLBACK (PID)"
            # On reset le guess du MPC pour ne pas rester bloqué sur une mauvaise solution
            self.last_u = np.zeros(3) 
            return self.simple_pid_fallback(tx, ty, target_psi)


class MultiRobotSimulation:
    def __init__(self):
        self.buoy1_pos = np.array([-0.4, 5.0])
        self.buoy2_pos = np.array([0.4, 5.0])
        mid_x = (self.buoy1_pos[0] + self.buoy2_pos[0]) / 2
        mid_y = (self.buoy1_pos[1] + self.buoy2_pos[1]) / 2
        self.target_pos = np.array([mid_x, mid_y - 1.0])
        
        start_positions = [
            (-2.5, 2.0, 0.0), (2.5, 2.0, np.pi),
            (0.1, -1.0, np.pi/2), (-1.5, 4.0, -np.pi/4)
        ]
        
        self.robots = []
        self.controllers = []
        self.colors = ['blue', 'red', 'green', 'orange']
        
        for pos in start_positions:
            self.robots.append(BlueROV(pos[0], pos[1], pos[2]))
            self.controllers.append(RobustMPC(dt=0.1)) # MPC dt
            
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.setup_plot()
        self.paused = False
        self.dt_sim = 0.05

    def setup_plot(self):
        self.ax.set_xlim(-4, 4)
        self.ax.set_ylim(-2, 7)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title("MPC Robuste (avec Fallback PID anti-blocage)")
        
        self.ax.add_patch(plt.Circle(self.buoy1_pos, 0.1, color='darkred'))
        self.ax.add_patch(plt.Circle(self.buoy2_pos, 0.1, color='darkred'))
        self.ax.add_patch(plt.Circle(self.target_pos, 0.15, color='gold', fill=False, lw=2))
        
        self.robot_patches = [[] for _ in range(4)]
        self.traj_lines = [self.ax.plot([], [], c=c, lw=1)[0] for c in self.colors]
        self.info_texts = []
        pos_txt = [(0.02, 0.95), (0.75, 0.95), (0.02, 0.10), (0.75, 0.10)]
        for i in range(4):
            self.info_texts.append(self.ax.text(pos_txt[i][0], pos_txt[i][1], '', transform=self.ax.transAxes, 
                                              fontsize=8, bbox=dict(facecolor='white', alpha=0.7)))

    def update(self, frame):
        if self.paused: return
        
        for i, robot in enumerate(self.robots):
            obs1, obs2 = robot.get_buoy_observations(self.buoy1_pos, self.buoy2_pos)
            u, v, r = self.controllers[i].control_step(robot, obs1, obs2)
            robot.move(u, v, r, self.dt_sim)
            
            # Affichage
            for p in self.robot_patches[i]: p.remove()
            self.robot_patches[i] = plot_bluerov(self.ax, robot.x, robot.y, robot.heading + np.pi/2, size=0.4, color=self.colors[i])
            self.traj_lines[i].set_data(robot.trajectory_x, robot.trajectory_y)
            
            status = self.controllers[i].status
            dist = np.linalg.norm([robot.x - self.target_pos[0], robot.y - self.target_pos[1]])
            self.info_texts[i].set_text(f"R{i+1}: {status}\nDist: {dist:.2f}m")

    def run(self):
        def on_key(event):
            if event.key == ' ': self.paused = not self.paused
        self.fig.canvas.mpl_connect('key_press_event', on_key)
        anim = FuncAnimation(self.fig, self.update, interval=50, blit=False)
        plt.show()

if __name__ == "__main__":
    sim = MultiRobotSimulation()
    sim.run()