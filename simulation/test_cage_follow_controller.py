#!/usr/bin/env python3
"""
Test du CageFollowController existant (control.py) avec simulation dynamique.

Ce contrôleur gère l'approche vers une cage définie par 2 bouées.
Il utilise 2 contrôleurs indépendants:
- Heading Controller: centre le robot entre les deux bouées
- Movement Controller: navigue vers un point 1m devant la cage

Usage:
    python test_cage_follow_controller.py
"""

import sys
import os

# Ajouter le chemin vers control.py
CONTROL_PATH = os.path.join(
    os.path.dirname(__file__), 
    '..', 'ros2_bluerov', 'src', 'bluerov_basics', 
    'bluerov_control0', 'bluerov_control0'
)
sys.path.insert(0, CONTROL_PATH)

# Import du contrôleur à tester
from control import CageFollowController

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Wedge, Rectangle, Circle, FancyArrowPatch
import matplotlib.transforms as transforms


# =============================================================================
# MODÈLE DYNAMIQUE SIMPLIFIÉ DU BLUEROV
# =============================================================================

def ssa(angle):
    """Smallest signed angle - ramène l'angle dans [-pi, pi]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class BlueROVDynamics:
    """
    Modèle dynamique du BlueROV - 3 DOF (surge, sway, yaw)
    """
    
    def __init__(self):
        self.m = 13.5
        self.Iz = 0.37
        self.Xu = -8.5
        self.Yv = -10.0
        self.Nr = -0.35
        self.T_control = 0.3
        self.vel_actual = np.array([0.0, 0.0, 0.0])
    
    def step(self, state, vel_cmd, dt):
        x, y, psi = state
        
        # Dynamique 1er ordre sur les vitesses
        vel_dot = (vel_cmd - self.vel_actual) / self.T_control
        self.vel_actual = self.vel_actual + vel_dot * dt
        
        u, v, r = self.vel_actual
        
        # Cinématique
        x_dot = u * np.cos(psi) - v * np.sin(psi)
        y_dot = u * np.sin(psi) + v * np.cos(psi)
        psi_dot = r
        
        x_new = x + x_dot * dt
        y_new = y + y_dot * dt
        psi_new = ssa(psi + psi_dot * dt)
        
        return np.array([x_new, y_new, psi_new])


# =============================================================================
# MODÈLE SONAR POUR 2 BOUÉES
# =============================================================================

class SonarModel:
    """Modèle de sonar pour détecter les 2 bouées de la cage."""
    
    def __init__(self):
        self.fov_deg = 120.0
        self.max_range = 15.0
        self.min_range = 0.3
        self.noise_range = 0.02
        self.noise_bearing = np.deg2rad(0.5)
    
    def measure_point(self, robot_state, target_world):
        """Mesure un point unique."""
        x, y, psi = robot_state
        x_t, y_t = target_world
        
        dx_w = x_t - x
        dy_w = y_t - y
        range_true = np.hypot(dx_w, dy_w)
        
        # Repère corps: x_b = devant, y_b = droite
        x_b = dx_w * np.cos(psi) + dy_w * np.sin(psi)
        y_b = -dx_w * np.sin(psi) + dy_w * np.cos(psi)
        
        bearing = np.arctan2(y_b, x_b)
        
        half_fov = np.deg2rad(self.fov_deg / 2)
        in_fov = np.abs(bearing) <= half_fov
        in_range = self.min_range <= range_true <= self.max_range
        
        visible = in_fov and in_range
        
        if visible:
            range_m = range_true + np.random.randn() * self.noise_range
            bearing_rad = bearing + np.random.randn() * self.noise_bearing
            return bearing_rad, range_m, True
        else:
            return 0.0, 0.0, False
    
    def measure_cage(self, robot_state, buoy1_world, buoy2_world):
        """
        Mesure les 2 bouées de la cage.
        
        Returns:
            obs1: (bearing, distance, visible) pour bouée 1
            obs2: (bearing, distance, visible) pour bouée 2
        """
        obs1 = self.measure_point(robot_state, buoy1_world)
        obs2 = self.measure_point(robot_state, buoy2_world)
        return obs1, obs2


# =============================================================================
# SIMULATION DU CAGEFOLLOW CONTROLLER
# =============================================================================

class CageFollowSimulation:
    """Test du CageFollowController avec 2 bouées."""
    
    def __init__(self, x0=0.0, y0=0.0, psi0=0.0, buoy1=(5.0, 2.0), buoy2=(5.0, 4.0)):
        self.dynamics = BlueROVDynamics()
        self.sonar = SonarModel()
        
        # ===== CONTRÔLEUR À TESTER =====
        self.controller = CageFollowController(target_distance_ahead=1.0)
        
        # État: [x, y, psi]
        self.state = np.array([x0, y0, psi0])
        
        # Position des 2 bouées (définissent la cage)
        self.buoy1 = np.array(buoy1)
        self.buoy2 = np.array(buoy2)
        
        # Calcul du point cible (1m devant la cage)
        self._compute_target()
        
        # Simulation
        self.dt = 0.02
        self.t = 0.0
        
        # Historique
        self.history = {
            't': [], 'x': [], 'y': [], 'psi': [],
            'u': [], 'v': [], 'r': [],
            'range1': [], 'bearing1': [], 'visible1': [],
            'range2': [], 'bearing2': [], 'visible2': [],
            'forward_cmd': [], 'lateral_cmd': [], 'angular_cmd': [],
            'ctrl_state': [], 'distance_to_target': []
        }
    
    def _compute_target(self):
        """Calcule le point cible 1m devant la cage."""
        # Centre de la cage
        cage_center = (self.buoy1 + self.buoy2) / 2
        
        # Vecteur entre les bouées
        bouee_vec = self.buoy2 - self.buoy1
        bouee_length = np.linalg.norm(bouee_vec)
        
        if bouee_length > 0:
            # Perpendiculaire normalisée
            perp = np.array([bouee_vec[1], -bouee_vec[0]]) / bouee_length
        else:
            perp = np.array([0, 1])
        
        # Point cible = centre + 1m * perpendiculaire
        self.target = cage_center + self.controller.target_distance_ahead * perp
        
    def step(self):
        """Un pas de simulation."""
        # Mesure sonar des 2 bouées
        obs1, obs2 = self.sonar.measure_cage(self.state, self.buoy1, self.buoy2)
        bearing1, range1, visible1 = obs1
        bearing2, range2, visible2 = obs2
        
        # ===== APPEL AU CONTRÔLEUR À TESTER =====
        forward_speed, lateral_speed, angular_speed = self.controller.control_step(obs1, obs2)
        
        # Appliquer la dynamique
        vel_cmd = np.array([forward_speed, lateral_speed, angular_speed])
        self.state = self.dynamics.step(self.state, vel_cmd, self.dt)
        
        # Distance au point cible
        dist_to_target = np.hypot(self.state[0] - self.target[0], 
                                   self.state[1] - self.target[1])
        
        # Enregistrement
        self.history['t'].append(self.t)
        self.history['x'].append(self.state[0])
        self.history['y'].append(self.state[1])
        self.history['psi'].append(np.degrees(self.state[2]))
        self.history['u'].append(self.dynamics.vel_actual[0])
        self.history['v'].append(self.dynamics.vel_actual[1])
        self.history['r'].append(np.degrees(self.dynamics.vel_actual[2]))
        self.history['range1'].append(range1 if visible1 else np.nan)
        self.history['bearing1'].append(np.degrees(bearing1) if visible1 else np.nan)
        self.history['visible1'].append(visible1)
        self.history['range2'].append(range2 if visible2 else np.nan)
        self.history['bearing2'].append(np.degrees(bearing2) if visible2 else np.nan)
        self.history['visible2'].append(visible2)
        self.history['forward_cmd'].append(forward_speed)
        self.history['lateral_cmd'].append(lateral_speed)
        self.history['angular_cmd'].append(angular_speed)
        self.history['ctrl_state'].append(self.controller.state)
        self.history['distance_to_target'].append(dist_to_target)
        
        self.t += self.dt
        
        return obs1, obs2


class InteractiveSimulation:
    """Interface graphique pour tester le CageFollowController."""
    
    def __init__(self):
        # Configuration initiale
        self.sim = CageFollowSimulation(
            x0=0.0, y0=3.0, psi0=np.deg2rad(50),
            buoy1=(6.0, 2.0), buoy2=(6.0, 4.0)
        )
        
        # Figure
        self.fig, self.axes = plt.subplots(2, 3, figsize=(16, 10))
        self.fig.suptitle('Test CageFollowController (control.py) - 2 Bouées', fontsize=14, fontweight='bold')
        
        # Axe trajectoire
        self.ax_traj = self.axes[0, 0]
        self.ax_traj.set_xlim(-2, 10)
        self.ax_traj.set_ylim(-1, 7)
        self.ax_traj.set_aspect('equal')
        self.ax_traj.grid(True, alpha=0.3)
        self.ax_traj.set_xlabel('X (m)')
        self.ax_traj.set_ylabel('Y (m)')
        self.ax_traj.set_title('Trajectoire & Cage')
        
        # ROV
        self.rov_body = Rectangle((-0.3, -0.2), 0.6, 0.4,
                                  facecolor='steelblue', edgecolor='navy', linewidth=2)
        self.ax_traj.add_patch(self.rov_body)
        
        # Cône sonar
        self.sonar_wedge = Wedge((0, 0), self.sim.sonar.max_range,
                                 -self.sim.sonar.fov_deg/2, self.sim.sonar.fov_deg/2,
                                 facecolor='cyan', alpha=0.1, edgecolor='cyan')
        self.ax_traj.add_patch(self.sonar_wedge)
        
        # Bouées
        self.buoy1_marker = Circle(self.sim.buoy1, 0.15, facecolor='orange', 
                                   edgecolor='darkorange', linewidth=2, label='Bouée 1')
        self.buoy2_marker = Circle(self.sim.buoy2, 0.15, facecolor='orange',
                                   edgecolor='darkorange', linewidth=2, label='Bouée 2')
        self.ax_traj.add_patch(self.buoy1_marker)
        self.ax_traj.add_patch(self.buoy2_marker)
        
        # Ligne entre les bouées (entrée de la cage)
        self.cage_line, = self.ax_traj.plot(
            [self.sim.buoy1[0], self.sim.buoy2[0]],
            [self.sim.buoy1[1], self.sim.buoy2[1]],
            'orange', linewidth=3, linestyle='--', label='Entrée cage'
        )
        
        # Point cible
        self.target_marker, = self.ax_traj.plot(
            self.sim.target[0], self.sim.target[1],
            'g*', markersize=20, label='Cible (1m devant)'
        )
        
        # Zone d'arrêt
        self.stop_zone = Circle(self.sim.target, self.sim.controller.stop_distance,
                                facecolor='lightgreen', alpha=0.3, edgecolor='green',
                                linestyle='--', linewidth=2)
        self.ax_traj.add_patch(self.stop_zone)
        
        # Flèche direction
        self.arrow = self.ax_traj.arrow(0, 0, 0.5, 0, head_width=0.15,
                                        head_length=0.1, fc='red', ec='red', zorder=10)
        
        # Trajectoire
        self.traj_line, = self.ax_traj.plot([], [], 'b-', alpha=0.5, linewidth=1.5)
        self.ax_traj.legend(loc='upper left', fontsize=8)
        
        # Sous-graphiques
        self.setup_subplots()
        
        # Animation
        self.anim = FuncAnimation(self.fig, self.update, interval=20, blit=False)
        
        # Interactions
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        self.click_mode = None  # Pour placer les bouées
        
    def setup_subplots(self):
        """Configure les sous-graphiques."""
        # Distance à la cible
        self.ax_dist = self.axes[0, 1]
        self.ax_dist.set_xlabel('Temps (s)')
        self.ax_dist.set_ylabel('Distance (m)')
        self.ax_dist.set_title('Distance à la cible')
        self.ax_dist.grid(True, alpha=0.3)
        self.line_dist, = self.ax_dist.plot([], [], 'g-', linewidth=2)
        self.ax_dist.axhline(y=self.sim.controller.stop_distance, color='r', 
                             linestyle='--', label=f'Stop ({self.sim.controller.stop_distance}m)')
        self.ax_dist.legend()
        
        # Bearings des 2 bouées
        self.ax_bearing = self.axes[0, 2]
        self.ax_bearing.set_xlabel('Temps (s)')
        self.ax_bearing.set_ylabel('Bearing (°)')
        self.ax_bearing.set_title('Bearings des bouées')
        self.ax_bearing.grid(True, alpha=0.3)
        self.line_b1, = self.ax_bearing.plot([], [], 'r-', label='Bouée 1', linewidth=2)
        self.line_b2, = self.ax_bearing.plot([], [], 'b-', label='Bouée 2', linewidth=2)
        self.ax_bearing.axhline(y=0, color='g', linestyle='--', alpha=0.5, label='Centre')
        self.ax_bearing.legend()
        
        # Commandes
        self.ax_cmd = self.axes[1, 0]
        self.ax_cmd.set_xlabel('Temps (s)')
        self.ax_cmd.set_ylabel('Vitesse commandée')
        self.ax_cmd.set_title('Commandes du contrôleur')
        self.ax_cmd.grid(True, alpha=0.3)
        self.line_fwd, = self.ax_cmd.plot([], [], 'b-', label='forward (m/s)')
        self.line_lat, = self.ax_cmd.plot([], [], 'g-', label='lateral (m/s)')
        self.line_ang, = self.ax_cmd.plot([], [], 'r-', label='angular (rad/s)')
        self.ax_cmd.legend()
        
        # Vitesses réelles
        self.ax_vel = self.axes[1, 1]
        self.ax_vel.set_xlabel('Temps (s)')
        self.ax_vel.set_ylabel('Vitesse réelle')
        self.ax_vel.set_title('Vitesses du robot')
        self.ax_vel.grid(True, alpha=0.3)
        self.line_u, = self.ax_vel.plot([], [], 'b-', label='u (m/s)')
        self.line_v, = self.ax_vel.plot([], [], 'g-', label='v (m/s)')
        self.line_r, = self.ax_vel.plot([], [], 'r-', label='r (°/s)')
        self.ax_vel.legend()
        
        # Info texte
        self.ax_info = self.axes[1, 2]
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(0.05, 0.95, '', transform=self.ax_info.transAxes,
                                           fontsize=9, verticalalignment='top', family='monospace')
        
    def update(self, frame):
        """Mise à jour de l'animation."""
        obs1, obs2 = self.sim.step()
        bearing1, range1, visible1 = obs1
        bearing2, range2, visible2 = obs2
        
        x, y, psi = self.sim.state
        
        # ROV
        transform = transforms.Affine2D().rotate(psi).translate(x, y)
        self.rov_body.set_transform(transform + self.ax_traj.transData)
        
        # Sonar
        wedge_transform = transforms.Affine2D().rotate(psi).translate(x, y)
        self.sonar_wedge.set_transform(wedge_transform + self.ax_traj.transData)
        
        # Flèche
        self.arrow.remove()
        dx = 0.6 * np.cos(psi)
        dy = 0.6 * np.sin(psi)
        self.arrow = self.ax_traj.arrow(x, y, dx, dy, head_width=0.15,
                                        head_length=0.1, fc='red', ec='red', zorder=10)
        
        # Trajectoire
        self.traj_line.set_data(self.sim.history['x'], self.sim.history['y'])
        
        # Graphiques
        t_data = self.sim.history['t']
        
        self.line_dist.set_data(t_data, self.sim.history['distance_to_target'])
        self.ax_dist.relim()
        self.ax_dist.autoscale_view()
        
        self.line_b1.set_data(t_data, self.sim.history['bearing1'])
        self.line_b2.set_data(t_data, self.sim.history['bearing2'])
        self.ax_bearing.relim()
        self.ax_bearing.autoscale_view()
        
        self.line_fwd.set_data(t_data, self.sim.history['forward_cmd'])
        self.line_lat.set_data(t_data, self.sim.history['lateral_cmd'])
        self.line_ang.set_data(t_data, self.sim.history['angular_cmd'])
        self.ax_cmd.relim()
        self.ax_cmd.autoscale_view()
        
        self.line_u.set_data(t_data, self.sim.history['u'])
        self.line_v.set_data(t_data, self.sim.history['v'])
        self.line_r.set_data(t_data, self.sim.history['r'])
        self.ax_vel.relim()
        self.ax_vel.autoscale_view()
        
        # Info
        ctrl_state = self.sim.controller.state.upper()
        dist_to_target = self.sim.history['distance_to_target'][-1] if self.sim.history['distance_to_target'] else 0
        
        # Calcul du bearing moyen (centre)
        if visible1 and visible2:
            center_bearing = (bearing1 + bearing2) / 2
            center_bearing_deg = np.degrees(center_bearing)
        else:
            center_bearing_deg = float('nan')
        
        info = f"""
CAGEFOLLOW CONTROLLER TEST
══════════════════════════════

ÉTAT DU ROBOT
─────────────────────────────
t = {self.sim.t:.2f} s
x = {x:.2f} m
y = {y:.2f} m
ψ = {np.degrees(psi):.1f}°

Distance à la cible: {dist_to_target:.2f} m

MESURES SONAR
─────────────────────────────
Bouée 1: {'✓' if visible1 else '✗'} r={range1:.2f}m b={np.degrees(bearing1):.1f}°
Bouée 2: {'✓' if visible2 else '✗'} r={range2:.2f}m b={np.degrees(bearing2):.1f}°
Centre bearing: {center_bearing_deg:.1f}°

CONTRÔLEUR
─────────────────────────────
État: {ctrl_state}
Gains:
  kp_heading: {self.sim.controller.kp_heading}
  kp_forward: {self.sim.controller.kp_forward}
  kp_lateral: {self.sim.controller.kp_lateral}

COMMANDES
─────────────────────────────
Forward: {self.sim.history['forward_cmd'][-1]:.3f} m/s
Lateral: {self.sim.history['lateral_cmd'][-1]:.3f} m/s
Angular: {self.sim.history['angular_cmd'][-1]:.3f} rad/s

CONTRÔLES
─────────────────────────────
R: Reset
1: Placer bouée 1
2: Placer bouée 2
G: Modifier gains
ESC: Quitter
"""
        self.info_text.set_text(info)
        
        return []
    
    def on_key(self, event):
        if event.key == 'r':
            self.reset()
        elif event.key == 'escape':
            plt.close()
        elif event.key == '1':
            self.click_mode = 'buoy1'
            print("Cliquez pour placer la bouée 1...")
        elif event.key == '2':
            self.click_mode = 'buoy2'
            print("Cliquez pour placer la bouée 2...")
        elif event.key == 'g':
            self.modify_gains()
    
    def on_click(self, event):
        if event.inaxes == self.ax_traj and event.xdata and event.ydata:
            if self.click_mode == 'buoy1':
                self.sim.buoy1 = np.array([event.xdata, event.ydata])
                print(f"Bouée 1 placée à ({event.xdata:.2f}, {event.ydata:.2f})")
                self.click_mode = None
                self.update_cage()
            elif self.click_mode == 'buoy2':
                self.sim.buoy2 = np.array([event.xdata, event.ydata])
                print(f"Bouée 2 placée à ({event.xdata:.2f}, {event.ydata:.2f})")
                self.click_mode = None
                self.update_cage()
    
    def update_cage(self):
        """Met à jour l'affichage de la cage et recalcule la cible."""
        self.sim._compute_target()
        self.buoy1_marker.center = self.sim.buoy1
        self.buoy2_marker.center = self.sim.buoy2
        self.cage_line.set_data(
            [self.sim.buoy1[0], self.sim.buoy2[0]],
            [self.sim.buoy1[1], self.sim.buoy2[1]]
        )
        self.target_marker.set_data([self.sim.target[0]], [self.sim.target[1]])
        self.stop_zone.center = self.sim.target
        self.reset(keep_cage=True)
    
    def reset(self, keep_cage=False):
        if keep_cage:
            buoy1 = tuple(self.sim.buoy1)
            buoy2 = tuple(self.sim.buoy2)
        else:
            buoy1 = (6.0, 2.0)
            buoy2 = (6.0, 4.0)
        
        self.sim = CageFollowSimulation(
            x0=0.0, y0=3.0, psi0=np.deg2rad(0),
            buoy1=buoy1, buoy2=buoy2
        )
        self.traj_line.set_data([], [])
        
        if not keep_cage:
            self.buoy1_marker.center = self.sim.buoy1
            self.buoy2_marker.center = self.sim.buoy2
            self.cage_line.set_data(
                [self.sim.buoy1[0], self.sim.buoy2[0]],
                [self.sim.buoy1[1], self.sim.buoy2[1]]
            )
            self.target_marker.set_data([self.sim.target[0]], [self.sim.target[1]])
            self.stop_zone.center = self.sim.target
    
    def modify_gains(self):
        """Permet de modifier les gains interactivement."""
        print("\n=== MODIFICATION DES GAINS ===")
        print(f"Gains actuels:")
        print(f"  kp_heading = {self.sim.controller.kp_heading}")
        print(f"  kp_forward = {self.sim.controller.kp_forward}")
        print(f"  kp_lateral = {self.sim.controller.kp_lateral}")
        try:
            kp_h = input("Nouveau kp_heading (Entrée pour garder): ")
            if kp_h: self.sim.controller.kp_heading = float(kp_h)
            kp_f = input("Nouveau kp_forward (Entrée pour garder): ")
            if kp_f: self.sim.controller.kp_forward = float(kp_f)
            kp_l = input("Nouveau kp_lateral (Entrée pour garder): ")
            if kp_l: self.sim.controller.kp_lateral = float(kp_l)
            print("Gains mis à jour!")
            self.reset(keep_cage=True)
        except ValueError:
            print("Valeur invalide, gains non modifiés.")
    
    def run(self):
        plt.tight_layout()
        plt.show()


# =============================================================================
# MAIN
# =============================================================================

def main():
    print("="*60)
    print("TEST DU CAGEFOLLOW CONTROLLER (control.py)")
    print("="*60)
    print("\nContrôleur importé depuis:")
    print(f"  {CONTROL_PATH}/control.py")
    print("\nParamètres du contrôleur:")
    ctrl = CageFollowController()
    print(f"  target_distance_ahead = {ctrl.target_distance_ahead} m")
    print(f"  stop_distance = {ctrl.stop_distance} m")
    print(f"  kp_heading = {ctrl.kp_heading}")
    print(f"  kp_forward = {ctrl.kp_forward}")
    print(f"  kp_lateral = {ctrl.kp_lateral}")
    print("\nContrôles:")
    print("  - 1 puis clic: placer bouée 1")
    print("  - 2 puis clic: placer bouée 2")
    print("  - R: reset")
    print("  - G: modifier les gains")
    print("  - ESC: quitter")
    print("\nLancement...")
    
    sim = InteractiveSimulation()
    sim.run()


if __name__ == '__main__':
    main()
