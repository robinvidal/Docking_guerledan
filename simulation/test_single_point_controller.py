#!/usr/bin/env python3
"""
Test du SinglePointController existant (control.py) avec simulation dynamique.

Ce script importe directement votre contrôleur et le teste dans un environnement
simulé pour valider son comportement avant déploiement sur le vrai BlueROV.

Usage:
    python test_single_point_controller.py
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
from control import SinglePointController, CageFollowController

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Wedge, Rectangle, Circle
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
    Convertit les vitesses (m/s, rad/s) en mouvement.
    """
    
    def __init__(self):
        # Paramètres physiques
        self.m = 13.5       # masse (kg)
        self.Iz = 0.37      # moment d'inertie en lacet
        
        # Coefficients hydrodynamiques (amortissement)
        self.Xu = -8.5
        self.Yv = -10.0
        self.Nr = -0.35
        self.Xuu = -22.0
        self.Yvv = -28.0
        self.Nrr = -1.9
        
        # Limites (comme dans control.py)
        self.max_forward_speed = 0.7  # m/s
        self.max_lateral_speed = 0.4  # m/s
        self.max_angular_speed = 0.7  # rad/s
        
        # Constante de temps des actionneurs
        self.T_control = 0.3
        
        # Vitesses actuelles (avec dynamique)
        self.vel_actual = np.array([0.0, 0.0, 0.0])  # [u, v, r]
    
    def step(self, state, vel_cmd, dt):
        """
        Intègre la dynamique d'un pas de temps.
        
        Args:
            state: [x, y, psi] position et cap
            vel_cmd: [forward_speed, lateral_speed, angular_speed] commandes de vitesse
            dt: pas de temps
            
        Returns:
            new_state: [x, y, psi] mis à jour
        """
        x, y, psi = state
        
        # Dynamique 1er ordre sur les vitesses (simule inertie)
        vel_dot = (vel_cmd - self.vel_actual) / self.T_control
        self.vel_actual = self.vel_actual + vel_dot * dt
        
        u, v, r = self.vel_actual  # surge, sway, yaw_rate
        
        # Cinématique: passage repère corps -> repère monde
        x_dot = u * np.cos(psi) - v * np.sin(psi)
        y_dot = u * np.sin(psi) + v * np.cos(psi)
        psi_dot = r
        
        # Intégration
        x_new = x + x_dot * dt
        y_new = y + y_dot * dt
        psi_new = ssa(psi + psi_dot * dt)
        
        return np.array([x_new, y_new, psi_new])


# =============================================================================
# MODÈLE SONAR
# =============================================================================

class SonarModel:
    """Modèle simplifié de sonar avec champ de vision conique."""
    
    def __init__(self):
        self.fov_deg = 120.0
        self.max_range = 15.0
        self.min_range = 0.3
        self.noise_range = 0.02  # m
        self.noise_bearing = np.deg2rad(0.5)
    
    def measure(self, robot_state, target_world):
        """
        Simule une mesure sonar vers un point.
        
        Returns:
            bearing: angle en repère corps (rad), 0 = devant, + = droite
            range_m: distance (m)
            visible: True si dans le FOV
        """
        x, y, psi = robot_state
        x_t, y_t = target_world
        
        # Vecteur vers la cible en repère monde
        dx_w = x_t - x
        dy_w = y_t - y
        
        # Distance
        range_true = np.hypot(dx_w, dy_w)
        
        # Transformation en repère corps
        # x_b = devant, y_b = droite (convention control.py)
        x_b = dx_w * np.cos(psi) + dy_w * np.sin(psi)
        y_b = -dx_w * np.sin(psi) + dy_w * np.cos(psi)
        
        # Bearing: 0 = devant, + = droite, - = gauche
        bearing = np.arctan2(y_b, x_b)
        
        # Vérification FOV
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


# =============================================================================
# SIMULATION DU SINGLEPOINT CONTROLLER
# =============================================================================

class SinglePointSimulation:
    """Test du SinglePointController avec visualisation."""
    
    def __init__(self, x0=0.0, y0=0.0, psi0=0.0, target=(5.0, 3.0)):
        # Composants
        self.dynamics = BlueROVDynamics()
        self.sonar = SonarModel()
        
        # ===== CONTRÔLEUR À TESTER =====
        self.controller = SinglePointController(stop_distance=0.5)
        
        # État: [x, y, psi]
        self.state = np.array([x0, y0, psi0])
        self.target = np.array(target)
        
        # Simulation
        self.dt = 0.02
        self.t = 0.0
        
        # Historique
        self.history = {
            't': [], 'x': [], 'y': [], 'psi': [],
            'u': [], 'v': [], 'r': [],
            'range': [], 'bearing': [],
            'forward_cmd': [], 'lateral_cmd': [], 'angular_cmd': [],
            'ctrl_state': []
        }
        
    def step(self):
        """Un pas de simulation."""
        # Mesure sonar (bearing, range, visible)
        bearing, range_m, visible = self.sonar.measure(self.state, self.target)
        
        # ===== APPEL AU CONTRÔLEUR À TESTER =====
        forward_speed, lateral_speed, angular_speed = self.controller.control_step(
            bearing, range_m, visible
        )
        # Calcul des PWM RC mappés comme en réalité (utilisé par le node ROV)
        yaw_pwm = int(1500 + angular_speed * 300)
        fwd_pwm = int(1500 + forward_speed * 400)
        lat_pwm = int(1500 + lateral_speed * 400)
        # Clip aux mêmes bornes que le node réel
        yaw_pwm = int(np.clip(yaw_pwm, 1100, 1900))
        fwd_pwm = int(np.clip(fwd_pwm, 1100, 1900))
        lat_pwm = int(np.clip(lat_pwm, 1100, 1900))
        print(f"[t={self.t:.2f}] RC  -> yaw: {yaw_pwm}, forward: {fwd_pwm}, lateral: {lat_pwm}")
        
        # Appliquer la dynamique
        vel_cmd = np.array([forward_speed, lateral_speed, angular_speed])
        self.state = self.dynamics.step(self.state, vel_cmd, self.dt)
        
        # Enregistrement
        self.history['t'].append(self.t)
        self.history['x'].append(self.state[0])
        self.history['y'].append(self.state[1])
        self.history['psi'].append(np.degrees(self.state[2]))
        self.history['u'].append(self.dynamics.vel_actual[0])
        self.history['v'].append(self.dynamics.vel_actual[1])
        self.history['r'].append(np.degrees(self.dynamics.vel_actual[2]))
        self.history['range'].append(range_m if visible else np.nan)
        self.history['bearing'].append(np.degrees(bearing) if visible else np.nan)
        self.history['forward_cmd'].append(forward_speed)
        self.history['lateral_cmd'].append(lateral_speed)
        self.history['angular_cmd'].append(angular_speed)
        self.history['ctrl_state'].append(self.controller.state)
        
        self.t += self.dt
        
        return bearing, range_m, visible


class InteractiveSimulation:
    """Interface graphique pour tester le contrôleur."""
    
    def __init__(self):
        self.sim = SinglePointSimulation(x0=0.0, y0=0.0, psi0=np.deg2rad(30), target=(6.0, 4.0))
        
        # Figure
        self.fig, self.axes = plt.subplots(2, 3, figsize=(16, 10))
        self.fig.suptitle('Test SinglePointController (control.py)', fontsize=14, fontweight='bold')
        
        # Axe trajectoire
        self.ax_traj = self.axes[0, 0]
        self.ax_traj.set_xlim(-2, 10)
        self.ax_traj.set_ylim(-2, 8)
        self.ax_traj.set_aspect('equal')
        self.ax_traj.grid(True, alpha=0.3)
        self.ax_traj.set_xlabel('X (m)')
        self.ax_traj.set_ylabel('Y (m)')
        self.ax_traj.set_title('Trajectoire')
        
        # ROV
        self.rov_body = Rectangle((-0.3, -0.2), 0.6, 0.4,
                                  facecolor='steelblue', edgecolor='navy', linewidth=2)
        self.ax_traj.add_patch(self.rov_body)
        
        # Cône sonar
        self.sonar_wedge = Wedge((0, 0), self.sim.sonar.max_range,
                                 -self.sim.sonar.fov_deg/2, self.sim.sonar.fov_deg/2,
                                 facecolor='cyan', alpha=0.12, edgecolor='cyan')
        self.ax_traj.add_patch(self.sonar_wedge)
        
        # Zone d'arrêt (stop_distance)
        self.stop_zone = Circle(self.sim.target, self.sim.controller.stop_distance,
                                facecolor='lightgreen', alpha=0.3, edgecolor='green',
                                linestyle='--', linewidth=2, label=f'Stop zone ({self.sim.controller.stop_distance}m)')
        self.ax_traj.add_patch(self.stop_zone)
        
        # Flèche direction
        self.arrow = self.ax_traj.arrow(0, 0, 0.5, 0, head_width=0.15,
                                        head_length=0.1, fc='red', ec='red', zorder=10)
        
        # Trajectoire et cible
        self.traj_line, = self.ax_traj.plot([], [], 'b-', alpha=0.5, linewidth=1.5)
        self.target_marker, = self.ax_traj.plot(self.sim.target[0], self.sim.target[1],
                                                'r*', markersize=20, label='Cible')
        self.ax_traj.legend(loc='upper left')
        
        # Sous-graphiques
        self.setup_subplots()
        
        # Animation
        self.anim = FuncAnimation(self.fig, self.update, interval=20, blit=False)
        
        # Interactions
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
    def setup_subplots(self):
        """Configure les sous-graphiques."""
        # Range
        self.ax_range = self.axes[0, 1]
        self.ax_range.set_xlabel('Temps (s)')
        self.ax_range.set_ylabel('Range (m)')
        self.ax_range.set_title('Distance à la cible')
        self.ax_range.grid(True, alpha=0.3)
        self.line_range, = self.ax_range.plot([], [], 'g-', linewidth=2)
        self.ax_range.axhline(y=self.sim.controller.stop_distance, color='r', 
                              linestyle='--', label=f'Stop ({self.sim.controller.stop_distance}m)')
        self.ax_range.legend()
        
        # Bearing
        self.ax_bearing = self.axes[0, 2]
        self.ax_bearing.set_xlabel('Temps (s)')
        self.ax_bearing.set_ylabel('Bearing (°)')
        self.ax_bearing.set_title('Angle vers cible (repère corps)')
        self.ax_bearing.grid(True, alpha=0.3)
        self.line_bearing, = self.ax_bearing.plot([], [], 'm-', linewidth=2)
        self.ax_bearing.axhline(y=0, color='g', linestyle='--', alpha=0.5, label='Aligné')
        self.ax_bearing.legend()
        
        # Commandes de vitesse
        self.ax_cmd = self.axes[1, 0]
        self.ax_cmd.set_xlabel('Temps (s)')
        self.ax_cmd.set_ylabel('Vitesse commandée')
        self.ax_cmd.set_title('Commandes du contrôleur (m/s, rad/s)')
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
                                           fontsize=10, verticalalignment='top', family='monospace')
        
    def update(self, frame):
        """Mise à jour de l'animation."""
        bearing, range_m, visible = self.sim.step()
        
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
        
        self.line_range.set_data(t_data, self.sim.history['range'])
        self.ax_range.relim()
        self.ax_range.autoscale_view()
        
        self.line_bearing.set_data(t_data, self.sim.history['bearing'])
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
        dist_to_target = np.hypot(x - self.sim.target[0], y - self.sim.target[1])
        
        info = f"""
SINGLEPOINT CONTROLLER TEST
════════════════════════════

ÉTAT DU ROBOT
─────────────────────
t = {self.sim.t:.2f} s
x = {x:.2f} m
y = {y:.2f} m
ψ = {np.degrees(psi):.1f}°

Distance réelle: {dist_to_target:.2f} m

MESURE SONAR
─────────────────────
Visible: {visible}
Range: {range_m:.2f} m
Bearing: {np.degrees(bearing):.1f}°

CONTRÔLEUR
─────────────────────
État: {ctrl_state}
Gains:
  kp_heading: {self.sim.controller.kp_heading}
  kp_forward: {self.sim.controller.kp_forward}
  kp_lateral: {self.sim.controller.kp_lateral}

COMMANDES (m/s, rad/s)
─────────────────────
Forward: {self.sim.history['forward_cmd'][-1]:.3f}
Lateral: {self.sim.history['lateral_cmd'][-1]:.3f}
Angular: {self.sim.history['angular_cmd'][-1]:.3f}

CONTRÔLES
─────────────────────
R: Reset
Click: Nouvelle cible
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
        elif event.key == 'g':
            self.modify_gains()
    
    def on_click(self, event):
        if event.inaxes == self.ax_traj and event.xdata and event.ydata:
            new_target = (event.xdata, event.ydata)
            print(f"Nouvelle cible: {new_target}")
            self.sim.target = np.array(new_target)
            self.target_marker.set_data([event.xdata], [event.ydata])
            self.stop_zone.center = new_target
            self.reset(keep_target=True)
    
    def reset(self, keep_target=False):
        target = tuple(self.sim.target) if keep_target else (6.0, 4.0)
        self.sim = SinglePointSimulation(x0=0.0, y0=0.0, psi0=np.deg2rad(30), target=target)
        self.traj_line.set_data([], [])
        if not keep_target:
            self.target_marker.set_data([target[0]], [target[1]])
            self.stop_zone.center = target
    
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
            self.reset(keep_target=True)
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
    print("TEST DU SINGLEPOINT CONTROLLER (control.py)")
    print("="*60)
    print("\nContrôleur importé depuis:")
    print(f"  {CONTROL_PATH}/control.py")
    print("\nParamètres du contrôleur:")
    ctrl = SinglePointController()
    print(f"  stop_distance = {ctrl.stop_distance} m")
    print(f"  kp_heading = {ctrl.kp_heading}")
    print(f"  kp_forward = {ctrl.kp_forward}")
    print(f"  kp_lateral = {ctrl.kp_lateral}")
    print(f"  max_forward_speed = {ctrl.max_forward_speed} m/s")
    print(f"  max_lateral_speed = {ctrl.max_lateral_speed} m/s")
    print(f"  max_angular_speed = {ctrl.max_angular_speed} rad/s")
    print("\nContrôles:")
    print("  - Clic: nouvelle cible")
    print("  - R: reset")
    print("  - G: modifier les gains")
    print("  - ESC: quitter")
    print("\nLancement...")
    
    sim = InteractiveSimulation()
    sim.run()


if __name__ == '__main__':
    main()
