#!/usr/bin/env python3
"""
BlueROV Simulator - 3 DOF at constant depth
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Wedge

def ssa(angle):
    """Smallest signed angle"""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

def attitudeEuler(eta, nu, sampleTime):
    """Integrate position/attitude"""
    psi = eta[5]
    u, v, r = nu[0], nu[1], nu[2]
    
    x_dot = u * np.cos(psi) - v * np.sin(psi)
    y_dot = u * np.sin(psi) + v * np.cos(psi)
    psi_dot = r
    
    eta_new = np.copy(eta)
    eta_new[0] += x_dot * sampleTime
    eta_new[1] += y_dot * sampleTime
    eta_new[5] += psi_dot * sampleTime
    
    return eta_new

class BlueROV:
    """BlueROV vehicle model - 3 DOF at constant depth (simplified)"""
    
    def __init__(self, v_current=0.0, beta_current=0.0):
        # Physical parameters
        self.m = 13.5
        self.Iz = 0.37
        
        # Hydrodynamic coefficients
        self.Xu = -8.5
        self.Yv = -10
        self.Nr = -0.35
        self.Xuu = -22
        self.Yvv = -28
        self.Nrr = -1.9
        
        # Control parameters (max forces/moments)
        self.max_force_surge = 100.0  # N
        self.max_force_sway = 100.0   # N
        self.max_moment_yaw = 15.0    # Nm
        
        # Time constant for force dynamics (1st order lag)
        self.T_control = 0.25
        
        # Current
        self.v_current = v_current
        self.beta_current = beta_current * np.pi / 180.0
        self.u_c = v_current * np.cos(self.beta_current)
        self.v_c = v_current * np.sin(self.beta_current)
        
        # States
        self.nu = np.array([0.0, 0.0, 0.0])  # [u, v, r]
        
        # Actual forces (with dynamics)
        self.tau_actual = np.array([0.0, 0.0, 0.0])  # [X, Y, N]
        
        # Control mode
        self.controlMode = 'manualInput'  # or 'autonomous'
        
        # Autonomous control parameters
        self.target_body = np.array([0.0, 0.0])  # Target in body frame [x_b, y_b]
        self.Kp_pos = 50.0      # Proportional gain for position
        self.Kp_heading = 5.0   # Proportional gain for heading
        self.Kd_heading = 2.0   # Derivative gain for heading
        self.heading_deadzone = np.deg2rad(5.0)  # Don't correct small heading errors

        # Sonar parameters
        self.sonar_fov_deg   = 120.0   # ouverture totale
        self.sonar_max_range = 15.0    # m
        self.sonar_min_range = 0.     # m
        self.sonar_sigma_xy  = 0.00 
    def controlDynamics(self, tau_actual, tau_control, sampleTime):
        """First-order control dynamics"""
        tau_dot = (tau_control - tau_actual) / self.T_control
        return tau_actual + tau_dot * sampleTime
    
    def dynamics(self, eta, nu, tau_actual, tau_control, sampleTime):
        """ROV dynamics - 3 DOF (surge, sway, yaw)"""
        # Update control forces with dynamics
        tau_actual = self.controlDynamics(tau_actual, tau_control, sampleTime)
        
        # Relative velocity to current
        psi = eta[5]
        u_r = nu[0] - (self.u_c * np.cos(psi) + self.v_c * np.sin(psi))
        v_r = nu[1] - (-self.u_c * np.sin(psi) + self.v_c * np.cos(psi))
        r = nu[2]
        
        # Hydrodynamic forces
        X = self.Xu * u_r + self.Xuu * u_r * abs(u_r)
        Y = self.Yv * v_r + self.Yvv * v_r * abs(v_r)
        N = self.Nr * r + self.Nrr * r * abs(r)
        
        # Equations of motion
        u_dot = (tau_actual[0] + X) / self.m + nu[2] * nu[1]
        v_dot = (tau_actual[1] + Y) / self.m - nu[2] * nu[0]
        r_dot = (tau_actual[2] + N) / self.Iz
        
        nu_new = np.array([
            nu[0] + u_dot * sampleTime,
            nu[1] + v_dot * sampleTime,
            nu[2] + r_dot * sampleTime
        ])
        
        return nu_new, tau_actual
    
    def manualInput(self, cmd_surge, cmd_sway, cmd_yaw):
        """
        Convert command inputs (%) to forces/moments
        cmd_surge, cmd_sway, cmd_yaw: [-100, 100] %
        Returns: [tau_X, tau_Y, tau_N] in N, N, Nm
        """
        tau_X = (cmd_surge / 100.0) * self.max_force_surge
        tau_Y = (cmd_sway / 100.0) * self.max_force_sway
        tau_N = (cmd_yaw / 100.0) * self.max_moment_yaw
        
        return np.array([tau_X, tau_Y, tau_N])
    
    def autonomousControl(self, target_body, r):
        """
        Autonomous controller to reach a target point
        
        Args:
            target_body: [x_b, y_b] target position in body frame (like sonar data)
            r: current yaw rate
            
        Returns:
            [tau_X, tau_Y, tau_N] control forces/moments
            
        Control strategy:
        1. If target is behind (x_b < 0), rotate to face it
        2. Once facing target, move towards it (surge + sway)
        3. Stop when close enough
        """
        x_b, y_b = target_body
        
        # Distance and angle to target in body frame
        distance = np.sqrt(x_b**2 + y_b**2)
        angle_to_target = np.arctan2(y_b, x_b)  # angle in body frame
        
        # Threshold for "reached"
        distance_threshold = 0.5  # meters
        
        if distance < distance_threshold:
            # Target reached, stop
            return np.array([0.0, 0.0, 0.0])
        
        # --- Heading control (align with target) ---
        # If target is not in front (large angle), rotate first
        if abs(angle_to_target) > self.heading_deadzone:
            # PD controller for heading
            tau_N = self.Kp_heading * angle_to_target - self.Kd_heading * r
            tau_N = np.clip(tau_N, -self.max_moment_yaw, self.max_moment_yaw)
            
            # Slow down translation when rotating
            if abs(angle_to_target) > np.deg2rad(30):
                # Large angle: only rotate
                tau_X = 0.0
                tau_Y = 0.0
            else:
                # Small angle: rotate + move slowly
                tau_X = 0.3 * self.Kp_pos * x_b
                tau_Y = 0.3 * self.Kp_pos * y_b
        else:
            # Well aligned, move towards target
            tau_X = self.Kp_pos * x_b
            tau_Y = self.Kp_pos * y_b
            tau_N = 0.0
        
        # Saturate forces
        tau_X = np.clip(tau_X, -self.max_force_surge, self.max_force_surge)
        tau_Y = np.clip(tau_Y, -self.max_force_sway, self.max_force_sway)
        
        return np.array([tau_X, tau_Y, tau_N])


class BlueROVSimulator:
    """Main simulator class"""
    
    def __init__(self, X0=0.0, Y0=0.0, theta_0=0.0):
        self.sampleTime = 0.02
        self.DOF = 6
        
        # Current
        self.v_current = 0.0
        self.beta_current = 0.0
        
        # Vehicle
        self.vehicle = BlueROV(self.v_current, self.beta_current)
        
        # States
        self.eta = np.array([X0, Y0, 0.0, 0.0, 0.0, theta_0])
        self.nu = self.vehicle.nu
        self.tau_actual = self.vehicle.tau_actual
        
        # Commands
        self.cmd_surge = 0.0    # forward/backward
        self.cmd_sway = 0.0     # lateral
        self.cmd_yaw = 0.0      # rotation
        
        # Target for autonomous mode (in world frame)
        self.target_world = np.array([5.0, 3.0])  # Example target
        
        self.t = 0.0
        self.i = 0
        
        self.history_x = []
        self.history_y = []
        # Search/Track state for sonar
        self.search_yaw_sign = +1.0     # sens de rotation pour chercher
        self.search_turn_cmd = 0.1      # fraction de moment max pour la recherche
        self.has_lock = False           # en vue sonar ?

        

    def world_to_body(self, target_world, eta):
        """
        Convert target from world frame to body frame (simulates sonar measurement)
        
        Args:
            target_world: [x_w, y_w] target in world frame
            eta: vehicle state [x, y, z, phi, theta, psi]
            
        Returns:
            [x_b, y_b] target in body frame
        """
        x_w, y_w = target_world
        x, y, psi = eta[0], eta[1], eta[5]
        
        # Vector from vehicle to target in world frame
        dx_w = x_w - x
        dy_w = y_w - y
        
        # Rotation matrix from world to body
        x_b = dx_w * np.cos(psi) + dy_w * np.sin(psi)
        y_b = -dx_w * np.sin(psi) + dy_w * np.cos(psi)
        
        return np.array([x_b, y_b])
    
    def one_step(self, eta, nu, tau_control, tau_actual):
        """One simulation step"""
        signals = np.concatenate([eta, nu, tau_control, tau_actual])
        
        nu, tau_actual = self.vehicle.dynamics(eta, nu, tau_actual, tau_control, self.sampleTime)
        eta = attitudeEuler(eta, nu, self.sampleTime)
        
        return eta, nu, tau_actual, signals
    
    def timer_callback(self):
        """Main loop"""
        self.t = self.i * self.sampleTime
        
        if self.vehicle.controlMode == 'manualInput':
            tau_control = self.vehicle.manualInput(self.cmd_surge, self.cmd_sway, self.cmd_yaw)
        elif self.vehicle.controlMode == 'autonomous':
            # Sonar "sense": cible mesurée en repère corps si dans le cône
            z_b, seen = self.sonar_measure(self.target_world, self.eta)
            if seen:
                self.has_lock = True
                self.vehicle.target_body = z_b
                # utilise ton contrôleur autonome existant (TRACK)
                tau_control = self.vehicle.autonomousControl(z_b, self.nu[2])
            else:
                self.has_lock = False
                # SEARCH: tourner sur place jusqu'à voir la cible (pas de translation)
                tau_X = 0.0
                tau_Y = 0.0
                tau_N = self.search_yaw_sign * self.search_turn_cmd * self.vehicle.max_moment_yaw
                tau_control = np.array([tau_X, tau_Y, tau_N])
        else:
            tau_control = np.array([0.0, 0.0, 0.0])
        
        self.eta, self.nu, self.tau_actual, signal = self.one_step(
            self.eta, self.nu, tau_control, self.tau_actual
        )
        
        self.eta[3] = ssa(self.eta[3])
        self.eta[4] = ssa(self.eta[4])
        self.eta[5] = ssa(self.eta[5])
        
        self.history_x.append(self.eta[0])
        self.history_y.append(self.eta[1])
        
        self.i += 1
        
        return signal
    
    def angle_in_fov(self, ang_rad, fov_deg):
        """Test si |angle| <= fov/2 (angles déjà en repère corps)"""
        half = np.deg2rad(fov_deg * 0.5)
        return abs(ang_rad) <= half

    def sonar_measure(self, target_world, eta):
        """
        Renvoie (z_body, detected) où z_body = [x_b, y_b] si détecté,
        sinon (None, False). Simule un sonar cônique 2D (plan horizontal).
        """
        # cible en repère corps (déjà ce que tu utilises pour l'autonomie)
        z_b = self.world_to_body(target_world, eta)    # [x_b, y_b]
        x_b, y_b = z_b
        rng = np.hypot(x_b, y_b)
        brg = np.arctan2(y_b, x_b)  # angle en repère corps

        in_range = (self.vehicle.sonar_min_range <= rng <= self.vehicle.sonar_max_range)
        in_fov   = self.angle_in_fov(brg, self.vehicle.sonar_fov_deg)

        if in_range and in_fov:
            if self.vehicle.sonar_sigma_xy > 0.0:
                z_b = z_b + np.random.randn(2) * self.vehicle.sonar_sigma_xy
            return z_b, True
        return None, False


class InteractiveSimulator:
    """Interactive matplotlib simulator"""
    
    def __init__(self):
        self.sim = BlueROVSimulator(X0=0.0, Y0=0.0, theta_0=0.0)
        
        # Setup figure
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(14, 7))
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('key_release_event', self.on_key_release)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        # Main plot
        self.ax1.set_xlim(-10, 10)
        self.ax1.set_ylim(-10, 10)
        self.ax1.set_aspect('equal')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.set_title('BlueROV Simulator ')
        
        self.ax2.axis('off')
        
        # ROV body
        self.rov_body = patches.Rectangle((-0.3, -0.2), 0.6, 0.4,
                                          facecolor='steelblue',
                                          edgecolor='navy', linewidth=2)
        self.ax1.add_patch(self.rov_body)
        
        # Direction arrow
        self.arrow = self.ax1.arrow(0, 0, 0.5, 0, head_width=0.15,
                                     head_length=0.15, fc='red', ec='red')
        
        # Trajectory
        self.trajectory, = self.ax1.plot([], [], 'b-', alpha=0.3, linewidth=1)
        
        # Target marker (for autonomous mode)
        self.target_marker = self.ax1.plot([], [], 'ro', markersize=15, label='Target')[0]
        
        # Force vectors (for visualization)
        self.force_surge = None
        self.force_sway = None
        
        # Sonar FOV wedge (affichage)
        self.sonar_wedge = Wedge(center=(0,0), r=self.sim.vehicle.sonar_max_range,
                                theta1=-self.sim.vehicle.sonar_fov_deg/2.0,
                                theta2=+self.sim.vehicle.sonar_fov_deg/2.0,
                                facecolor='cyan', alpha=0.12, edgecolor='cyan', linewidth=1.0)
        self.ax1.add_patch(self.sonar_wedge)

        # Animation
        self.anim = FuncAnimation(self.fig, self.update, interval=20, blit=False)
    
    def update(self, frame):
        self.sim.timer_callback()
        
        x, y = self.sim.eta[0], self.sim.eta[1]
        psi = self.sim.eta[5]
        # Mettre à jour le cône sonar (orienté par psi et centré sur le ROV)
        wedge_transform = plt.matplotlib.transforms.Affine2D().rotate_around(0,0, psi).translate(x, y)
        self.sonar_wedge.set_transform(wedge_transform + self.ax1.transData)

        # Update ROV body
        transform = plt.matplotlib.transforms.Affine2D().rotate_around(0, 0, psi).translate(x, y)
        self.rov_body.set_transform(transform + self.ax1.transData)
        
        # Update arrow
        self.arrow.remove()
        dx = 0.5 * np.cos(psi)
        dy = 0.5 * np.sin(psi)
        self.arrow = self.ax1.arrow(x, y, dx, dy, head_width=0.15,
                                     head_length=0.15, fc='red', ec='red', zorder=10)
        
        # Remove old force vectors
        if self.force_surge is not None:
            try:
                self.force_surge.remove()
            except:
                pass
        if self.force_sway is not None:
            try:
                self.force_sway.remove()
            except:
                pass
        
        self.force_surge = None
        self.force_sway = None
        
        # Draw force vectors (in body frame)
        scale = 0.01
        tau_X = self.sim.tau_actual[0] * scale
        tau_Y = self.sim.tau_actual[1] * scale
        
        # Transform to world frame
        fx_world = tau_X * np.cos(psi) - tau_Y * np.sin(psi)
        fy_world = tau_X * np.sin(psi) + tau_Y * np.cos(psi)
        
        if abs(fx_world) > 0.01 or abs(fy_world) > 0.01:
            self.force_surge = self.ax1.arrow(x, y, fx_world, fy_world, 
                                             head_width=0.1, head_length=0.1,
                                             fc='green', ec='green', alpha=0.6, zorder=5)
        
        # Update trajectory
        self.trajectory.set_data(self.sim.history_x, self.sim.history_y)
        
        # Update target marker (only in autonomous mode)
        if self.sim.vehicle.controlMode == 'autonomous':
            self.target_marker.set_data([self.sim.target_world[0]], [self.sim.target_world[1]])
        else:
            self.target_marker.set_data([], [])
        
        # Update info
        self.ax2.clear()
        self.ax2.axis('off')
        
        # Build info text depending on mode
        if self.sim.vehicle.controlMode == 'autonomous':
            target_body = self.sim.vehicle.target_body
            distance = np.sqrt(target_body[0]**2 + target_body[1]**2)
            angle = np.degrees(np.arctan2(target_body[1], target_body[0]))
            
            mode_info = f"""
MODE: AUTONOMOUS
────────────────────────
Target (world): ({self.sim.target_world[0]:.1f}, {self.sim.target_world[1]:.1f})
Target (body):  ({target_body[0]:.2f}, {target_body[1]:.2f})
Distance: {distance:.2f} m
Angle:    {angle:.1f}°
"""
        else:
            mode_info = f"""
MODE: MANUAL
────────────────────────
Surge (fwd/back) = {self.sim.cmd_surge:.0f}%
Sway  (lateral)  = {self.sim.cmd_sway:.0f}%
Yaw   (rotation) = {self.sim.cmd_yaw:.0f}%
"""
        
        info_text = f"""
BLUEROV STATE:
────────────────────────
t = {self.sim.t:.2f} s
x = {self.sim.eta[0]:.2f} m
y = {self.sim.eta[1]:.2f} m
ψ = {np.degrees(self.sim.eta[5]):.1f}°

u = {self.sim.nu[0]:.2f} m/s (surge)
v = {self.sim.nu[1]:.2f} m/s (sway)
r = {np.degrees(self.sim.nu[2]):.1f}°/s (yaw rate)
{mode_info}
FORCES/MOMENTS:
────────────────────────
τ_X (surge) = {self.sim.tau_actual[0]:.1f} N
τ_Y (sway)  = {self.sim.tau_actual[1]:.1f} N
τ_N (yaw)   = {self.sim.tau_actual[2]:.2f} Nm


CONTROLS:
────────────────────────
↑     : Forward (surge +)
↓     : Backward (surge -)
←     : Left (sway +)
→     : Right (sway -)
A     : Rotate left (yaw +)
E     : Rotate right (yaw -)
SPACE : Stop
M     : Manual/Autonomous
T     : Set target 
R     : Reset
ESC   : Quit
        """
        
        self.ax2.text(0.05, 0.95, info_text, transform=self.ax2.transAxes,
                     fontsize=9, verticalalignment='top', family='monospace')
        
        return [self.rov_body, self.arrow, self.trajectory]
    
    def on_key_press(self, event):
        step = 100
        
        if event.key in ['up', 'z']:
            self.sim.cmd_surge = min(100, self.sim.cmd_surge + step)
        elif event.key in ['down', 's']:
            self.sim.cmd_surge = max(-100, self.sim.cmd_surge - step)
        elif event.key in ['left', 'q']:
            self.sim.cmd_sway = min(100, self.sim.cmd_sway + step)
        elif event.key in ['right', 'd']:
            self.sim.cmd_sway = max(-100, self.sim.cmd_sway - step)
        elif event.key == 'a':
            self.sim.cmd_yaw = min(100, self.sim.cmd_yaw + step)
        elif event.key == 'e':
            self.sim.cmd_yaw = max(-100, self.sim.cmd_yaw - step)
        elif event.key == ' ':
            self.sim.cmd_surge = 0.0
            self.sim.cmd_sway = 0.0
            self.sim.cmd_yaw = 0.0
        elif event.key == 'm':
            # Toggle between manual and autonomous
            if self.sim.vehicle.controlMode == 'manualInput':
                self.sim.vehicle.controlMode = 'autonomous'
                print(f"→ Mode AUTONOMOUS - Target: {self.sim.target_world}")
            else:
                self.sim.vehicle.controlMode = 'manualInput'
                print("→ Mode MANUAL")
        elif event.key == 't':
            print("Click on the plot to set a new target...")
        elif event.key == 'r':
            self.reset()
        elif event.key == 'escape':
            plt.close()
    
    def on_key_release(self, event):
        if event.key in ['up', 'down', 'z', 's']:
            self.sim.cmd_surge = 0.0
        elif event.key in ['left', 'right', 'q', 'd']:
            self.sim.cmd_sway = 0.0
        elif event.key in ['a', 'e']:
            self.sim.cmd_yaw = 0.0
    
    def on_click(self, event):
        """Set target on click"""
        if event.inaxes == self.ax1 and event.xdata and event.ydata:
            self.sim.target_world = np.array([event.xdata, event.ydata])
            print(f"New target set: ({event.xdata:.2f}, {event.ydata:.2f})")
    
    def reset(self):
        self.sim = BlueROVSimulator(X0=0.0, Y0=0.0, theta_0=0.0)
        self.trajectory.set_data([], [])
    
    def run(self):
        plt.show()


def main():  
    sim = InteractiveSimulator()
    sim.run()


if __name__ == '__main__':
    main()