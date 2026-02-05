# Copyright 2024, Christophe VIEL
# 
# Redistribution and use in source and binary forms, with or without modification, are permitted # provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


# Note : by default, angles provid by sensor are in deg. Variable with a "rad" are expressed in radian

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

import time

import os

#import parameters as param
from scipy.spatial.transform import Rotation 

import numpy as np
import math

from mavros import command as mavros_command
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool, Float32MultiArray
from docking_msgs.msg import TrackedObject

# Import du contrôleur de mission (version simple: un seul point)
from .control import SinglePointController, OrientedApproachController

import importlib

import time

#################################################################

def light_control(self,light_modif_value):


    adresse_ip0 = self.adresse_ip

    pwm_light0 = self.pwm_light+light_modif_value
    if (pwm_light0 > 1900):
        pwm_light0 = 1000.0
    if (pwm_light0 < 1000):
        pwm_light0 = 1900.0

    self.pwm_light = pwm_light0

    print(f"Light PWM value set to: {self.pwm_light}")
    
    # # on ecrit la valeur que l'on veut dans le fichier test.txt   TODO: retirer ce passage pour version publique
    msg_lum = str(pwm_light0) 
    password = 'companion' 
    file3 = '/home/pi/pwm_light.txt'
    cmd = 'sshpass -p '+password+' ssh '+ adresse_ip0 +' "echo '+ msg_lum +' > '+file3+'"'
    os.system(cmd) 

    # for classic bluerov
    self.commands[8] = self.pwm_light
    print(f"Light command set to: {self.commands[8]}")


def clip(val, min_val, max_val):
    if val <= min_val:
        return min_val
    elif val >= max_val:
        return max_val
    return val

def angle_diff_deg(target_deg, current_deg):
    # return smallest signed difference target - current in range [-180,180)
    d = (target_deg - current_deg + 180.0) % 360.0 - 180.0
    return d

class ROV(Node):

    def __init__(self):
    
        super().__init__('Control_ROV')


        self.dt = 1/10
        
        self.declare_parameter('ROV_name', '')
        ROV_name = self.get_parameter('ROV_name').get_parameter_value().string_value
        self.declare_parameter('ID', 1.0)
        ID = self.get_parameter('ID').get_parameter_value().double_value
        self.declare_parameter('ip_adress_light', "192.168.2")
        ip_adress_light = self.get_parameter('ip_adress_light').get_parameter_value().string_value


        # identification
        self.ROV_name = ROV_name #rospy.get_param('~ROV_name',"inky")
        self.ID = str(ID)
        self.ID0 = ID
        self.IDnum = str(self.ID0+1)

        if not(self.ROV_name == ''):
            self.ns = "/"+ self.ROV_name # TODO
        else:
            self.ns = ""

        
        # pour lumiere (configuration maison)
        self.adresse_ip_short = ip_adress_light
        self.adresse_ip = 'pi@' + self.adresse_ip_short  +'.2'
        self.pwm_light = 1000.0
            

        # Robot modes
        self.armed = False

        # Robot parameter
        self.depth = 0.0 # Depth
        self.heading = 0.0 # Heading
        self._have_compass = False #booléen pour savoir si on a une boussole (surtout utile au demarrage)

        # Heading-hold state: X => enable to a fixed target (120°); LH+X => disable
        self.heading_hold = False
        self.desired_heading = 200.0
        # Internal flag: True while operator is moving yaw stick; used to capture last heading on release
        self._heading_move_active = False
        # Heading PID state and gains (tunable)
        self._heading_integral = 0.0
        self._heading_error_prev = 0.0
        self.Kp_heading = 2.0
        self.Ki_heading = 0.01
        self.Kd_heading = 0.5
        self._heading_integral_min = -100.0
        self._heading_integral_max = 100.0
        self.integrator_tau_heading = 10.0  # seconds, for leaky integrator
        self.integrator_tau_depth = 10.0

        # Depth-hold mode flag (ensure attribute exists before use in run)
        self.depth_hold = False
        # Rosbag recording flag
        self.recording = False

        # Point follow mission: A => enable; LH+A => disable
        self.point_follow_mission = False
        
        # Données du tracker - Point cible unique
        self.target_range = 0.0    # Distance à la cible (m)
        self.target_bearing = 0.0  # Bearing de la cible (rad)
        self.target_visible = False
        
        # Instanciation du contrôleur simple (un seul point)
        self.point_controller = SinglePointController(stop_distance=0.5)

        # Oriented approach mission: Y => enable; LH+Y => disable
        self.oriented_approach_mission = False
        
        # Angle d'orientation de la cage (valeur fixe, à ajuster selon la cage)
        # Convention: angle en radians, repère monde mathématique (0° = Est, 90° = Nord)
        # Exemple: np.pi/4 = 45° = cage orientée vers le Nord-Est
        self.cage_angle = np.pi / 4  # TODO: Ajuster selon l'orientation réelle de la cage
        
        # Instanciation du contrôleur d'approche orientée
        self.oriented_controller = OrientedApproachController(
            stop_distance=0.5,
            orbit_distance=1.5,
            angle_tolerance_deg=5.0
        )

                # Desired depth to maintain when depth_hold is True.
        # Initialized to current depth; will be updated after manual joystick moves.
        self.desired_depth = self.depth
        # Internal flag: True while operator is moving depth stick; used to capture last depth on release.
        self._depth_move_active = False

        # PID state for depth control
        self._depth_integral = 0.0
        self._depth_error_prev = 0.0
        # PID gains (tunable)
        self.Kp_depth = 100.0
        self.Ki_depth = 5.0
        self.Kd_depth = 10.0
        # Integral windup limits (tunable)
        self._depth_integral_min = -50.0
        self._depth_integral_max = 50.0

        # IMU
        self.Phi_rad = 0.0
        self.Theta_rad = 0.0
        self.Psy_rad = 0.0
        self.angular_velocity_y = 0.0  # vitesse tangage
        self.angular_velocity_x = 0.0  # vitesse rouli
        

        # Buttons and Axes
        self.frame_id = 0
        self.frame_id_last = 0
        self.joy_time = 0.0
        self.joy_time_old = 0.0
        self.letter_to_indice = {"A": 0, "B": 1, "X": 2, "Y": 3, "LH": 4,
                                 "RH": 5, "Back": 6, "Start": 7, "?": 8, "L3": 9, "R3": 10}
        # "?" cest le bouton logitech; "LH" et "RH" cest "lB" et "RB"
        self.button_values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # A, B, X, Y, LH , RH , back, start, ?, L3, R3
        
        # L3 (x,y) sur (0,1) WARNING axe des x va de 1 à -1 (de gauche à droite) (axe y de -1 à 1)  
        # R3 (x,y) sur (3,4) WARNING axe des x va de 1 à -1 (de gauche à droite) 
        # LT : axe 2 (1 neutre à -1 pressé) et RT pareil sur axe 5
        # axe 6 flèche gauche : 1 et flèche droite : -1 
        # axe 7 flèche bas : -1 et flèche haut : 1 
        
        self.axes = [0, 0, 0, 0, 0, 0, 0, 0]
         
        # ROS
        #### listener
        self.qos_profile = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.SYSTEM_DEFAULT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.queue_listener = 10
        self.listener()  # Subscribes to the messages

        ######################
        # liste des publishers
        self.command_pub = self.create_publisher(OverrideRCIn,self.ns+'/mavros/rc/override', self.queue_listener)

        #####################

        self.nb_channels = 18 # rospy.get_param(rospy.get_name() + "/nb_channels", 18) # TODO: chercher pk ca ne marche pas...
        self.commands = [1500] * self.nb_channels
        if self.nb_channels > 8:     # if the MAVROS version can control the channels 8, i.e. the light
            self.commands[8] = 1000  # light pwm : 1000 = off, 2000 = max luminosity

        
        # MAVROS
        self.client_arm = self.create_client(CommandBool, self.ns+'/mavros/cmd/arming')
        self.send_request_arm(False)


        timer_period = self.dt # seconds
        self.timer = self.create_timer(timer_period, self.run)


    def button(self, letter):
        return self.button_values[self.letter_to_indice[letter]]


    def send_commands(self):

        msg = OverrideRCIn()
        for i in range(self.nb_channels):
            self.commands[i] = clip(self.commands[i], 1000, 2000)
            msg.channels[i] = self.commands[i] 
        #msg.channels = self.commands
        self.command_pub.publish(msg)
        
        

    def callback_heading(self, msg):
        self.heading = msg.data
        self._have_compass = True

    def callback_heading_reference(self, data):
        self.heading_reference = data.data
        self.heading_global_target = self.heading_reference + self.heading_offset

    def callback_imu(self, msg):
        self.angular_velocity_z = msg.angular_velocity.z  # vitesse de lacet
        self.angular_velocity_y = msg.angular_velocity.y  # (vitesse de tangage ?)
        self.angular_velocity_x = msg.angular_velocity.x  # (vitesse de roulis ?)
        W = msg.orientation.w
        X = msg.orientation.x
        Y = msg.orientation.y
        Z = msg.orientation.z
        orientq=(W, X, Y, Z)
        ### Conversion quaternions in rotation matrix
        self.Phi_rad, self.Theta_rad, self.Psy_rad = Rotation.from_quat([orientq[1], orientq[2], orientq[3],   orientq[0]]).as_euler("xyz") # Roulis, Tangage, Lacet 
        
                
    def callback_joy(self, msg):

        new_time = msg.header.stamp.sec +  msg.header.stamp.nanosec*10**(-9)

        if new_time - self.joy_time_old > 0.1:
            self.button_values = msg.buttons
            self.axes = msg.axes
            
            self.joy_time_old = self.joy_time
            self.joy_time = new_time

            self.frame_id = self.frame_id + 1

    def callback_depth(self, msg):
        self.depth = -msg.data

        
    def callback_ping1d(self, data):
        self.distance_ping = data.data[0]
        self.confidence_ping = data.data[1]
        self.ping = data.data[2]

    def callback_tracker(self, msg):
        """Callback for tracker data - single point (center of tracked object)."""
        # Récupérer les coordonnées polaires du centre de l'objet tracké
        self.target_range = msg.range
        self.target_bearing = msg.bearing
        
        # Détecter si la cible est visible (tracker publie 0,0 quand pas de tracking)
        self.target_visible = not (msg.range == 0.0 and msg.bearing == 0.0)
                     
           
    def listener(self):

        self.subscription = self.create_subscription(
            Float64,
            self.ns+'/mavros/global_position/compass_hdg',
            self.callback_heading,
            self.qos_profile)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.callback_joy,
            self.queue_listener)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Float64,
            self.ns+'/mavros/global_position/rel_alt',
            self.callback_depth,
            self.qos_profile)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning
      
        self.subscription = self.create_subscription(
            Imu,
            self.ns+"/mavros/imu/data",
            self.callback_imu,
            self.qos_profile)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning


        self.subscription = self.create_subscription(
            Float32MultiArray,
            self.ns+'/msg_ping1d',
            self.callback_ping1d,
            self.queue_listener)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            TrackedObject,
            '/docking/tracking/tracked_object',
            self.callback_tracker,
            self.queue_listener)
        self.subscription  # prevent unused variable warning



    def send_request_arm(self, arming):

        while not self.client_arm.wait_for_service(timeout_sec=0.5):
                    self.get_logger().info('service not available, waiting again...')
        req = CommandBool.Request()
        req.value = bool(arming)
        self.client_arm.call_async(req)


    # =====================================================================
    # FONCTIONS AUXILIAIRES POUR LA BOUCLE RUN
    # =====================================================================

    def _reset_commands(self):
        """Remet les 8 premières commandes à neutre (1500)."""
        for i in range(8):
            self.commands[i] = 1500

    def _handle_arm_disarm(self):
        """Gère l'armement/désarmement via les boutons Start/Back."""
        if (self.button("Start") == 1) | (self.button("Back") != 0):
            
            if (self.button("Start") == 1):
                self.armed = True
                print("[ARMING] ROV Armed ✓")
            if self.button("Back") != 0:
                self.armed = False
                print("[DISARMING] ROV Disarmed ✓")
            response = self.send_request_arm(self.armed)

            self.commands = [1500] * self.nb_channels
            if self.nb_channels > 8:
                self.commands[8] = 1000

    def _handle_camera_and_light(self):
        """Gère l'inclinaison caméra et le contrôle lumière."""
        # Camera inclination
        if (self.axes[7] != 0):  # fleche haut/bas
            self.commands[7] = int(100 * self.axes[7] + 1500)
        else:
            self.commands[7] = 1500

        # Light control
        if self.button("?") != 0:  # button Logitech : control light intensity
            light_modif_value = 100
            light_control(self, light_modif_value)

    def _handle_depth_hold_toggle(self):
        """Active/désactive le maintien de profondeur (Bouton B)."""
        if self.button("B") != 0:
            if self.button("LH") != 0:
                # LH + B : deactivate (only if currently active)
                if self.depth_hold:
                    self.depth_hold = False
                    print("[DEPTH HOLD] Deactivated (LH + B) → Manual depth mode")
            else:
                # B alone : activate (only if currently inactive)
                if not self.depth_hold:
                    self.depth_hold = True
                    self.desired_depth = self.depth
                    print(f"[DEPTH HOLD] Activated (B) → Target depth: {self.desired_depth:.2f}m")

    def _handle_point_follow_toggle(self):
        """Active/désactive la mission de suivi de point (Bouton A)."""
        if self.button("A") != 0:
            if self.button("LH") != 0:
                # LH + A : deactivate (only if currently active)
                if self.point_follow_mission:
                    self.point_follow_mission = False
                    print("[MISSION] Point follow deactivated (LH + A) → Manual mode")
            else:
                # A alone : activate (only if currently inactive)
                if not self.point_follow_mission:
                    self.point_follow_mission = True
                    print("[MISSION] Point follow ACTIVATED (A) → Auto-tracking enabled")

    def _handle_heading_hold_toggle(self):
        """Active/désactive le maintien de cap (Bouton X)."""
        if self.button("X") != 0:
            if self.button("LH") != 0:
                # LH + X -> deactivate heading hold
                if self.heading_hold:
                    self.heading_hold = False
                    print("[HEADING HOLD] Deactivated (LH + X) → Free manual yaw mode")
            else:
                # X alone -> activate heading hold and capture current heading
                if not self.heading_hold:
                    self.heading_hold = True
                    self.desired_heading = self.heading
                    self._heading_integral = 0.0
                    self._heading_error_prev = 0.0
                    print(f"[HEADING HOLD] Activated (X) → Target: {self.desired_heading:.1f}°")

    def _handle_oriented_approach_toggle(self):
        """Active/désactive la mission d'approche orientée (Bouton Y)."""
        if self.button("Y") != 0:
            if self.button("LH") != 0:
                # LH + Y : deactivate (only if currently active)
                if self.oriented_approach_mission:
                    self.oriented_approach_mission = False
                    print("[MISSION] Oriented approach deactivated (LH + Y) → Manual mode")
            else:
                # Y alone : activate (only if currently inactive)
                if not self.oriented_approach_mission:
                    # Désactiver la mission A si active
                    if self.point_follow_mission:
                        self.point_follow_mission = False
                        print("[MISSION] Point follow deactivated → Switching to Oriented Approach")
                    self.oriented_approach_mission = True
                    print(f"[MISSION] Oriented Approach ACTIVATED (Y) → Cage angle: {np.degrees(self.cage_angle):.0f}°")

    def _handle_point_follow_mission(self):
        """Exécute la mission de suivi de point (contrôle auto yaw, avance, latéral)."""
        # Appel au contrôleur simple
        forward_speed, lateral_speed, angular_speed = self.point_controller.control_step(
            self.target_bearing, self.target_range, self.target_visible
        )
        
        # Conversion des vitesses (m/s et rad/s) vers commandes RC (PWM 1000-2000)
        self.commands[3] = int(1500 + angular_speed * 170) 
        
        self.commands[4] = int(1500 + forward_speed * 200)
        
        self.commands[5] = int(1500 + lateral_speed * 200)
        
        # Affichage de l'état
        state_name = self.point_controller.state.upper()
        if self.target_visible:
            print(f"[{state_name}] Target @ {self.target_range:.2f}m, {np.degrees(self.target_bearing):+.0f}° | "
                  f"Robot state: Depth :({self.depth:.2f}m, Heading: {self.heading:.1f}°) | "
                  f"Cmds: Yaw={self.commands[3]} Fwd={self.commands[4]} Lat={self.commands[5]}")
        else:
            print(f"[{state_name}] Target not visible - waiting...")

    def _handle_oriented_approach_mission(self):
        """Exécute la mission d'approche orientée (contrôle auto yaw, avance, latéral).
        
        Cette mission permet d'approcher une cible en arrivant par un angle spécifique,
        comme pour entrer dans une cage par l'ouverture.
        
        Phases:
        1. APPROACHING: Approche jusqu'à orbit_distance (1.5m)
        2. ORBITING: Tourne autour de la cible pour s'aligner avec l'angle de la cage
        3. FINAL_APPROACH: Approche finale jusqu'à stop_distance (0.5m)
        """
        # Calcul de l'erreur d'angle d'approche
        # heading est en degrés (0° = Nord), on le convertit en radians (repère math)
        heading_rad = math.radians(self.heading)
        # Conversion: heading navigation (0°=Nord) vers math (0°=Est): math_angle = 90° - nav_angle
        heading_math = math.pi/2 - heading_rad
        
        # L'angle du robot autour de la cible = heading_math + bearing + π
        # (direction depuis laquelle le robot voit la cible, dans le repère monde)
        robot_angle_around_target = heading_math + self.target_bearing + math.pi
        
        # L'angle d'approche cible = cage_angle + π (le robot doit arriver FACE à la cage)
        desired_approach_angle = self.cage_angle + math.pi
        
        # Erreur = angle_cible - position_actuelle
        approach_angle_error = desired_approach_angle - robot_angle_around_target
        # Normaliser entre -π et π
        approach_angle_error = math.atan2(math.sin(approach_angle_error), math.cos(approach_angle_error))
        
        # Appel au contrôleur d'approche orientée
        forward_speed, lateral_speed, angular_speed = self.oriented_controller.control_step(
            self.target_bearing, self.target_range, approach_angle_error, self.target_visible
        )
        
        # Conversion des vitesses (m/s et rad/s) vers commandes RC (PWM 1000-2000)
        self.commands[3] = int(1500 + angular_speed * 170)
        self.commands[4] = int(1500 + forward_speed * 200)
        self.commands[5] = int(1500 + lateral_speed * 200)
        
        # Affichage de l'état
        state_name = self.oriented_controller.state.upper()
        if self.target_visible:
            print(f"[{state_name}] Target @ {self.target_range:.2f}m, brg={np.degrees(self.target_bearing):+.0f}° | "
                  f"Angle err: {np.degrees(approach_angle_error):+.0f}° | "
                  f"Cmds: Yaw={self.commands[3]} Fwd={self.commands[4]} Lat={self.commands[5]}")
        else:
            print(f"[{state_name}] Target not visible - waiting...")

    def _handle_manual_movement(self):
        """Gère les contrôles manuels (avance, latéral) quand la mission n'est pas active.
        
        Note: Le yaw est géré séparément par _handle_yaw_control() pour permettre
        le heading hold même en mode manuel.
        """
        # Move forward/backward
        if self.axes[1] != 0:  
            self.commands[4] = int(200 * self.axes[1] + 1500)
        else:
            self.commands[4] = 1500

        # Move left/right translation
        if self.axes[0] != 0:  
            self.commands[5] = int(200 * -self.axes[0] + 1500)
        else:
            self.commands[5] = 1500

    def _handle_yaw_control(self):
        """Gère le contrôle du yaw (manuel ou maintien de cap via PID)."""
        if self.axes[3] != 0:  # joy right left/right
            self.commands[3] = int(200 * -self.axes[3] + 1500)
            # mark manual move active so that on release we can capture new desired heading
            if self.heading_hold:
                self._heading_move_active = True
            print(f"[YAW] Manual control - Cmd: {self.commands[3]} PWM")
        else:
            if self.heading_hold:
                # if operator just released the yaw stick after moving it, capture current heading
                if self._heading_move_active:
                    current_heading_deg = self.heading
                    self.desired_heading = current_heading_deg
                    self._heading_integral = 0.0
                    self._heading_error_prev = 0.0
                    self._heading_move_active = False
                    print(f"[HEADING HOLD] New target captured: {self.desired_heading:.1f}°")
                # PID de maintien de cap basé sur la boussole MAVROS
                if not self._have_compass:
                    print("[HEADING HOLD] ⚠ Waiting for compass data...")
                    self.commands[3] = 1500
                else:
                    self._compute_heading_pid()
            else:
                print("[YAW] Free manual mode (no heading hold)")
                self.commands[3] = 1500

    def _compute_heading_pid(self):
        """Calcule la commande PID pour le maintien de cap."""
        current_heading_deg = self.heading
        err_deg = angle_diff_deg(self.desired_heading, current_heading_deg)
        dt = self.dt if self.dt > 0 else 0.1
        
        # integral with anti-windup
        self._heading_integral += err_deg * dt
        self._heading_integral = max(self._heading_integral_min, min(self._heading_integral, self._heading_integral_max))
        
        # derivative
        derivative = (err_deg - self._heading_error_prev) / dt if dt > 0 else 0.0
        self._heading_error_prev = err_deg
        
        # PID output (deg -> RC mapping)
        pid_out = (self.Kp_heading * err_deg) + (self.Ki_heading * self._heading_integral) + (self.Kd_heading * derivative)
        print(f"[HEADING HOLD] Target: {self.desired_heading:.1f}° | Current: {current_heading_deg:.1f}° | Err: {err_deg:+.1f}° | PID: {pid_out:.1f} | Cmd: {int(1500 + pid_out)} PWM")
        
        self.commands[3] = int(1500 + pid_out)
        self.commands[3] = clip(self.commands[3], 1300, 1700)

    def _handle_depth_control(self):
        """Gère le contrôle de profondeur (manuel ou maintien via PID)."""
        # Use LT (axis 2) to go down and RT (axis 5) to go up
        # LT and RT: 1 when neutral, -1 when fully pressed
        # depth_input: positive to go up (RT), negative to go down (LT)
        depth_input = (self.axes[2] - self.axes[5]) / 2.0
        
        if abs(depth_input) > 0.05:  # deadzone to avoid drift when triggers are neutral
            # Direct manual control while trigger is pressed
            self.commands[2] = int(200 * depth_input + 1500)
            print(f"[DEPTH] Manual control - Current: {self.depth:.2f}m | Cmd: {self.commands[2]} PWM")
            # If depth hold is active, mark that operator is changing depth
            if self.depth_hold:
                self._depth_move_active = True
        else:
            if self.depth_hold:
                # If operator just released the stick after moving it, capture the current depth
                if self._depth_move_active:
                    self.desired_depth = self.depth
                    self._depth_integral = -9  # seems to be approximately the value reached when depth is stable
                    self._depth_error_prev = 0.0
                    self._depth_move_active = False
                    print(f"[DEPTH HOLD] New target captured: {self.desired_depth:.2f}m")

                self._compute_depth_pid()
            else:
                print("[DEPTH] Free manual mode (no depth hold)")
                self.commands[2] = 1500

    def _compute_depth_pid(self):
        """Calcule la commande PID pour le maintien de profondeur."""
        error = self.desired_depth - self.depth
        dt = self.dt if self.dt > 0 else 0.1

        # Integrate with anti-windup
        self._depth_integral += error * dt
        self._depth_integral = max(self._depth_integral_min, min(self._depth_integral, self._depth_integral_max))
        
        # Derivative
        derivative = (error - self._depth_error_prev) / dt if dt > 0 else 0.0
        self._depth_error_prev = error

        # PID output
        pid_out = (self.Kp_depth * error) + (self.Ki_depth * self._depth_integral) + (self.Kd_depth * derivative)

        print(f"[DEPTH HOLD] Target: {self.desired_depth:.2f}m | Current: {self.depth:.2f}m | Err: {error:+.3f}m | PID: {pid_out:.1f} | Cmd: {int(1500 - pid_out)} PWM")

        # Convert PID output to RC command and bound
        self.commands[2] = int(1500 - pid_out)
        self.commands[2] = clip(self.commands[2], 1300, 1700)

    def _print_state_summary(self):
        """Affiche le résumé de l'état actuel du ROV."""
        print(f"[STATE] Armed: {self.armed} | Depth: {self.depth:.2f}m | Heading: {self.heading:.1f}°")
        print(f"[MODES] DepthHold: {self.depth_hold} | HeadingHold: {self.heading_hold} | PointMission(A): {self.point_follow_mission} | OrientedApproach(Y): {self.oriented_approach_mission}")
        print(f"[COMMANDS] Elev: {self.commands[2]} | Yaw: {self.commands[3]} | Forward: {self.commands[4]} | Lateral: {self.commands[5]} | Light: {self.commands[8]}")
        if self.target_visible:
            print(f"[TARGET] Range: {self.target_range:.2f}m | Bearing: {np.degrees(self.target_bearing):+.0f}°")
        print("=" * 80 + "\n")

    # =====================================================================
    # BOUCLE PRINCIPALE
    # =====================================================================

    def run(self):
        """
        Boucle principale de contrôle du ROV.
        
        Structure:
        1. Traitement des nouvelles commandes joystick
           - Armement/Désarmement
           - Caméra et lumière
           - Toggles des modes (depth hold, heading hold, mission)
           - Mission automatique ou contrôles manuels
           - Contrôle yaw et profondeur
        2. Affichage du résumé d'état
        3. Envoi des commandes
        """
        print("[ROV] --- Control loop running ---")

        # Traitement uniquement si nouvelle trame joystick reçue
        if self.frame_id_last != self.frame_id:
            self.frame_id_last = self.frame_id

            # Reset des commandes à neutre
            self._reset_commands()

            # Gestion armement/désarmement (Start/Back)
            self._handle_arm_disarm()

            # Gestion caméra et lumière
            self._handle_camera_and_light()

            # Toggles des modes
            self._handle_depth_hold_toggle()       # Bouton B
            self._handle_point_follow_toggle()     # Bouton A
            self._handle_heading_hold_toggle()     # Bouton X
            self._handle_oriented_approach_toggle() # Bouton Y

            # Contrôle du mouvement (mission auto ou manuel)
            if self.oriented_approach_mission:
                # Mission Y: approche orientée (yaw, avance, latéral automatiques)
                self._handle_oriented_approach_mission()
            elif self.point_follow_mission:
                # Mission A: suivi de point simple (yaw, avance, latéral automatiques)
                self._handle_point_follow_mission()
            else:
                # Mode manuel: translation (avance/latéral)
                self._handle_manual_movement()
                # Mode manuel: yaw (avec support heading hold)
                self._handle_yaw_control()

            # Contrôle profondeur (priorité gâchettes sur PID depth hold)
            self._handle_depth_control()

        # Affichage résumé état
        self._print_state_summary()

        # Envoi des commandes au ROV
        self.send_commands()
                    
            
def main(args=None):


    rclpy.init(args=args)


    node_rov = ROV()
    
    rclpy.spin(node_rov)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node_rov.destroy_node()
    rclpy.shutdown()

          

if __name__ == '__main__':
    main()