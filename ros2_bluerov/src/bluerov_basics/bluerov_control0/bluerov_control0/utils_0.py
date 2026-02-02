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



    def send_request_arm(self, arming):

        while not self.client_arm.wait_for_service(timeout_sec=0.5):
                    self.get_logger().info('service not available, waiting again...')
        req = CommandBool.Request()
        req.value = bool(arming)
        self.client_arm.call_async(req)


###############################
    def run(self):

        print("[ROV] --- Control loop running ---")


        if (self.frame_id_last != self.frame_id):  # Check if new commands are given with joystick  : + on regarde si on obéit ou non à la télécommande
            self.frame_id_last = self.frame_id


            for i in range(8):
                self.commands[i] = 1500 # on remets les commandes a zero

            if (self.button("Start") == 1)|(self.button("Back") != 0):  # Arm and disarm
                
                if (self.button("Start") == 1):
                    self.armed = True 
                if self.button("Back") != 0:
                    self.armed = False # not self.armed
                response = self.send_request_arm(self.armed) # armement/desarmement

                self.commands = [1500] * self.nb_channels
                if self.nb_channels > 8:
                    self.commands[8] = 1000

            # camera inclination 
            if (self.axes[7] != 0):  # fleche haut/bas
                self.commands[7] = int(100 * self.axes[7] + 1500)
            else:
                self.commands[7] = 1500

            # light control 
            if self.button("?") != 0:  # button Back : control light intensity
                light_modif_value = 100
                light_control(self,light_modif_value)

            # Bouton B : Activer/ désactiver maintien de profondeur
            # B alone -> activate depth hold; LH + B -> deactivate depth hold
            if self.button("B") != 0:
                if self.button("LH") != 0:
                    # LH + B : deactivate (only if currently active)
                    if self.depth_hold:
                        self.depth_hold = False
                        self.get_logger().info("Depth hold deactivated (LH + B)")
                else:
                    # B alone : activate (only if currently inactive)
                    if not self.depth_hold:
                        self.depth_hold = True
                        self.get_logger().info("Depth hold activated (B)")

            ##### Lecture des input de la manette

            # Example : move forward/backward
            if self.axes[1] != 0:  # joy right up/down
                self.commands[4] = int(200 * self.axes[1] + 1500)
                self.commands_front = self.commands[4]

            else:
                self.commands[4] = 1500
                self.commands_front = self.commands[4]

            # Example : move left/right translation
            if self.axes[0] != 0:  # joy right up/down
                self.commands[5] = int(200 * -self.axes[0] + 1500)
                self.commands_front = self.commands[5]

            else:
                self.commands[5] = 1500
                self.commands_front = self.commands[5]

            # Example : move yaw
            # Heading hold control using X / LH+X
            if self.button("X") != 0:
                if self.button("LH") != 0:
                    # LH + X -> deactivate heading hold
                    if self.heading_hold:
                        self.heading_hold = False
                        self.get_logger().info("Heading hold deactivated (LH + X)")
                else:
                    # X alone -> activate heading hold to fixed 120°
                    if not self.heading_hold:
                        self.heading_hold = True
                        self.desired_heading = 200.0
                        self.desired_heading = self.heading
                        self._heading_integral = 0.0
                        self._heading_error_prev = 0.0
                        self.get_logger().info(f"Heading hold activated (X) -> target {self.desired_heading}°")

            # Manual yaw input takes precedence
            if self.axes[3] != 0:  # joy right left/right
                self.commands[3] = int(100 * -self.axes[3] + 1500)
                self.commands[3] = clip(self.commands[3], 1300, 1700)
                self.commands_front = self.commands[3]
                # mark manual move active so that on release we can capture new desired heading
                if self.heading_hold:
                    self._heading_move_active = True
            else:
                if self.heading_hold:
                    # if operator just released the yaw stick after moving it, capture current heading
                    if self._heading_move_active:
                        current_heading_deg = self.heading 
                        self.desired_heading = 200.0 #mettre current_heading_deg pour garder le cap actuel
                        self.desired_heading = current_heading_deg
                        self._heading_integral = 0.0
                        self._heading_error_prev = 0.0
                        self._heading_move_active = False
                    # PID de maintien de cap basé sur la boussole MAVROS
                    if not self._have_compass:
                        print("[HEADING] ⚠ Waiting for compass data...")
                        # pas encore de donnée -> neutre
                        self.commands[3] = 1500
                    else:
                        # PID to maintain desired_heading
                        current_heading_deg = self.heading
                        err_deg = angle_diff_deg(self.desired_heading, current_heading_deg)
                        dt = self.dt if self.dt > 0 else 0.1
                        # integral with anti-windup
                        # tau = self.integrator_tau_heading  # constante de temps, par ex. 10.0 secondes
                        # alpha = math.exp(-dt / tau)  # leaky integrator pour suppimer les anciennes erreures
                        # self._heading_integral = self._heading_integral * alpha + err_deg * dt
                        self._heading_integral +=  err_deg * dt

                        self._heading_integral = max(self._heading_integral_min, min(self._heading_integral, self._heading_integral_max))
                        # derivative
                        derivative = (err_deg - self._heading_error_prev) / dt if dt > 0 else 0.0
                        self._heading_error_prev = err_deg
                        # PID output (deg -> RC mapping)
                        pid_out = (self.Kp_heading * err_deg) + (self.Ki_heading * self._heading_integral) + (self.Kd_heading * derivative)
                        print(f"[HEADING HOLD] Target: {self.desired_heading:.1f}° | Current: {current_heading_deg:.1f}° | Err: {err_deg:.1f}° | PID: {pid_out:.1f}")
                        self.commands[3] = int(1500 + pid_out)
                        self.commands[3] = clip(self.commands[3], 1300, 1700)
                        self.commands_front = self.commands[3]
                else:
                    print("[HEADING] Manual mode (free)")
                    self.commands[3] = 1500
                    self.commands_front = self.commands[3]


            # Example : move elevation
            # Use LT (axis 2) to go down and RT (axis 5) to go up
            # LT and RT: 1 when neutral, -1 when fully pressed
            # depth_input: positive to go up (RT), negative to go down (LT)
            depth_input = (self.axes[2] - self.axes[5]) / 2.0
            
            if abs(depth_input) > 0.05:  # deadzone to avoid drift when triggers are neutral
                # Direct manual control while trigger is pressed
                self.commands[2] = int(200 * depth_input + 1500)
                # Bound manual command to safe range
                self.commands[2] = clip(self.commands[2], 1300, 1700)
                self.commands_front = self.commands[2]
                # If depth hold is active, mark that operator is changing depth
                if self.depth_hold:
                    self._depth_move_active = True

            else:
                if self.depth_hold:
                    # If operator just released the stick after moving it, capture the current depth
                    if self._depth_move_active:
                        self.desired_depth = self.depth
                        # reset integral and previous error to avoid jump
                        self._depth_integral = -9 #  seems to be approximally the value reached when depth is stable so it avoid jump
                        self._depth_error_prev = 0.0
                        self._depth_move_active = False
                    # self.desired_depth = 2

                    # maintain desired depth using PID
                    # error = desired - current (positive => need to go deeper)
                    error = self.desired_depth - self.depth
                    dt = self.dt if self.dt > 0 else 0.1

                    # Integrate with anti-windup
                    # tau = self.integrator_tau_depth  # constante de temps, par ex. 10.0 secondes
                    # alpha = math.exp(-dt / tau)
                    # self._depth_integral = self._depth_integral * alpha + error * dt
                    self._depth_integral += error * dt
                    self._depth_integral = max(self._depth_integral_min, min(self._depth_integral, self._depth_integral_max))
                    # Derivative
                    derivative = (error - self._depth_error_prev) / dt if dt > 0 else 0.0
                    self._depth_error_prev = error

                    # PID output (maps to same sign as previous P-only behavior)
                    pid_out = (self.Kp_depth * error) + (self.Ki_depth * self._depth_integral) + (self.Kd_depth * derivative)

                    print(f"[DEPTH HOLD] Target: {self.desired_depth:.2f}m | Current: {self.depth:.2f}m | Err: {error:.3f} | PID: {pid_out:.1f}")
                    

                    # Convert PID output to RC command and bound
                    self.commands[2] = int(1500 - pid_out)
                    self.commands[2] = clip(self.commands[2], 1300, 1700)
                    # Optional debug
                    # print(f"Depth err:{error:.3f} I:{self._depth_integral:.3f} D:{derivative:.3f} pid:{pid_out:.3f} cmd:{self.commands[2]}")
                    self.commands_front = self.commands[2]
                else:
                    print("[DEPTH] Manual mode (free)")
                    self.commands[2] = 1500
                    self.commands_front = self.commands[2]





        # Reste du code
        ##############################################
        
        # (...)

        # (Programme lié au bouton B retiré)
        print(f"[LIGHT] PWM: {self.commands[8]}")

        self.send_commands()
        self.commands_old = self.commands


            
            
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