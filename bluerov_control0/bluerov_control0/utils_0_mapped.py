
# Copyright 2024, Christophe VIEL
# (modified mapping by ChatGPT, 2025-10-03) 
#
# License: BSD-3-Clause (same as original)
#
# NOTE: Angles provided by sensor are in deg. Variables with a "rad" are expressed in radians.

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

import os
import time
from scipy.spatial.transform import Rotation 
import numpy as np
import math

from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float64, String, Bool, Float32MultiArray

###############################################################
# Helper functions

def clip(val, min_val, max_val):
    if val <= min_val:
        return min_val
    elif val >= max_val:
        return max_val
    return val

def pwm_from_axis(axis_value, gain=1.0):
    # axis_value in [-1,1] -> PWM about 1500 with +/- 500*gain
    axis_value = clip(axis_value * gain, -1.0, 1.0)
    return int(1500 + 500 * axis_value)

###############################################################

class ROV(Node):

    def __init__(self):
        super().__init__('Control_ROV')

        # -- timing
        self.dt = 1/10

        # -- parameters
        self.declare_parameter('ROV_name', '')
        ROV_name = self.get_parameter('ROV_name').get_parameter_value().string_value
        self.declare_parameter('ID', 1.0)
        ID = self.get_parameter('ID').get_parameter_value().double_value
        self.declare_parameter('ip_adress_light', "192.168.2")
        ip_adress_light = self.get_parameter('ip_adress_light').get_parameter_value().string_value

        # -- identification
        self.ROV_name = ROV_name
        self.ID = str(ID)
        self.ID0 = ID
        self.IDnum = str(self.ID0+1)

        self.ns = ("/" + self.ROV_name) if self.ROV_name != '' else ""

        # -- light (user custom setup: ssh write to /home/pi/pwm_light.txt + RC CH8 for classic)
        self.adresse_ip_short = ip_adress_light
        self.adresse_ip = 'pi@' + self.adresse_ip_short  + '.2'
        self.pwm_light = 1000.0  # [1000-2000]

        # -- robot state
        self.armed = False
        self.program_B = False
        self.input_hold = False

        # -- vehicle state
        self.depth = 0.0
        self.heading = 0.0

        # -- IMU
        self.Phi_rad = 0.0
        self.Theta_rad = 0.0
        self.Psy_rad = 0.0
        self.angular_velocity_y = 0.0  # pitch rate
        self.angular_velocity_x = 0.0  # roll  rate
        self.angular_velocity_z = 0.0  # yaw   rate

        # -- joystick cache
        self.frame_id = 0
        self.frame_id_last = 0
        self.joy_time = 0.0
        self.joy_time_old = 0.0

        # Button name -> index mapping (keep original naming for compatibility)
        self.letter_to_indice = {
            "A": 0, "B": 1, "X": 2, "Y": 3, "LH": 4, "RH": 5, "Back": 6, "Start": 7, "?": 8, "L3": 9, "R3": 10
        }
        # values updated by callback_joy
        self.button_values = [0]*11  # A, B, X, Y, LH , RH , Back, Start, ?, L3, R3
        self.axes = [0]*8            # 0:LS LR, 1:LS UD, 2:LT, 3:RS LR, 4:RS UD, 5:RT, 6:Dpad LR, 7:Dpad UD

        # -- ROS I/O
        self.qos_profile = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.SYSTEM_DEFAULT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.queue_listener = 10
        self.listener()  # subscriptions

        self.command_pub = self.create_publisher(OverrideRCIn, self.ns + '/mavros/rc/override', self.queue_listener)
        self.program_B_name_pub = self.create_publisher(String, self.ns + "/program_B_name", self.queue_listener)

        # -- RC channels (ArduSub typical: CH7 camera tilt servo, CH8 lights PWM)
        self.nb_channels = 18
        self.commands = [1500] * self.nb_channels
        if self.nb_channels > 8:
            self.commands[8] = 1000  # lights off

        # -- services (arm/disarm + mode)
        self.client_arm = self.create_client(CommandBool, self.ns + '/mavros/cmd/arming')
        self.client_mode = self.create_client(SetMode, self.ns + '/mavros/set_mode')
        self.send_request_arm(False)

        # -- camera tilt control (bumpers) & gain
        self.camera_pwm = 1500         # CH7 current PWM
        self.camera_step = 50          # per button press
        self.camera_min = 1100
        self.camera_max = 1900
        self.commands[7] = self.camera_pwm

        self.gain = 1.0
        self.gain_min = 0.3
        self.gain_max = 1.5
        self.gain_step = 0.1

        # -- timer loop
        self.timer = self.create_timer(self.dt, self.run)

    # ---------------------------- utils ----------------------------
    def button(self, letter):
        return self.button_values[self.letter_to_indice[letter]]

    def send_commands(self):
        msg = OverrideRCIn()
        for i in range(self.nb_channels):
            self.commands[i] = clip(self.commands[i], 1000, 2000)
            msg.channels[i] = self.commands[i]
        self.command_pub.publish(msg)

        name_msg = String()
        name_msg.data = getattr(self, 'name_program_B', '')
        self.program_B_name_pub.publish(name_msg)

    def set_mode(self, custom_mode: str):
        # Wait for service, then try to set mode
        while not self.client_mode.wait_for_service(timeout_sec=0.2):
            self.get_logger().info('set_mode service not available, waiting...')
        req = SetMode.Request(base_mode=0, custom_mode=custom_mode)
        self.client_mode.call_async(req)

    def send_request_arm(self, arming: bool):
        while not self.client_arm.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('arming service not available, waiting...')
        req = CommandBool.Request()
        req.value = bool(arming)
        self.client_arm.call_async(req)

    # -------------------------- callbacks --------------------------
    def callback_heading(self, msg):
        self.heading = msg.data

    def callback_imu(self, msg: Imu):
        self.angular_velocity_z = msg.angular_velocity.z
        self.angular_velocity_y = msg.angular_velocity.y
        self.angular_velocity_x = msg.angular_velocity.x
        W = msg.orientation.w
        X = msg.orientation.x
        Y = msg.orientation.y
        Z = msg.orientation.z
        self.Phi_rad, self.Theta_rad, self.Psy_rad = Rotation.from_quat([X, Y, Z, W]).as_euler("xyz")

    def callback_joy(self, msg: Joy):
        new_time = msg.header.stamp.sec +  msg.header.stamp.nanosec*10**(-9)
        if new_time - self.joy_time_old > 0.05:  # debounce
            self.button_values = msg.buttons
            self.axes = msg.axes
            self.joy_time_old = self.joy_time
            self.joy_time = new_time
            self.frame_id += 1

    def callback_depth(self, msg: Float64):
        self.depth = -msg.data

    def callback_ping1d(self, data: Float32MultiArray):
        self.distance_ping = data.data[0]
        self.confidence_ping = data.data[1]
        self.ping = data.data[2]

    def listener(self):
        self.create_subscription(Float64, self.ns + '/mavros/global_position/compass_hdg', self.callback_heading, self.qos_profile)
        self.create_subscription(Joy, '/joy', self.callback_joy, self.queue_listener)
        self.create_subscription(Float64, self.ns + '/mavros/global_position/rel_alt', self.callback_depth, self.qos_profile)
        self.create_subscription(Imu, self.ns + '/mavros/imu/data', self.callback_imu, self.qos_profile)
        self.create_subscription(Float32MultiArray, self.ns + '/msg_ping1d', self.callback_ping1d, self.queue_listener)

    # ----------------------------- I/O helpers -----------------------------
    def light_control(self, delta_pwm: int):
        # wrap-around behavior (as in original)
        pwm = self.pwm_light + delta_pwm
        if pwm > 1900:
            pwm = 1000.0
        if pwm < 1000:
            pwm = 1900.0
        self.pwm_light = pwm

        # user custom ssh write (can be removed for public version)
        try:
            password = 'companion'
            file3 = '/home/pi/pwm_light.txt'
            cmd = f'sshpass -p {password} ssh {self.adresse_ip} "echo {int(pwm)} > {file3}"'
            os.system(cmd)
        except Exception as e:
            self.get_logger().warn(f'Light SSH write failed: {e}')

        # RC CH8 for classic BlueROV
        if self.nb_channels > 8:
            self.commands[8] = int(self.pwm_light)

    # ------------------------------ main loop ------------------------------
    def run(self):
        if self.frame_id_last == self.frame_id:
            # no fresh joystick event
            self.send_commands()
            return
        self.frame_id_last = self.frame_id

        # Reset motion channels to neutral unless input_hold keeps them
        if not self.input_hold:
            for i in range(8):
                self.commands[i] = 1500

        # ---------------- ARM / DISARM ----------------
        if (self.button("Start") == 1) or (self.button("Back") != 0):
            if self.button("Start") == 1:
                self.armed = True
            if self.button("Back") != 0:
                self.armed = False
            self.send_request_arm(self.armed)
            # reset neutral + lights off on state change
            self.commands = [1500] * self.nb_channels
            if self.nb_channels > 8:
                self.commands[8] = 1000

        # ---------------- MODES (no Shift required) ----------------
        # X -> Depth Hold, Y -> Stabilize, B -> Manual (A is Shift only)
        if self.button('X'):
            self.set_mode('ALT_HOLD')   # Depth Hold
        if self.button('Y'):
            self.set_mode('STABILIZE')
        if self.button('B'):
            self.set_mode('MANUAL')

        # ---------------- CAMERA TILT ----------------
        # LB (LH) -> tilt down, RB (RH) -> tilt up, L3 -> center
        if self.button('LH'):
            self.camera_pwm = clip(self.camera_pwm - self.camera_step, self.camera_min, self.camera_max)
        if self.button('RH'):
            self.camera_pwm = clip(self.camera_pwm + self.camera_step, self.camera_min, self.camera_max)
        if self.button('L3'):
            self.camera_pwm = 1500
        self.commands[7] = int(self.camera_pwm)

        # ---------------- GAIN / LIGHTS on D-PAD ----------------
        dpad_lr = int(np.sign(self.axes[6])) if abs(self.axes[6]) > 0.3 else 0   # -1 left, +1 right
        dpad_ud = int(np.sign(self.axes[7])) if abs(self.axes[7]) > 0.3 else 0   # -1 down, +1 up

        # Up/Down: gain +/-
        if dpad_ud == +1:
            self.gain = clip(self.gain + self.gain_step, self.gain_min, self.gain_max)
        elif dpad_ud == -1:
            self.gain = clip(self.gain - self.gain_step, self.gain_min, self.gain_max)

        # Left/Right: lights dimmer/brighter
        if dpad_lr == -1:
            self.light_control(-100)
        elif dpad_lr == +1:
            self.light_control(+100)

        # ---------------- INPUT HOLD TOGGLE ----------------
        if self.button('R3'):
            self.input_hold = not self.input_hold

        # ---------------- STICKS MAPPING ----------------
        # As per BlueRobotics diagram:
        #   Left Stick  : Forward/Reverse (UD) + Lateral Left/Right (LR)
        #   Right Stick : Yaw Left/Right (LR) + Ascend/Descend (UD)
        # Map to RC channels (adjust here if your ArduSub RCx_FUNCTION differs):
        #   CH0: Lateral (sway)   <- LS LR (axes[0])
        #   CH4: Forward (surge)  <- LS UD (axes[1])  [kept CH4 from original code for forward]
        #   CH3: Yaw              <- RS LR (axes[3])
        #   CH2: Vertical (heave) <- RS UD (axes[4])
        if not self.input_hold:
            self.commands[0] = pwm_from_axis(self.axes[0], self.gain)  # lateral
            self.commands[4] = pwm_from_axis(-self.axes[1], self.gain) # forward (invert UD so up=forward)
            self.commands[3] = pwm_from_axis(self.axes[3], self.gain)  # yaw
            self.commands[2] = pwm_from_axis(-self.axes[4], self.gain) # vertical (up=ascend)

        # ---------------- Program B placeholder ----------------
        self.name_program_B = "Operator Control"

        # send
        self.send_commands()

# ------------------------------- main ---------------------------------

def main(args=None):
    rclpy.init(args=args)
    node_rov = ROV()
    rclpy.spin(node_rov)
    node_rov.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
