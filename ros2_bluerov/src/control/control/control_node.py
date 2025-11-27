"""
Nœud d'asservissement PID pour le docking autonome.
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import PoseRelative, State
from geometry_msgs.msg import Twist
import numpy as np


class PIDController:
    """Contrôleur PID simple avec anti-windup."""
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_min: float = -1.0, output_max: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
    
    def reset(self):
        """Réinitialise l'état du PID."""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
    
    def compute(self, error: float, dt: float) -> float:
        """Calcule la commande PID."""
        # Proportionnel
        p_term = self.kp * error
        
        # Intégral avec anti-windup
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Dérivée
        if self.last_time is not None:
            derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        else:
            derivative = 0.0
        d_term = self.kd * derivative
        
        # Commande totale
        output = p_term + i_term + d_term
        
        # Saturation
        output = np.clip(output, self.output_min, self.output_max)
        
        # Anti-windup: si saturé, stopper intégration
        if output == self.output_min or output == self.output_max:
            self.integral -= error * dt
        
        self.last_error = error
        
        return output


class ControlNode(Node):
    """Asservissement PID pour maintenir position relative à la cage."""
    
    def __init__(self):
        super().__init__('control_node')
        
        # Paramètres PID
        self.declare_parameter('pid_x_kp', 0.5)
        self.declare_parameter('pid_x_ki', 0.01)
        self.declare_parameter('pid_x_kd', 0.1)
        
        self.declare_parameter('pid_y_kp', 0.3)
        self.declare_parameter('pid_y_ki', 0.005)
        self.declare_parameter('pid_y_kd', 0.05)
        
        self.declare_parameter('pid_yaw_kp', 1.0)
        self.declare_parameter('pid_yaw_ki', 0.02)
        self.declare_parameter('pid_yaw_kd', 0.2)
        
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 0.5)  # rad/s
        
        # Contrôleurs PID
        self.pid_x = PIDController(
            self.get_parameter('pid_x_kp').value,
            self.get_parameter('pid_x_ki').value,
            self.get_parameter('pid_x_kd').value,
            -self.get_parameter('max_linear_speed').value,
            self.get_parameter('max_linear_speed').value
        )
        
        self.pid_y = PIDController(
            self.get_parameter('pid_y_kp').value,
            self.get_parameter('pid_y_ki').value,
            self.get_parameter('pid_y_kd').value,
            -self.get_parameter('max_linear_speed').value,
            self.get_parameter('max_linear_speed').value
        )
        
        self.pid_yaw = PIDController(
            self.get_parameter('pid_yaw_kp').value,
            self.get_parameter('pid_yaw_ki').value,
            self.get_parameter('pid_yaw_kd').value,
            -self.get_parameter('max_angular_speed').value,
            self.get_parameter('max_angular_speed').value
        )
        
        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseRelative,
            '/docking/localisation/pose',
            self.pose_callback,
            10
        )
        
        self.state_sub = self.create_subscription(
            State,
            '/docking/mission/state',
            self.state_callback,
            10
        )
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # État
        self.control_enabled = False
        self.last_pose_time = None
        
        self.get_logger().info('Control node démarré')
    
    def state_callback(self, msg: State):
        """Active/désactive le contrôle selon l'état de la mission."""
        # Activer seulement en APPROACH et DOCKING
        should_enable = msg.current_state in [State.APPROACH, State.DOCKING]
        
        if should_enable and not self.control_enabled:
            self.get_logger().info('Contrôle activé')
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_yaw.reset()
        elif not should_enable and self.control_enabled:
            self.get_logger().info('Contrôle désactivé')
            # Publier commande nulle
            self.cmd_pub.publish(Twist())
        
        self.control_enabled = should_enable
    
    def pose_callback(self, msg: PoseRelative):
        """Calcule et publie les commandes de vitesse."""
        if not self.control_enabled or not msg.is_valid:
            return
        
        # Calcul dt
        current_time = self.get_clock().now()
        if self.last_pose_time is not None:
            dt = (current_time - self.last_pose_time).nanoseconds / 1e9
        else:
            dt = 0.1  # Défaut
        self.last_pose_time = current_time
        
        if dt <= 0 or dt > 1.0:  # Sécurité
            dt = 0.1
        
        # Erreurs (négatif car on veut aller vers (0,0,0))
        error_x = -msg.x
        error_y = -msg.y  
        error_yaw = -msg.yaw
        
        # Calcul commandes PID
        cmd_x = self.pid_x.compute(error_x, dt)
        cmd_y = self.pid_y.compute(error_y, dt)
        cmd_yaw = self.pid_yaw.compute(error_yaw, dt)
        
        # Publication
        twist = Twist()
        twist.linear.x = cmd_y  # Frontal
        twist.linear.y = cmd_x  # Latéral
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = cmd_yaw
        
        self.cmd_pub.publish(twist)
        
        self.get_logger().debug(
            f'Erreurs: x={error_x:.2f}, y={error_y:.2f}, yaw={np.rad2deg(error_yaw):.1f}° | '
            f'Cmd: vx={cmd_y:.2f}, vy={cmd_x:.2f}, wz={cmd_yaw:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
