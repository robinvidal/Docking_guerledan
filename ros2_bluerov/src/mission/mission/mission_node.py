"""
Nœud de gestion de la machine d'états de la mission de docking.
"""

import rclpy
from rclpy.node import Node
from docking_msgs.msg import State, PoseRelative, Borders
from std_msgs.msg import Bool


class MissionNode(Node):
    """
    Machine d'états pour la mission de docking autonome.
    
    États:
    - IDLE: Attente commande de démarrage
    - LOCK_ON: Acquisition de la cage
    - APPROACH: Approche vers la cage
    - DOCKING: Phase finale d'amarrage
    - DOCKED: Amarré avec succès
    - RECOVERY: Récupération après perte de tracking
    - ABORT: Mission annulée
    """
    
    def __init__(self):
        super().__init__('mission_node')
        
        # Paramètres
        self.declare_parameter('lock_on_timeout', 10.0)  # s
        self.declare_parameter('approach_distance', 1.0)  # m
        self.declare_parameter('docking_distance', 0.3)  # m
        self.declare_parameter('alignment_threshold', 0.1)  # rad (~6°)
        
        # État actuel
        self.current_state = State.IDLE
        self.state_start_time = None
        
        # Données des capteurs
        self.cage_detected = False
        self.pose_valid = False
        self.current_pose = None
        self.abort_requested = False
        
        # Subscriptions
        self.borders_sub = self.create_subscription(
            Borders,
            '/docking/tracking/borders',
            self.borders_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseRelative,
            '/docking/localisation/pose',
            self.pose_callback,
            10
        )
        
        self.abort_sub = self.create_subscription(
            Bool,
            '/docking/mission/abort',
            self.abort_callback,
            10
        )
        
        # Publisher
        self.state_pub = self.create_publisher(State, '/docking/mission/state', 10)
        
        # Timer pour machine d'états (10 Hz)
        self.timer = self.create_timer(0.1, self.state_machine_update)
        
        self.get_logger().info('Mission node démarré (état: IDLE)')
    
    def borders_callback(self, msg: Borders):
        """Met à jour la détection de cage."""
        self.cage_detected = msg.is_valid
    
    def pose_callback(self, msg: PoseRelative):
        """Met à jour la pose relative."""
        self.pose_valid = msg.is_valid
        if msg.is_valid:
            self.current_pose = msg
    
    def abort_callback(self, msg: Bool):
        """Gère demande d'annulation."""
        if msg.data:
            self.get_logger().warn('Abort demandé par opérateur')
            self.abort_requested = True
    
    def transition_to(self, new_state: int, reason: str = ""):
        """Transition vers un nouvel état."""
        if new_state != self.current_state:
            old_state_name = self.get_state_name(self.current_state)
            new_state_name = self.get_state_name(new_state)
            self.get_logger().info(f'Transition: {old_state_name} -> {new_state_name} ({reason})')
            
            self.current_state = new_state
            self.state_start_time = self.get_clock().now()
    
    def get_state_name(self, state: int) -> str:
        """Retourne le nom d'un état."""
        names = {
            State.IDLE: 'IDLE',
            State.LOCK_ON: 'LOCK_ON',
            State.APPROACH: 'APPROACH',
            State.DOCKING: 'DOCKING',
            State.DOCKED: 'DOCKED',
            State.RECOVERY: 'RECOVERY',
            State.ABORT: 'ABORT'
        }
        return names.get(state, f'UNKNOWN({state})')
    
    def state_machine_update(self):
        """Mise à jour de la machine d'états."""
        # Vérification abort
        if self.abort_requested and self.current_state != State.ABORT:
            self.transition_to(State.ABORT, "abort demandé")
            self.publish_state()
            return
        
        # Logique de transition selon état actuel
        if self.current_state == State.IDLE:
            # TODO: Attendre commande de démarrage (service ou topic)
            # Pour l'instant, démarrer automatiquement si cage détectée
            if self.cage_detected:
                self.transition_to(State.LOCK_ON, "cage détectée")
        
        elif self.current_state == State.LOCK_ON:
            # Acquisition de la cage
            if self.pose_valid:
                self.transition_to(State.APPROACH, "pose valide acquise")
            else:
                # Timeout
                elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
                timeout = self.get_parameter('lock_on_timeout').value
                if elapsed > timeout:
                    self.transition_to(State.RECOVERY, "timeout acquisition")
        
        elif self.current_state == State.APPROACH:
            # Approche vers la cage
            if not self.pose_valid:
                self.transition_to(State.RECOVERY, "perte de pose")
            elif self.current_pose is not None:
                # Vérifier distance
                distance = self.current_pose.y
                docking_dist = self.get_parameter('docking_distance').value
                approach_dist = self.get_parameter('approach_distance').value
                
                if distance <= docking_dist:
                    # Assez proche pour docking final
                    alignment_ok = abs(self.current_pose.x) < 0.2 and abs(self.current_pose.yaw) < self.get_parameter('alignment_threshold').value
                    if alignment_ok:
                        self.transition_to(State.DOCKING, "distance et alignement OK")
                    else:
                        self.get_logger().warn(f'Proche mais mal aligné: x={self.current_pose.x:.2f}, yaw={self.current_pose.yaw:.2f}')
        
        elif self.current_state == State.DOCKING:
            # Phase finale
            if not self.pose_valid:
                self.transition_to(State.RECOVERY, "perte de pose en docking")
            elif self.current_pose is not None:
                # TODO: Détecter contact physique (capteur de force, IMU)
                # Pour l'instant, valider si très proche
                if self.current_pose.y < 0.1:
                    self.transition_to(State.DOCKED, "contact détecté")
        
        elif self.current_state == State.DOCKED:
            # Amarré, mission terminée
            pass
        
        elif self.current_state == State.RECOVERY:
            # Tentative de récupération
            if self.pose_valid:
                self.transition_to(State.APPROACH, "pose récupérée")
            else:
                # Timeout recovery -> abort
                elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
                if elapsed > 30.0:  # 30s max
                    self.transition_to(State.ABORT, "recovery timeout")
        
        elif self.current_state == State.ABORT:
            # Mission annulée, rien à faire
            pass
        
        # Publication état
        self.publish_state()
    
    def publish_state(self):
        """Publie l'état actuel de la mission."""
        msg = State()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.current_state = self.current_state
        
        msg.cage_detected = self.cage_detected
        msg.pose_valid = self.pose_valid
        
        # Alignement
        if self.current_pose is not None:
            threshold = self.get_parameter('alignment_threshold').value
            msg.alignment_ok = abs(self.current_pose.x) < 0.2 and abs(self.current_pose.yaw) < threshold
        else:
            msg.alignment_ok = False
        
        # TODO: Contact physique
        msg.contact_detected = False
        
        # Progression (simplifiée)
        progress_map = {
            State.IDLE: 0.0,
            State.LOCK_ON: 0.2,
            State.APPROACH: 0.5,
            State.DOCKING: 0.8,
            State.DOCKED: 1.0,
            State.RECOVERY: 0.3,
            State.ABORT: 0.0
        }
        msg.progress = progress_map.get(self.current_state, 0.0)
        
        msg.status_message = self.get_state_name(self.current_state)
        msg.abort_requested = self.abort_requested
        
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
