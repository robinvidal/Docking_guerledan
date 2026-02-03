#!/usr/bin/env python3
"""
Simple controller node that subscribes to `/docking/tracking/tracked_object`
and publishes manual-style commands on `/simulator/cmd` as a Float32MultiArray:
[surge_percent, sway_percent, yaw_percent].

Behaviour:
- If no target detected -> rotate in place to search
- If detected -> proportional control on range (forward) and bearing (yaw)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from docking_msgs.msg import TrackedObject

import numpy as np


class SimpleApproachController(Node):
    def __init__(self):
        super().__init__('bluerov_simple_controller')
        self.sub = self.create_subscription(TrackedObject, '/docking/tracking/tracked_object', self.cb_tracked, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/simulator/cmd', 10)

        # Controller params
        self.desired_range = 0.8  # meters
        self.kp_range = 25.0      # percent per meter error (tune)
        self.kp_bearing_deg = 1.5 # percent per degree of bearing
        self.search_yaw_percent = 30.0

        self.last_msg = None

    def cb_tracked(self, msg: TrackedObject):
        self.last_msg = msg
        cmd = Float32MultiArray()
        # default zero
        surge = 0.0
        sway = 0.0
        yaw = 0.0

        if not msg.is_tracking or msg.confidence < 0.1:
            # Search behaviour: rotate slowly
            yaw = self.search_yaw_percent
        else:
            rng = float(msg.range)
            brg = float(msg.bearing)  # radians

            # Range control: go forward if farther than desired
            range_err = rng - self.desired_range
            surge = np.clip(self.kp_range * range_err, -100.0, 100.0)

            # Bearing control: rotate to reduce bearing
            yaw = np.clip(self.kp_bearing_deg * np.degrees(brg), -100.0, 100.0)

            # If close enough, stop
            if rng <= self.desired_range + 0.05:
                surge = 0.0
                yaw = 0.0

        cmd.data = [float(surge), float(sway), float(yaw)]
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleApproachController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
