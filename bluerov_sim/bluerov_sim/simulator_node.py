#!/usr/bin/env python3
"""
ROS2 simulator node that runs the existing `bluerov_dynamic.BlueROVSimulator`
and exposes a simple command input topic and a `docking_msgs/TrackedObject`
publisher for tracker/testing.
"""
import os
import sys
import importlib.util
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from docking_msgs.msg import TrackedObject

import numpy as np


def load_bluerov_dynamic():
    """Try to load the existing bluerov_dynamic.py as a module from likely locations."""
    candidates = [
        # relative to this file: ../../simulation/bluerov_dynamic.py
        Path(__file__).resolve().parents[2] / 'simulation' / 'bluerov_dynamic.py',
        Path(__file__).resolve().parents[3] / 'simulation' / 'bluerov_dynamic.py',
        Path.cwd() / 'src' / 'simulation' / 'bluerov_dynamic.py',
        Path('/home/clementdunot/Documents/Docking/src/simulation/bluerov_dynamic.py')
    ]
    for p in candidates:
        if p.exists():
            spec = importlib.util.spec_from_file_location('bluerov_dynamic', str(p))
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    raise FileNotFoundError('Could not find bluerov_dynamic.py in expected locations')


class SimulatorNode(Node):
    def __init__(self):
        super().__init__('bluerov_simulator')
        self.get_logger().info('Loading simulator module...')
        mod = load_bluerov_dynamic()
        self.BlueROVSimulator = mod.BlueROVSimulator

        self.sim = self.BlueROVSimulator(X0=0.0, Y0=0.0, theta_0=0.0)

        # Topic to receive manual commands [surge%, sway%, yaw%]
        self.sub_cmd = self.create_subscription(Float32MultiArray, '/simulator/cmd', self.cb_cmd, 10)

        # Publish tracked object
        self.pub_tracked = self.create_publisher(TrackedObject, '/docking/tracking/tracked_object', 10)

        self.timer = self.create_timer(self.sim.sampleTime, self.timer_cb)

    def cb_cmd(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) >= 3:
            self.sim.cmd_surge = float(data[0])
            self.sim.cmd_sway = float(data[1])
            self.sim.cmd_yaw = float(data[2])

    def timer_cb(self):
        # Advance simulation
        self.sim.timer_callback()

        # Build tracked object msg using simulator's sonar model
        z_b, seen = self.sim.sonar_measure(self.sim.target_world, self.sim.eta)

        msg = TrackedObject()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sim'
        msg.is_tracking = bool(seen)
        msg.is_initialized = True

        if seen and z_b is not None:
            x_b, y_b = z_b
            rng = float(np.hypot(x_b, y_b))
            brg = float(np.arctan2(y_b, x_b))

            # center in world coords
            msg.center_x = float(self.sim.target_world[0])
            msg.center_y = float(self.sim.target_world[1])
            msg.range = rng
            msg.bearing = brg
            msg.confidence = 1.0
        else:
            msg.center_x = 0.0
            msg.center_y = 0.0
            msg.range = 0.0
            msg.bearing = 0.0
            msg.confidence = 0.0

        self.pub_tracked.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
