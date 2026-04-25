#!/usr/bin/env python3
"""
fake_hardware - Simulates the Arduino hardware for testing Nav2 without real robot.

Publishes:
  /encoder_ticks  — Fake encoder ticks (cumulative, changes when /cmd_vel received)
  /scan           — Fake laser scan (empty, but valid — enough for AMCL to start)

Subscribes:
  /cmd_vel        — Reads velocity commands and simulates encoder motion

This lets you verify the full Nav2 pipeline works:
  - Map loads in RViz
  - AMCL localizes 
  - You can set a Nav Goal
  - Path gets planned and drawn
  - Controller sends /cmd_vel
  - Odometry updates (via encoder_odometry node reading our fake ticks)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan
import math
import time


class FakeHardware(Node):
    def __init__(self):
        super().__init__('fake_hardware')

        self.declare_parameter('wheel_separation', 0.488)
        self.declare_parameter('wheel_radius', 0.10)
        self.declare_parameter('ticks_per_revolution', 8384)

        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('ticks_per_revolution').value
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

        # Simulated cumulative ticks
        self.cum_left = 0
        self.cum_right = 0
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Subscribe to /cmd_vel
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        # Publish encoder ticks at 64Hz (like Arduino)
        self.enc_pub = self.create_publisher(Int32MultiArray, '/encoder_ticks', 10)
        self.enc_timer = self.create_timer(1.0 / 64.0, self.publish_encoder)

        # Publish fake laser scan at 10Hz
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.scan_timer = self.create_timer(0.1, self.publish_scan)

        self.get_logger().info(
            '====================================\n'
            '  FAKE HARDWARE MODE (no Arduino)\n'
            '  Publishing fake /encoder_ticks + /scan\n'
            '  Use this to TEST Nav2 pipeline\n'
            '===================================='
        )

    def cmd_vel_cb(self, msg: Twist):
        self.current_linear = msg.linear.x
        self.current_angular = msg.angular.z

    def publish_encoder(self):
        """Simulate encoder ticks based on current velocity."""
        dt = 1.0 / 64.0  # Timer period

        # Inverse kinematics: V, W → wheel velocities
        v_left = self.current_linear - (self.current_angular * self.wheel_separation / 2.0)
        v_right = self.current_linear + (self.current_angular * self.wheel_separation / 2.0)

        # Distance per tick period
        dist_left = v_left * dt
        dist_right = v_right * dt

        # Convert to ticks
        delta_left = int(dist_left / self.meters_per_tick)
        delta_right = int(dist_right / self.meters_per_tick)

        self.cum_left += delta_left
        self.cum_right += delta_right

        msg = Int32MultiArray()
        msg.data = [self.cum_left, self.cum_right]
        self.enc_pub.publish(msg)

    def publish_scan(self):
        """Publish a valid-but-empty scan so AMCL can start."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / 450.0
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.5
        scan.range_max = 12.0
        # All ranges at max (no obstacles — open space)
        scan.ranges = [12.0] * 450
        scan.intensities = [100.0] * 450
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = FakeHardware()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
