#!/usr/bin/env python3
"""
encoder_odometry - Compute odometry from wheel encoder ticks and publish
                   /odom topic + odom→base_footprint TF transform.

Subscribes to encoder ticks (std_msgs/Int32MultiArray with [left, right])
and computes differential-drive forward kinematics to produce the robot's
pose (x, y, θ) and velocity.

This replaces the Gazebo diff_drive plugin's odometry for the real robot.

Frame convention:
    map → odom → base_footprint → base_link → lidar_link
    
    This node publishes: odom → base_footprint
    AMCL publishes:      map → odom
    robot_state_publisher publishes: base_footprint → base_link → lidar_link
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math
import time


class EncoderOdometry(Node):
    def __init__(self):
        super().__init__('encoder_odometry')

        # ── Declare parameters ──
        self.declare_parameter('encoder_topic', '/encoder_ticks')
        self.declare_parameter('wheel_separation', 0.488)      # meters (from URDF)
        self.declare_parameter('wheel_radius', 0.10)           # meters (from URDF)
        self.declare_parameter('ticks_per_revolution', 8384)   # 64 CPR * 131:1 gear ratio
        self.declare_parameter('publish_rate', 64.0)           # Hz (matches Arduino E: output rate)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # ── Get parameters ──
        self.encoder_topic = self.get_parameter('encoder_topic').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('ticks_per_revolution').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # ── Derived constants ──
        # Distance per tick = (2 * pi * wheel_radius) / ticks_per_revolution
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

        # ── Odometry state ──
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0

        # ── Encoder state ──
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_time = self.get_clock().now()

        # ── Publishers ──
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)

        # ── TF Broadcaster ──
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Subscriber ──
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            self.encoder_topic,
            self.encoder_callback,
            10
        )

        self.get_logger().info(
            f'encoder_odometry started:\n'
            f'  Encoder topic: {self.encoder_topic}\n'
            f'  Wheel separation: {self.wheel_separation}m\n'
            f'  Wheel radius: {self.wheel_radius}m\n'
            f'  Ticks/rev: {self.ticks_per_rev}\n'
            f'  Meters/tick: {self.meters_per_tick:.6f}\n'
            f'  Frames: {self.odom_frame} → {self.base_frame}'
        )

    def encoder_callback(self, msg: Int32MultiArray):
        """
        Process encoder ticks and update odometry.
        
        Expected message data: [left_ticks, right_ticks]
        These are CUMULATIVE tick counts (not deltas).
        """
        if len(msg.data) < 2:
            self.get_logger().warn('Encoder message has < 2 values, ignoring')
            return

        left_ticks = msg.data[0]
        right_ticks = msg.data[1]
        current_time = self.get_clock().now()

        # ── First reading: initialize and return ──
        if self.last_left_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_time = current_time
            return

        # ── Compute tick deltas ──
        delta_left = left_ticks - self.last_left_ticks
        delta_right = right_ticks - self.last_right_ticks

        # ── Compute time delta ──
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        # ── Convert ticks to distance traveled by each wheel ──
        dist_left = delta_left * self.meters_per_tick
        dist_right = delta_right * self.meters_per_tick

        # ── Differential drive forward kinematics ──
        # Linear distance traveled by the robot center
        dist_center = (dist_left + dist_right) / 2.0
        # Angular rotation of the robot
        delta_theta = (dist_right - dist_left) / self.wheel_separation

        # ── Update pose ──
        # Use midpoint integration for better accuracy
        if abs(delta_theta) < 1e-6:
            # Straight-line motion
            self.x += dist_center * math.cos(self.theta)
            self.y += dist_center * math.sin(self.theta)
        else:
            # Arc motion
            radius = dist_center / delta_theta
            self.x += radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y -= radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))

        self.theta += delta_theta
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # ── Compute velocities ──
        self.vx = dist_center / dt
        self.vth = delta_theta / dt

        # ── Create quaternion from yaw ──
        q = self._yaw_to_quaternion(self.theta)

        # ── Publish TF: odom → base_footprint ──
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # ── Publish Odometry message ──
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = q

        # Pose covariance (6x6, row-major)
        # [x, y, z, roll, pitch, yaw]
        odom_msg.pose.covariance[0] = 0.01   # x variance
        odom_msg.pose.covariance[7] = 0.01   # y variance
        odom_msg.pose.covariance[14] = 1e6   # z variance (large = don't trust)
        odom_msg.pose.covariance[21] = 1e6   # roll variance
        odom_msg.pose.covariance[28] = 1e6   # pitch variance
        odom_msg.pose.covariance[35] = 0.03  # yaw variance

        # Velocity
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.vth

        # Twist covariance
        odom_msg.twist.covariance[0] = 0.01   # vx variance
        odom_msg.twist.covariance[7] = 0.01   # vy variance
        odom_msg.twist.covariance[14] = 1e6   # vz variance
        odom_msg.twist.covariance[21] = 1e6   # roll rate variance
        odom_msg.twist.covariance[28] = 1e6   # pitch rate variance
        odom_msg.twist.covariance[35] = 0.03  # yaw rate variance

        self.odom_pub.publish(odom_msg)

        # ── Save state for next iteration ──
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time = current_time

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """Convert a yaw angle (radians) to a Quaternion message."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
