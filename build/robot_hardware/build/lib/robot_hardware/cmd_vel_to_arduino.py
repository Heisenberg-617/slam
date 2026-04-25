#!/usr/bin/env python3
"""
cmd_vel_to_arduino - Bidirectional bridge between ROS2 Nav2 and Arduino.

This node adapts to the EXISTING Arduino motor controller protocol:

OUTBOUND (ROS2 → Arduino):
    Subscribes to /cmd_vel (geometry_msgs/Twist)
    Sends: "M<linear_x>:<angular_z>\n"
    The Arduino already handles inverse kinematics, PID, and ramping.

INBOUND (Arduino → ROS2):
    Reads encoder tick data:  "E:<left_ticks>,<right_ticks>\n"
    Publishes to /encoder_ticks (std_msgs/Int32MultiArray)
    
    Also reads battery/debug data: "RPM:...,Speed:...,Steering:...,Volt:...,Bat%:..."
    Publishes battery percentage to /battery_percent (std_msgs/Int32)

The Arduino ALREADY does:
    - Differential-drive inverse kinematics (V,W → wheel speeds)
    - PID control on each motor
    - Velocity ramping (smooth acceleration/deceleration)
    - Battery monitoring with emergency stop
    - Low-pass filtering on encoder RPM

Safety watchdog: if no /cmd_vel received for timeout period,
sends M0:0 (stop) to the Arduino.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Int32
import serial
import time
import threading


class CmdVelToArduino(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_arduino')

        # ── Declare parameters ──
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('encoder_publish_topic', '/encoder_ticks')

        # ── Get parameters ──
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.encoder_topic = self.get_parameter('encoder_publish_topic').value

        # ── State ──
        self.last_cmd_vel_time = time.time()
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.serial_conn = None
        self.running = True
        self.serial_lock = threading.Lock()

        # ── Open serial connection ──
        self._connect_serial()

        # ── Subscribe to /cmd_vel ──
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ── Publishers ──
        self.encoder_pub = self.create_publisher(
            Int32MultiArray,
            self.encoder_topic,
            10
        )
        self.battery_pub = self.create_publisher(
            Int32,
            '/battery_percent',
            10
        )

        # ── Timer for sending commands to Arduino at fixed rate ──
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # ── Serial reader thread ──
        self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.serial_thread.start()

        self.get_logger().info(
            f'cmd_vel_to_arduino started:\n'
            f'  Serial: {self.serial_port} @ {self.baud_rate} baud\n'
            f'  Protocol: M<V>:<W> (Arduino does PID + kinematics)\n'
            f'  Timeout: {self.cmd_vel_timeout}s\n'
            f'  Encoder topic: {self.encoder_topic}'
        )

    def _connect_serial(self):
        """Attempt to open the serial connection to the Arduino."""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            # Wait for Arduino to reset after serial connection
            time.sleep(2.0)
            # Flush any startup messages
            if self.serial_conn.in_waiting:
                self.serial_conn.read(self.serial_conn.in_waiting)
            self.get_logger().info(f'Serial connected to {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(
                f'Failed to open serial port {self.serial_port}: {e}\n'
                f'Will retry on next timer tick...'
            )
            self.serial_conn = None

    def _serial_reader(self):
        """
        Background thread that reads lines from Arduino serial.
        
        Parses two types of messages:
          - "E:<left>,<right>"      → encoder ticks → /encoder_ticks
          - "RPM:...,Bat%:<pct>"    → battery info  → /battery_percent
        """
        while self.running:
            if self.serial_conn is None or not self.serial_conn.is_open:
                time.sleep(1.0)
                continue

            try:
                with self.serial_lock:
                    if self.serial_conn.in_waiting > 0:
                        line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    else:
                        line = None

                if line is None:
                    time.sleep(0.002)  # 2ms sleep when no data
                    continue

                if line.startswith('E:'):
                    self._parse_encoder_data(line)
                elif 'Bat%:' in line:
                    self._parse_battery_data(line)
                elif 'ALERTE' in line:
                    self.get_logger().error(f'Arduino: {line}')

            except serial.SerialException as e:
                self.get_logger().warn(f'Serial read error: {e}')
                self.serial_conn = None
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().debug(f'Serial reader error: {e}')
                time.sleep(0.01)

    def _parse_encoder_data(self, line: str):
        """Parse "E:<left_ticks>,<right_ticks>" and publish."""
        try:
            data = line[2:]  # Strip "E:" prefix
            parts = data.split(',')
            if len(parts) == 2:
                left_ticks = int(parts[0])
                right_ticks = int(parts[1])
                msg = Int32MultiArray()
                msg.data = [left_ticks, right_ticks]
                self.encoder_pub.publish(msg)
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f'Failed to parse encoder data "{line}": {e}')

    def _parse_battery_data(self, line: str):
        """Parse "RPM:...,Bat%:<pct>" and publish battery percentage."""
        try:
            bat_idx = line.index('Bat%:')
            pct_str = line[bat_idx + 5:].strip()
            pct = int(pct_str)
            msg = Int32()
            msg.data = pct
            self.battery_pub.publish(msg)
        except (ValueError, IndexError):
            pass

    def cmd_vel_callback(self, msg: Twist):
        """Store the latest velocity command and update timestamp."""
        self.current_linear = msg.linear.x
        self.current_angular = msg.angular.z
        self.last_cmd_vel_time = time.time()

    def timer_callback(self):
        """Periodically send velocity command to Arduino."""

        # ── Watchdog: stop if no cmd_vel received recently ──
        elapsed = time.time() - self.last_cmd_vel_time
        if elapsed > self.cmd_vel_timeout:
            self.current_linear = 0.0
            self.current_angular = 0.0

        # ── Send to Arduino using its native M<V>:<W> protocol ──
        # The Arduino handles: inverse kinematics, PID, ramping, battery safety
        self._send_command(self.current_linear, self.current_angular)

    def _send_command(self, linear_vel: float, angular_vel: float):
        """Send velocity command to Arduino in M<V>:<W> format."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self._connect_serial()
            if self.serial_conn is None:
                return

        # Format: M<linear_velocity>:<angular_velocity>\n
        # Arduino parseCommand() expects uppercase M, float values
        command = f'M{linear_vel:.4f}:{angular_vel:.4f}\n'
        try:
            with self.serial_lock:
                self.serial_conn.write(command.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write error: {e}')
            self.serial_conn = None

    def destroy_node(self):
        """Send stop command and close serial on shutdown."""
        self.get_logger().info('Shutting down — sending stop command to Arduino')
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(b'M0:0\n')
                time.sleep(0.1)
                self.serial_conn.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToArduino()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
