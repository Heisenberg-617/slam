#!/usr/bin/env python3
"""
nav_goal_receiver — Bridges /navigation_goal (JSON from Railway UI via rosbridge)
to Nav2's NavigateToPose action server.

Data flow:
  Railway Streamlit → websocket-client → Cloudflare Tunnel
  → rosbridge_server :9090 → /navigation_goal (std_msgs/String)
  → this node → NavigateToPose action → Nav2

Expected JSON payload (from NavigationService._dispatch_navigation_command):
{
  "destination":  "Cafétéria",
  "matched_name": "Cafétéria",
  "coordinates":  { "latitude": <map_x>,  "longitude": <map_y> },
  "building":     "Bâtiment Emines",
  "floor":        "RDC"
}

COORDINATE NOTE:
  locations.json currently uses "latitude"/"longitude" as placeholder field names.
  For indoor SLAM navigation these should contain MAP-FRAME x/y values (metres,
  relative to the map origin).  To find a location's map coordinates:
    1. Load the SLAM map in RViz (ros2 launch robot_hardware real_robot_nav.launch.py)
    2. Enable the "Publish Point" tool in RViz toolbar
    3. Click on each room/location on the map → read x/y from the terminal
    4. Update data/locations.json in the chatbot repo with those values
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavGoalReceiver(Node):
    def __init__(self):
        super().__init__('nav_goal_receiver')

        self.declare_parameter('map_frame',    'map')
        self.declare_parameter('arrival_yaw',  0.0)   # robot heading on arrival (radians)

        self.map_frame   = self.get_parameter('map_frame').value
        self.arrival_yaw = self.get_parameter('arrival_yaw').value

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._sub = self.create_subscription(
            String,
            '/navigation_goal',
            self._on_goal_received,
            10
        )

        self.get_logger().info(
            f'nav_goal_receiver ready.\n'
            f'  Listening : /navigation_goal (std_msgs/String JSON)\n'
            f'  Action    : navigate_to_pose (Nav2)\n'
            f'  Map frame : {self.map_frame}'
        )

    # ──────────────────────────────────────────────────────────────
    def _on_goal_received(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Cannot parse /navigation_goal JSON: {exc}\nRaw: {msg.data}')
            return

        destination = data.get('destination', '<unknown>')
        coords      = data.get('coordinates', {})

        # "latitude" / "longitude" hold map-frame x / y until locations.json
        # is updated with real indoor coordinates.
        x = float(coords.get('latitude',  0.0))
        y = float(coords.get('longitude', 0.0))

        if x == 0.0 and y == 0.0:
            self.get_logger().warn(
                f'Destination "{destination}" has coordinates (0, 0). '
                'Update locations.json with real map-frame x/y values.'
            )

        self.get_logger().info(
            f'Received goal: "{destination}"  '
            f'[{data.get("building","?")} / {data.get("floor","?")}]  '
            f'→ map ({x:.3f}, {y:.3f})'
        )

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'navigate_to_pose action server unavailable after 5 s — is Nav2 running?'
            )
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose(x, y, self.arrival_yaw)

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_feedback
        )
        send_future.add_done_callback(
            lambda f: self._on_goal_response(f, destination)
        )

    # ──────────────────────────────────────────────────────────────
    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    # ──────────────────────────────────────────────────────────────
    def _on_goal_response(self, future, destination: str):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'Goal "{destination}" was REJECTED by Nav2.')
            return
        self.get_logger().info(f'Goal "{destination}" ACCEPTED — robot navigating.')
        handle.get_result_async().add_done_callback(
            lambda f: self._on_result(f, destination)
        )

    def _on_result(self, future, destination: str):
        status = future.result().status
        label  = {4: 'SUCCEEDED', 5: 'CANCELLED', 6: 'ABORTED'}.get(status, str(status))
        self.get_logger().info(f'Navigation to "{destination}": {label}')

    def _on_feedback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f'  Distance remaining: {dist:.2f} m',
            throttle_duration_sec=3.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
