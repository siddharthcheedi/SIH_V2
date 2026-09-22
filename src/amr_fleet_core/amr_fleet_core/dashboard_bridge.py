"""
dashboard_bridge.py — Telemetry bridge for the observer-only dashboard.

Subscribes to robot telemetry topics (position, battery, task, explainability
log per robot) and forwards them to the dashboard via a WebSocket.

OBSERVER-ONLY: This node has NO publishers to any robot command topic.
It structurally cannot issue commands. Killing it has zero effect on fleet
behavior.
"""

from __future__ import annotations

import json
from typing import Dict

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Float32
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

from amr_fleet_core.scenario import ROBOTS


class DashboardBridgeNode(Node):
    """
    Collects telemetry from all robots and serves it to the dashboard.

    STRUCTURAL GUARANTEE:
    - This node creates only Subscribers, never Publishers.
    - It has no ActionClients, no service clients.
    - Killing this process has zero effect on fleet behavior.
    """

    def __init__(self) -> None:
        super().__init__('dashboard_bridge')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self._robot_data: Dict[str, dict] = {}

        for robot in ROBOTS:
            rid = robot.robot_id
            self._robot_data[rid] = {
                'robot_id': rid,
                'color': robot.color_name,
                'x': robot.spawn_x,
                'y': robot.spawn_y,
                'battery': 1.0,
                'status': 'initializing',
                'explain': '',
            }

            # Subscribe to each robot's telemetry
            self.create_subscription(
                Odometry,
                f'/{rid}/odom',
                lambda msg, r=rid: self._odom_cb(r, msg),
                10,
            )
            self.create_subscription(
                String,
                f'/{rid}/explain',
                lambda msg, r=rid: self._explain_cb(r, msg),
                10,
            )
            self.create_subscription(
                Float32,
                f'/{rid}/battery',
                lambda msg, r=rid: self._battery_cb(r, msg),
                10,
            )

        # Periodic data push
        self.create_timer(0.5, self._push_data)

        self.get_logger().info(
            f"Dashboard bridge started for {len(ROBOTS)} robots "
            f"(observer-only, no command capability)"
        )

    def _odom_cb(self, robot_id: str, msg: Odometry) -> None:
        self._robot_data[robot_id]['x'] = msg.pose.pose.position.x
        self._robot_data[robot_id]['y'] = msg.pose.pose.position.y

    def _explain_cb(self, robot_id: str, msg: String) -> None:
        self._robot_data[robot_id]['explain'] = msg.data

    def _battery_cb(self, robot_id: str, msg: Float32) -> None:
        self._robot_data[robot_id]['battery'] = msg.data

    def _push_data(self) -> None:
        """Push aggregated data to the FastAPI dashboard."""
        import urllib.request
        try:
            payload = json.dumps({'robots': self._robot_data}).encode('utf-8')
            req = urllib.request.Request(
                'http://127.0.0.1:8000/api/telemetry',
                data=payload,
                headers={'Content-Type': 'application/json'},
                method='POST',
            )
            with urllib.request.urlopen(req, timeout=0.2):
                pass
        except Exception:
            # Dashboard server may not be running; non-blocking
            pass

    def get_all_data(self) -> dict:
        """Get current telemetry for all robots (used by dashboard)."""
        return dict(self._robot_data)


def main(args=None):
    if not HAS_ROS2:
        print("ERROR: rclpy not available.")
        return

    rclpy.init(args=args)
    node = DashboardBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
