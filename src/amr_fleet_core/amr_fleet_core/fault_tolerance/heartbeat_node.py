"""
heartbeat_node.py — ROS2 thin wrapper for heartbeat publishing/monitoring.
"""

from __future__ import annotations

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

from amr_fleet_core.fault_tolerance.heartbeat import HeartbeatMonitor
from amr_fleet_core.scenario import ROBOTS


class HeartbeatNode(Node):
    def __init__(self, robot_id: str) -> None:
        super().__init__(f'{robot_id}_heartbeat')
        self.robot_id = robot_id
        self.monitor = HeartbeatMonitor(
            heartbeat_interval=1.0,
            timeout_factor=3,
        )

        self.pub = self.create_publisher(String, 'heartbeat', 10)
        for r in ROBOTS:
            if r.robot_id != robot_id:
                self.create_subscription(
                    String,
                    f'/{r.robot_id}/heartbeat',
                    lambda msg, rid=r.robot_id: self._peer_cb(rid, msg),
                    10,
                )
        self.create_timer(1.0, self._publish)
        self.create_timer(2.0, self._check)

    def _publish(self):
        msg = String()
        msg.data = self.robot_id
        self.pub.publish(msg)

    def _peer_cb(self, rid, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        self.monitor.on_heartbeat(rid, t)

    def _check(self):
        t = self.get_clock().now().nanoseconds / 1e9
        dead = self.monitor.check_timeouts(t)
        for d in dead:
            self.get_logger().warn(f"Peer {d} timed out")


def main(args=None):
    if not HAS_ROS2:
        return
    rclpy.init(args=args)
    import sys
    rid = sys.argv[1] if len(sys.argv) > 1 else 'amr_1'
    node = HeartbeatNode(rid)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
