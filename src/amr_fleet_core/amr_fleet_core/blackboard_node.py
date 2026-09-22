"""
blackboard_node.py — Lightweight shared blackboard (information only).

Publishes global map state (blocked aisles, open task queue) that any
robot can read. This is explicitly NOT a control authority — it cannot
arbitrate conflicts, make reservation decisions, or issue any commands.

STRUCTURAL GUARANTEE:
- No reservation or priority logic exists in this module.
- No subscribers to any robot's intent or negotiation topics.
- The fleet must keep operating if this process is killed (degraded
  local-only mode). This is demonstrated with an actual kill-the-process
  test — not asserted.

The blackboard is a convenience for task distribution and map updates,
not a coordination mechanism.
"""

from __future__ import annotations

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    import json
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

from amr_fleet_core.scenario import ROBOTS


class BlackboardNode(Node):
    """
    Map/task-queue publisher — read-only by design.

    What it publishes:
      - /blackboard/map_updates: aisle blockage status (JSON)
      - /blackboard/task_queue: available tasks for CNP bidding (JSON)

    What it does NOT do:
      - Subscribe to any robot's /intent, /heartbeat, or /cmd_vel topics
      - Make any reservation, priority, or collision-avoidance decisions
      - Send any commands to any robot

    By construction, killing this node cannot affect navigation — robots
    negotiate directly with each other via peer broadcasts.
    """

    def __init__(self) -> None:
        super().__init__('blackboard')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # Publishers only — no subscribers to robot topics
        self.map_pub = self.create_publisher(
            String, '/blackboard/map_updates', 10
        )
        self.task_pub = self.create_publisher(
            String, '/blackboard/task_queue', 10
        )

        self._blocked_aisles = []
        self._task_queue = []

        # Periodic updates
        self.create_timer(2.0, self._publish_state)

        self.get_logger().info(
            "Blackboard node started (information only, no control authority)"
        )

    def _publish_state(self) -> None:
        # Map updates
        map_msg = String()
        map_msg.data = json.dumps({
            'blocked_aisles': self._blocked_aisles,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        })
        self.map_pub.publish(map_msg)

        # Task queue
        task_msg = String()
        task_msg.data = json.dumps({
            'tasks': self._task_queue,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        })
        self.task_pub.publish(task_msg)


def main(args=None):
    if not HAS_ROS2:
        print("ERROR: rclpy not available.")
        return

    rclpy.init(args=args)
    node = BlackboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
