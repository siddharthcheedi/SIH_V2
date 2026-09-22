"""
task_manager_node.py — ROS2 node for decentralized task management via CNP.
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

from amr_fleet_core.task_allocation.cnp import TaskAllocator, TaskAnnouncement, Bid
from amr_fleet_core.task_allocation.rl_policy import TabularQPolicy, make_state
from amr_fleet_core.scenario import ROBOTS


class TaskManagerNode(Node):
    def __init__(self, robot_id: str) -> None:
        super().__init__(f'{robot_id}_task_manager')
        self.robot_id = robot_id
        self.allocator = TaskAllocator()
        self.rl_policy = TabularQPolicy(seed=hash(robot_id) % 2**31)

        self.announcement_pub = self.create_publisher(
            String, '/task_announcements', 10
        )
        self.bid_pub = self.create_publisher(String, '/task_bids', 10)
        self.award_pub = self.create_publisher(String, '/task_awards', 10)

        self.create_subscription(
            String, '/task_announcements', self._on_announcement, 10
        )
        self.create_subscription(String, '/task_bids', self._on_bid, 10)
        self.create_subscription(String, '/task_awards', self._on_award, 10)

        self.get_logger().info(f"Task manager for {robot_id} initialized")

    def _on_announcement(self, msg):
        try:
            data = json.loads(msg.data)
            task = TaskAnnouncement(**data)
            self.allocator.announce_task(task)
        except Exception as e:
            self.get_logger().warn(f"Bad announcement: {e}")

    def _on_bid(self, msg):
        try:
            data = json.loads(msg.data)
            bid = Bid(**data)
            self.allocator.submit_bid(bid)
        except Exception as e:
            self.get_logger().warn(f"Bad bid: {e}")

    def _on_award(self, msg):
        pass  # Handle task award notifications


def main(args=None):
    if not HAS_ROS2:
        return
    rclpy.init(args=args)
    import sys
    rid = sys.argv[1] if len(sys.argv) > 1 else 'amr_1'
    node = TaskManagerNode(rid)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
