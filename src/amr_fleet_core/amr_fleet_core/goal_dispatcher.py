"""
goal_dispatcher.py — Baseline uncoordinated goal dispatch.

This is the STOP-AND-WAIT BASELINE that the 20% task-completion-time
improvement is measured against. It sends NavigateToPose goals to all
robots simultaneously with NO negotiation, NO conflict avoidance, and
NO coordination. Robots rely solely on DWB's local obstacle avoidance
to (try to) avoid each other.

This is implemented FIRST — before any negotiated behavior — because
it's the comparison point. The benchmark harness runs this baseline,
then runs the negotiated version, and compares task-completion times.
"""

from __future__ import annotations

import time
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import String
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

from amr_fleet_core.scenario import ROBOTS


class GoalDispatcherNode(Node):
    """
    Sends NavigateToPose goals to all robots simultaneously.
    No negotiation, no coordination — pure baseline.
    """

    def __init__(self) -> None:
        super().__init__('goal_dispatcher')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self._action_clients = {}
        self._task_start_times = {}
        self._task_end_times = {}

        for robot in ROBOTS:
            client = ActionClient(
                self,
                NavigateToPose,
                f'/{robot.robot_id}/navigate_to_pose',
            )
            self._action_clients[robot.robot_id] = client

        # Check action servers periodically until all are ready, then dispatch
        self.create_timer(2.0, self._dispatch_all_goals)
        self._dispatched = False

        self.get_logger().info(
            f"Goal dispatcher initialized for {len(ROBOTS)} robots (BASELINE mode)"
        )

    def _dispatch_all_goals(self) -> None:
        if self._dispatched:
            return

        # Check if all action servers are ready
        not_ready = [
            robot.robot_id for robot in ROBOTS
            if not self._action_clients[robot.robot_id].server_is_ready()
        ]
        if not_ready:
            self.get_logger().info(
                f"Waiting for action servers: {len(not_ready)} robots not ready yet "
                f"({', '.join(not_ready)})"
            )
            return

        self._dispatched = True
        sim_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            f"All {len(ROBOTS)} NavigateToPose servers ready! Dispatching baseline goals..."
        )

        for robot in ROBOTS:
            client = self._action_clients[robot.robot_id]
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = robot.goal_x
            goal_msg.pose.pose.position.y = robot.goal_y
            goal_msg.pose.pose.orientation.w = 1.0

            self._task_start_times[robot.robot_id] = sim_time

            future = client.send_goal_async(goal_msg)
            future.add_done_callback(
                lambda f, rid=robot.robot_id: self._goal_response_cb(f, rid)
            )

            self.get_logger().info(
                f"[BASELINE] Dispatched goal to {robot.robot_id}: "
                f"({robot.goal_x:.1f}, {robot.goal_y:.1f})"
            )

    def _goal_response_cb(self, future, robot_id: str) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Goal rejected for {robot_id}")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, rid=robot_id: self._goal_result_cb(f, rid)
        )

    def _goal_result_cb(self, future, robot_id: str) -> None:
        sim_time = self.get_clock().now().nanoseconds / 1e9
        self._task_end_times[robot_id] = sim_time
        start = self._task_start_times.get(robot_id, sim_time)
        elapsed = sim_time - start
        self.get_logger().info(
            f"[BASELINE] {robot_id} reached goal in {elapsed:.1f}s"
        )

        # Check if all robots are done
        if len(self._task_end_times) == len(ROBOTS):
            self._log_results()

    def _log_results(self) -> None:
        """Log task completion summary."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("BASELINE TRIAL COMPLETE")
        total_time = 0.0
        for robot in ROBOTS:
            start = self._task_start_times.get(robot.robot_id, 0)
            end = self._task_end_times.get(robot.robot_id, 0)
            elapsed = end - start
            total_time += elapsed
            self.get_logger().info(
                f"  {robot.robot_id}: {elapsed:.1f}s"
            )
        self.get_logger().info(f"  Total: {total_time:.1f}s")
        self.get_logger().info("=" * 60)


def main(args=None):
    if not HAS_ROS2:
        print("ERROR: rclpy not available.")
        return

    rclpy.init(args=args)
    node = GoalDispatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
