"""
global_planner_node.py — ROS2 node wrapping custom A* + spline smoother.

Provides ComputePathToPose action server interface, replacing NavfnPlanner.
"""

from __future__ import annotations

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionServer
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from nav2_msgs.action import ComputePathToPose
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

from amr_fleet_core.planners.astar import OccupancyGrid, GridCell, astar
from amr_fleet_core.planners.spline_smoother import smooth_path


class GlobalPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('custom_global_planner')
        self._grid = None  # Loaded from map

        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            self._compute_path_cb,
        )
        self.get_logger().info("Custom A* + spline global planner ready")

    def _compute_path_cb(self, goal_handle):
        """Compute path using A* + spline smoothing."""
        request = goal_handle.request

        start = GridCell(row=0, col=0)  # Would be computed from current pose
        goal = GridCell(row=0, col=0)   # Would be computed from goal pose

        if self._grid is not None:
            start = self._grid.world_to_grid(
                request.start.pose.position.x,
                request.start.pose.position.y,
            )
            goal = self._grid.world_to_grid(
                request.goal.pose.position.x,
                request.goal.pose.position.y,
            )

            grid_path = astar(self._grid, start, goal)
            if grid_path is None:
                goal_handle.abort()
                return ComputePathToPose.Result()

            # Convert grid path to world coordinates
            waypoints = [self._grid.grid_to_world(cell) for cell in grid_path]

            # Smooth with cubic spline
            smooth_waypoints = smooth_path(waypoints, point_spacing=0.05)

            # Build Path message
            result = ComputePathToPose.Result()
            result.path = Path()
            result.path.header.frame_id = 'map'
            result.path.header.stamp = self.get_clock().now().to_msg()

            for x, y in smooth_waypoints:
                pose = PoseStamped()
                pose.header = result.path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                result.path.poses.append(pose)

            goal_handle.succeed()
            return result

        goal_handle.abort()
        return ComputePathToPose.Result()


def main(args=None):
    if not HAS_ROS2:
        return
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
