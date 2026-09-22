"""
initial_pose_publisher.py — Seed AMCL with each robot's known spawn pose.

AMCL initial pose cannot be hardcoded in nav2_params_template.yaml because
it's shared across robots with different spawn points. This node publishes
once to /<robot_ns>/initialpose with the known ground-truth spawn pose
from scenario.py, then shuts down.

Valid because this is simulation — real hardware would need a different
initial-pose strategy (fiducial-based, docking station, etc.).
"""

from __future__ import annotations

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseWithCovarianceStamped
    import math
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

from amr_fleet_core.scenario import ROBOT_MAP


class InitialPosePublisher(Node):
    """Publishes initial pose to AMCL, then shuts down."""

    def __init__(self, robot_id: str) -> None:
        super().__init__(f'{robot_id}_initial_pose_pub')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.robot_id = robot_id
        self.config = ROBOT_MAP.get(robot_id)

        if not self.config:
            self.get_logger().error(f"Unknown robot_id: {robot_id}")
            return

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )

        # Delay slightly to ensure AMCL is up
        self.create_timer(1.0, self._publish_once)
        self._published = False

    def _publish_once(self) -> None:
        if self._published:
            return
        self._published = True

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.config.spawn_x
        msg.pose.pose.position.y = self.config.spawn_y
        msg.pose.pose.position.z = 0.0

        # Yaw to quaternion
        yaw = self.config.spawn_yaw
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Small covariance — we know the pose exactly in sim
        msg.pose.covariance[0] = 0.1   # x
        msg.pose.covariance[7] = 0.1   # y
        msg.pose.covariance[35] = 0.05 # yaw

        self.pub.publish(msg)
        self.get_logger().info(
            f"Published initial pose for {self.robot_id}: "
            f"({self.config.spawn_x:.1f}, {self.config.spawn_y:.1f}, "
            f"yaw={self.config.spawn_yaw:.2f})"
        )

        # Schedule shutdown after a few more publishes (for reliability)
        self.create_timer(3.0, self._shutdown)

    def _shutdown(self) -> None:
        self.get_logger().info("Initial pose published, shutting down")
        raise SystemExit(0)


def main(args=None):
    if not HAS_ROS2:
        print("ERROR: rclpy not available.")
        return

    rclpy.init(args=args)
    import sys
    robot_id = sys.argv[1] if len(sys.argv) > 1 else 'amr_1'
    node = InitialPosePublisher(robot_id)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
