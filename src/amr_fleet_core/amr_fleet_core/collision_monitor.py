"""
collision_monitor.py — Ground-truth collision/near-miss monitor.

Subscribes to Gazebo's world-frame ModelStates (NOT per-robot odometry)
and computes true pairwise distances between all AMR models. Logs every
collision and near-miss event to CSV with simulation timestamp.

Uses Gazebo's /gazebo/model_states (via libgazebo_ros_state.so) which
provides world-frame positions — NOT any robot's own odometry, which is
in each robot's local drifting odom frame and NOT comparable across robots
(§9 pitfall).
"""

from __future__ import annotations

import csv
import math
import os
from typing import Dict, List, Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from gazebo_msgs.msg import ModelStates
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

from amr_fleet_core.scenario import ROBOTS

# Thresholds (meters, center-to-center distance between robot models)
COLLISION_THRESHOLD = 0.45    # Less than sum of radii + small margin
NEAR_MISS_THRESHOLD = 0.85   # Close call but not touching

AMR_NAMES = {r.robot_id for r in ROBOTS}


class CollisionMonitorNode(Node):
    """Ground-truth collision/near-miss logger."""

    def __init__(self, output_dir: str = 'benchmarks/raw') -> None:
        super().__init__('collision_monitor')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        self._csv_path = os.path.join(output_dir, 'collision_events.csv')
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'sim_time', 'event_type', 'robot_a', 'robot_b',
            'distance', 'ax', 'ay', 'bx', 'by',
        ])

        self._collision_count = 0
        self._near_miss_count = 0

        # Subscribe to Gazebo's world-frame model states
        self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self._model_states_cb,
            10,
        )

        self.get_logger().info(
            f"Collision monitor active, logging to {self._csv_path}"
        )

    def _model_states_cb(self, msg: ModelStates) -> None:
        """Process model states and check pairwise distances."""
        sim_time = self.get_clock().now().nanoseconds / 1e9

        # Extract AMR positions from model states
        amr_positions: Dict[str, Tuple[float, float]] = {}
        for i, name in enumerate(msg.name):
            if name in AMR_NAMES:
                amr_positions[name] = (
                    msg.pose[i].position.x,
                    msg.pose[i].position.y,
                )

        # Check all pairwise distances
        amr_list = list(amr_positions.items())
        for i in range(len(amr_list)):
            for j in range(i + 1, len(amr_list)):
                name_a, (ax, ay) = amr_list[i]
                name_b, (bx, by) = amr_list[j]

                dx = bx - ax
                dy = by - ay
                dist = math.sqrt(dx * dx + dy * dy)

                if dist < COLLISION_THRESHOLD:
                    self._log_event(
                        sim_time, 'COLLISION', name_a, name_b,
                        dist, ax, ay, bx, by
                    )
                    self._collision_count += 1
                elif dist < NEAR_MISS_THRESHOLD:
                    self._log_event(
                        sim_time, 'NEAR_MISS', name_a, name_b,
                        dist, ax, ay, bx, by
                    )
                    self._near_miss_count += 1

    def _log_event(self, sim_time, event_type, robot_a, robot_b,
                    dist, ax, ay, bx, by) -> None:
        self._csv_writer.writerow([
            f"{sim_time:.3f}", event_type, robot_a, robot_b,
            f"{dist:.4f}", f"{ax:.3f}", f"{ay:.3f}",
            f"{bx:.3f}", f"{by:.3f}",
        ])
        self._csv_file.flush()
        self.get_logger().warn(
            f"{event_type}: {robot_a}↔{robot_b} dist={dist:.3f}m "
            f"at t={sim_time:.1f}s"
        )

    def destroy_node(self) -> None:
        self.get_logger().info(
            f"Monitor summary: {self._collision_count} collisions, "
            f"{self._near_miss_count} near-misses"
        )
        self._csv_file.close()
        super().destroy_node()


def main(args=None):
    if not HAS_ROS2:
        print("ERROR: rclpy not available.")
        return

    rclpy.init(args=args)
    node = CollisionMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
