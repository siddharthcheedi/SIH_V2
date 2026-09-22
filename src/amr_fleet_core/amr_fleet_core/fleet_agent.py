"""
fleet_agent.py — Per-robot fleet coordination agent (ROS2 node).

This is the main node each robot runs. It orchestrates:
  1. Receive task (from scenario or CNP task allocation)
  2. Plan path waypoints through warehouse corridors
  3. Map path waypoints to space-time cell reservations
  4. Negotiate against peer reservation table:
     - If PROCEED → broadcast intent and NavigateToPose
     - If YIELD → wait and retry evaluation periodically
     - If DEADLOCK_BACKOFF → backoff and replan
  5. Monitor execution and clear reservations on completion
  6. Log human-readable explainability strings to the dashboard

Every robot runs an identical copy. No robot is ever special or a hidden
leader. All collision/deadlock decisions are computed locally from peer
broadcasts.
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

# ROS2 imports — this file is a thin wrapper around the pure-Python core
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Path, Odometry
    from std_msgs.msg import String, Float32
    from nav2_msgs.action import ComputePathToPose, NavigateToPose, NavigateThroughPoses
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

from amr_fleet_core.negotiation.grid import path_to_cell_reservations
from amr_fleet_core.negotiation.reservation import Reservation, ReservationTable
from amr_fleet_core.negotiation.priority import make_priority
from amr_fleet_core.negotiation.deadlock import WaitForGraph
from amr_fleet_core.negotiation.negotiator import Decision, Negotiator
from amr_fleet_core.negotiation.orca import AgentState, compute_rvo_adjustment
from amr_fleet_core.fault_tolerance.heartbeat import HeartbeatMonitor
from amr_fleet_core.scenario import ROBOT_MAP, ROBOTS

# Constants
NEGOTIATE_RETRY_INTERVAL = 1.5   # seconds between re-evaluations when yielding
HEARTBEAT_INTERVAL = 1.0         # seconds between heartbeat broadcasts
HEARTBEAT_CHECK_INTERVAL = 2.0   # seconds between timeout checks
ROBOT_SPEED = 0.9                # nominal speed for ETA estimation
ORCA_ACTIVATION_DISTANCE = 2.0   # meters — activate ORCA when peer is this close


def _ros2_path_to_waypoints(path_msg) -> List[Tuple[float, float]]:
    """Extract (x, y) waypoints from a nav_msgs/Path."""
    return [
        (pose.pose.position.x, pose.pose.position.y)
        for pose in path_msg.poses
    ]


class FleetAgentNode(Node):
    """
    Per-robot fleet coordination node.

    Subscribes to peer intent/heartbeat topics, publishes own intent/heartbeat,
    and uses Nav2 action clients for path computation and navigation.
    """

    def __init__(self, robot_id: str) -> None:
        super().__init__(f'{robot_id}_fleet_agent')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.robot_id = robot_id
        self.config = ROBOT_MAP.get(robot_id)

        # ─── Core negotiation state (pure Python) ───────────────────
        self.reservation_table = ReservationTable()
        self.wait_for_graph = WaitForGraph()
        self.negotiator = Negotiator(
            robot_id=robot_id,
            reservation_table=self.reservation_table,
            wait_for_graph=self.wait_for_graph,
        )
        self.heartbeat_monitor = HeartbeatMonitor(
            heartbeat_interval=HEARTBEAT_INTERVAL,
            timeout_factor=3,
            on_peer_dead=self._on_peer_dead,
        )

        # ─── Robot state ────────────────────────────────────────────
        self.current_x = float(self.config.spawn_x) if self.config else 0.0
        self.current_y = float(self.config.spawn_y) if self.config else 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_task_id: Optional[str] = f"scenario_{robot_id}"
        self.current_priority = make_priority(0.0, robot_id)
        self.battery_level = 1.0
        self.peer_states: dict = {}  # robot_id → AgentState
        self.state = 'WAIT_FOR_SERVERS'
        self._goal_handle = None

        # Determine startup delay based on priority order to prevent startup race condition
        robot_idx = 0
        for idx, r in enumerate(ROBOTS):
            if r.robot_id == robot_id:
                robot_idx = idx
                break
        self._startup_delay = 1.0 + float(robot_idx) * 2.0
        self._nav_ready = False

        # ─── Publishers (scoped with robot_id) ──────────────────────
        self.intent_pub = self.create_publisher(String, f'/{robot_id}/intent', 10)
        self.heartbeat_pub = self.create_publisher(String, f'/{robot_id}/heartbeat', 10)
        self.explain_pub = self.create_publisher(String, f'/{robot_id}/explain', 10)
        self.battery_pub = self.create_publisher(Float32, f'/{robot_id}/battery', 10)

        # ─── Subscribers ────────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry, f'/{robot_id}/odom', self._odom_cb, 10
        )

        # Subscribe to all peers' intent and heartbeat topics
        for r in ROBOTS:
            if r.robot_id == robot_id:
                continue
            self.create_subscription(
                String,
                f'/{r.robot_id}/intent',
                lambda msg, rid=r.robot_id: self._peer_intent_cb(rid, msg),
                10,
            )
            self.create_subscription(
                String,
                f'/{r.robot_id}/heartbeat',
                lambda msg, rid=r.robot_id: self._peer_heartbeat_cb(rid, msg),
                10,
            )
            self.create_subscription(
                Odometry,
                f'/{r.robot_id}/odom',
                lambda msg, rid=r.robot_id: self._peer_odom_cb(rid, msg),
                10,
            )

        # ─── Nav2 action clients ────────────────────────────────────
        self.compute_path_client = ActionClient(
            self, ComputePathToPose, f'/{robot_id}/compute_path_to_pose'
        )
        self.navigate_client = ActionClient(
            self, NavigateToPose, f'/{robot_id}/navigate_to_pose'
        )
        self.navigate_through_poses_client = ActionClient(
            self, NavigateThroughPoses, f'/{robot_id}/navigate_through_poses'
        )

        # ─── Timers ─────────────────────────────────────────────────
        self.create_timer(HEARTBEAT_INTERVAL, self._publish_heartbeat)
        self.create_timer(HEARTBEAT_CHECK_INTERVAL, self._check_heartbeats)
        self.create_timer(NEGOTIATE_RETRY_INTERVAL, self._step_state_machine)

        self._publish_explain("Fleet agent initialized, waiting for Nav2 action servers")
        self.get_logger().info(f"Fleet agent {robot_id} initialized")

    def _step_state_machine(self) -> None:
        """Main autonomous coordination state machine."""
        if not self.config:
            return

        if self.state == 'WAIT_FOR_SERVERS':
            if not self.navigate_client.server_is_ready():
                return
            if not self._nav_ready:
                self._nav_ready = True
                self.get_logger().info(
                    f"Nav2 server ready for {self.robot_id}. Staggering priority negotiation window ({self._startup_delay:.1f}s delay)..."
                )
            if self._startup_delay > 0:
                self._startup_delay -= NEGOTIATE_RETRY_INTERVAL
                return

            self.state = 'PLAN_AND_NEGOTIATE'
            self._publish_explain(
                f"Action server ready. Preparing mission to ({self.config.goal_x:.1f}, {self.config.goal_y:.1f})"
            )

        if self.state in ('PLAN_AND_NEGOTIATE', 'YIELDING', 'BACKOFF'):
            self._negotiate_and_dispatch()
        elif self.state == 'NAVIGATING':
            self._check_proximity_yield()

    def _check_proximity_yield(self) -> None:
        """Safety check: if a higher-priority peer is close ahead, yield immediately."""
        if self.state != 'NAVIGATING' or self._goal_handle is None:
            return
        for peer_id, pstate in self.peer_states.items():
            dist = math.hypot(pstate.x - self.current_x, pstate.y - self.current_y)
            if dist < 1.8:
                peer_priority = make_priority(0.0, peer_id)
                if peer_priority < self.current_priority:
                    self._publish_explain(
                        f"YIELD: Proximity collision alert with higher-priority {peer_id} ({dist:.2f}m). Pausing..."
                    )
                    self._goal_handle.cancel_goal_async()
                    self._goal_handle = None
                    self.state = 'YIELDING'
                    break

    def _get_corridor_waypoints(
        self, start_x: float, start_y: float, goal_x: float, goal_y: float, step: float = 0.3
    ) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
        """
        Generate collision-free corridor waypoints matching warehouse aisles.

        Rules:
        1. Aisles are at x in [-8.0, -4.0, 0.0, 4.0, 8.0].
        2. South cross-aisle (y <= -4.0) has parked robots at y = -6.0; robots must
           NOT travel laterally across y = -6.0.
        3. North cross-aisle (y: 3.5 .. 6.5) has ample clearance (4m wide).
           Eastbound lateral moves (start_x < goal_x) use y = 4.8.
           Westbound lateral moves (start_x > goal_x) use y = 5.8.
        4. South-to-North: straight UP departure aisle (start_x), then lateral
           in North cross-aisle to goal_x, then to goal_y.
        5. North-to-South: lateral in North cross-aisle to goal_x, then straight
           DOWN destination aisle (goal_x) to goal_y.

        Returns (fine_waypoints, corner_waypoints).
        """
        aisles = [-8.0, -4.0, 0.0, 4.0, 8.0]
        snap_start_x = min(aisles, key=lambda ax: abs(ax - start_x))
        snap_goal_x = min(aisles, key=lambda ax: abs(ax - goal_x))

        # Lane separation in North cross-aisle (3.5 to 6.5)
        # Eastbound traffic (increasing x) takes y = 4.8; Westbound takes y = 5.8
        cross_y = 4.8 if snap_start_x <= snap_goal_x else 5.8

        corners: List[Tuple[float, float]] = [(start_x, start_y)]

        # South-to-North: straight UP departure aisle (start_x), then cross-aisle to goal_x, then goal_y
        if start_y < -4.0 and goal_y > 2.0:
            corners.append((snap_start_x, cross_y))
            if abs(snap_start_x - snap_goal_x) > 0.5:
                corners.append((snap_goal_x, cross_y))
            corners.append((goal_x, goal_y))

        # North-to-South: cross-aisle to goal_x first, then straight DOWN destination aisle
        elif start_y > 2.0 and goal_y < -4.0:
            if abs(snap_start_x - snap_goal_x) > 0.5:
                corners.append((snap_start_x, cross_y))
                corners.append((snap_goal_x, cross_y))
            corners.append((snap_goal_x, goal_y))
            corners.append((goal_x, goal_y))

        # South-to-South: bypass parked peers via North cross-aisle
        elif start_y < -4.0 and goal_y < -4.0:
            corners.append((snap_start_x, cross_y))
            corners.append((snap_goal_x, cross_y))
            corners.append((goal_x, goal_y))

        # North-to-North
        elif start_y > 2.0 and goal_y > 2.0:
            corners.append((snap_start_x, cross_y))
            corners.append((snap_goal_x, cross_y))
            corners.append((goal_x, goal_y))

        else:
            corners.append((goal_x, goal_y))

        # Deduplicate consecutive corners
        dedup_corners: List[Tuple[float, float]] = []
        for pt in corners:
            if not dedup_corners or math.hypot(pt[0] - dedup_corners[-1][0], pt[1] - dedup_corners[-1][1]) > 0.1:
                dedup_corners.append(pt)

        # Fine-grained interpolation for space-time cell reservations
        waypoints: List[Tuple[float, float]] = []
        for i in range(len(dedup_corners) - 1):
            x1, y1 = dedup_corners[i]
            x2, y2 = dedup_corners[i + 1]
            dist = math.hypot(x2 - x1, y2 - y1)
            num_pts = max(2, int(dist / step))
            for j in range(num_pts):
                alpha = j / float(num_pts)
                waypoints.append((x1 + alpha * (x2 - x1), y1 + alpha * (y2 - y1)))
        waypoints.append((goal_x, goal_y))
        return waypoints, dedup_corners

    def _negotiate_and_dispatch(self) -> None:
        """Evaluate path reservations against peers and proceed or yield."""
        now = self.get_clock().now().nanoseconds / 1e9
        goal_x = self.config.goal_x
        goal_y = self.config.goal_y

        waypoints, corners = self._get_corridor_waypoints(
            self.current_x, self.current_y, goal_x, goal_y
        )
        if not waypoints:
            return

        cell_reservations = path_to_cell_reservations(
            waypoints, speed=ROBOT_SPEED, start_time=now, time_margin=1.0
        )

        candidate_reservations = [
            Reservation(
                robot_id=self.robot_id,
                cell_id=cr.cid,
                t_start=cr.t_start,
                t_end=cr.t_end,
                priority_key=self.current_priority,
            )
            for cr in cell_reservations
        ]

        result = self.negotiator.evaluate(
            candidate_reservations=candidate_reservations,
            my_priority=self.current_priority,
            path_waypoints=waypoints,
            current_time=now,
        )

        if result.decision == Decision.PROCEED:
            self.reservation_table.update_robot(self.robot_id, candidate_reservations)
            self._publish_intent(candidate_reservations)
            self._publish_explain(
                f"PROCEED: Path clear. Navigating to ({goal_x:.1f}, {goal_y:.1f})"
            )
            self._send_navigation_goal(goal_x, goal_y, corners=corners)
            self.state = 'NAVIGATING'

        elif result.decision == Decision.YIELD:
            if self._goal_handle is not None and self.state == 'NAVIGATING':
                self.get_logger().info(f"Cancelling active goal to yield to {result.yielding_to}")
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None
            self.state = 'YIELDING'
            self._publish_explain(
                f"YIELD: Yielding to {result.yielding_to} ({result.yield_reason}). Waiting for corridor clearance..."
            )

        elif result.decision == Decision.DEADLOCK_BACKOFF:
            if self._goal_handle is not None and self.state == 'NAVIGATING':
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None
            self.state = 'BACKOFF'
            self._publish_explain(
                f"DEADLOCK: Cycle detected {result.deadlock_cycle}. Backing off."
            )

    def _send_navigation_goal(
        self, goal_x: float, goal_y: float, corners: Optional[List[Tuple[float, float]]] = None
    ) -> None:
        """Send navigation action goal to Nav2 (NavigateThroughPoses if available and corners exist, else NavigateToPose)."""
        if corners and len(corners) > 1 and self.navigate_through_poses_client.server_is_ready():
            goal_msg = NavigateThroughPoses.Goal()
            poses = []
            for cx, cy in corners[1:]:
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.header.stamp = self.get_clock().now().to_msg()
                p.pose.position.x = float(cx)
                p.pose.position.y = float(cy)
                p.pose.orientation.w = 1.0
                poses.append(p)
            goal_msg.poses = poses
            future = self.navigate_through_poses_client.send_goal_async(goal_msg)
            future.add_done_callback(self._on_goal_response)
        else:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = float(goal_x)
            goal_msg.pose.pose.position.y = float(goal_y)
            goal_msg.pose.pose.orientation.w = 1.0

            future = self.navigate_client.send_goal_async(goal_msg)
            future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._publish_explain("Goal rejected by Nav2. Retrying...")
            self.state = 'PLAN_AND_NEGOTIATE'
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        status = future.result().status
        # Status 4 corresponds to GoalStatus.STATUS_SUCCEEDED
        if status == 4:
            self._publish_explain(
                f"SUCCESS: Goal reached at ({self.config.goal_x:.1f}, {self.config.goal_y:.1f})! Mission complete."
            )
            self.negotiator.clear_own_reservations()
            self._publish_intent([])
            self.state = 'COMPLETED'
        else:
            self._publish_explain(
                f"Navigation ended with status {status}. Re-evaluating path..."
            )
            self.negotiator.clear_own_reservations()
            self._publish_intent([])
            self.state = 'PLAN_AND_NEGOTIATE'

    def _odom_cb(self, msg: Odometry) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_vx = msg.twist.twist.linear.x
        self.current_vy = msg.twist.twist.linear.y

    def _peer_odom_cb(self, robot_id: str, msg: Odometry) -> None:
        self.peer_states[robot_id] = AgentState(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            vx=msg.twist.twist.linear.x,
            vy=msg.twist.twist.linear.y,
        )

    def _peer_intent_cb(self, robot_id: str, msg: String) -> None:
        """Handle a peer's reservation broadcast."""
        import json
        try:
            data = json.loads(msg.data)
            reservations = []
            for r in data.get('reservations', []):
                reservations.append(Reservation(
                    robot_id=robot_id,
                    cell_id=r['cell_id'],
                    t_start=r['t_start'],
                    t_end=r['t_end'],
                    priority_key=tuple(r['priority_key']),
                ))
            self.negotiator.on_peer_reservations_updated(robot_id, reservations)
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f"Bad intent from {robot_id}: {e}")

    def _peer_heartbeat_cb(self, robot_id: str, msg: String) -> None:
        sim_time = self.get_clock().now().nanoseconds / 1e9
        self.heartbeat_monitor.on_heartbeat(robot_id, sim_time)

    def _publish_heartbeat(self) -> None:
        msg = String()
        msg.data = self.robot_id
        self.heartbeat_pub.publish(msg)

        # Also publish battery telemetry
        bat_msg = Float32()
        bat_msg.data = float(self.battery_level)
        self.battery_pub.publish(bat_msg)

    def _check_heartbeats(self) -> None:
        sim_time = self.get_clock().now().nanoseconds / 1e9
        newly_dead = self.heartbeat_monitor.check_timeouts(sim_time)
        for dead_id in newly_dead:
            self._publish_explain(
                f"Peer {dead_id} declared dead after heartbeat timeout"
            )

    def _on_peer_dead(self, robot_id: str, timestamp: float) -> None:
        """Callback when heartbeat monitor declares a peer dead."""
        self.negotiator.on_peer_timeout(robot_id)
        self.get_logger().warn(
            f"Peer {robot_id} timed out at t={timestamp:.1f}, reservations released"
        )

    def _publish_explain(self, reason: str) -> None:
        """Publish human-readable explainability string for logs and dashboard."""
        msg = String()
        msg.data = f"[{self.robot_id}] {reason}"
        self.explain_pub.publish(msg)
        self.get_logger().info(reason)

    def _publish_intent(self, reservations: List[Reservation]) -> None:
        """Broadcast current reservations to peers."""
        import json
        data = {
            'robot_id': self.robot_id,
            'reservations': [
                {
                    'cell_id': r.cell_id,
                    't_start': r.t_start,
                    't_end': r.t_end,
                    'priority_key': list(r.priority_key),
                }
                for r in reservations
            ],
        }
        msg = String()
        msg.data = json.dumps(data)
        self.intent_pub.publish(msg)

    def execute_task(self, goal_x: float, goal_y: float,
                     task_id: str, task_timestamp: float) -> None:
        """Explicit task assignment interface (e.g. from CNP)."""
        self.current_task_id = task_id
        self.current_priority = make_priority(task_timestamp, self.robot_id)
        if self.config:
            from dataclasses import replace
            self.config = replace(self.config, goal_x=goal_x, goal_y=goal_y)
        self.state = 'PLAN_AND_NEGOTIATE'
        self._publish_explain(
            f"Task {task_id} assigned, planning path to ({goal_x:.1f}, {goal_y:.1f})"
        )


def main(args=None):
    if not HAS_ROS2:
        print("ERROR: rclpy not available. This node requires ROS2.")
        print("The pure-Python logic is tested separately via pytest.")
        return

    rclpy.init(args=args)

    import sys
    robot_id = sys.argv[1] if len(sys.argv) > 1 else 'amr_1'
    node = FleetAgentNode(robot_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
