"""
cbaa.py — Consensus-Based Auction Algorithm with pluggable utility function.

Each robot bids on unassigned tasks using a utility score provided by its RobotBrain.
Greedy consensus: highest bidder per task wins (no auctioneer needed).

A bid <= 0 means "I can't / shouldn't take this" (low battery, recently dropped it, ...)
and never wins, so a task stays in the queue rather than being forced onto a bad robot.
"""
from __future__ import annotations
import math
from typing import Callable, Dict, List, Optional, Tuple

from .robot import Robot, RobotState


class Task:
    _next_id = 0

    def __init__(self, pickup: Tuple[int,int], dropoff: Tuple[int,int], priority: int = 1):
        self.id          = Task._next_id
        Task._next_id   += 1
        self.pickup      = pickup
        self.dropoff     = dropoff
        self.priority    = priority
        self.assigned_to: Optional[int] = None
        self.created_at:   float = 0.0
        self.started_at:   float = 0.0
        self.completed_at: float = 0.0
        # robot_id -> tick until which that robot may not bid on this task again
        self.banned: Dict[int, int] = {}

    def to_dict(self) -> Dict:
        return {
            "id":          self.id,
            "pickup":      list(self.pickup),
            "dropoff":     list(self.dropoff),
            "priority":    self.priority,
            "assigned_to": self.assigned_to,
        }

    @classmethod
    def reset_ids(cls):
        cls._next_id = 0


class CBAA:
    def __init__(self):
        self.tasks: List[Task] = []
        self.viz: Dict = {"bids": {}, "assignments": {}, "events": []}

    def add_task(self, task: Task) -> None:
        if task not in self.tasks:
            self.tasks.append(task)

    def remove_task(self, task: Task) -> None:
        if task in self.tasks:
            self.tasks.remove(task)

    def run_auction(self, idle_robots: List[Robot],
                    all_robots: List[Robot],
                    utility_fn: Optional[Callable] = None) -> Dict[int, Task]:
        """
        Assigns unassigned tasks to idle robots.
        utility_fn(robot, task) -> float  (higher = better match, <=0 = abstain)
        """
        pending = [t for t in self.tasks if t.assigned_to is None]
        if not idle_robots or not pending:
            return {}

        # bid_matrix[task_id] = [(bid, robot_id, robot), ...] (positive bids only)
        bid_matrix: Dict[int, List[Tuple]] = {}
        for task in pending:
            bids = []
            for robot in idle_robots:
                if utility_fn:
                    score = utility_fn(robot, task)
                else:
                    dist  = math.hypot(robot.x - task.pickup[0],
                                       robot.y - task.pickup[1])
                    bat   = max(0.1, robot.battery / 100.0)
                    score = bat / (dist + 0.01) * task.priority
                robot.viz["bid_values"][task.id] = round(score, 3)
                if score > 0:
                    bids.append((score, robot.id, robot))
            bid_matrix[task.id] = sorted(bids, key=lambda b: (-b[0], b[1]))

        # Greedy consensus: the task with the strongest single bid is settled first
        sorted_tasks = sorted(
            [t for t in pending if bid_matrix[t.id]],
            key=lambda t: -bid_matrix[t.id][0][0])
        assigned_robots: set = set()
        new_assignments: Dict[int, Task] = {}
        events: List[str] = []

        for task in sorted_tasks:
            for score, robot_id, robot in bid_matrix[task.id]:
                if robot_id not in assigned_robots:
                    task.assigned_to        = robot_id
                    robot.task              = task
                    robot.state             = RobotState.MOVING_TO_PICKUP
                    assigned_robots.add(robot_id)
                    new_assignments[robot_id] = task
                    events.append(f"R{robot_id}→Task{task.id} "
                                  f"(score={score:.3f}, p={task.priority})")
                    break

        self.viz = {
            "bids": {str(tid): {str(rid): round(b, 3) for b, rid, _ in bids}
                     for tid, bids in bid_matrix.items()},
            "assignments": {str(rid): t.id for rid, t in new_assignments.items()},
            "events": events,
        }
        return new_assignments

    def drop_task(self, task: Optional[Task], robot: Robot,
                  reason: str = "re-route", tick: int = 0,
                  ban_ticks: int = 120) -> None:
        """Release a task back to the pool. The dropping robot can't re-bid on it for a while."""
        if task:
            task.assigned_to = None
            if ban_ticks > 0:
                task.banned[robot.id] = tick + ban_ticks
        robot.task  = None
        robot.path  = []
        robot.state = RobotState.IDLE
        robot.task_age = 0
        robot.viz["bid_values"] = {}

    def get_viz(self) -> Dict:
        return self.viz
