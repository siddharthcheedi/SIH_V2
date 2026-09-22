"""
cnp.py — Contract Net Protocol (auction-based bidding) for task allocation.

This is the deterministic baseline allocator. The RL policy (rl_policy.py)
only influences bid priority — it cannot touch motion, collision avoidance,
or reservation logic.

Protocol:
  1. A task is announced (pickup location, dropoff location).
  2. Every idle (or about-to-be-idle) robot computes its estimated cost
     (distance to pickup + pickup-to-dropoff) and submits a bid.
  3. Bids are evaluated deterministically: lowest cost wins, ties broken
     by robot_id (lexicographic).
  4. The winning robot is awarded the task.

No central manager: each robot that has a task to dispatch runs the protocol
for that task. The "manager" role rotates naturally.

Zero ROS2 dependencies — pure Python, tested with plain pytest.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple


class TaskStatus(Enum):
    """Status of a task in the system."""
    PENDING = auto()      # announced, awaiting bids
    AWARDED = auto()      # awarded to a robot
    IN_PROGRESS = auto()  # robot is executing
    COMPLETED = auto()    # successfully completed
    FAILED = auto()       # failed, needs re-announcement


class RobotStatus(Enum):
    """Status of a robot for task allocation."""
    IDLE = auto()
    BUSY = auto()
    DEAD = auto()


@dataclass(frozen=True)
class TaskAnnouncement:
    """A task available for bidding."""
    task_id: str
    pickup_x: float
    pickup_y: float
    dropoff_x: float
    dropoff_y: float
    announced_at: float  # simulation timestamp

    @property
    def transport_distance(self) -> float:
        """Euclidean distance from pickup to dropoff."""
        dx = self.dropoff_x - self.pickup_x
        dy = self.dropoff_y - self.pickup_y
        return math.sqrt(dx * dx + dy * dy)


@dataclass(frozen=True)
class Bid:
    """A robot's bid on a task."""
    robot_id: str
    task_id: str
    estimated_cost: float  # total distance: robot→pickup + pickup→dropoff
    bid_priority: float    # from RL policy (lower = more eager to take task)
    submitted_at: float    # simulation timestamp


@dataclass
class TaskRecord:
    """Internal record tracking a task's lifecycle."""
    announcement: TaskAnnouncement
    status: TaskStatus = TaskStatus.PENDING
    bids: List[Bid] = field(default_factory=list)
    awarded_to: Optional[str] = None
    started_at: Optional[float] = None
    completed_at: Optional[float] = None


def compute_bid_cost(
    robot_x: float, robot_y: float,
    task: TaskAnnouncement,
) -> float:
    """
    Compute a robot's estimated cost for a task:
    distance(robot → pickup) + distance(pickup → dropoff).
    """
    dx1 = task.pickup_x - robot_x
    dy1 = task.pickup_y - robot_y
    to_pickup = math.sqrt(dx1 * dx1 + dy1 * dy1)
    return to_pickup + task.transport_distance


def evaluate_bids(bids: List[Bid]) -> Optional[str]:
    """
    Evaluate bids for a task and return the winning robot_id.

    Selection criteria (deterministic):
      1. Lowest estimated_cost wins.
      2. Ties broken by lowest bid_priority (from RL — lower = more eager).
      3. Final tiebreak by robot_id (lexicographic).

    Returns None if no bids received.
    """
    if not bids:
        return None

    winner = min(bids, key=lambda b: (b.estimated_cost, b.bid_priority, b.robot_id))
    return winner.robot_id


class TaskAllocator:
    """
    Manages the Contract Net Protocol lifecycle for task allocation.

    Each robot maintains its own instance. Tasks are announced, bids
    collected, and awards made in a decentralized fashion.
    """

    def __init__(self) -> None:
        self._tasks: Dict[str, TaskRecord] = {}
        self._robot_status: Dict[str, RobotStatus] = {}

    def announce_task(self, task: TaskAnnouncement) -> None:
        """Register a new task for bidding."""
        self._tasks[task.task_id] = TaskRecord(announcement=task)

    def submit_bid(self, bid: Bid) -> bool:
        """
        Submit a bid on a task. Returns True if accepted (task still pending).
        """
        record = self._tasks.get(bid.task_id)
        if record is None or record.status != TaskStatus.PENDING:
            return False
        record.bids.append(bid)
        return True

    def close_bidding(self, task_id: str) -> Optional[str]:
        """
        Close bidding on a task and award it to the winner.
        Returns the winning robot_id, or None if no bids.
        """
        record = self._tasks.get(task_id)
        if record is None or record.status != TaskStatus.PENDING:
            return None

        winner = evaluate_bids(record.bids)
        if winner is None:
            return None

        record.status = TaskStatus.AWARDED
        record.awarded_to = winner
        return winner

    def mark_in_progress(self, task_id: str, timestamp: float) -> None:
        """Mark a task as in progress."""
        record = self._tasks.get(task_id)
        if record:
            record.status = TaskStatus.IN_PROGRESS
            record.started_at = timestamp

    def mark_completed(self, task_id: str, timestamp: float) -> None:
        """Mark a task as completed."""
        record = self._tasks.get(task_id)
        if record:
            record.status = TaskStatus.COMPLETED
            record.completed_at = timestamp

    def mark_failed(self, task_id: str) -> None:
        """Mark a task as failed (needs re-announcement)."""
        record = self._tasks.get(task_id)
        if record:
            record.status = TaskStatus.FAILED
            record.awarded_to = None

    def get_pending_tasks(self) -> List[TaskAnnouncement]:
        """Return all tasks still awaiting bids."""
        return [
            r.announcement for r in self._tasks.values()
            if r.status == TaskStatus.PENDING
        ]

    def get_task_record(self, task_id: str) -> Optional[TaskRecord]:
        return self._tasks.get(task_id)

    def set_robot_status(self, robot_id: str, status: RobotStatus) -> None:
        self._robot_status[robot_id] = status

    def get_idle_robots(self) -> List[str]:
        return [rid for rid, s in self._robot_status.items()
                if s == RobotStatus.IDLE]
