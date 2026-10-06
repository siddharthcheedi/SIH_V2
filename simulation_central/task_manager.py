"""
task_manager.py — Task generation, lifecycle tracking, and performance metrics.
"""

from __future__ import annotations
import random
import time
from typing import List, Dict, Optional, Tuple

from .warehouse import Warehouse, PICKUP, DROP, FREE, UNREACHABLE
from .cbaa import Task


class TaskManager:
    def __init__(self, warehouse: Warehouse, seed: int = 42):
        self.wh   = warehouse
        self.rng  = random.Random(seed)
        self.active_tasks:    List[Task] = []
        self.completed_tasks: List[Task] = []
        # Simulation clock (seconds). The engine replaces this with tick*DT so metrics are
        # independent of wall-clock speed / sim_speed. Defaults to wall time standalone.
        self.clock = time.time
        self._now = self.clock()
        self._target_pending  = 4   # keep this many tasks in queue

    def set_warehouse(self, wh: Warehouse) -> None:
        """Hot-swap warehouse (on layout change)."""
        self.wh = wh

    # ── task generation ───────────────────────────────────────────────────────

    def _random_pickup(self) -> Optional[Tuple[int,int]]:
        pts = self.wh.pickup_points
        if pts:
            return self.rng.choice(pts)
        # Fallback: any passable cell
        candidates = []
        for y in range(self.wh.height):
            for x in range(self.wh.width):
                if self.wh.is_passable(x, y):
                    candidates.append((x, y))
        return self.rng.choice(candidates) if candidates else None

    def _random_dropoff(self) -> Optional[Tuple[int,int]]:
        dz = self.wh.drop_zones
        if dz:
            return self.rng.choice(dz)
        # Fallback
        candidates = []
        for y in range(self.wh.height):
            for x in range(self.wh.width):
                if self.wh.is_passable(x, y):
                    candidates.append((x, y))
        return self.rng.choice(candidates) if candidates else None

    def is_feasible(self, pickup: Tuple[int,int], dropoff: Tuple[int,int]) -> bool:
        """
        A task is only worth queueing if a robot can actually do it: the dropoff is
        reachable from the pickup, and the pickup is reachable from where robots start.
        Without this, walled-off pockets in a layout produce tasks nobody can take; they
        sit in the queue forever and (because the queue is kept topped up to N pending)
        stop any new task from being generated.
        """
        if self.wh.dist(pickup, dropoff) >= UNREACHABLE:
            return False
        spawns = self.wh.spawn_points
        return (not spawns) or any(self.wh.dist(sp, pickup) < UNREACHABLE for sp in spawns)

    def generate_task(self) -> Optional[Task]:
        pickup = dropoff = None
        for _ in range(12):                       # re-draw until the pair is feasible
            p, d = self._random_pickup(), self._random_dropoff()
            if p is not None and d is not None and p != d and self.is_feasible(p, d):
                pickup, dropoff = p, d
                break
        if pickup is None:
            return None
        t = Task(pickup, dropoff, priority=self.rng.randint(1, 3))
        t.created_at = self.clock()
        self.active_tasks.append(t)
        return t

    def add_manual_task(self, pickup: Tuple[int,int],
                        dropoff: Tuple[int,int]) -> Task:
        t = Task(pickup, dropoff, priority=2)
        t.created_at = self.clock()
        self.active_tasks.append(t)
        return t

    def update(self) -> List[Task]:
        """
        Ensure `_target_pending` unassigned tasks exist.
        Returns newly created tasks.
        """
        unassigned = [t for t in self.active_tasks
                      if t.assigned_to is None and self.is_feasible(t.pickup, t.dropoff)]
        new = []
        attempts = 0
        while len(unassigned) < self._target_pending and attempts < 10:
            t = self.generate_task()
            if t:
                new.append(t)
                unassigned.append(t)
            attempts += 1
        return new

    def mark_completed(self, task: Task) -> None:
        task.completed_at = self.clock()
        if task in self.active_tasks:
            self.active_tasks.remove(task)
        self.completed_tasks.append(task)

    def mark_started(self, task: Task) -> None:
        if task.started_at == 0.0:
            task.started_at = self.clock()

    # ── metrics ───────────────────────────────────────────────────────────────

    def get_metrics(self) -> Dict:
        completed = self.completed_tasks
        n         = len(completed)

        times = []
        latencies = []
        for t in completed:
            if t.started_at > 0 and t.completed_at > t.started_at:
                times.append(t.completed_at - t.started_at)
            if t.completed_at > t.created_at:
                latencies.append(t.completed_at - t.created_at)   # queue wait + execution

        avg_time  = sum(times) / len(times) if times else 0.0
        min_time  = min(times) if times else 0.0
        max_time  = max(times) if times else 0.0

        elapsed   = self.clock() - self._now
        throughput = n / (elapsed / 60.0) if elapsed > 0 else 0.0

        return {
            "completed":          n,
            "active":             len(self.active_tasks),
            "avg_completion_time": round(avg_time, 2),
            "avg_latency":        round(sum(latencies) / len(latencies), 2) if latencies else 0.0,
            "min_time":           round(min_time, 2),
            "max_time":           round(max_time, 2),
            "throughput_per_min": round(throughput, 2),
            "recent_times":       [round(t, 2) for t in times[-20:]],
        }

    def reset(self) -> None:
        self.active_tasks    = []
        self.completed_tasks = []
        self._now = self.clock()
        Task.reset_ids()
