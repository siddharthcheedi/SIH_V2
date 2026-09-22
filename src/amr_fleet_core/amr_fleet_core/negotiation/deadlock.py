"""
deadlock.py — Wait-for-graph cycle detection + priority-based backoff.

This is the second of two complementary deadlock defenses:

1. Resource ordering (priority.py, structural prevention):
   Makes the classic two-agent circular-wait impossible by construction.

2. Wait-for-graph cycle detection (THIS MODULE, reactive detection + break):
   Catches the N-robot cycle shape that resource ordering doesn't cover —
   where each robot is waiting on exactly one other robot's single resource.

Every robot broadcasts its current yield state ("I am yielding to robot X").
Every robot reconstructs the identical wait-for graph from these broadcasts
and independently reaches the same conclusion about which cycle exists and
which robot (lowest priority in the cycle) breaks it.

What this combination does NOT guarantee:
Fixed-priority prioritized planning can still cause a low-priority robot to
wait a long time in adversarial layouts. That is a known property of the
approach, not a bug to hide.

Zero ROS2 dependencies — pure Python, tested with plain pytest.
"""

from __future__ import annotations

import csv
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from amr_fleet_core.negotiation.priority import PriorityKey


@dataclass
class YieldState:
    """A robot's current yield relationship."""
    robot_id: str
    yielding_to: Optional[str]  # None if not yielding
    priority: PriorityKey


@dataclass
class DeadlockEvent:
    """A logged deadlock detection/prevention event."""
    timestamp: float
    cycle: List[str]
    backoff_robot: str
    reason: str


class WaitForGraph:
    """
    Directed graph of yield relationships: edge (A → B) means robot A is
    currently yielding to (waiting for) robot B.

    A cycle in this graph means deadlock. The robot with the lowest priority
    (highest tuple value) in the cycle backs off.
    """

    def __init__(self) -> None:
        # robot_id → YieldState
        self._states: Dict[str, YieldState] = {}
        # Log of all deadlock events for benchmarking
        self._events: List[DeadlockEvent] = []

    def update(self, robot_id: str, yielding_to: Optional[str],
               priority: PriorityKey) -> None:
        """Update a robot's yield state from its broadcast."""
        self._states[robot_id] = YieldState(
            robot_id=robot_id,
            yielding_to=yielding_to,
            priority=priority,
        )

    def remove(self, robot_id: str) -> None:
        """Remove a robot from the graph (e.g. after heartbeat timeout)."""
        self._states.pop(robot_id, None)

    def detect_cycles(self) -> List[List[str]]:
        """
        Find all cycles in the wait-for graph using DFS.

        Returns
        -------
        list of list of str
            Each inner list is a cycle of robot IDs, in the order they
            form the cycle (A → B → C → A would be [A, B, C]).
        """
        visited: Set[str] = set()
        in_stack: Set[str] = set()
        stack: List[str] = []
        cycles: List[List[str]] = []

        def dfs(node: str) -> None:
            if node in in_stack:
                # Found a cycle — extract it
                cycle_start = stack.index(node)
                cycle = stack[cycle_start:]
                cycles.append(list(cycle))
                return
            if node in visited:
                return

            visited.add(node)
            in_stack.add(node)
            stack.append(node)

            state = self._states.get(node)
            if state and state.yielding_to and state.yielding_to in self._states:
                dfs(state.yielding_to)

            stack.pop()
            in_stack.discard(node)

        for robot_id in self._states:
            if robot_id not in visited:
                dfs(robot_id)

        return cycles

    def select_backoff_robot(self, cycle: List[str]) -> Optional[str]:
        """
        Select the robot in the cycle that should back off: the one with
        the lowest priority (highest tuple value — worst priority).

        Parameters
        ----------
        cycle : list of str
            Robot IDs forming a deadlock cycle.

        Returns
        -------
        str or None
            The robot_id that should back off, or None if the cycle is empty.
        """
        if not cycle:
            return None

        worst_priority: Optional[PriorityKey] = None
        worst_robot: Optional[str] = None

        for robot_id in cycle:
            state = self._states.get(robot_id)
            if state is None:
                continue
            if worst_priority is None or state.priority > worst_priority:
                worst_priority = state.priority
                worst_robot = robot_id

        return worst_robot

    def detect_and_resolve(self, timestamp: float) -> List[DeadlockEvent]:
        """
        Run cycle detection + backoff selection, logging events.

        Returns
        -------
        list of DeadlockEvent
            All deadlock events detected in this round.
        """
        events: List[DeadlockEvent] = []
        cycles = self.detect_cycles()

        for cycle in cycles:
            backoff = self.select_backoff_robot(cycle)
            if backoff:
                event = DeadlockEvent(
                    timestamp=timestamp,
                    cycle=list(cycle),
                    backoff_robot=backoff,
                    reason=(
                        f"Deadlock cycle {cycle}: {backoff} backs off "
                        f"(lowest priority in cycle)"
                    ),
                )
                events.append(event)
                self._events.append(event)

        return events

    def get_all_events(self) -> List[DeadlockEvent]:
        """Return all logged deadlock events."""
        return list(self._events)

    def export_events_csv(self, filepath: str) -> None:
        """Export deadlock events to CSV for benchmarking."""
        os.makedirs(os.path.dirname(filepath) or '.', exist_ok=True)
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'cycle', 'backoff_robot', 'reason',
            ])
            for ev in self._events:
                writer.writerow([
                    ev.timestamp,
                    ';'.join(ev.cycle),
                    ev.backoff_robot,
                    ev.reason,
                ])

    def __repr__(self) -> str:
        edges = []
        for rid, state in self._states.items():
            if state.yielding_to:
                edges.append(f"{rid}→{state.yielding_to}")
        return f"WaitForGraph([{', '.join(edges)}])"
