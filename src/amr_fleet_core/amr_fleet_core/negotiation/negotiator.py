"""
negotiator.py — Main negotiation evaluator combining all modules.

This is the single entry point that a ROS2 node calls to decide whether
to proceed with a planned path or yield. It combines:
  1. Resource ordering (structural deadlock prevention)
  2. Space-time reservation conflict detection
  3. Wait-for-graph deadlock detection (reactive)

The evaluate() function returns one of three results:
  - PROCEED: no conflicts with equal-or-higher priority robots → go.
  - YIELD: conflict with a higher-priority robot → wait, report who.
  - DEADLOCK_BACKOFF: cycle detected in wait-for graph → lowest priority
    robot in the cycle backs off (cancels and replans).

Conflict resolution is ALL-OR-NOTHING per evaluation: a conflict against
an equal-or-higher-priority robot means yield entirely (don't partially
commit a path). A conflict only against lower-priority robots means
proceed — they're expected to yield independently.

Zero ROS2 dependencies — pure Python, tested with plain pytest.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Set, Tuple

from amr_fleet_core.negotiation.deadlock import WaitForGraph, DeadlockEvent
from amr_fleet_core.negotiation.grid import (
    CellReservation, path_to_cell_reservations, zones_for_path,
)
from amr_fleet_core.negotiation.priority import (
    PriorityKey, has_priority_over, order_zones_for_acquisition,
)
from amr_fleet_core.negotiation.reservation import (
    Conflict, Reservation, ReservationTable,
)


class Decision(Enum):
    """Outcome of a negotiation evaluation."""
    PROCEED = auto()
    YIELD = auto()
    DEADLOCK_BACKOFF = auto()


@dataclass
class NegotiationResult:
    """Full result of a negotiation evaluation."""
    decision: Decision
    yielding_to: Optional[str] = None
    yield_reason: Optional[str] = None
    deadlock_cycle: Optional[List[str]] = None
    deadlock_reason: Optional[str] = None
    conflicts: List[Conflict] = field(default_factory=list)
    zone_acquisition_order: Optional[List[int]] = None

    @property
    def explanation(self) -> str:
        """Human-readable explanation for the explainability log."""
        if self.decision == Decision.PROCEED:
            return "Proceeding to goal — no conflicts with higher-priority robots"
        elif self.decision == Decision.YIELD:
            return (
                f"Yielding to {self.yielding_to}: {self.yield_reason}"
            )
        elif self.decision == Decision.DEADLOCK_BACKOFF:
            return (
                f"Deadlock detected in cycle {self.deadlock_cycle}: "
                f"{self.deadlock_reason}"
            )
        return "Unknown decision"


class Negotiator:
    """
    Combines reservation table, priority, resource ordering, and deadlock
    detection into a single evaluate() call.

    Each robot creates its own Negotiator instance. The instance holds
    references to the shared (locally-maintained) ReservationTable and
    WaitForGraph.
    """

    def __init__(
        self,
        robot_id: str,
        reservation_table: ReservationTable,
        wait_for_graph: WaitForGraph,
    ) -> None:
        self.robot_id = robot_id
        self.table = reservation_table
        self.wfg = wait_for_graph

    def evaluate(
        self,
        candidate_reservations: List[Reservation],
        my_priority: PriorityKey,
        path_waypoints: Optional[List[Tuple[float, float]]] = None,
        current_time: float = 0.0,
    ) -> NegotiationResult:
        """
        Evaluate whether to proceed with or yield on a candidate path.

        Parameters
        ----------
        candidate_reservations : list of Reservation
            The reservations for the path the robot wants to take.
        my_priority : PriorityKey
            This robot's priority for the current task.
        path_waypoints : list of (x, y), optional
            The path waypoints, used for zone ordering check.
        current_time : float
            Current simulation time, for deadlock event logging.

        Returns
        -------
        NegotiationResult
        """
        # Step 1: Resource ordering check
        zone_order = None
        if path_waypoints:
            zones = zones_for_path(path_waypoints)
            zone_order = order_zones_for_acquisition(zones)

        # Step 2: Find conflicts with all peers
        conflicts = self.table.find_conflicts(
            candidate_reservations, exclude_robot=self.robot_id
        )

        if not conflicts:
            # No conflicts at all — clear any yield state and proceed
            self.wfg.update(self.robot_id, yielding_to=None,
                            priority=my_priority)
            return NegotiationResult(
                decision=Decision.PROCEED,
                zone_acquisition_order=zone_order,
            )

        # Step 3: Check if ANY conflict is with an equal-or-higher-priority robot
        higher_priority_conflicts = [
            c for c in conflicts if c.other_has_priority
        ]

        if not higher_priority_conflicts:
            # All conflicts are with lower-priority robots — proceed
            # (they're expected to yield to us independently)
            self.wfg.update(self.robot_id, yielding_to=None,
                            priority=my_priority)
            return NegotiationResult(
                decision=Decision.PROCEED,
                conflicts=conflicts,
                zone_acquisition_order=zone_order,
            )

        # Step 4: We must yield — find the highest-priority conflicting robot
        highest_other = min(
            higher_priority_conflicts,
            key=lambda c: c.other_priority,
        )
        yielding_to = highest_other.other_robot_id

        # Update wait-for graph with our yield state
        self.wfg.update(
            self.robot_id,
            yielding_to=yielding_to,
            priority=my_priority,
        )

        # Step 5: Check for deadlock cycles
        events = self.wfg.detect_and_resolve(timestamp=current_time)

        for event in events:
            if self.robot_id in event.cycle:
                if event.backoff_robot == self.robot_id:
                    return NegotiationResult(
                        decision=Decision.DEADLOCK_BACKOFF,
                        yielding_to=yielding_to,
                        deadlock_cycle=event.cycle,
                        deadlock_reason=event.reason,
                        conflicts=conflicts,
                        zone_acquisition_order=zone_order,
                    )

        # Yield (no deadlock detected)
        return NegotiationResult(
            decision=Decision.YIELD,
            yielding_to=yielding_to,
            yield_reason=(
                f"lower priority {my_priority} vs "
                f"{highest_other.other_priority}"
            ),
            conflicts=conflicts,
            zone_acquisition_order=zone_order,
        )

    def on_peer_reservations_updated(
        self,
        peer_id: str,
        reservations: List[Reservation],
    ) -> None:
        """
        Called when a peer broadcasts new reservations.
        Atomically replaces (not appends) that peer's reservation set.
        """
        self.table.update_robot(peer_id, reservations)

    def on_peer_timeout(self, peer_id: str) -> None:
        """Called when a peer is declared dead via heartbeat timeout."""
        self.table.clear_robot(peer_id)
        self.wfg.remove(peer_id)

    def clear_own_reservations(self) -> None:
        """Clear this robot's own reservations (e.g. when cancelling a path)."""
        self.table.clear_robot(self.robot_id)
        self.wfg.update(self.robot_id, yielding_to=None,
                        priority=(float('inf'), self.robot_id))
