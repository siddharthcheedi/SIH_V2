"""
reservation.py — Reservation dataclass + ReservationTable.

The ReservationTable is the data structure every robot maintains locally,
populated entirely from peer broadcasts. There is no central process holding
"the" table — every robot holds its own copy, all converging to the same
content because they all see the same broadcasts.

Zero ROS2 dependencies — pure Python, tested with plain pytest.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from amr_fleet_core.negotiation.grid import CellReservation, cell_id


@dataclass(frozen=True)
class Reservation:
    """
    A time-windowed cell reservation belonging to a specific robot.

    Attributes
    ----------
    robot_id : str
        Which robot owns this reservation.
    cell_id : int
        Coarse grid cell ID (from grid.cell_id).
    t_start : float
        Simulation time the robot expects to enter this cell.
    t_end : float
        Simulation time the robot expects to leave this cell.
    priority_key : tuple
        (task_assigned_at, robot_id) — lower tuple wins. Fixed for one task.
    """
    robot_id: str
    cell_id: int
    t_start: float
    t_end: float
    priority_key: Tuple[float, str]


@dataclass
class Conflict:
    """A detected conflict between two reservations."""
    own_reservation: Reservation
    other_reservation: Reservation
    other_robot_id: str
    other_priority: Tuple[float, str]

    @property
    def other_has_priority(self) -> bool:
        """True if the other robot has equal or higher (lower-value) priority."""
        return self.other_priority <= self.own_reservation.priority_key


def _time_overlaps(a_start: float, a_end: float,
                   b_start: float, b_end: float) -> bool:
    """Check if two time intervals overlap (inclusive boundaries)."""
    return a_start <= b_end and b_start <= a_end


class ReservationTable:
    """
    Stores per-robot reservation sets and detects conflicts.

    Each robot's reservation set is replaced atomically (not appended to)
    when a new broadcast arrives — this prevents stale reservation buildup.
    """

    def __init__(self) -> None:
        # robot_id → list of Reservation
        self._tables: Dict[str, List[Reservation]] = {}

    def update_robot(self, robot_id: str,
                     reservations: List[Reservation]) -> None:
        """
        Replace (not append) a robot's entire reservation set.

        A fresh broadcast completely supersedes the robot's previous
        reservations. This is critical — if we appended, old reservations
        from a previous (now-obsolete) plan would linger and cause false
        conflicts.
        """
        self._tables[robot_id] = list(reservations)

    def clear_robot(self, robot_id: str) -> None:
        """
        Remove all reservations for a robot (e.g. after heartbeat timeout).
        """
        self._tables.pop(robot_id, None)

    def get_robot_reservations(self, robot_id: str) -> List[Reservation]:
        """Return the current reservation set for a robot."""
        return list(self._tables.get(robot_id, []))

    def get_all_robots(self) -> Set[str]:
        """Return the set of all robot IDs with active reservations."""
        return set(self._tables.keys())

    def find_conflicts(
        self,
        candidate_reservations: List[Reservation],
        exclude_robot: str,
    ) -> List[Conflict]:
        """
        Check a set of candidate reservations against all other robots'
        reservations, returning every detected conflict.

        Parameters
        ----------
        candidate_reservations : list of Reservation
            The reservations the calling robot wants to commit.
        exclude_robot : str
            The calling robot's own ID (skip self-conflicts).

        Returns
        -------
        list of Conflict
        """
        conflicts: List[Conflict] = []
        for other_id, other_reservations in self._tables.items():
            if other_id == exclude_robot:
                continue
            for mine in candidate_reservations:
                for theirs in other_reservations:
                    if (mine.cell_id == theirs.cell_id and
                            _time_overlaps(mine.t_start, mine.t_end,
                                           theirs.t_start, theirs.t_end)):
                        conflicts.append(Conflict(
                            own_reservation=mine,
                            other_reservation=theirs,
                            other_robot_id=other_id,
                            other_priority=theirs.priority_key,
                        ))
        return conflicts

    def build_reservations_from_cells(
        self,
        robot_id: str,
        cell_reservations: List[CellReservation],
        priority_key: Tuple[float, str],
    ) -> List[Reservation]:
        """
        Convert CellReservation list (from grid.path_to_cell_reservations)
        into Reservation list with robot_id and priority attached.
        """
        return [
            Reservation(
                robot_id=robot_id,
                cell_id=cr.cid,
                t_start=cr.t_start,
                t_end=cr.t_end,
                priority_key=priority_key,
            )
            for cr in cell_reservations
        ]

    def __len__(self) -> int:
        return sum(len(v) for v in self._tables.values())

    def __repr__(self) -> str:
        counts = {k: len(v) for k, v in self._tables.items()}
        return f"ReservationTable({counts})"
