"""
grid.py — Coarse reservation grid + chokepoint zone geometry.

This module provides the spatial data structures that the reservation and
negotiation layers operate on. The grid resolution is deliberately coarse
(0.5m cells) because DWB already handles fine-grained local obstacle
avoidance — this layer only needs "roughly where, roughly when."

Zero ROS2 dependencies — pure Python, tested with plain pytest.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from amr_fleet_core.scenario import CHOKE_ZONES, ChokeZone


# ── Coarse grid ──────────────────────────────────────────────────────────────

CELL_SIZE = 0.5  # meters per cell edge

# World bounds matching warehouse.world (with margin)
WORLD_MIN_X = -10.0
WORLD_MAX_X = 10.0
WORLD_MIN_Y = -7.0
WORLD_MAX_Y = 7.0

GRID_COLS = int((WORLD_MAX_X - WORLD_MIN_X) / CELL_SIZE)  # 40
GRID_ROWS = int((WORLD_MAX_Y - WORLD_MIN_Y) / CELL_SIZE)  # 28


@dataclass(frozen=True)
class CellCoord:
    """Integer (row, col) in the coarse grid."""
    row: int
    col: int

    def __lt__(self, other: 'CellCoord') -> bool:
        return (self.row, self.col) < (other.row, other.col)


def world_to_cell(x: float, y: float) -> CellCoord:
    """Convert world-frame (x, y) to the coarse grid cell."""
    col = int((x - WORLD_MIN_X) / CELL_SIZE)
    row = int((WORLD_MAX_Y - y) / CELL_SIZE)
    col = max(0, min(col, GRID_COLS - 1))
    row = max(0, min(row, GRID_ROWS - 1))
    return CellCoord(row=row, col=col)


def cell_to_world(cell: CellCoord) -> Tuple[float, float]:
    """Convert a coarse grid cell to world-frame center (x, y)."""
    x = WORLD_MIN_X + (cell.col + 0.5) * CELL_SIZE
    y = WORLD_MAX_Y - (cell.row + 0.5) * CELL_SIZE
    return (x, y)


def cell_id(cell: CellCoord) -> int:
    """Unique integer ID for a cell, for use as a dict key."""
    return cell.row * GRID_COLS + cell.col


def cell_from_id(cid: int) -> CellCoord:
    """Recover CellCoord from a cell_id."""
    return CellCoord(row=cid // GRID_COLS, col=cid % GRID_COLS)


# ── Chokepoint zone lookups ──────────────────────────────────────────────────

def point_in_zone(x: float, y: float, zone: ChokeZone) -> bool:
    """Check if world-frame (x, y) falls within a chokepoint zone."""
    return (abs(x - zone.center_x) <= zone.half_size and
            abs(y - zone.center_y) <= zone.half_size)


def zones_for_point(x: float, y: float) -> List[int]:
    """Return sorted list of zone_ids that contain the given point."""
    return sorted(
        z.zone_id for z in CHOKE_ZONES if point_in_zone(x, y, z)
    )


def zones_for_path(waypoints: List[Tuple[float, float]]) -> List[int]:
    """Return sorted, deduplicated list of zone_ids the path passes through."""
    seen = set()
    for x, y in waypoints:
        for z in CHOKE_ZONES:
            if point_in_zone(x, y, z):
                seen.add(z.zone_id)
    return sorted(seen)


# ── Path → cell-time sequence ────────────────────────────────────────────────

@dataclass(frozen=True)
class CellReservation:
    """A single cell the robot occupies during a time interval."""
    cell: CellCoord
    t_start: float
    t_end: float

    @property
    def cid(self) -> int:
        return cell_id(self.cell)


def path_to_cell_reservations(
    waypoints: List[Tuple[float, float]],
    speed: float,
    start_time: float = 0.0,
    time_margin: float = 0.5,
) -> List[CellReservation]:
    """
    Convert a continuous path (list of (x, y) waypoints) into a sequence of
    coarse-grid cell reservations with time intervals.

    Each cell the path passes through is reserved from the time the robot
    enters it to the time it leaves, plus a configurable margin on both sides
    for safety.

    Parameters
    ----------
    waypoints : list of (x, y) tuples in world frame
    speed : robot nominal speed in m/s
    start_time : simulation time at which the robot starts the path
    time_margin : seconds of padding on each reservation interval

    Returns
    -------
    list of CellReservation, in path order, deduplicated by cell
    """
    if not waypoints or speed <= 0:
        return []

    reservations: List[CellReservation] = []
    current_time = start_time
    prev_cell: Optional[CellCoord] = None
    cell_enter_time = current_time

    for i, (x, y) in enumerate(waypoints):
        # Accumulate travel time
        if i > 0:
            dx = x - waypoints[i - 1][0]
            dy = y - waypoints[i - 1][1]
            dist = math.sqrt(dx * dx + dy * dy)
            current_time += dist / speed

        c = world_to_cell(x, y)
        if c != prev_cell:
            # Close out the previous cell's reservation
            if prev_cell is not None:
                reservations.append(CellReservation(
                    cell=prev_cell,
                    t_start=max(0.0, cell_enter_time - time_margin),
                    t_end=current_time + time_margin,
                ))
            prev_cell = c
            cell_enter_time = current_time

    # Close out the last cell
    if prev_cell is not None:
        reservations.append(CellReservation(
            cell=prev_cell,
            t_start=max(0.0, cell_enter_time - time_margin),
            t_end=current_time + time_margin,
        ))

    return reservations
