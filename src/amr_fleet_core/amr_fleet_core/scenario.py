"""
scenario.py — Single source of truth for robot configurations.

Every robot's spawn pose, goal pose, chassis color, and ID are defined here.
Launch files, test fixtures, and the dashboard all import from this module.
Nothing else in the repo hard-codes robot positions or colors.

Warehouse layout (matching worlds/warehouse.world):
  x: -10 .. 10   (east–west, 20m)
  y: -7  ..  7   (north–south, 14m)

  4 shelf rows at x = -6, -2, 2, 6 (each 1m wide × 8m long, y: -5 .. 3)
  3 aisles between shelves (centers at x ≈ -4, 0, 4)
  South cross-aisle: y ∈ (-7, -5)
  North cross-aisle: y ∈ (3, 7)
  6 chokepoints: each aisle × each cross-aisle end

Spawn and goal positions are chosen so paths genuinely interleave through
every aisle and every chokepoint — this is what makes conflict/deadlock
negotiation visible and testable.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple


@dataclass(frozen=True)
class RobotConfig:
    """Immutable configuration for one AMR."""

    robot_id: str
    spawn_x: float
    spawn_y: float
    spawn_yaw: float
    goal_x: float
    goal_y: float
    # RGBA color tuple for Gazebo visual material
    color_rgba: Tuple[float, float, float, float]
    color_name: str  # human-readable, for logs and dashboard


# ── 9-robot scenario ────────────────────────────────────────────────────────
# Spawn points spread across the south cross-aisle and aisle entries.
# Goals spread across the north cross-aisle and opposite aisle entries.
# Every aisle and every chokepoint gets traffic in both directions.

ALL_ROBOTS: List[RobotConfig] = [
    RobotConfig(
        robot_id='amr_1',
        spawn_x=-8.0, spawn_y=-6.0, spawn_yaw=0.0,
        goal_x=8.0, goal_y=5.0,
        color_rgba=(0.9, 0.1, 0.1, 1.0), color_name='red',
    ),
    RobotConfig(
        robot_id='amr_2',
        spawn_x=-4.0, spawn_y=-6.0, spawn_yaw=1.5708,
        goal_x=-4.0, goal_y=5.0,
        color_rgba=(0.1, 0.4, 0.9, 1.0), color_name='blue',
    ),
    RobotConfig(
        robot_id='amr_3',
        spawn_x=0.0, spawn_y=-6.0, spawn_yaw=1.5708,
        goal_x=0.0, goal_y=5.0,
        color_rgba=(0.1, 0.8, 0.2, 1.0), color_name='green',
    ),
    RobotConfig(
        robot_id='amr_4',
        spawn_x=4.0, spawn_y=-6.0, spawn_yaw=1.5708,
        goal_x=-8.0, goal_y=5.0,
        color_rgba=(1.0, 0.55, 0.0, 1.0), color_name='orange',
    ),
    RobotConfig(
        robot_id='amr_5',
        spawn_x=8.0, spawn_y=-6.0, spawn_yaw=3.1416,
        goal_x=-4.0, goal_y=6.0,
        color_rgba=(0.6, 0.1, 0.9, 1.0), color_name='purple',
    ),
    RobotConfig(
        robot_id='amr_6',
        spawn_x=8.0, spawn_y=5.0, spawn_yaw=3.1416,
        goal_x=-8.0, goal_y=-6.0,
        color_rgba=(0.0, 0.8, 0.8, 1.0), color_name='cyan',
    ),
    RobotConfig(
        robot_id='amr_7',
        spawn_x=4.0, spawn_y=5.0, spawn_yaw=-1.5708,
        goal_x=0.0, goal_y=-6.0,
        color_rgba=(0.95, 0.85, 0.1, 1.0), color_name='yellow',
    ),
    RobotConfig(
        robot_id='amr_8',
        spawn_x=-4.0, spawn_y=5.0, spawn_yaw=-1.5708,
        goal_x=4.0, goal_y=-6.0,
        color_rgba=(0.9, 0.1, 0.6, 1.0), color_name='magenta',
    ),
    RobotConfig(
        robot_id='amr_9',
        spawn_x=-8.0, spawn_y=5.0, spawn_yaw=0.0,
        goal_x=8.0, goal_y=-6.0,
        color_rgba=(0.9, 0.9, 0.9, 1.0), color_name='white',
    ),
]

# Active fleet for benchmark (3-robot conflict & negotiation trial: amr_1, amr_2, amr_6)
ROBOTS: List[RobotConfig] = [
    r for r in ALL_ROBOTS if r.robot_id in ('amr_1', 'amr_2', 'amr_6')
]

# Quick-access dict keyed by robot_id.
ROBOT_MAP = {r.robot_id: r for r in ROBOTS}

# ── Chokepoint zone definitions ─────────────────────────────────────────────
# 6 intersection zones where aisles meet cross-aisles. Zone IDs are assigned
# in a fixed ascending order — this is the global ordering used for the
# resource-ordering deadlock prevention (§4.1.4 / §2 point 2).
#
# Zone format: (zone_id, center_x, center_y, half_size)
# half_size defines a square zone around the intersection center.

ZONE_HALF_SIZE = 1.0  # 2m × 2m zone around each intersection

@dataclass(frozen=True)
class ChokeZone:
    """A named chokepoint intersection zone."""
    zone_id: int
    name: str
    center_x: float
    center_y: float
    half_size: float = ZONE_HALF_SIZE

CHOKE_ZONES: List[ChokeZone] = [
    # South cross-aisle intersections (y ≈ -5)
    ChokeZone(zone_id=0, name='aisle_1_south', center_x=-4.0, center_y=-5.0),
    ChokeZone(zone_id=1, name='aisle_2_south', center_x=0.0,  center_y=-5.0),
    ChokeZone(zone_id=2, name='aisle_3_south', center_x=4.0,  center_y=-5.0),
    # North cross-aisle intersections (y ≈ 3)
    ChokeZone(zone_id=3, name='aisle_1_north', center_x=-4.0, center_y=3.0),
    ChokeZone(zone_id=4, name='aisle_2_north', center_x=0.0,  center_y=3.0),
    ChokeZone(zone_id=5, name='aisle_3_north', center_x=4.0,  center_y=3.0),
]

ZONE_MAP = {z.zone_id: z for z in CHOKE_ZONES}


def get_robot_ids() -> List[str]:
    """Return all robot IDs in scenario order."""
    return [r.robot_id for r in ROBOTS]


def get_robot_count() -> int:
    """Return the number of robots in the scenario."""
    return len(ROBOTS)
