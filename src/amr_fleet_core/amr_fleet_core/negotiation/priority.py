"""
priority.py — Deterministic priority key + resource-ordering helper.

Priority
--------
PriorityKey = (task_assigned_at: float, robot_id: str).
Lower tuple wins. This is fixed for the duration of one navigation task —
not recomputed on every replan (§9: if priority were "most recent request
wins," a robot that keeps getting bumped would keep re-stamping a fresher
timestamp and could end up perpetually losing to itself).

Resource ordering (structural deadlock prevention)
--------------------------------------------------
When a robot's path needs to acquire more than one contested chokepoint zone
at overlapping times, it must request them in ascending zone_id order. This
makes the classic two-agent opposite-order circular wait structurally
impossible — not unlikely, impossible by construction.

What this does NOT cover: an N-robot cycle where each robot waits on exactly
one other robot's single resource. That shape is caught by the wait-for-graph
cycle detection in deadlock.py.

Zero ROS2 dependencies — pure Python, tested with plain pytest.
"""

from __future__ import annotations

from typing import List, Tuple


# Type alias for readability.
PriorityKey = Tuple[float, str]   # (task_assigned_at, robot_id)


def make_priority(task_assigned_at: float, robot_id: str) -> PriorityKey:
    """Create a priority key. Lower tuple wins."""
    return (task_assigned_at, robot_id)


def higher_priority(a: PriorityKey, b: PriorityKey) -> PriorityKey:
    """Return whichever key has higher priority (lower tuple value)."""
    return a if a <= b else b


def has_priority_over(mine: PriorityKey, theirs: PriorityKey) -> bool:
    """True if `mine` has strictly higher priority (lower value) than `theirs`."""
    return mine < theirs


def order_zones_for_acquisition(zone_ids: List[int]) -> List[int]:
    """
    Sort zone IDs into the fixed global acquisition order (ascending).

    Resource ordering rule: when a robot needs to reserve multiple
    chokepoint zones, it must acquire them in this order. This prevents
    the classic circular-wait deadlock where robot A holds zone 1 and
    wants zone 2, while robot B holds zone 2 and wants zone 1.

    By always acquiring in ascending order, no such cycle can form.

    Parameters
    ----------
    zone_ids : list of int
        Zone IDs the robot's path passes through.

    Returns
    -------
    list of int
        Same zone IDs, sorted ascending (the mandatory acquisition order).
    """
    return sorted(set(zone_ids))
