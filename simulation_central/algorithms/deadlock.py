"""
deadlock.py — Deadlock Detection (Wait-For Graph) + Resolution

PIBT's priority inheritance already prevents most deadlocks. This module is the safety
net for the ones that survive (e.g. a ring of robots in a junction all blocked by each
other with no straight-line escape).

Detection:
  Wait-For Graph: edge A -> B when A is at rest, has been blocked for a few ticks, and
  the cell A wants is one of B's cells. Every node has at most one outgoing edge, so a
  deadlock is simply a cycle in a functional graph.

Resolution, tried in order:
  1. RETREAT  — the LOWEST-priority robot in the cycle steps to a free adjacent cell and
                comes back ([safe, current] is spliced into its path, so the path stays
                contiguous). The robot that has waited longest is NOT the one that backs
                off — it is the one that gets through.
  2. BOOST    — the highest-priority robot in the cycle gets a temporary priority boost.
  3. Last resort (engine): the brain drops the task after it has been blocked too long.

It never touches robot.state (an earlier version reset it to MOVING_TO_PICKUP, which made
loaded robots turn around and drive back to the pickup).
"""
from __future__ import annotations
import math
from typing import Dict, List, Optional, Set, Tuple

BLOCKED_TICKS_FOR_WFG = 3     # a robot must be blocked this long to count as "waiting"
RETREAT_COOLDOWN      = 15


# ─────────────────────────────────────────────────────────────
#  Wait-For Graph
# ─────────────────────────────────────────────────────────────

def build_wfg(robots) -> Dict[int, int]:
    """{robot_id: robot_id_it_waits_for}"""
    cell_owner: Dict[Tuple[int,int], int] = {}
    for r in robots:
        if not r.active:
            continue
        cell_owner[r.grid_cell] = r.id
        if r.target_cell is not None:
            cell_owner[r.target_cell] = r.id

    wfg: Dict[int, int] = {}
    for r in robots:
        if not r.active or r.target_cell is not None:
            continue
        if r.blocked_ticks < BLOCKED_TICKS_FOR_WFG or r.desired_cell is None:
            continue
        blocker = cell_owner.get(r.desired_cell)
        if blocker is not None and blocker != r.id:
            wfg[r.id] = blocker
    return wfg


def find_cycles(wfg: Dict[int, int]) -> List[List[int]]:
    """All cycles (length >= 2) in the functional graph."""
    visited:  Set[int] = set()
    in_cycle: Set[int] = set()
    cycles: List[List[int]] = []

    for start in wfg:
        if start in visited:
            continue
        path: List[int] = []
        seen: Dict[int, int] = {}
        cur: Optional[int] = start
        while cur is not None and cur not in visited:
            if cur in seen:
                cycle = path[seen[cur]:]
                if len(cycle) >= 2 and not any(n in in_cycle for n in cycle):
                    cycles.append(cycle)
                    in_cycle.update(cycle)
                break
            seen[cur] = len(path)
            path.append(cur)
            visited.add(cur)
            cur = wfg.get(cur)
    return cycles


# ─────────────────────────────────────────────────────────────
#  Resolution
# ─────────────────────────────────────────────────────────────

class DeadlockResolver:
    def __init__(self, warehouse):
        self.wh = warehouse

    def resolve(self, cycle: List[int], robots_by_id: Dict[int, "Robot"],
                mapf_planner=None, tick: int = 0) -> List[Dict]:
        members = [robots_by_id[i] for i in cycle if i in robots_by_id]
        if not members:
            return []
        actions: List[Dict] = []

        occupied: Set[Tuple[int,int]] = set()
        for r in robots_by_id.values():
            occupied.add(r.grid_cell)
            if r.target_cell is not None:
                occupied.add(r.target_cell)

        # 1. RETREAT: lowest priority first
        for retreat in sorted(members, key=lambda r: r.priority_score()):
            if retreat.target_cell is not None:
                continue
            safe = self._find_retreat_cell(retreat, occupied)
            if safe is None:
                continue
            retreat.path = [safe, retreat.grid_cell] + list(retreat.path)
            retreat.blocked_ticks = 0
            actions.append({"type": "RETREAT", "robot_id": retreat.id,
                            "to": list(safe), "cycle": cycle})
            # let the one that stays have the right of way
            winner = max(members, key=lambda r: r.priority_score())
            if winner is not retreat:
                winner.priority_boost = max(winner.priority_boost, 40)
            return actions

        # 2. BOOST
        winner = max(members, key=lambda r: r.priority_score())
        winner.priority_boost = max(winner.priority_boost, 60)
        actions.append({"type": "PRIORITY_BOOST", "robot_id": winner.id, "cycle": cycle})
        return actions

    def _find_retreat_cell(self, robot, occupied: Set[Tuple[int,int]]
                           ) -> Optional[Tuple[int,int]]:
        """Adjacent, passable, currently unoccupied; prefer low-traffic, far from others."""
        cx, cy = robot.grid_cell
        options = [n for n in self.wh.get_neighbors(cx, cy) if n not in occupied]
        if not options:
            return None
        wanted = robot.desired_cell

        def score(cell):
            # never retreat INTO the cell we were trying to reach; prefer a roomy cell
            penalty = 100 if cell == wanted else 0
            room = len(self.wh.get_neighbors(*cell))
            return (penalty, -room)

        options.sort(key=score)
        return options[0]


# ─────────────────────────────────────────────────────────────
#  Main Detector Class
# ─────────────────────────────────────────────────────────────

class DeadlockDetector:
    """High-level interface: detect + resolve deadlocks each tick."""

    def __init__(self, warehouse):
        self.warehouse = warehouse
        self.resolver  = DeadlockResolver(warehouse)
        self._recently_resolved: Dict[frozenset, int] = {}
        self._COOLDOWN = RETREAT_COOLDOWN

    def update_warehouse(self, warehouse) -> None:
        self.warehouse   = warehouse
        self.resolver.wh = warehouse

    def run(self, robots, tick: int,
            mapf_planner=None) -> Tuple[List[List[int]], List[Dict]]:
        robots_by_id = {r.id: r for r in robots if r.active}
        wfg    = build_wfg(list(robots_by_id.values()))
        cycles = find_cycles(wfg)

        all_actions: List[Dict] = []
        fresh_cycles: List[List[int]] = []
        for cycle in cycles:
            key = frozenset(cycle)
            if tick - self._recently_resolved.get(key, -999) < self._COOLDOWN:
                continue
            actions = self.resolver.resolve(cycle, robots_by_id,
                                            mapf_planner=mapf_planner, tick=tick)
            if actions:
                all_actions.extend(actions)
                self._recently_resolved[key] = tick
                fresh_cycles.append(cycle)
        # forget old entries
        self._recently_resolved = {k: t for k, t in self._recently_resolved.items()
                                   if tick - t < 200}
        return fresh_cycles, all_actions

    def get_viz(self, cycles: List[List[int]], actions: List[Dict]) -> Dict:
        return {"cycles": cycles, "actions": actions, "n_deadlocks": len(cycles)}
