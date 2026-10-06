"""
space_time_astar.py — Space-Time A* with a Reservation Table (congestion-aware routing).

Each robot plans in (x, y, t) space against the paths other robots have already
committed to, so it can route AROUND expected traffic or schedule a wait instead of
meeting someone head-on. PIBT (pibt.py) remains the authority on what actually happens
each step; this planner supplies the *route* PIBT prefers.

Design notes (what changed vs. the first version, and why):
  * TIME UNIT = ONE CELL-STEP (not one tick). A robot needs ~4 ticks per cell, but the
    old planner assumed 1 cell per tick, so every reservation was off by 4x.
  * Heuristic = true BFS walking distance (exact in free space). Manhattan distance is
    badly misleading around long shelf rows and made the search blow its node budget.
  * Returned paths contain NO wait steps. The old paths had repeated cells; the robot
    could never "consume" them and stood still forever.
  * If the reservation-aware search fails, we fall back to a plain shortest path
    instead of returning nothing. Returning None means the goal is truly unreachable.
  * Each robot builds its table from what its neighbours broadcast (LocalRoutePlanner).
"""
from __future__ import annotations
import heapq
from typing import Callable, Dict, List, Optional, Set, Tuple

Cell = Tuple[int, int]
UNREACHABLE = 10**6


# ─────────────────────────────────────────────────────────────
#  Reservation Table
# ─────────────────────────────────────────────────────────────

class ReservationTable:
    """(x, y, step) -> robot_id, plus directed edge reservations to catch swaps."""
    GOAL_HOLD = 2          # steps a goal cell stays reserved after arrival

    def __init__(self):
        self._vertex: Dict[Tuple[int,int,int], int] = {}
        self._edge:   Dict[Tuple[int,int,int,int,int], int] = {}

    def clear_robot(self, robot_id: int) -> None:
        self._vertex = {k: v for k, v in self._vertex.items() if v != robot_id}
        self._edge   = {k: v for k, v in self._edge.items()   if v != robot_id}

    def reserve_path(self, path: List[Cell], robot_id: int, start_t: int = 0) -> None:
        for i, (x, y) in enumerate(path):
            t = start_t + i
            self._vertex[(x, y, t)] = robot_id
            if i < len(path) - 1:
                nx, ny = path[i + 1]
                if (nx, ny) != (x, y):
                    self._edge[(x, y, nx, ny, t)] = robot_id
        if path:
            gx, gy = path[-1]
            gt = start_t + len(path) - 1
            for extra in range(1, self.GOAL_HOLD + 1):
                self._vertex[(gx, gy, gt + extra)] = robot_id

    def is_vertex_free(self, x: int, y: int, t: int, robot_id: int) -> bool:
        occ = self._vertex.get((x, y, t))
        return occ is None or occ == robot_id

    def is_edge_free(self, x1: int, y1: int, x2: int, y2: int,
                     t: int, robot_id: int) -> bool:
        """No robot may traverse the reverse edge in the same step (swap)."""
        occ = self._edge.get((x2, y2, x1, y1, t))
        return occ is None or occ == robot_id

    def snapshot_for_viz(self) -> List[Dict]:
        return [{"x": x, "y": y, "t": t, "robot_id": rid}
                for (x, y, t), rid in list(self._vertex.items())[:200]]


# ─────────────────────────────────────────────────────────────
#  Space-Time A*
# ─────────────────────────────────────────────────────────────

class SpaceTimeAStar:
    MAX_T     = 70     # steps of look-ahead
    MAX_NODES = 4000   # search budget per robot

    def __init__(self, warehouse):
        self.warehouse = warehouse

    def plan(self, robot_id: int, start: Cell, goal: Cell,
             table: ReservationTable, start_t: int = 0,
             blocked: Optional[Set[Cell]] = None,
             soft_cost: Optional[Dict[Cell, int]] = None) -> Optional[List[Cell]]:
        """
        Returns the cells to visit after `start` (goal included, no wait steps).
        []   -> already there.
        None -> no path found within budget (caller falls back to shortest path).
        `blocked`   : hard-impassable cells (dead robots).
        `soft_cost` : extra cost for entering a cell (e.g. an idle robot parked there).
        """
        if start == goal:
            return []
        wh = self.warehouse
        dm = wh.dist_map(goal)
        if dm[start[1]][start[0]] >= UNREACHABLE:
            return None
        blocked   = blocked or set()
        soft_cost = soft_cost or {}

        h0 = dm[start[1]][start[0]]
        # heap entries: (f, h, g, x, y, t)
        open_q: List[Tuple] = [(h0, h0, 0, start[0], start[1], start_t)]
        came_from: Dict[Tuple[int,int,int], Optional[Tuple[int,int,int]]] = \
            {(start[0], start[1], start_t): None}
        g_score: Dict[Tuple[int,int,int], int] = {(start[0], start[1], start_t): 0}

        expanded = 0
        while open_q:
            f, h, g, x, y, t = heapq.heappop(open_q)
            if g > g_score.get((x, y, t), UNREACHABLE):
                continue                      # stale entry
            expanded += 1
            if expanded > self.MAX_NODES:
                return None

            if (x, y) == goal:
                return self._reconstruct(came_from, (x, y, t), start)
            if t >= start_t + self.MAX_T:
                continue

            nt = t + 1
            for nx, ny in wh.get_neighbors(x, y) + [(x, y)]:
                if (nx, ny) in blocked:
                    continue
                if not table.is_vertex_free(nx, ny, nt, robot_id):
                    continue
                if (nx, ny) != (x, y) and not table.is_edge_free(x, y, nx, ny, t, robot_id):
                    continue
                step = 1 + (soft_cost.get((nx, ny), 0) if (nx, ny) != (x, y) else 0)
                ng = g + step
                state = (nx, ny, nt)
                if ng < g_score.get(state, UNREACHABLE):
                    g_score[state]   = ng
                    came_from[state] = (x, y, t)
                    nh = dm[ny][nx]
                    heapq.heappush(open_q, (ng + nh, nh, ng, nx, ny, nt))
        return None

    @staticmethod
    def _reconstruct(came_from: Dict, end_state: Tuple, start: Cell) -> List[Cell]:
        cells: List[Cell] = []
        cur = end_state
        while cur is not None:
            cells.append((cur[0], cur[1]))
            cur = came_from[cur]
        cells.reverse()
        # strip wait steps (consecutive duplicates) and the start cell
        path: List[Cell] = []
        prev = start
        for c in cells:
            if c != prev:
                path.append(c)
                prev = c
        return path


# ─────────────────────────────────────────────────────────────
#  Local route planner (one per robot)
# ─────────────────────────────────────────────────────────────

class LocalRoutePlanner:
    """
    A robot plans ITS OWN route. The "reservation table" is built only from what its
    neighbours have broadcast (their next few path cells), so it is a local, partial view:
    distributed prioritised planning with no shared table and no coordinator.
    """

    def __init__(self, warehouse):
        self.warehouse = warehouse
        self.sta       = SpaceTimeAStar(warehouse)
        self.fallbacks = 0

    def plan(self, me_id: int, anchor: Cell, anchor_t: int, goal: Cell,
             neighbors, dead_cells) -> Optional[List[Cell]]:
        """Returns cells after `anchor` (no waits); None if goal unreachable."""
        table = ReservationTable()
        soft: Dict[Cell, int] = {}
        for n in neighbors:
            if getattr(n, "dead", False):
                continue
            a  = n.target_cell if n.target_cell is not None else n.grid_cell
            t0 = 1 if n.target_cell is not None else 0
            if n.path:
                table.reserve_path([a] + list(n.path), n.id, start_t=t0)
            else:
                soft[a] = soft.get(a, 0) + 3        # parked robot: avoid if cheap
        path = self.sta.plan(me_id, anchor, goal, table, start_t=anchor_t,
                             blocked=set(dead_cells), soft_cost=soft)
        if path is None:
            path = self.warehouse.shortest_path(anchor, goal)
            if path is not None:
                self.fallbacks += 1
        return path
