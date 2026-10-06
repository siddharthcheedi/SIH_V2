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
  * The reservation table is rebuilt from robots' live paths whenever we plan, so it
    can't go stale (the old one was only refreshed every 15 ticks).
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
#  Windowed MAPF Planner
# ─────────────────────────────────────────────────────────────

class WindowedMAPFPlanner:
    """
    plan() takes the robots that NEED a route (new task, blocked, invalid path) and plans
    them one by one in priority order, each against the live paths of everybody else
    plus the routes planned a moment ago in the same call.
    """

    def __init__(self, warehouse, step_ticks: int = 4):
        self.warehouse  = warehouse
        self.sta        = SpaceTimeAStar(warehouse)
        self.table      = ReservationTable()
        self.step_ticks = max(1, step_ticks)      # ticks per cell-step (info only)
        self.fallbacks  = 0                       # how often STA* gave up (diagnostics)

    def set_warehouse(self, warehouse) -> None:
        self.warehouse = warehouse
        self.sta.warehouse = warehouse

    # ---- helpers -----------------------------------------------------------

    @staticmethod
    def _anchor(robot) -> Tuple[Cell, int]:
        """(cell the robot will stand on next, steps from now until it does)."""
        if robot.target_cell is not None:
            return robot.target_cell, 1
        return robot.grid_cell, 0

    # ---- main entry --------------------------------------------------------

    def plan(self, to_plan: List, all_robots: List, tick: int,
             goal_fn: Callable) -> Dict[int, Optional[List[Cell]]]:
        """
        Returns {robot_id: path}; path None = goal unreachable.
        Everything is expressed in cell-steps relative to "now".
        """
        table = ReservationTable()
        planning_ids = {r.id for r in to_plan}
        blocked: Set[Cell] = set()
        soft: Dict[Cell, int] = {}

        for r in all_robots:
            if not r.active:
                continue
            if r.battery <= 0:                       # dead robot = permanent wall
                blocked.add(r.grid_cell)
                if r.target_cell is not None:
                    blocked.add(r.target_cell)
                continue
            if r.id in planning_ids:
                continue
            anchor, t0 = self._anchor(r)
            if r.path:
                table.reserve_path([anchor] + list(r.path), r.id, start_t=t0)
            else:
                # parked / idle robot: soft-avoid its cell (PIBT can still nudge it)
                soft[anchor] = soft.get(anchor, 0) + 3

        results: Dict[int, Optional[List[Cell]]] = {}
        for r in sorted(to_plan, key=lambda x: (-x.priority_score(), x.id)):
            goal = goal_fn(r)
            anchor, t0 = self._anchor(r)
            if goal is None:
                results[r.id] = []
                continue
            if anchor == goal:
                results[r.id] = []
                continue
            path = self.sta.plan(r.id, anchor, goal, table, start_t=t0,
                                 blocked=blocked, soft_cost=soft)
            if path is None:
                # search budget / reservations boxed us in: plain shortest path instead
                path = self.warehouse.shortest_path(anchor, goal)
                if path is not None:
                    self.fallbacks += 1
            results[r.id] = path
            if path:
                table.reserve_path([anchor] + path, r.id, start_t=t0)

        self.table = table
        return results

    # compatibility with older callers
    def invalidate_robot(self, robot_id: int) -> None:
        pass   # the table is rebuilt from live paths on every plan(); nothing to invalidate

    def get_viz_snapshot(self) -> List[Dict]:
        return self.table.snapshot_for_viz()
