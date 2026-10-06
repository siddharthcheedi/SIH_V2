"""
pibt.py — Priority Inheritance with Backtracking (PIBT) for MAPF.

Reference: Okumura et al., "Priority Inheritance with Backtracking
for Iterative Multi-Agent Path Finding", AIJ 2022.

One call to solve() decides ONE step for every robot that is AT REST.
Robots that are in transit are not touched: they keep moving, and both of the cells
they occupy (from + to) are treated as walls for everyone else.

Safety rules enforced here (each one closes a real collision case):
  1. A cell is claimed by at most one robot  (no vertex conflicts).
  2. A pushed robot may only move STRAIGHT AHEAD, away from the robot pushing it.
     Hence no swaps (head-on pass-through) and no sideways "corner cutting".
  3. A robot may step into a cell its occupant is leaving only if both move in the same
     direction (convoy). Turning a corner behind another robot puts the two bodies
     0.71 cells apart, which is less than the 0.8 combined radius -> not allowed.
  4. A follower must not be faster than the leader it follows.
  5. No rotation cycles (same corner geometry problem as rule 3).

Priority inheritance: if robot A wants a cell where robot B sits, B is asked to move
on A's behalf (recursively). If B cannot, A tries its next-best cell, and B is flagged
so that, when B is idle, it voluntarily steps aside next tick.

Ranking of cells uses TRUE walking distance to the goal (BFS map), not Manhattan.
"""

from __future__ import annotations
import heapq
import random
from collections import Counter
from typing import Dict, List, Tuple, Optional, Set

from .warehouse import Warehouse, FREE, SPAWN, DROP, PICKUP
from .robot import Robot

Cell = Tuple[int, int]
YIELD_REQUEST_TTL = 12   # ticks an "please step aside" request stays valid


class PIBT:
    def __init__(self, warehouse: Warehouse, seed: int = 7):
        self.wh  = warehouse
        self.rng = random.Random(seed)
        self._yield_requests: Dict[int, int] = {}      # robot_id -> tick requested
        # Visualization output from last solve()
        self.viz: Dict = {
            "priorities": {}, "claimed_cells": [], "conflicts": [], "waiting": [],
        }

    # ──────────────────────────────────────────────────────────────────────────
    # Main solver
    # ──────────────────────────────────────────────────────────────────────────

    def solve(self, robots: List[Robot], tick: int = 0) -> Dict[int, Cell]:
        """
        Returns {robot_id: next_cell} for every robot that is at rest and alive.
        next_cell == robot.grid_cell means "stay".
        """
        self._tick      = tick
        self._claims:   Dict[Cell, int]   = {}   # cell -> robot_id (next tick)
        self._decided:  Dict[int, Cell]   = {}   # robot_id -> next cell
        self._stack:    Set[int]          = set()
        self._blocked:  Set[Cell]         = set()
        self._rest:     Dict[Cell, Robot] = {}   # cell -> movable robot sitting there
        self._conflicts: List[Dict]       = []

        for r in robots:
            if not r.active:
                continue
            if r.battery <= 0 or r.target_cell is not None:
                # dead, or in transit: occupies both its cells until it arrives
                self._blocked.add(r.grid_cell)
                if r.target_cell is not None:
                    self._blocked.add(r.target_cell)
            else:
                self._rest[r.grid_cell] = r

        # Where do robots plan to be soon? Used to nudge idle robots off busy lanes.
        self._traffic: Counter = Counter()
        for r in robots:
            if r.active:
                for c in r.path[:3]:
                    self._traffic[c] += 1

        # forget stale yield requests
        self._yield_requests = {rid: t for rid, t in self._yield_requests.items()
                                if tick - t <= YIELD_REQUEST_TTL}

        order = sorted(self._rest.values(),
                       key=lambda r: (-r.priority_score(), r.id))
        for rank, r in enumerate(order):
            r.viz["priority"] = rank
            r.viz["in_conflict"] = False
            if r.id not in self._decided:
                self._plan(r, None)

        waiting = [r.id for r in order
                   if self._decided.get(r.id) == r.grid_cell
                   and r.goal is not None and r.goal != r.grid_cell]
        for r in order:
            r.viz["claimed_cell"] = list(self._decided.get(r.id, r.grid_cell))

        self.viz = {
            "priorities":    {r.id: i for i, r in enumerate(order)},
            "claimed_cells": [[c[0], c[1], rid] for c, rid in self._claims.items()],
            "conflicts":     self._conflicts,
            "waiting":       waiting,
        }
        return dict(self._decided)

    # ──────────────────────────────────────────────────────────────────────────
    # Recursive step
    # ──────────────────────────────────────────────────────────────────────────

    def _plan(self, r: Robot, pusher: Optional[Robot]) -> bool:
        """
        Decide robot r's next cell. `pusher` is the robot that wants r's cell (priority
        inheritance). Returns True if r moves away / found a cell, False if r had to stay.
        """
        cur = r.grid_cell
        self._stack.add(r.id)

        if pusher is None:
            candidates = self._candidates(r)
        else:
            # Pushed: ONLY straight ahead (same direction the pusher is travelling).
            px, py = pusher.grid_cell
            candidates = [(cur[0] + (cur[0] - px), cur[1] + (cur[1] - py))]

        first_choice = candidates[0] if candidates else cur

        for v in candidates:
            if v == cur and pusher is None:
                # staying: legal as long as nobody (permanently) holds my cell
                if self._claims.get(cur, r.id) != r.id:
                    continue
                self._claims[cur] = r.id
                self._decided[r.id] = cur
                self._stack.discard(r.id)
                self._note_conflict(r, first_choice, v)
                return True

            if v in self._blocked or not self.wh.is_passable(*v):
                continue
            holder = self._claims.get(v)
            if holder is not None and holder != r.id:
                continue

            occ = self._rest.get(v)
            if occ is not None and occ is not r:
                if occ.id in self._stack:                     # rule 5: no cycles
                    continue
                if occ.preferred_speed + 1e-9 < r.preferred_speed:   # rule 4
                    continue
                if occ.id in self._decided:
                    w = self._decided[occ.id]
                    if w == v:
                        continue                              # occupant stays
                    # rule 3: convoy only if moving in the same direction
                    if (w[0] - v[0], w[1] - v[1]) != (v[0] - cur[0], v[1] - cur[1]):
                        continue
                else:
                    self._claims[v] = r.id                    # tentative
                    if not self._plan(occ, r):                # ask occupant to move on
                        # Occupant can't go straight ahead. That does NOT mean it can't
                        # move at all, so we leave it UNDECIDED (it plans for itself on
                        # its own turn) and just withdraw our tentative claim. Marking it
                        # "stay" here froze robots that had a free cell beside them.
                        if self._claims.get(v) == r.id:
                            del self._claims[v]
                        # an idle occupant will step aside on its own next tick
                        self._yield_requests[occ.id] = self._tick
                        continue

            self._claims[v] = r.id
            self._decided[r.id] = v
            self._stack.discard(r.id)
            self._note_conflict(r, first_choice, v)
            return True

        self._stack.discard(r.id)
        if pusher is not None:
            return False      # pushed robot: report failure, stay undecided (see above)

        # top-level robot with nothing workable -> stay put
        self._claims[cur] = r.id
        self._decided[r.id] = cur
        self._note_conflict(r, first_choice, cur)
        return False

    def _note_conflict(self, r: Robot, first: Cell, got: Cell) -> None:
        if pusher_free(first, got):
            return
        r.viz["in_conflict"] = True
        occ = self._rest.get(first)
        self._conflicts.append({"from_id": r.id,
                                "to_id": occ.id if occ else -1,
                                "cell": list(first)})

    # ──────────────────────────────────────────────────────────────────────────
    # Candidate ordering
    # ──────────────────────────────────────────────────────────────────────────

    def _candidates(self, r: Robot) -> List[Cell]:
        wh, cur = self.wh, r.grid_cell
        nbrs = wh.get_neighbors(*cur)
        goal = r.goal

        if goal is not None and goal != cur:
            dm = wh.dist_map(goal)
            dv = lambda c: dm[c[1]][c[0]]
            ranked = sorted(nbrs, key=lambda c: (dv(c), self._traffic[c], self.rng.random()))
            stay_key = dv(cur) + 0.5      # staying beats sidesteps, loses to progress
            out: List[Cell] = []
            placed = False
            for c in ranked:
                if not placed and dv(c) >= stay_key:
                    out.append(cur); placed = True
                out.append(c)
            if not placed:
                out.append(cur)
            # Prefer the globally planned (congestion-aware) route cell if it is valid
            if r.path and r.path[0] in nbrs:
                out.remove(r.path[0])
                out.insert(0, r.path[0])
            r.desired_cell = out[0] if out[0] != cur else None
            return out

        # No goal (idle) or already at goal.
        r.desired_cell = None
        special = wh.cell(*cur) in (DROP, PICKUP)
        asked   = r.id in self._yield_requests
        spare   = sorted(nbrs, key=lambda c: (self._traffic[c], self.rng.random()))
        if asked or special:
            # Step aside, preferably onto an ordinary floor cell (don't hop DROP->DROP).
            ordinary = [c for c in spare if wh.cell(*c) in (FREE, SPAWN)]
            if ordinary:
                return ordinary + [cur]
            if asked:
                return spare + [cur]
        return [cur] + spare

    # ──────────────────────────────────────────────────────────────────────────
    # A* pathfinder (kept for API compatibility)
    # ──────────────────────────────────────────────────────────────────────────

    def a_star(self, start: Cell, goal: Cell,
               avoid: Optional[Set[Cell]] = None) -> List[Cell]:
        if start == goal:
            return []
        avoid = avoid or set()
        open_set: List[Tuple[int, Cell]] = [(0, start)]
        came_from: Dict[Cell, Optional[Cell]] = {start: None}
        g: Dict[Cell, int] = {start: 0}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                break
            for nxt in self.wh.get_neighbors(*current):
                if nxt in avoid and nxt != goal:
                    continue
                new_g = g[current] + 1
                if nxt not in g or new_g < g[nxt]:
                    g[nxt] = new_g
                    h = abs(goal[0]-nxt[0]) + abs(goal[1]-nxt[1])
                    heapq.heappush(open_set, (new_g + h, nxt))
                    came_from[nxt] = current
        if goal not in came_from:
            return []
        path, cur = [], goal
        while cur != start:
            path.append(cur)
            cur = came_from[cur]  # type: ignore
        path.reverse()
        return path

    def get_viz(self) -> Dict:
        return self.viz


def pusher_free(first: Cell, got: Cell) -> bool:
    """True when the robot got its first-choice cell (no conflict to report)."""
    return first == got
