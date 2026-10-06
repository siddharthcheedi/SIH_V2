"""
warehouse.py — Configurable warehouse grid with layout support.

Cell types:
  FREE (0)     — passable floor
  OBSTACLE (1) — static wall / shelf body (impassable)
  DROP (2)     — drop-zone (passable, robots deliver here)
  PICKUP (3)   — pickup point (passable, shelf edge where goods are)
  SPAWN (4)    — robot spawn point (passable)
"""

from __future__ import annotations
import numpy as np
import json
from collections import deque
from pathlib import Path
from typing import List, Optional, Tuple, Set, Dict

FREE     = 0
OBSTACLE = 1
DROP     = 2
PICKUP   = 3
SPAWN    = 4

UNREACHABLE = 10**6

CELL_NAMES = {FREE: "free", OBSTACLE: "obstacle", DROP: "drop", PICKUP: "pickup", SPAWN: "spawn"}


class Warehouse:
    def __init__(self, width: int = 25, height: int = 20):
        self.width  = width
        self.height = height
        self.grid   = np.zeros((height, width), dtype=np.int8)  # grid[y][x]
        self.dynamic_obstacles: Set[Tuple[int,int]] = set()
        self.name   = "Custom Warehouse"
        # BFS distance maps, keyed by goal cell. Invalidated on ANY map change.
        self._dist_cache: Dict[Tuple[int,int], List[List[int]]] = {}

    # ── cell access ─────────────────────────────────────────────────────────

    def cell(self, x: int, y: int) -> int:
        if 0 <= x < self.width and 0 <= y < self.height:
            return int(self.grid[y][x])
        return OBSTACLE

    def set_cell(self, x: int, y: int, ctype: int) -> None:
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = ctype
            self._dist_cache.clear()

    def is_passable(self, x: int, y: int) -> bool:
        if not (0 <= x < self.width and 0 <= y < self.height):
            return False
        if (x, y) in self.dynamic_obstacles:
            return False
        return int(self.grid[y][x]) != OBSTACLE

    def get_neighbors(self, x: int, y: int) -> List[Tuple[int,int]]:
        result = []
        for dx, dy in ((0,1),(0,-1),(1,0),(-1,0)):
            nx, ny = x+dx, y+dy
            if self.is_passable(nx, ny):
                result.append((nx, ny))
        return result

    # ── special cell lists ───────────────────────────────────────────────────

    @property
    def drop_zones(self) -> List[Tuple[int,int]]:
        ys, xs = np.where(self.grid == DROP)
        return list(zip(xs.tolist(), ys.tolist()))

    @property
    def pickup_points(self) -> List[Tuple[int,int]]:
        ys, xs = np.where(self.grid == PICKUP)
        return list(zip(xs.tolist(), ys.tolist()))

    @property
    def spawn_points(self) -> List[Tuple[int,int]]:
        ys, xs = np.where(self.grid == SPAWN)
        return list(zip(xs.tolist(), ys.tolist()))

    # ── dynamic obstacles ────────────────────────────────────────────────────

    def toggle_obstacle(self, x: int, y: int) -> bool:
        """Toggle a dynamic obstacle. Returns True if now blocked."""
        self._dist_cache.clear()
        if (x, y) in self.dynamic_obstacles:
            self.dynamic_obstacles.discard((x, y))
            return False
        else:
            if self.is_passable(x, y):
                self.dynamic_obstacles.add((x, y))
                return True
            return False

    def block(self, x: int, y: int) -> None:
        self.dynamic_obstacles.add((x, y))
        self._dist_cache.clear()

    def unblock(self, x: int, y: int) -> None:
        self.dynamic_obstacles.discard((x, y))
        self._dist_cache.clear()

    def clear_dynamic_obstacles(self) -> None:
        self.dynamic_obstacles.clear()
        self._dist_cache.clear()

    # ── serialization ────────────────────────────────────────────────────────

    def to_dict(self) -> Dict:
        return {
            "name":   self.name,
            "width":  self.width,
            "height": self.height,
            "grid":   self.grid.tolist(),
        }

    @classmethod
    def from_dict(cls, data: Dict) -> "Warehouse":
        w = cls(data["width"], data["height"])
        w.name  = data.get("name", "Warehouse")
        w.grid  = np.array(data["grid"], dtype=np.int8)
        return w

    def save(self, path: str) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2))

    @classmethod
    def load(cls, path: str) -> "Warehouse":
        data = json.loads(Path(path).read_text())
        return cls.from_dict(data)

    def get_state(self) -> Dict:
        """Serializable state for the frontend."""
        return {
            "name":              self.name,
            "width":             self.width,
            "height":            self.height,
            "grid":              self.grid.tolist(),
            "dynamic_obstacles": [list(o) for o in self.dynamic_obstacles],
            "drop_zones":        [list(z) for z in self.drop_zones],
            "pickup_points":     [list(p) for p in self.pickup_points],
            "spawn_points":      [list(s) for s in self.spawn_points],
        }

    def reset_dynamic(self) -> None:
        self.dynamic_obstacles.clear()
        self._dist_cache.clear()

    # ── true (walking) distances ─────────────────────────────────────────────
    # Euclidean / Manhattan distance is badly wrong in a warehouse: a robot can
    # be 2 cells from a pickup "as the crow flies" and 30 cells away by aisle.
    # Everything that ranks cells or robots (PIBT, STA*, bidding) uses this.

    def dist_map(self, goal: Tuple[int,int]) -> List[List[int]]:
        """BFS distance (in steps) from every cell to `goal`. dist_map(g)[y][x]."""
        d = self._dist_cache.get(goal)
        if d is not None:
            return d
        d = [[UNREACHABLE] * self.width for _ in range(self.height)]
        gx, gy = goal
        if 0 <= gx < self.width and 0 <= gy < self.height:
            d[gy][gx] = 0
            q = deque([(gx, gy)])
            while q:
                x, y = q.popleft()
                nd = d[y][x] + 1
                for nx, ny in self.get_neighbors(x, y):
                    if d[ny][nx] == UNREACHABLE:
                        d[ny][nx] = nd
                        q.append((nx, ny))
        self._dist_cache[goal] = d
        return d

    def dist(self, a: Tuple[int,int], goal: Tuple[int,int]) -> int:
        x, y = a
        if not (0 <= x < self.width and 0 <= y < self.height):
            return UNREACHABLE
        return self.dist_map(goal)[y][x]

    def shortest_path(self, start: Tuple[int,int],
                      goal: Tuple[int,int]) -> Optional[List[Tuple[int,int]]]:
        """Plain shortest path (start excluded, goal included). None if unreachable."""
        if start == goal:
            return []
        dm = self.dist_map(goal)
        if dm[start[1]][start[0]] >= UNREACHABLE:
            return None
        path, cur = [], start
        while cur != goal:
            cur = min(self.get_neighbors(*cur), key=lambda c: dm[c[1]][c[0]])
            path.append(cur)
        return path
