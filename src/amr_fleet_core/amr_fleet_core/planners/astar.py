"""
astar.py — Custom A* pathfinding on a grid derived from the static map.

8-connected grid with Euclidean heuristic. Returns a list of grid waypoints
that are then smoothed by spline_smoother.py into a continuous path.

Zero ROS2 dependencies — pure Python, tested with plain pytest.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple


@dataclass(frozen=True)
class GridCell:
    """Integer (row, col) on the planning grid."""
    row: int
    col: int


@dataclass(order=True)
class _Node:
    """Priority queue entry for A*."""
    f: float
    g: float = field(compare=False)
    cell: GridCell = field(compare=False)
    parent: Optional[GridCell] = field(compare=False, default=None)


# 8-connected neighbors: (drow, dcol, cost)
_NEIGHBORS_8 = [
    (-1,  0, 1.0),   # N
    ( 1,  0, 1.0),   # S
    ( 0, -1, 1.0),   # W
    ( 0,  1, 1.0),   # E
    (-1, -1, 1.414), # NW
    (-1,  1, 1.414), # NE
    ( 1, -1, 1.414), # SW
    ( 1,  1, 1.414), # SE
]


def _euclidean(a: GridCell, b: GridCell) -> float:
    """Euclidean distance heuristic."""
    dr = a.row - b.row
    dc = a.col - b.col
    return math.sqrt(dr * dr + dc * dc)


class OccupancyGrid:
    """
    Binary occupancy grid for pathfinding.

    Can be constructed from a PGM map image or from a simple 2D bool array
    for testing.
    """

    def __init__(self, width: int, height: int,
                 occupied: Optional[Set[Tuple[int, int]]] = None) -> None:
        self.width = width
        self.height = height
        # (row, col) → True if occupied
        self._occupied: Set[Tuple[int, int]] = occupied or set()

    def is_free(self, row: int, col: int) -> bool:
        """Check if a cell is free (not occupied and in bounds)."""
        if row < 0 or row >= self.height or col < 0 or col >= self.width:
            return False
        return (row, col) not in self._occupied

    def is_occupied(self, row: int, col: int) -> bool:
        return not self.is_free(row, col)

    @classmethod
    def from_pgm(cls, pgm_path: str, resolution: float,
                 origin_x: float, origin_y: float,
                 occupied_threshold: int = 128) -> 'OccupancyGrid':
        """
        Load an occupancy grid from a PGM file.
        Pixels with value < occupied_threshold are considered occupied.
        """
        try:
            from PIL import Image
        except ImportError:
            raise ImportError("Pillow required: pip install Pillow")

        img = Image.open(pgm_path).convert('L')
        width, height = img.size
        occupied = set()
        for row in range(height):
            for col in range(width):
                if img.getpixel((col, row)) < occupied_threshold:
                    occupied.add((row, col))

        grid = cls(width=width, height=height, occupied=occupied)
        grid.resolution = resolution
        grid.origin_x = origin_x
        grid.origin_y = origin_y
        return grid

    def world_to_grid(self, x: float, y: float) -> GridCell:
        """Convert world coordinates to grid cell (requires from_pgm metadata)."""
        col = int((x - self.origin_x) / self.resolution)
        row = self.height - 1 - int((y - self.origin_y) / self.resolution)
        col = max(0, min(col, self.width - 1))
        row = max(0, min(row, self.height - 1))
        return GridCell(row=row, col=col)

    def grid_to_world(self, cell: GridCell) -> Tuple[float, float]:
        """Convert grid cell to world coordinates center."""
        x = self.origin_x + (cell.col + 0.5) * self.resolution
        y = self.origin_y + (self.height - 1 - cell.row + 0.5) * self.resolution
        return (x, y)


def astar(
    grid: OccupancyGrid,
    start: GridCell,
    goal: GridCell,
) -> Optional[List[GridCell]]:
    """
    A* search on an 8-connected occupancy grid.

    Parameters
    ----------
    grid : OccupancyGrid
    start : GridCell
    goal : GridCell

    Returns
    -------
    list of GridCell from start to goal (inclusive), or None if no path exists.
    """
    if grid.is_occupied(start.row, start.col):
        return None
    if grid.is_occupied(goal.row, goal.col):
        return None

    open_set: List[_Node] = []
    start_node = _Node(f=_euclidean(start, goal), g=0.0, cell=start)
    heapq.heappush(open_set, start_node)

    came_from: Dict[GridCell, GridCell] = {}
    g_score: Dict[GridCell, float] = {start: 0.0}

    while open_set:
        current_node = heapq.heappop(open_set)
        current = current_node.cell

        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for dr, dc, step_cost in _NEIGHBORS_8:
            nr, nc = current.row + dr, current.col + dc
            neighbor = GridCell(row=nr, col=nc)

            if not grid.is_free(nr, nc):
                continue

            # For diagonal moves, check that both adjacent cells are free
            # (prevents corner-cutting through obstacles)
            if dr != 0 and dc != 0:
                if not grid.is_free(current.row + dr, current.col):
                    continue
                if not grid.is_free(current.row, current.col + dc):
                    continue

            tentative_g = g_score[current] + step_cost

            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + _euclidean(neighbor, goal)
                heapq.heappush(open_set, _Node(f=f, g=tentative_g,
                                                cell=neighbor))

    return None  # No path found
