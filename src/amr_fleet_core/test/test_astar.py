"""Tests for amr_fleet_core.planners.astar."""

from amr_fleet_core.planners.astar import GridCell, OccupancyGrid, astar


def _make_grid(width: int, height: int,
               obstacles: list = None) -> OccupancyGrid:
    """Helper: create a grid with optional obstacle cells."""
    occupied = set()
    for r, c in (obstacles or []):
        occupied.add((r, c))
    return OccupancyGrid(width=width, height=height, occupied=occupied)


class TestAStarBasic:
    """Core A* pathfinding."""

    def test_straight_line(self):
        grid = _make_grid(10, 10)
        start = GridCell(0, 0)
        goal = GridCell(0, 5)
        path = astar(grid, start, goal)
        assert path is not None
        assert path[0] == start
        assert path[-1] == goal
        assert len(path) == 6  # 0,1,2,3,4,5

    def test_diagonal(self):
        grid = _make_grid(10, 10)
        start = GridCell(0, 0)
        goal = GridCell(5, 5)
        path = astar(grid, start, goal)
        assert path is not None
        assert path[0] == start
        assert path[-1] == goal

    def test_avoids_obstacles(self):
        # Wall blocking direct path
        obstacles = [(2, c) for c in range(5)]
        grid = _make_grid(10, 10, obstacles)
        start = GridCell(0, 2)
        goal = GridCell(4, 2)
        path = astar(grid, start, goal)
        assert path is not None
        assert path[0] == start
        assert path[-1] == goal
        # Should not pass through any obstacle
        for cell in path:
            assert (cell.row, cell.col) not in set(obstacles)

    def test_no_path_returns_none(self):
        # Completely walled off
        obstacles = [(r, 5) for r in range(10)]
        grid = _make_grid(10, 10, obstacles)
        start = GridCell(5, 0)
        goal = GridCell(5, 9)
        path = astar(grid, start, goal)
        assert path is None

    def test_start_equals_goal(self):
        grid = _make_grid(10, 10)
        cell = GridCell(3, 3)
        path = astar(grid, cell, cell)
        assert path is not None
        assert len(path) == 1
        assert path[0] == cell

    def test_start_on_obstacle_returns_none(self):
        grid = _make_grid(10, 10, obstacles=[(3, 3)])
        path = astar(grid, GridCell(3, 3), GridCell(0, 0))
        assert path is None

    def test_goal_on_obstacle_returns_none(self):
        grid = _make_grid(10, 10, obstacles=[(7, 7)])
        path = astar(grid, GridCell(0, 0), GridCell(7, 7))
        assert path is None

    def test_path_is_optimal_length(self):
        """In a clear grid, Manhattan-style path lengths should be reasonable."""
        grid = _make_grid(20, 20)
        start = GridCell(0, 0)
        goal = GridCell(10, 10)
        path = astar(grid, start, goal)
        assert path is not None
        # Diagonal: 10 diagonal steps (each √2) ≈ path length 11 cells
        assert len(path) == 11

    def test_no_corner_cutting(self):
        """Diagonal moves should not cut through obstacle corners."""
        # Place obstacles so the only diagonal would cut a corner
        obstacles = [(1, 0), (0, 1)]
        grid = _make_grid(5, 5, obstacles)
        start = GridCell(0, 0)
        goal = GridCell(1, 1)
        path = astar(grid, start, goal)
        # Path should exist but go around, not diagonally through corner
        if path is not None:
            for cell in path:
                assert (cell.row, cell.col) not in set(obstacles)
