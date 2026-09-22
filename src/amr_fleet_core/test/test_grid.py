"""Tests for amr_fleet_core.negotiation.grid — coarse grid + chokepoint zones."""

import math

from amr_fleet_core.negotiation.grid import (
    CELL_SIZE,
    GRID_COLS,
    GRID_ROWS,
    CellCoord,
    CellReservation,
    cell_from_id,
    cell_id,
    cell_to_world,
    path_to_cell_reservations,
    point_in_zone,
    world_to_cell,
    zones_for_path,
    zones_for_point,
)
from amr_fleet_core.scenario import CHOKE_ZONES


class TestWorldToCellConversions:
    """Verify world↔cell coordinate conversions."""

    def test_origin_maps_to_center(self):
        c = world_to_cell(0.0, 0.0)
        # Origin (0,0) should be near the center of the grid
        assert 10 <= c.row <= 18
        assert 15 <= c.col <= 25

    def test_round_trip_consistency(self):
        """world→cell→world should land within one cell of the original."""
        for x, y in [(-8.0, -6.0), (0.0, 0.0), (8.0, 5.0), (-4.0, 3.0)]:
            c = world_to_cell(x, y)
            rx, ry = cell_to_world(c)
            assert abs(rx - x) <= CELL_SIZE
            assert abs(ry - y) <= CELL_SIZE

    def test_cell_id_round_trip(self):
        c = CellCoord(row=5, col=10)
        cid = cell_id(c)
        recovered = cell_from_id(cid)
        assert recovered == c

    def test_clamp_to_bounds(self):
        """Out-of-bounds world coords should clamp to grid edges."""
        c = world_to_cell(-100.0, -100.0)
        assert c.row >= 0
        assert c.col >= 0
        c2 = world_to_cell(100.0, 100.0)
        assert c2.row <= GRID_ROWS - 1
        assert c2.col <= GRID_COLS - 1

    def test_grid_dimensions(self):
        assert GRID_COLS == 40  # 20m / 0.5m
        assert GRID_ROWS == 28  # 14m / 0.5m


class TestChokeZones:
    """Verify chokepoint zone geometry."""

    def test_six_zones_defined(self):
        assert len(CHOKE_ZONES) == 6

    def test_zone_ids_are_sequential(self):
        ids = [z.zone_id for z in CHOKE_ZONES]
        assert ids == list(range(6))

    def test_point_in_south_aisle1_zone(self):
        # Aisle 1 south intersection at (-4, -5)
        assert point_in_zone(-4.0, -5.0, CHOKE_ZONES[0])
        assert point_in_zone(-3.5, -4.5, CHOKE_ZONES[0])  # within half_size
        assert not point_in_zone(0.0, -5.0, CHOKE_ZONES[0])  # wrong x

    def test_point_outside_all_zones(self):
        # Center of an aisle, not near any intersection
        zones = zones_for_point(0.0, 0.0)
        assert zones == []

    def test_zones_for_path_through_two_intersections(self):
        # Path from south cross-aisle through aisle 1 to north cross-aisle
        path = [(-4.0, -6.0), (-4.0, -5.0), (-4.0, 0.0), (-4.0, 3.0)]
        zones = zones_for_path(path)
        assert 0 in zones  # aisle_1_south
        assert 3 in zones  # aisle_1_north


class TestPathToCellReservations:
    """Verify path→cell reservation conversion."""

    def test_straight_line_produces_reservations(self):
        # Simple horizontal path
        waypoints = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        reservations = path_to_cell_reservations(waypoints, speed=1.0)
        assert len(reservations) > 0
        # All reservations should have positive time intervals
        for r in reservations:
            assert r.t_end > r.t_start

    def test_single_point_path(self):
        reservations = path_to_cell_reservations([(0.0, 0.0)], speed=1.0)
        assert len(reservations) == 1

    def test_empty_path(self):
        assert path_to_cell_reservations([], speed=1.0) == []

    def test_zero_speed_returns_empty(self):
        assert path_to_cell_reservations([(0, 0)], speed=0.0) == []

    def test_time_increases_along_path(self):
        waypoints = [(-8.0, -6.0), (-4.0, -6.0), (0.0, -6.0)]
        reservations = path_to_cell_reservations(waypoints, speed=0.9)
        # Each successive reservation's t_start should be >= previous t_start
        for i in range(1, len(reservations)):
            assert reservations[i].t_start >= reservations[i - 1].t_start

    def test_time_margin_applied(self):
        waypoints = [(0.0, 0.0), (2.0, 0.0)]
        margin = 1.0
        reservations = path_to_cell_reservations(
            waypoints, speed=1.0, start_time=5.0, time_margin=margin
        )
        # First reservation t_start should be start_time - margin
        assert reservations[0].t_start == 5.0 - margin

    def test_different_cells_produce_different_reservations(self):
        # Dense waypoints simulating a real Nav2 path (0.1m spacing over 16m)
        waypoints = [(-8.0 + i * 0.1, -6.0) for i in range(161)]
        reservations = path_to_cell_reservations(waypoints, speed=1.0)
        # Should produce many distinct cell reservations
        cells = set(r.cid for r in reservations)
        assert len(cells) > 5  # At least 5 distinct cells for 16m at 0.5m/cell
