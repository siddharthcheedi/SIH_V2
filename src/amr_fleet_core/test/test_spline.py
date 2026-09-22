"""Tests for amr_fleet_core.planners.spline_smoother."""

import math
from amr_fleet_core.planners.spline_smoother import smooth_path, path_smoothness


class TestSmoothPath:
    """Spline path smoothing tests."""

    def test_two_points_returns_dense_path(self):
        result = smooth_path([(0.0, 0.0), (1.0, 0.0)], point_spacing=0.1)
        assert len(result) >= 2
        # Endpoints should be preserved
        assert abs(result[0][0] - 0.0) < 0.01
        assert abs(result[-1][0] - 1.0) < 0.01

    def test_single_point_returns_itself(self):
        result = smooth_path([(3.0, 4.0)])
        assert len(result) == 1

    def test_empty_returns_empty(self):
        result = smooth_path([])
        assert result == []

    def test_zigzag_gets_smoother(self):
        """A raw A* zig-zag should be measurably smoother after spline."""
        zigzag = [
            (0.0, 0.0), (0.5, 0.5), (1.0, 0.0), (1.5, 0.5),
            (2.0, 0.0), (2.5, 0.5), (3.0, 0.0),
        ]
        raw_smoothness = path_smoothness(zigzag)
        smoothed = smooth_path(zigzag, point_spacing=0.05)
        smoothed_smoothness = path_smoothness(smoothed)
        # Smoothed path should have lower average turning angle
        assert smoothed_smoothness < raw_smoothness

    def test_straight_path_stays_straight(self):
        """A straight path should remain straight after smoothing."""
        straight = [(float(i), 0.0) for i in range(10)]
        smoothed = smooth_path(straight, point_spacing=0.1)
        # All y coordinates should stay near 0
        for x, y in smoothed:
            assert abs(y) < 0.01

    def test_linear_fallback(self):
        """smooth=False should produce a linearly interpolated path."""
        points = [(0.0, 0.0), (2.0, 0.0)]
        result = smooth_path(points, point_spacing=0.5, smooth=False)
        assert len(result) >= 4  # At least 4 points for 2m at 0.5m spacing
        for _, y in result:
            assert abs(y) < 0.01

    def test_output_is_dense(self):
        """Output spacing should be approximately point_spacing."""
        path = [(0.0, 0.0), (5.0, 0.0)]
        spacing = 0.1
        result = smooth_path(path, point_spacing=spacing)
        # Check that consecutive points are approximately `spacing` apart
        for i in range(1, min(5, len(result))):
            dx = result[i][0] - result[i - 1][0]
            dy = result[i][1] - result[i - 1][1]
            dist = math.sqrt(dx * dx + dy * dy)
            assert dist < spacing * 2  # Within 2× tolerance
