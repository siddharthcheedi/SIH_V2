"""
spline_smoother.py — Cubic spline path smoothing.

Takes raw A* grid waypoints and produces a smooth, continuous path via
cubic spline interpolation. The output is dense enough for DWB to follow
without zig-zag artifacts from the grid.

Zero ROS2 dependencies — pure Python + scipy/numpy (both available in the
ROS2 environment). Falls back to simple linear interpolation if scipy is
not installed (e.g. in a minimal test environment).
"""

from __future__ import annotations

import math
from typing import List, Tuple


def _linear_interpolate(
    waypoints: List[Tuple[float, float]],
    point_spacing: float,
) -> List[Tuple[float, float]]:
    """Fallback: densify path via linear interpolation between waypoints."""
    if len(waypoints) < 2:
        return list(waypoints)

    result: List[Tuple[float, float]] = [waypoints[0]]
    for i in range(1, len(waypoints)):
        x0, y0 = waypoints[i - 1]
        x1, y1 = waypoints[i]
        dx, dy = x1 - x0, y1 - y0
        segment_len = math.sqrt(dx * dx + dy * dy)

        if segment_len < 1e-9:
            continue

        n_points = max(1, int(segment_len / point_spacing))
        for j in range(1, n_points + 1):
            t = j / n_points
            result.append((x0 + dx * t, y0 + dy * t))

    return result


def smooth_path(
    waypoints: List[Tuple[float, float]],
    point_spacing: float = 0.05,
    smooth: bool = True,
) -> List[Tuple[float, float]]:
    """
    Smooth a list of (x, y) waypoints into a dense, continuous path.

    Uses cubic spline interpolation if scipy is available, otherwise
    falls back to linear interpolation with densification.

    Parameters
    ----------
    waypoints : list of (x, y) tuples
        Coarse waypoints from A*.
    point_spacing : float
        Target spacing between output points (meters).
    smooth : bool
        If False, skip smoothing and just densify linearly.

    Returns
    -------
    list of (x, y) tuples
        Dense path suitable for DWB to follow.
    """
    if len(waypoints) < 2:
        return list(waypoints)

    if not smooth:
        return _linear_interpolate(waypoints, point_spacing)

    try:
        import numpy as np
        from scipy.interpolate import CubicSpline
    except ImportError:
        # Fall back to linear if scipy not available
        return _linear_interpolate(waypoints, point_spacing)

    # Compute cumulative arc-length as the spline parameter
    xs = [p[0] for p in waypoints]
    ys = [p[1] for p in waypoints]

    dists = [0.0]
    for i in range(1, len(waypoints)):
        dx = xs[i] - xs[i - 1]
        dy = ys[i] - ys[i - 1]
        dists.append(dists[-1] + math.sqrt(dx * dx + dy * dy))

    total_len = dists[-1]
    if total_len < 1e-6:
        return list(waypoints)

    t = np.array(dists)

    # Fit cubic splines parametrized by arc length
    cs_x = CubicSpline(t, xs, bc_type='natural')
    cs_y = CubicSpline(t, ys, bc_type='natural')

    # Sample at the desired point spacing
    n_points = max(2, int(total_len / point_spacing))
    t_dense = np.linspace(0, total_len, n_points)

    x_dense = cs_x(t_dense)
    y_dense = cs_y(t_dense)

    return list(zip(x_dense.tolist(), y_dense.tolist()))


def path_smoothness(path: List[Tuple[float, float]]) -> float:
    """
    Compute a smoothness metric: average absolute turning angle (radians).
    Lower is smoother. Used for testing.
    """
    if len(path) < 3:
        return 0.0

    total_angle = 0.0
    count = 0
    for i in range(1, len(path) - 1):
        dx1 = path[i][0] - path[i - 1][0]
        dy1 = path[i][1] - path[i - 1][1]
        dx2 = path[i + 1][0] - path[i][0]
        dy2 = path[i + 1][1] - path[i][1]

        mag1 = math.sqrt(dx1 * dx1 + dy1 * dy1)
        mag2 = math.sqrt(dx2 * dx2 + dy2 * dy2)

        if mag1 < 1e-9 or mag2 < 1e-9:
            continue

        cos_angle = (dx1 * dx2 + dy1 * dy2) / (mag1 * mag2)
        cos_angle = max(-1.0, min(1.0, cos_angle))  # clamp for float error
        angle = math.acos(cos_angle)
        total_angle += abs(angle)
        count += 1

    return total_angle / max(1, count)
