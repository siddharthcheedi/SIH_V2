"""
orca.py — Optimal Reciprocal Collision Avoidance (ORCA) for AMR fleets.

Reference: van den Berg et al., "Reciprocal n-Body Collision Avoidance", ISRR 2011.

Each robot independently computes a safe velocity by:
1. Computing velocity obstacles (VOs) with each neighbor
2. Converting to ORCA half-planes (each robot takes half the avoidance responsibility)
3. Solving a 2D linear program to find the closest safe velocity to its preferred velocity
4. Falling back to zero velocity if no feasible velocity exists

Returns velocity corrections and visualization data (half-planes, velocity vectors).
"""

from __future__ import annotations
import math
from typing import List, Tuple, Dict, Optional

from .robot import Robot


# ── geometry helpers ──────────────────────────────────────────────────────────

def dot2(ax, ay, bx, by):    return ax*bx + ay*by
def cross2(ax, ay, bx, by):  return ax*by - ay*bx
def norm2(x, y):             return math.sqrt(x*x + y*y)


def closest_point_on_line_segment(px, py, ax, ay, bx, by):
    """Closest point on segment [A,B] to point P."""
    abx, aby = bx-ax, by-ay
    ab2 = abx*abx + aby*aby
    if ab2 < 1e-12:
        return ax, ay
    t = max(0.0, min(1.0, dot2(px-ax, py-ay, abx, aby) / ab2))
    return ax + t*abx, ay + t*aby


# ── half-plane LP ─────────────────────────────────────────────────────────────

def solve_lp_2d(half_planes, pref_vx, pref_vy, max_speed):
    """
    Solve 2D LP: find point in intersection of half-planes closest to (pref_vx, pref_vy).
    Half-plane format: (nx, ny, px, py) meaning (v - P) · N >= 0.
    Falls back to stopping if infeasible.
    """
    vx, vy = pref_vx, pref_vy

    # Clamp preferred to max speed
    spd = norm2(vx, vy)
    if spd > max_speed:
        vx, vy = vx/spd*max_speed, vy/spd*max_speed

    for i, (nx, ny, px, py) in enumerate(half_planes):
        # Check if current velocity satisfies this plane
        if dot2(vx-px, vy-py, nx, ny) >= -1e-6:
            continue

        # Current velocity violates plane i — project onto its boundary
        # The boundary line passes through (px,py) with direction perpendicular to (nx,ny)
        # i.e., direction = (-ny, nx)
        # The closest point on this line to pref that also satisfies all previous planes

        # Line direction tangent to plane i
        tx, ty = -ny, nx

        # Project origin of line that also satisfies max-speed
        # Parametric: (px + t*tx, py + t*ty)
        # |pt|^2 <= max_speed^2
        dot_pp = dot2(px, py, px, py)
        dot_pt = dot2(px, py, tx, ty)
        if max_speed*max_speed - dot_pp + dot_pt*dot_pt < 0:
            # Cannot satisfy speed constraint — stop
            return 0.0, 0.0

        sqrt_d = math.sqrt(max(0, max_speed*max_speed - dot_pp + dot_pt*dot_pt))
        t_lo, t_hi = -dot_pt - sqrt_d, -dot_pt + sqrt_d

        # Further constrain with earlier half-planes
        for j in range(i):
            nx2, ny2, px2, py2 = half_planes[j]
            # (px+t*tx - px2)·n2 >= 0
            denom = dot2(tx, ty, nx2, ny2)
            num   = dot2(px2-px, py2-py, nx2, ny2)
            if abs(denom) < 1e-10:
                if num < -1e-6:
                    return 0.0, 0.0
                continue
            t_val = num / denom
            if denom > 0:
                t_hi = min(t_hi, t_val)
            else:
                t_lo = max(t_lo, t_val)
            if t_lo > t_hi + 1e-6:
                return 0.0, 0.0

        # Optimal t along the boundary line = closest to preferred velocity
        t_opt = dot2(pref_vx-px, pref_vy-py, tx, ty)
        t_opt = max(t_lo, min(t_hi, t_opt))
        vx    = px + t_opt*tx
        vy    = py + t_opt*ty

    return vx, vy


# ── ORCA main ────────────────────────────────────────────────────────────────

TIME_HORIZON      = 2.5   # seconds — look-ahead for VO computation
SAFETY_MARGIN     = 0.08  # extra clearance added to combined radius
SENSING_RADIUS    = 6.0   # cells — only consider neighbors within this range


def compute_orca(robots: List[Robot], dt: float = 0.1) -> Dict:
    """
    Compute ORCA velocities for all active robots.
    Returns:
        viz_data: dict with per-robot visualization info (preferred vel, orca vel, half-planes)
    """
    viz_data = {}

    active = [r for r in robots if r.active and r.battery > 0]

    for robot in active:
        # Preferred velocity: toward target_cell (or zero if no target)
        if robot.target_cell:
            tx, ty = robot.target_cell
            dx, dy = tx - robot.x, ty - robot.y
            d = norm2(dx, dy)
            if d > 1e-4:
                pref_vx = (dx/d) * robot.preferred_speed
                pref_vy = (dy/d) * robot.preferred_speed
            else:
                pref_vx, pref_vy = 0.0, 0.0
        else:
            pref_vx, pref_vy = 0.0, 0.0

        robot.viz["pref_vx"] = pref_vx
        robot.viz["pref_vy"] = pref_vy

        half_planes  = []
        viz_planes   = []

        neighbors = [
            r for r in active
            if r.id != robot.id and robot.distance_to(r) < SENSING_RADIUS
        ]

        for other in neighbors:
            rel_px = other.x - robot.x
            rel_py = other.y - robot.y
            rel_vx = robot.vx - other.vx
            rel_vy = robot.vy - other.vy

            dist      = norm2(rel_px, rel_py)
            comb_r    = robot.radius + other.radius + SAFETY_MARGIN

            # Truncated VO: treat as circle at rel_p/τ with radius r/τ
            inv_tau   = 1.0 / TIME_HORIZON
            w_x = rel_vx - rel_px * inv_tau
            w_y = rel_vy - rel_py * inv_tau
            w_len = norm2(w_x, w_y)

            if dist < comb_r:
                # Already overlapping — strong push directly apart
                u_len  = comb_r - dist + 0.01
                if dist > 1e-6:
                    nx_plane = -rel_px / dist
                    ny_plane = -rel_py / dist
                else:
                    nx_plane, ny_plane = 1.0, 0.0
                # Half-plane point and normal
                pp_x = robot.vx + nx_plane * u_len
                pp_y = robot.vy + ny_plane * u_len
                half_planes.append((nx_plane, ny_plane, pp_x, pp_y))
                viz_planes.append({"nx": nx_plane, "ny": ny_plane,
                                   "px": pp_x,     "py": pp_y,
                                   "other_id": other.id})
                continue

            # Distance from w to VO boundary circle
            # VO center in velocity space: c = rel_p * inv_tau
            cx = rel_px * inv_tau
            cy = rel_py * inv_tau
            r_VO = comb_r * inv_tau

            dcx = w_x - 0.0  # w relative to VO center (VO center relative to origin)
            dcy = w_y - 0.0
            # Actually: w = (rel_v - cx_VO, rel_v - cy_VO) is already relative
            # w_to_center:
            wx_c = w_x - cx
            wy_c = w_y - cy
            wc_len = norm2(wx_c, wy_c)

            if wc_len < r_VO:
                # w is inside VO — find closest boundary point
                if wc_len < 1e-9:
                    nx_plane, ny_plane = 1.0, 0.0
                else:
                    nx_plane = wx_c / wc_len
                    ny_plane = wy_c / wc_len
                # u = vector from w to VO boundary
                u_x = (cx + nx_plane * r_VO) - w_x
                u_y = (cy + ny_plane * r_VO) - w_y
            else:
                # Check tangent lines — simplified: use straight line from origin
                # If w is outside VO, no collision — skip
                continue

            # ORCA half-plane: robot takes half responsibility
            pp_x = robot.vx + 0.5 * u_x
            pp_y = robot.vy + 0.5 * u_y
            half_planes.append((nx_plane, ny_plane, pp_x, pp_y))
            viz_planes.append({"nx": nx_plane, "ny": ny_plane,
                               "px": pp_x,     "py": pp_y,
                               "other_id": other.id})

        # Solve LP
        new_vx, new_vy = solve_lp_2d(half_planes, pref_vx, pref_vy, robot.preferred_speed)

        robot.vx = new_vx
        robot.vy = new_vy
        robot.viz["orca_vx"]   = new_vx
        robot.viz["orca_vy"]   = new_vy
        robot.viz["orca_planes"] = viz_planes

        viz_data[robot.id] = {
            "pref_vel":   [pref_vx,  pref_vy],
            "orca_vel":   [new_vx,   new_vy],
            "half_planes": viz_planes,
            "neighbor_ids": [n.id for n in neighbors],
        }

    return viz_data
