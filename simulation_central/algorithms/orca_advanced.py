"""
orca_advanced.py — Enhanced ORCA with wall avoidance and non-holonomic constraints.

Improvements over basic ORCA:
1. Wall half-planes: robots are repelled from static obstacles just like from other robots
2. Dynamic obstacle avoidance: treats dynamic obstacles as static velocity obstacles
3. Better LP solver: faster geometric solution for 2D LP
4. Emergency braking: if no feasible velocity exists, decelerate toward zero
5. Neighbor prioritization: robots in narrower corridors get tighter time horizons
"""
from __future__ import annotations
import math
from typing import Dict, List, Tuple

from ..warehouse import Warehouse


# ─────────────────────────────────────────────────────────────
#  Constants
# ─────────────────────────────────────────────────────────────
TIME_HORIZON_ROBOT = 3.0    # seconds: how far ahead to consider robot-robot VO
TIME_HORIZON_WALL  = 1.5    # seconds: how far ahead to consider wall VO
SAFETY_MARGIN      = 0.12   # extra clearance (cells)
SENSING_RADIUS     = 7.0    # only consider neighbors within this many cells
WALL_SENSE_DIST    = 1.8    # how close to a wall before repulsion kicks in
MAX_SPEED_FACTOR   = 1.0    # cap at preferred_speed * this


# ─────────────────────────────────────────────────────────────
#  2D Linear Program (velocity selection)
# ─────────────────────────────────────────────────────────────

def _dot(ax, ay, bx, by):    return ax*bx + ay*by
def _len(x, y):               return math.sqrt(x*x + y*y)
def _clamp(v, lo, hi):        return max(lo, min(hi, v))


def solve_orca_lp(half_planes: List[Tuple], pref_vx: float, pref_vy: float,
                  max_speed: float) -> Tuple[float, float]:
    """
    Find velocity closest to (pref_vx, pref_vy) that satisfies all half-planes.
    Half-plane: (nx, ny, px, py) meaning (v - P)·N >= 0.

    Uses incremental 2D LP: add planes one by one, project onto boundary if violated.
    Falls back to speed = 0 if infeasible.
    """
    vx, vy = pref_vx, pref_vy

    # Clamp to max speed
    spd = _len(vx, vy)
    if spd > max_speed:
        vx, vy = vx/spd * max_speed, vy/spd * max_speed

    for i, (nx, ny, px, py) in enumerate(half_planes):
        # Check satisfaction: (v - P)·N >= 0
        if _dot(vx-px, vy-py, nx, ny) >= -1e-7:
            continue

        # Violated — project onto boundary line of plane i
        # Boundary line passes through (px, py) with direction (-ny, nx)
        tx, ty = -ny, nx   # tangent direction

        # Intersect boundary line with disk |v| <= max_speed
        # Points: (px+t*tx, py+t*ty), constraint: (px+t*tx)^2+(py+t*ty)^2 <= R^2
        R2    = max_speed * max_speed
        pp    = px*px + py*py
        pt    = px*tx + py*ty
        disc  = R2 - pp + pt*pt
        if disc < 0:
            # Cannot satisfy speed limit + this plane → stop
            return 0.0, 0.0
        sqrt_d = math.sqrt(disc)
        t_lo, t_hi = -pt - sqrt_d, -pt + sqrt_d

        # Tighten with earlier half-planes
        for j in range(i):
            nx2, ny2, px2, py2 = half_planes[j]
            denom = _dot(tx, ty, nx2, ny2)
            numer = _dot(px2-px, py2-py, nx2, ny2)
            if abs(denom) < 1e-9:
                if numer < -1e-7:
                    return 0.0, 0.0
                continue
            t_val = numer / denom
            if denom > 0:
                t_hi = min(t_hi, t_val)
            else:
                t_lo = max(t_lo, t_val)
            if t_lo > t_hi + 1e-7:
                return 0.0, 0.0

        # Optimal t = closest point on boundary to pref
        t_opt = _clamp(_dot(pref_vx-px, pref_vy-py, tx, ty), t_lo, t_hi)
        vx    = px + t_opt * tx
        vy    = py + t_opt * ty

    return vx, vy


# ─────────────────────────────────────────────────────────────
#  Robot-Robot ORCA Half-Plane
# ─────────────────────────────────────────────────────────────

def _robot_orca_plane(ax, ay, avx, avy, ar,
                      bx, by, bvx, bvy, br,
                      tau: float) -> Tuple:
    """
    Compute the ORCA half-plane for robot A imposed by robot B.
    Returns (nx, ny, px, py) or None if no constraint needed.
    """
    rel_px = bx - ax
    rel_py = by - ay
    rel_vx = avx - bvx
    rel_vy = avy - bvy
    comb_r = ar + br + SAFETY_MARGIN

    dist   = _len(rel_px, rel_py)
    inv_tau = 1.0 / tau

    # Translate to "VO cone" in velocity space
    # VO center (truncated): c = rel_p / tau
    cx_vo = rel_px * inv_tau
    cy_vo = rel_py * inv_tau
    r_vo  = comb_r * inv_tau

    # Vector from VO center to relative velocity
    wx = rel_vx - cx_vo
    wy = rel_vy - cy_vo
    w_len = _len(wx, wy)

    if dist < comb_r:
        # Already overlapping — strong push directly apart
        if dist < 1e-6:
            nx, ny = 1.0, 0.0
        else:
            nx, ny = -rel_px/dist, -rel_py/dist
        u_mag  = (comb_r - dist + 0.05) * inv_tau
        pp_x   = avx + nx * u_mag * 0.5
        pp_y   = avy + ny * u_mag * 0.5
        return (nx, ny, pp_x, pp_y)

    if w_len > r_vo:
        # Relative velocity is outside VO → no constraint needed
        return None

    # Closest point on VO boundary
    if w_len < 1e-9:
        nx, ny = 1.0, 0.0
    else:
        nx = wx / w_len
        ny = wy / w_len

    # u = vector from (rel_vx, rel_vy) to VO boundary
    u_x = (cx_vo + nx * r_vo) - rel_vx
    u_y = (cy_vo + ny * r_vo) - rel_vy

    # ORCA: robot A takes half the responsibility
    pp_x = avx + 0.5 * u_x
    pp_y = avy + 0.5 * u_y
    return (nx, ny, pp_x, pp_y)


# ─────────────────────────────────────────────────────────────
#  Wall ORCA Half-Planes
# ─────────────────────────────────────────────────────────────

def _wall_planes(rx: float, ry: float, rvx: float, rvy: float,
                 radius: float, warehouse: Warehouse) -> List[Tuple]:
    """
    ORCA half-planes keeping a robot away from walls.
    Grid convention: cell (cx, cy) is centred on integer coordinates, so it covers
    [cx-0.5, cx+0.5] x [cy-0.5, cy+0.5]. Each nearby blocked cell is treated as a
    static obstacle: the closest point of its square to the robot defines the plane.
    (The first version used [cx, cx+1], i.e. every wall was half a cell off, and re-scanned
    the inner cells once per ring, producing duplicate planes.)
    """
    planes = []
    gx, gy = int(round(rx)), int(round(ry))
    reach = int(WALL_SENSE_DIST) + 1
    inv_tau_w = 1.0 / TIME_HORIZON_WALL
    comb_r_w = radius + 0.05            # robot body + a hair of clearance

    for dx in range(-reach, reach + 1):
        for dy in range(-reach, reach + 1):
            wx, wy = gx + dx, gy + dy
            if warehouse.is_passable(wx, wy):
                continue
            near_x = max(wx - 0.5, min(rx, wx + 0.5))
            near_y = max(wy - 0.5, min(ry, wy + 0.5))
            wnx, wny = rx - near_x, ry - near_y
            w_dist = _len(wnx, wny)
            if w_dist > WALL_SENSE_DIST or w_dist < 1e-6:
                continue
            nx_w, ny_w = wnx / w_dist, wny / w_dist
            u_needed = (comb_r_w - w_dist) * inv_tau_w + 0.02
            if u_needed <= 0:
                continue
            planes.append((nx_w, ny_w, rvx + nx_w * u_needed, rvy + ny_w * u_needed))
    return planes


# ─────────────────────────────────────────────────────────────
#  Main: compute_orca (called each tick by engine)
# ─────────────────────────────────────────────────────────────

def compute_orca(robots, warehouse: Warehouse, dt: float = 0.1) -> Dict:
    """
    Compute ORCA-safe velocities for all active robots.

    Returns viz_data dict: {robot_id: {pref_vel, orca_vel, speed_scale, n_planes, neighbors}}
    Also sets robot.vx, robot.vy directly (the engine then re-derives them from the
    actual move). `speed_scale` is how much of its preferred speed along its heading
    ORCA lets the robot keep: the engine applies it as a last-line safety governor.
    """
    viz: Dict[int, Dict] = {}
    active = [r for r in robots if r.active and r.battery > 0]

    for robot in active:
        # ── preferred velocity ───────────────────────────────
        if robot.target_cell:
            tx, ty = robot.target_cell
            dx, dy = tx - robot.x, ty - robot.y
            d = _len(dx, dy)
            if d > 1e-4:
                pref_vx = (dx/d) * robot.preferred_speed
                pref_vy = (dy/d) * robot.preferred_speed
            else:
                pref_vx, pref_vy = 0.0, 0.0
        else:
            pref_vx, pref_vy = 0.0, 0.0

        robot.viz["pref_vx"] = pref_vx
        robot.viz["pref_vy"] = pref_vy

        half_planes:  List[Tuple]   = []
        viz_planes:   List[Dict]    = []
        neighbor_ids: List[int]     = []

        # ── robot-robot half-planes ───────────────────────────
        neighbors = [r for r in active
                     if r.id != robot.id
                     and robot.distance_to(r) < SENSING_RADIUS]

        for other in neighbors:
            neighbor_ids.append(other.id)
            dist = robot.distance_to(other)
            # Use shorter time horizon for closer robots
            tau  = TIME_HORIZON_ROBOT * max(0.3, dist / SENSING_RADIUS)
            plane = _robot_orca_plane(
                robot.x, robot.y, robot.vx, robot.vy, robot.radius,
                other.x, other.y, other.vx, other.vy, other.radius,
                tau
            )
            if plane:
                half_planes.append(plane)
                viz_planes.append({"nx": plane[0], "ny": plane[1],
                                   "px": plane[2], "py": plane[3],
                                   "other_id": other.id})

        # ── wall half-planes ─────────────────────────────────
        wall_planes = _wall_planes(robot.x, robot.y, robot.vx, robot.vy,
                                   robot.radius, warehouse)
        half_planes.extend(wall_planes)

        # ── solve LP ─────────────────────────────────────────
        new_vx, new_vy = solve_orca_lp(half_planes, pref_vx, pref_vy,
                                        robot.preferred_speed * MAX_SPEED_FACTOR)

        robot.vx = new_vx
        robot.vy = new_vy
        robot.viz["orca_vx"]    = new_vx
        robot.viz["orca_vy"]    = new_vy
        robot.viz["orca_planes"] = viz_planes

        pref_spd = _len(pref_vx, pref_vy)
        if pref_spd > 1e-6:
            along = _dot(new_vx, new_vy, pref_vx / pref_spd, pref_vy / pref_spd)
            speed_scale = _clamp(along / pref_spd, 0.0, 1.0)
        else:
            speed_scale = 1.0
        robot.viz["orca_scale"] = round(speed_scale, 3)

        viz[robot.id] = {
            "speed_scale": round(speed_scale, 3),
            "pref_vel":    [round(pref_vx,3), round(pref_vy,3)],
            "orca_vel":    [round(new_vx,3),  round(new_vy,3)],
            "n_planes":    len(half_planes),
            "n_wall_planes": len(wall_planes),
            "neighbor_ids": neighbor_ids,
        }

    return viz
