"""
orca.py — Simplified reciprocal velocity obstacle (RVO) for reactive avoidance.

This is a LAST-RESORT safety net, active only if live execution drifts from
the negotiated reservation plan (e.g. a peer's odometry noise, a late replan).
It does NOT replace the reservation layer — it's the fallback for when
reality diverges from the plan.

Implementation note:
This is a simplified RVO (reciprocal velocity obstacle) approach, not the
full ORCA linear-programming formulation. The simplification computes a
velocity adjustment by projecting the relative velocity onto the collision
cone and applying a half-plane correction. This is adequate for the safety-net
role because:
  - The reservation layer has already resolved most conflicts.
  - This only needs to handle small execution drift, not full path planning.
  - Full ORCA's LP solver adds complexity disproportionate to benefit here.

What's implemented: simplified RVO (velocity-cone half-plane correction).
What's NOT implemented: full ORCA (Optimal Reciprocal Collision Avoidance
with LP-based velocity selection across multiple agents simultaneously).

Zero ROS2 dependencies — pure Python, tested with plain pytest.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class AgentState:
    """Position and velocity of an agent in 2D world frame."""
    x: float
    y: float
    vx: float
    vy: float
    radius: float = 0.28  # robot collision radius (meters)


@dataclass
class VelocityAdjustment:
    """A recommended velocity change to avoid collision."""
    vx: float
    vy: float
    reason: str


# Minimum safe distance between robot centers (sum of radii + margin)
SAFETY_MARGIN = 0.15  # extra margin beyond contact distance


def _distance(a: AgentState, b: AgentState) -> float:
    """Euclidean distance between two agents."""
    dx = b.x - a.x
    dy = b.y - a.y
    return math.sqrt(dx * dx + dy * dy)


def _normalize(vx: float, vy: float) -> Tuple[float, float]:
    """Normalize a 2D vector. Returns (0, 0) for zero-length."""
    mag = math.sqrt(vx * vx + vy * vy)
    if mag < 1e-9:
        return (0.0, 0.0)
    return (vx / mag, vy / mag)


def compute_rvo_adjustment(
    ego: AgentState,
    peers: List[AgentState],
    time_horizon: float = 2.0,
    max_adjustment: float = 0.5,
) -> Optional[VelocityAdjustment]:
    """
    Compute a velocity adjustment for the ego agent to avoid collisions
    with all peer agents using simplified reciprocal velocity obstacles.

    Parameters
    ----------
    ego : AgentState
        The robot computing the avoidance.
    peers : list of AgentState
        All other visible robots.
    time_horizon : float
        How far into the future to check for collisions (seconds).
    max_adjustment : float
        Maximum velocity adjustment magnitude (m/s).

    Returns
    -------
    VelocityAdjustment or None
        A velocity adjustment if any peer is within the collision cone,
        or None if no adjustment is needed.
    """
    total_adj_x = 0.0
    total_adj_y = 0.0
    collision_peers: List[str] = []

    for peer in peers:
        # Vector from ego to peer
        dx = peer.x - ego.x
        dy = peer.y - ego.y
        dist = math.sqrt(dx * dx + dy * dy)

        combined_radius = ego.radius + peer.radius + SAFETY_MARGIN

        if dist < 1e-6:
            # Overlapping — emergency push away
            total_adj_x -= 0.3
            total_adj_y -= 0.3
            collision_peers.append('overlap')
            continue

        if dist > combined_radius + (ego.vx * ego.vx + ego.vy * ego.vy) ** 0.5 * time_horizon + 2.0:
            # Too far away to matter in this time horizon
            continue

        # Relative velocity (ego relative to peer)
        rel_vx = ego.vx - peer.vx
        rel_vy = ego.vy - peer.vy

        # Project the relative position into the future
        # Check if the relative velocity points toward the peer
        # within the collision cone

        # Time to closest approach
        dot_rv_dp = rel_vx * dx + rel_vy * dy
        rel_speed_sq = rel_vx * rel_vx + rel_vy * rel_vy

        if rel_speed_sq < 1e-9:
            # Not moving relative to each other
            if dist < combined_radius * 1.5:
                # But too close — nudge away
                nx, ny = _normalize(-dx, -dy)
                adj_mag = min(max_adjustment, combined_radius * 1.5 - dist)
                total_adj_x += nx * adj_mag * 0.5
                total_adj_y += ny * adj_mag * 0.5
                collision_peers.append('static_close')
            continue

        t_closest = dot_rv_dp / rel_speed_sq
        t_closest = max(0.0, min(t_closest, time_horizon))

        # Distance at closest approach
        closest_x = dx - rel_vx * t_closest
        closest_y = dy - rel_vy * t_closest
        closest_dist = math.sqrt(closest_x * closest_x + closest_y * closest_y)

        if closest_dist < combined_radius:
            # Collision predicted — compute avoidance
            # Push velocity perpendicular to the line of approach
            # (simplified RVO: each agent takes half the correction)
            if closest_dist < 1e-6:
                # Head-on — dodge perpendicular
                perp_x, perp_y = -dy / dist, dx / dist
            else:
                perp_x, perp_y = _normalize(
                    closest_x - dx * (closest_dist / dist),
                    closest_y - dy * (closest_dist / dist),
                )
                if abs(perp_x) < 1e-6 and abs(perp_y) < 1e-6:
                    # Fallback: perpendicular to connection line
                    perp_x, perp_y = -dy / dist, dx / dist

            # Correction magnitude proportional to how deep in the cone
            penetration = combined_radius - closest_dist
            urgency = min(1.0, penetration / combined_radius)
            adj_mag = min(max_adjustment, urgency * max_adjustment)

            # Reciprocal: each agent takes half the correction
            total_adj_x += perp_x * adj_mag * 0.5
            total_adj_y += perp_y * adj_mag * 0.5
            collision_peers.append(f"t={t_closest:.1f}s")

    if not collision_peers:
        return None

    # Clamp total adjustment
    adj_mag = math.sqrt(total_adj_x * total_adj_x + total_adj_y * total_adj_y)
    if adj_mag > max_adjustment:
        scale = max_adjustment / adj_mag
        total_adj_x *= scale
        total_adj_y *= scale

    return VelocityAdjustment(
        vx=total_adj_x,
        vy=total_adj_y,
        reason=f"RVO avoidance ({', '.join(collision_peers)})",
    )
