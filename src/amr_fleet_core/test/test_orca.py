"""Tests for amr_fleet_core.negotiation.orca — simplified RVO."""

import math

from amr_fleet_core.negotiation.orca import (
    AgentState,
    VelocityAdjustment,
    compute_rvo_adjustment,
)


class TestRVOAdjustment:
    """Simplified reciprocal velocity obstacle tests."""

    def test_no_adjustment_when_no_peers(self):
        ego = AgentState(x=0, y=0, vx=1.0, vy=0)
        result = compute_rvo_adjustment(ego, peers=[])
        assert result is None

    def test_no_adjustment_when_peers_far_away(self):
        ego = AgentState(x=0, y=0, vx=1.0, vy=0)
        far_peer = AgentState(x=100, y=100, vx=0, vy=0)
        result = compute_rvo_adjustment(ego, peers=[far_peer])
        assert result is None

    def test_adjustment_on_head_on_collision_course(self):
        """Two agents heading straight at each other should get an adjustment."""
        ego = AgentState(x=0, y=0, vx=1.0, vy=0, radius=0.28)
        peer = AgentState(x=2.0, y=0, vx=-1.0, vy=0, radius=0.28)

        result = compute_rvo_adjustment(ego, peers=[peer], time_horizon=3.0)
        assert result is not None
        # The adjustment should have a non-zero component
        adj_mag = math.sqrt(result.vx ** 2 + result.vy ** 2)
        assert adj_mag > 0.01

    def test_adjustment_near_static_obstacle(self):
        """Ego moving toward a stationary peer should get an adjustment."""
        ego = AgentState(x=0, y=0, vx=0.5, vy=0, radius=0.28)
        static_peer = AgentState(x=1.0, y=0, vx=0, vy=0, radius=0.28)

        result = compute_rvo_adjustment(ego, peers=[static_peer], time_horizon=3.0)
        assert result is not None

    def test_perpendicular_crossing_adjustment(self):
        """Two agents on a perpendicular collision course."""
        ego = AgentState(x=0, y=0, vx=1.0, vy=0, radius=0.28)
        peer = AgentState(x=1.5, y=-1.5, vx=0, vy=1.0, radius=0.28)

        result = compute_rvo_adjustment(ego, peers=[peer], time_horizon=3.0)
        # They will collide near (1.5, 0) at t≈1.5 — should detect
        assert result is not None

    def test_parallel_same_direction_no_adjustment(self):
        """Agents moving in the same direction, not converging."""
        ego = AgentState(x=0, y=0, vx=1.0, vy=0, radius=0.28)
        peer = AgentState(x=0, y=2.0, vx=1.0, vy=0, radius=0.28)

        result = compute_rvo_adjustment(ego, peers=[peer], time_horizon=2.0)
        # Parallel paths with 2m separation — no collision risk
        assert result is None

    def test_max_adjustment_clamped(self):
        """Adjustment magnitude should not exceed max_adjustment."""
        ego = AgentState(x=0, y=0, vx=2.0, vy=0, radius=0.28)
        peer = AgentState(x=0.8, y=0, vx=-2.0, vy=0, radius=0.28)

        max_adj = 0.3
        result = compute_rvo_adjustment(
            ego, peers=[peer], max_adjustment=max_adj
        )
        if result is not None:
            adj_mag = math.sqrt(result.vx ** 2 + result.vy ** 2)
            assert adj_mag <= max_adj + 0.01  # small epsilon for floating point

    def test_multiple_peers_combined(self):
        """Adjustments from multiple peers should combine."""
        ego = AgentState(x=0, y=0, vx=0.5, vy=0, radius=0.28)
        peers = [
            AgentState(x=1.0, y=0.3, vx=-0.5, vy=0, radius=0.28),
            AgentState(x=1.0, y=-0.3, vx=-0.5, vy=0, radius=0.28),
        ]
        result = compute_rvo_adjustment(ego, peers=peers, time_horizon=3.0)
        assert result is not None
