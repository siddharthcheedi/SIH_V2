"""Tests for amr_fleet_core.task_allocation.rl_policy."""

from amr_fleet_core.task_allocation.rl_policy import (
    ACTION_HIGH,
    ACTION_LOW,
    ACTION_MEDIUM,
    ACTION_TO_PRIORITY,
    N_ACTIONS,
    TabularQPolicy,
    make_state,
)


class TestMakeState:
    """State discretization."""

    def test_low_battery(self):
        state = make_state(battery_level=0.1, distance_to_task=5.0,
                           num_nearby_robots=2)
        assert state.battery_bin == 0  # below 0.2 threshold

    def test_high_battery(self):
        state = make_state(battery_level=0.9, distance_to_task=5.0,
                           num_nearby_robots=2)
        assert state.battery_bin == 3  # above all thresholds

    def test_close_distance(self):
        state = make_state(0.5, distance_to_task=1.0, num_nearby_robots=0)
        assert state.distance_bin == 0  # below 3.0

    def test_far_distance(self):
        state = make_state(0.5, distance_to_task=20.0, num_nearby_robots=0)
        assert state.distance_bin == 3  # above all thresholds

    def test_state_as_tuple(self):
        state = make_state(0.5, 5.0, 2)
        t = state.as_tuple()
        assert isinstance(t, tuple)
        assert len(t) == 3


class TestTabularQPolicy:
    """Q-learning policy behavior."""

    def test_initial_q_values_are_zero(self):
        policy = TabularQPolicy(seed=42)
        state = make_state(0.5, 5.0, 2)
        q = policy._get_q(state.as_tuple())
        assert all(v == 0.0 for v in q)

    def test_get_bid_priority_returns_valid_range(self):
        policy = TabularQPolicy(seed=42)
        state = make_state(0.5, 5.0, 2)
        priority = policy.get_bid_priority(state)
        assert priority in ACTION_TO_PRIORITY.values()

    def test_update_changes_q(self):
        policy = TabularQPolicy(epsilon=0.0, seed=42)
        state = make_state(0.5, 5.0, 2)
        key = state.as_tuple()

        old_q = policy._get_q(key)[ACTION_HIGH]
        policy.update(state, ACTION_HIGH, reward=-5.0)
        new_q = policy._get_q(key)[ACTION_HIGH]
        assert new_q != old_q

    def test_greedy_selects_best_action(self):
        """With epsilon=0, should select the action with highest Q."""
        policy = TabularQPolicy(epsilon=0.0, seed=42)
        state = make_state(0.5, 5.0, 2)

        # Set Q values so ACTION_MEDIUM is best
        q = policy._get_q(state.as_tuple())
        q[ACTION_HIGH] = -10.0
        q[ACTION_MEDIUM] = -2.0
        q[ACTION_LOW] = -5.0

        action = policy.select_action(state)
        assert action == ACTION_MEDIUM

    def test_epsilon_exploration(self):
        """With epsilon=1.0, should explore randomly (not always same action)."""
        policy = TabularQPolicy(epsilon=1.0, seed=42)
        state = make_state(0.5, 5.0, 2)

        actions = set()
        for _ in range(100):
            actions.add(policy.select_action(state))

        # With full exploration, should hit at least 2 different actions
        assert len(actions) >= 2

    def test_epsilon_decay(self):
        policy = TabularQPolicy(epsilon=0.5, seed=42)
        policy.decay_epsilon(min_epsilon=0.01, decay_rate=0.9)
        assert policy.epsilon < 0.5
        assert policy.epsilon >= 0.01

    def test_epsilon_decay_respects_minimum(self):
        policy = TabularQPolicy(epsilon=0.02, seed=42)
        policy.decay_epsilon(min_epsilon=0.01, decay_rate=0.1)
        assert policy.epsilon >= 0.01

    def test_structural_isolation(self):
        """
        STRUCTURAL GUARANTEE: The policy has no methods that could output
        motion commands, reservations, or collision-avoidance data.
        """
        policy = TabularQPolicy(seed=42)
        public_methods = [m for m in dir(policy) if not m.startswith('_')]
        # Should NOT have anything motion-related
        forbidden = ['cmd_vel', 'publish', 'navigate', 'reserve', 'collision']
        for method_name in public_methods:
            for bad in forbidden:
                assert bad not in method_name.lower(), (
                    f"Policy has method '{method_name}' that suggests "
                    f"access to motion/safety systems"
                )
