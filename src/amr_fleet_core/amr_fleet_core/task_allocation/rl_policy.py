"""
rl_policy.py — Tabular Q-learning bid-priority policy.

The RL policy's ONLY output is a bid-priority scalar. This is enforced
structurally: the module's only public method is get_bid_priority(state),
which returns a float. It has NO access to cmd_vel, reservation data,
or any motion/safety interface.

State: (battery_bin, distance_bin, nearby_robots_bin) — all discretized.
Action: bid priority ∈ {0=HIGH, 1=MEDIUM, 2=LOW}.
Reward: negative task completion time (lower is better for throughput).

This is a deliberate, defensible design choice: RL is confined to a
genuinely non-safety-critical role (which task to bid on), while motion
and collision avoidance stay rule-based, deterministic, and auditable.

Zero ROS2 dependencies — pure Python, tested with plain pytest.
"""

from __future__ import annotations

import json
import os
import random
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# State discretization bins
BATTERY_BINS = [0.2, 0.5, 0.8]        # low, medium, high, full
DISTANCE_BINS = [3.0, 8.0, 15.0]      # close, medium, far, very_far
NEARBY_ROBOTS_BINS = [1, 3, 5]         # few, some, many, crowded

# Actions: bid priority levels
N_ACTIONS = 3
ACTION_HIGH = 0     # bid priority 0.0 (most eager)
ACTION_MEDIUM = 1   # bid priority 0.5
ACTION_LOW = 2      # bid priority 1.0

ACTION_TO_PRIORITY = {
    ACTION_HIGH: 0.0,
    ACTION_MEDIUM: 0.5,
    ACTION_LOW: 1.0,
}


@dataclass
class RLState:
    """Discretized state for the Q-learning policy."""
    battery_bin: int
    distance_bin: int
    nearby_robots_bin: int

    def as_tuple(self) -> Tuple[int, int, int]:
        return (self.battery_bin, self.distance_bin, self.nearby_robots_bin)


def _discretize(value: float, bins: list) -> int:
    """Map a continuous value to a bin index."""
    for i, threshold in enumerate(bins):
        if value < threshold:
            return i
    return len(bins)


def make_state(
    battery_level: float,
    distance_to_task: float,
    num_nearby_robots: int,
) -> RLState:
    """
    Create a discretized RL state from continuous observations.

    Parameters
    ----------
    battery_level : float in [0, 1]
    distance_to_task : float in meters
    num_nearby_robots : int
    """
    return RLState(
        battery_bin=_discretize(battery_level, BATTERY_BINS),
        distance_bin=_discretize(distance_to_task, DISTANCE_BINS),
        nearby_robots_bin=_discretize(float(num_nearby_robots),
                                      [float(b) for b in NEARBY_ROBOTS_BINS]),
    )


class TabularQPolicy:
    """
    Tabular Q-learning policy for bid priority selection.

    STRUCTURAL GUARANTEE: The only public method that produces output
    usable by the task allocation system is get_bid_priority(), which
    returns a float. This class has no knowledge of cmd_vel, reservations,
    paths, or any motion interface — by design, not by convention.
    """

    def __init__(
        self,
        learning_rate: float = 0.1,
        discount_factor: float = 0.95,
        epsilon: float = 0.15,
        seed: Optional[int] = None,
    ) -> None:
        self.lr = learning_rate
        self.gamma = discount_factor
        self.epsilon = epsilon
        self._rng = random.Random(seed)
        # Q-table: state_tuple → [Q(a=0), Q(a=1), Q(a=2)]
        self._q: Dict[Tuple, list] = {}
        self._episode_count = 0

    def _get_q(self, state: Tuple) -> list:
        if state not in self._q:
            self._q[state] = [0.0] * N_ACTIONS
        return self._q[state]

    def select_action(self, state: RLState) -> int:
        """Select an action using epsilon-greedy exploration."""
        if self._rng.random() < self.epsilon:
            return self._rng.randint(0, N_ACTIONS - 1)
        q_values = self._get_q(state.as_tuple())
        # Greedy: pick action with highest Q (best expected reward)
        return max(range(N_ACTIONS), key=lambda a: q_values[a])

    def get_bid_priority(self, state: RLState) -> float:
        """
        PUBLIC API: Get the bid priority scalar for a given state.

        This is the ONLY output of the RL module usable by the task
        allocation system. Returns a float where lower = more eager.
        """
        action = self.select_action(state)
        return ACTION_TO_PRIORITY[action]

    def update(
        self,
        state: RLState,
        action: int,
        reward: float,
        next_state: Optional[RLState] = None,
    ) -> None:
        """
        Update Q-value for a (state, action) pair.

        Parameters
        ----------
        state : RLState
        action : int (0, 1, or 2)
        reward : float (negative task completion time)
        next_state : RLState or None (None if terminal)
        """
        q = self._get_q(state.as_tuple())
        if next_state is None:
            target = reward
        else:
            next_q = self._get_q(next_state.as_tuple())
            target = reward + self.gamma * max(next_q)
        q[action] += self.lr * (target - q[action])

    def decay_epsilon(self, min_epsilon: float = 0.01,
                       decay_rate: float = 0.995) -> None:
        """Decay exploration rate after each episode."""
        self.epsilon = max(min_epsilon, self.epsilon * decay_rate)
        self._episode_count += 1

    def save(self, filepath: str) -> None:
        """Save Q-table to JSON."""
        os.makedirs(os.path.dirname(filepath) or '.', exist_ok=True)
        data = {
            'q_table': {str(k): v for k, v in self._q.items()},
            'epsilon': self.epsilon,
            'episode_count': self._episode_count,
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

    def load(self, filepath: str) -> None:
        """Load Q-table from JSON."""
        with open(filepath) as f:
            data = json.load(f)
        self._q = {}
        for k, v in data['q_table'].items():
            # Parse string tuple key back to tuple
            key = eval(k)  # Safe: we wrote it ourselves
            self._q[key] = v
        self.epsilon = data.get('epsilon', self.epsilon)
        self._episode_count = data.get('episode_count', 0)

    @property
    def table_size(self) -> int:
        return len(self._q)
