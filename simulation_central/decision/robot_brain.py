"""
robot_brain.py — Per-Robot Autonomous Decision-Making Engine

Each robot has a RobotBrain that runs every tick:

1. PROGRESS MONITORING  — uses the engine's blocked_ticks (ticks at rest, wanting to move,
                          unable to). A robot that is travelling is never "stuck".
2. ESCALATION LADDER    — blocked a while  -> REPLAN (find another route)
                          blocked long     -> DROP_TASK (let a better-placed robot take it)
3. TASK UTILITY         — multi-factor bid for the auction, using WALKING distance
4. BATTERY MANAGEMENT   — refuse tasks it can't finish AND get home from; go charge when
                          idle and low; abandon the task and charge if critical
5. YIELDING             — if stuck on the way to a pickup and a clearly closer robot is
                          idle, hand the task over

Decision outputs (returned to the engine each tick):
  CONTINUE        — nominal
  REPLAN          — compute a fresh congestion-aware route
  DROP_TASK       — abandon task, re-enter auction (can't re-bid on it for a while)
  GO_CHARGE       — idle and low on battery: go to a charger
  EMERGENCY_STOP  — battery critical with a task: drop it and go to a charger
"""
from __future__ import annotations
import math
from typing import Dict, List, Optional, Tuple, Any

from ..robot import RobotState, BATTERY_PER_CELL
from ..warehouse import UNREACHABLE


# ─────────────────────────────────────────────────────────────
#  Constants
# ─────────────────────────────────────────────────────────────
REPLAN_BLOCKED_TICKS   = 8     # blocked this long -> ask for a new route
REPLAN_COOLDOWN        = 20    # ...but not more often than this
TASK_DROP_THRESHOLD    = 60    # blocked this long -> give the task up
YIELD_BLOCKED_TICKS    = 12    # blocked this long on the way to pickup -> consider yielding
YIELD_DISTANCE_RATIO   = 0.5   # another robot must be < 50% as far from the pickup...
YIELD_MIN_GAIN         = 6     # ...and at least this many cells closer
BATTERY_WARN           = 30    # % — idle robots below this go charging
BATTERY_CRITICAL       = 8     # % — abandon everything and charge
BATTERY_MARGIN         = 1.25  # safety factor on energy estimates
CHARGE_RETRY_TICKS     = 25    # don't spam GO_CHARGE when every charger is taken


class Decision:
    CONTINUE       = "CONTINUE"
    DROP_TASK      = "DROP_TASK"
    CHAIN_TASK     = "CHAIN_TASK"       # unused; kept so old dashboards don't break
    RETREAT        = "RETREAT"          # unused here (deadlock module does this)
    EMERGENCY_STOP = "EMERGENCY_STOP"
    REPLAN         = "REPLAN"
    GO_CHARGE      = "GO_CHARGE"


class BrainDecision:
    def __init__(self, action: str, data: Any = None, reason: str = ""):
        self.action = action
        self.data   = data
        self.reason = reason

    def __repr__(self):
        return f"Decision({self.action}: {self.reason})"


# ─────────────────────────────────────────────────────────────
#  Robot Brain
# ─────────────────────────────────────────────────────────────

class RobotBrain:
    def __init__(self, robot):
        self.robot = robot
        self._task_ticks:   int           = 0
        self._task_id_last: Optional[int] = None
        self._tick:         int           = 0
        self._last_charge_try: int        = -999
        self.current_decision = Decision.CONTINUE
        self.current_reason   = "Initializing"
        self.congestion_score: float = 0.0

    # ── main entry point ────────────────────────────────────────────────────

    def think(self, all_robots: List, available_tasks: List,
              warehouse, tick: int) -> BrainDecision:
        r = self.robot
        self._tick = tick
        self._update_progress()

        # Charging / heading to a charger: nothing to decide
        if r.state == RobotState.LOW_BATTERY:
            docked = r.charge_cell is not None and r.is_at_cell(*r.charge_cell)
            return self._decide(Decision.CONTINUE,
                                "Charging" if docked else "Heading to charger")

        # Battery critical with work in hand: drop it, go charge
        if r.battery < BATTERY_CRITICAL and r.task:
            return self._decide(Decision.EMERGENCY_STOP,
                                f"Battery {r.battery:.0f}% < {BATTERY_CRITICAL}%", r.task)

        # Idle and low: go charge (rate-limited if every charger is busy)
        if (r.task is None and r.state == RobotState.IDLE and r.at_rest
                and r.battery < BATTERY_WARN
                and tick - self._last_charge_try >= CHARGE_RETRY_TICKS):
            self._last_charge_try = tick
            return self._decide(Decision.GO_CHARGE, f"Battery {r.battery:.0f}% — charging")

        if r.task:
            # Last resort: stuck far too long
            if r.blocked_ticks >= TASK_DROP_THRESHOLD:
                return self._decide(Decision.DROP_TASK,
                                    f"Blocked {r.blocked_ticks} ticks", r.task)

            # Yield the pickup to a clearly better-placed idle robot
            if (r.state == RobotState.MOVING_TO_PICKUP
                    and r.blocked_ticks >= YIELD_BLOCKED_TICKS):
                better = self._find_better_robot(r.task, all_robots, warehouse)
                if better is not None:
                    return self._decide(Decision.DROP_TASK,
                                        f"Yielding to Robot {better.id}", r.task)

            # Blocked for a while: try another route
            if (r.blocked_ticks >= REPLAN_BLOCKED_TICKS
                    and tick - r.last_plan_tick >= REPLAN_COOLDOWN):
                return self._decide(Decision.REPLAN,
                                    f"Blocked {r.blocked_ticks} ticks — new route")

        return self._decide(Decision.CONTINUE, "Nominal operation")

    # ── utility / bid computation ────────────────────────────────────────────

    def compute_task_utility(self, task, all_robots: List, warehouse) -> float:
        """
        Utility for the auction. Higher = more willing. 0 = abstain.
          45% proximity (walking distance to pickup)
          20% battery headroom after the whole job + trip to a charger
          20% task priority
          10% congestion around the pickup
           5% trip efficiency (short deliveries finish sooner)
        """
        r = self.robot

        if task.banned.get(r.id, -1) > self._tick:
            return 0.0

        d_pick = warehouse.dist(r.grid_cell, task.pickup)
        d_trip = warehouse.dist(task.pickup, task.dropoff)
        if d_pick >= UNREACHABLE or d_trip >= UNREACHABLE:
            return 0.0

        # Energy: pickup + trip + getting to the nearest charger afterwards + reserve
        d_home = min((warehouse.dist(task.dropoff, sp) for sp in warehouse.spawn_points),
                     default=0)
        if d_home >= UNREACHABLE:
            d_home = 0
        bat_needed = (d_pick + d_trip + d_home) * BATTERY_PER_CELL * BATTERY_MARGIN \
                     + BATTERY_CRITICAL
        headroom = r.battery - bat_needed
        if headroom <= 0:
            return 0.0
        bat_score = min(1.0, headroom / 40.0)

        dist_score     = 1.0 / (1.0 + d_pick / 4.0)
        priority_score = task.priority / 3.0
        trip_score     = 1.0 / (1.0 + d_trip / 10.0)

        crowd = sum(1 for o in all_robots
                    if o.id != r.id
                    and math.hypot(o.x - task.pickup[0], o.y - task.pickup[1]) < 3.5)
        congestion_score = 1.0 / (1 + crowd)
        self.congestion_score = congestion_score

        return (0.45 * dist_score + 0.20 * bat_score + 0.20 * priority_score
                + 0.10 * congestion_score + 0.05 * trip_score)

    # ── internals ────────────────────────────────────────────────────────────

    def _update_progress(self) -> None:
        r = self.robot
        if r.task:
            if r.task.id != self._task_id_last:
                self._task_ticks   = 0
                self._task_id_last = r.task.id
            else:
                self._task_ticks += 1
        else:
            self._task_ticks   = 0
            self._task_id_last = None
        r.viz["stuck_ticks"] = r.blocked_ticks
        r.viz["task_ticks"]  = self._task_ticks

    def _find_better_robot(self, task, all_robots: List, warehouse):
        r = self.robot
        my_d = warehouse.dist(r.anchor, task.pickup)
        for other in all_robots:
            if other.id == r.id or not other.active:
                continue
            if other.state != RobotState.IDLE or not other.at_rest:
                continue
            if other.battery < BATTERY_WARN:
                continue
            if task.banned.get(other.id, -1) > self._tick:
                continue
            od = warehouse.dist(other.grid_cell, task.pickup)
            if od < my_d * YIELD_DISTANCE_RATIO and my_d - od >= YIELD_MIN_GAIN:
                return other
        return None

    def _decide(self, action: str, reason: str, data: Any = None) -> BrainDecision:
        self.current_decision = action
        self.current_reason   = reason
        self.robot.viz["brain_action"] = action
        self.robot.viz["brain_reason"] = reason
        return BrainDecision(action, data, reason)

    # ── state export ─────────────────────────────────────────────────────────

    def get_state(self) -> Dict:
        return {
            "robot_id":    self.robot.id,
            "decision":    self.current_decision,
            "reason":      self.current_reason,
            "stuck_ticks": self.robot.blocked_ticks,
            "task_ticks":  self._task_ticks,
            "congestion":  self.congestion_score,
        }
