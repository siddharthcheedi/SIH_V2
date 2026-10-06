"""
robot.py — AMR Robot with smooth lerp-based movement, state machine, and broadcast.

MOVEMENT MODEL (important — the planners all rely on this):
  A robot is always in exactly one of two modes:
    * AT REST     : target_cell is None, it sits on grid_cell. Planners may give it a move.
    * IN TRANSIT  : target_cell is set. It is travelling grid_cell -> target_cell and
                    CANNOT be re-targeted. grid_cell only changes on arrival.
  While in transit it occupies BOTH cells (grid_cell and target_cell); the planners
  treat both as blocked for everyone else. This one rule is what removes collisions.

PATH CONVENTION:
  robot.path = cells still to visit AFTER the cell the robot will be standing on at its
  next decision (i.e. after target_cell if in transit, else after grid_cell).
  It never contains wait steps or the current cell.
"""

from __future__ import annotations
import math
from typing import Optional, List, Tuple, Dict, Any

ROBOT_COLORS = [
    "#e74c3c","#3498db","#2ecc71","#f39c12","#9b59b6",
    "#1abc9c","#e67e22","#e91e63","#00bcd4","#cddc39",
    "#ff5722","#607d8b","#8bc34a","#ff9800","#673ab7",
    "#009688","#795548","#f44336","#2196f3","#4caf50",
]

# ── energy model (shared with the robot brain's bid / viability checks) ──────
BATTERY_PER_CELL = 0.20    # % consumed per cell travelled
IDLE_DRAIN       = 0.004   # % per tick just for being switched on
CHARGE_RATE      = 1.5     # % per tick while docked
CHARGE_FULL      = 95.0    # leave the charger at this level


class RobotState:
    IDLE             = "IDLE"
    MOVING_TO_PICKUP  = "MOVING_TO_PICKUP"
    PICKING_UP        = "PICKING_UP"
    MOVING_TO_DROPOFF = "MOVING_TO_DROPOFF"
    DROPPING_OFF      = "DROPPING_OFF"
    WAITING           = "WAITING"
    LOW_BATTERY       = "LOW_BATTERY"      # heading to / sitting on a charger


class Robot:
    def __init__(self, robot_id: int, x: float, y: float,
                 preferred_speed: float = 3.0, battery_capacity: float = 100.0):
        self.id    = robot_id
        self.x     = float(x)
        self.y     = float(y)
        self.grid_cell: Tuple[int,int] = (int(round(x)), int(round(y)))
        self.target_cell: Optional[Tuple[int,int]] = None  # set only while in transit

        self.vx = 0.0
        self.vy = 0.0
        self.preferred_speed  = preferred_speed
        self.radius           = 0.4
        self.battery          = battery_capacity
        self.battery_capacity = battery_capacity

        self.state = RobotState.IDLE
        self.task  = None          # Task object or None
        self.path: List[Tuple[int,int]] = []   # see PATH CONVENTION above

        self.color = ROBOT_COLORS[robot_id % len(ROBOT_COLORS)]
        self.active = True

        # ── planning bookkeeping (written by engine / PIBT / brain) ──────────
        self.goal: Optional[Tuple[int,int]]         = None  # where this robot is heading now
        self.desired_cell: Optional[Tuple[int,int]] = None  # PIBT's first-choice next cell
        self.blocked_ticks   = 0     # consecutive ticks at rest, wanting to move, unable to
        self.task_age        = 0     # ticks since the current task was assigned
        self.priority_boost  = 0     # temporary priority bump (deadlock resolver), decays
        self.last_plan_tick  = -999
        self.need_replan     = False
        self.charge_cell: Optional[Tuple[int,int]]  = None

        # Metrics
        self.tasks_completed  = 0
        self.collision_count  = 0
        self.ticks_waiting    = 0
        self.total_distance   = 0.0

        # Smooth animation lerp
        self._lerp_t    = 1.0
        self._from_x    = float(x)
        self._from_y    = float(y)

        # Algorithm visualization data (filled by algorithms each tick)
        self.viz = self.fresh_viz()

    @staticmethod
    def fresh_viz() -> Dict[str, Any]:
        return {
            "priority":      0,
            "claimed_cell":  None,
            "in_conflict":   False,
            "pref_vx":       0.0,
            "pref_vy":       0.0,
            "orca_vx":       0.0,
            "orca_vy":       0.0,
            "bid_values":    {},   # task_id -> bid
            "orca_planes":   [],   # list of {nx, ny, pt_x, pt_y} for visualization
            "brain_action":  "CONTINUE",
            "brain_reason":  "",
            "stuck_ticks":   0,
            "task_ticks":    0,
        }

    def reset_runtime(self) -> None:
        """Clear everything that belongs to a run (used by engine.reset / layout change)."""
        self.state = RobotState.IDLE
        self.task = None
        self.path = []
        self.target_cell = None
        self.goal = None
        self.desired_cell = None
        self.blocked_ticks = 0
        self.task_age = 0
        self.priority_boost = 0
        self.last_plan_tick = -999
        self.need_replan = False
        self.charge_cell = None
        self.vx = self.vy = 0.0
        self._lerp_t = 1.0
        self._from_x, self._from_y = self.x, self.y
        self.viz = self.fresh_viz()

    # ── movement ─────────────────────────────────────────────────────────────

    @property
    def at_rest(self) -> bool:
        return self.target_cell is None

    @property
    def anchor(self) -> Tuple[int,int]:
        """The cell this robot will be standing on at its next decision point."""
        return self.target_cell if self.target_cell is not None else self.grid_cell

    def begin_move(self, tx: int, ty: int) -> bool:
        """
        Start moving to an adjacent cell. REFUSED while already in transit
        (this is what used to cause the 'robot snaps back to its start' glitch).
        """
        if self.target_cell is not None:
            return False
        if (tx, ty) == self.grid_cell:
            return False
        if abs(tx - self.grid_cell[0]) + abs(ty - self.grid_cell[1]) != 1:
            return False   # only 4-neighbour moves
        self._from_x = self.x
        self._from_y = self.y
        self._lerp_t = 0.0
        self.target_cell = (tx, ty)
        return True

    def advance(self, dt: float, speed_scale: float = 1.0) -> bool:
        """
        Advance the current move. Returns True if the robot is at rest afterwards.
        `speed_scale` (0..1) lets a safety layer slow the robot down.
        """
        if self.target_cell is None:
            self.vx = 0.0; self.vy = 0.0
            return True

        tx, ty = self.target_cell
        step = self.preferred_speed * dt * max(0.0, speed_scale)
        self._lerp_t = min(1.0, self._lerp_t + step)

        prev_x, prev_y = self.x, self.y
        self.x = self._from_x + (tx - self._from_x) * self._lerp_t
        self.y = self._from_y + (ty - self._from_y) * self._lerp_t

        moved = math.hypot(self.x - prev_x, self.y - prev_y)
        self.total_distance += moved
        self.battery -= BATTERY_PER_CELL * moved

        if self._lerp_t >= 1.0:
            self.x, self.y = float(tx), float(ty)
            self.vx = 0.0; self.vy = 0.0
            self.grid_cell   = (tx, ty)
            self.target_cell = None
            return True

        seg_x, seg_y = tx - self._from_x, ty - self._from_y
        seg = math.hypot(seg_x, seg_y) or 1.0
        self.vx = seg_x / seg * self.preferred_speed * speed_scale
        self.vy = seg_y / seg * self.preferred_speed * speed_scale
        return False

    def is_at_cell(self, cx: int, cy: int) -> bool:
        return self.grid_cell == (cx, cy) and self.target_cell is None

    # ── priority (shared by PIBT, deadlock resolver) ─────────────────────────

    def priority_score(self) -> float:
        """
        Higher = goes first. Tasked robots outrank idle ones. Among tasked robots the
        one that has been working longest goes first — this ageing is what guarantees
        nobody is starved forever (PIBT's liveness argument relies on it).
        """
        s = float(self.priority_boost)
        if self.task is not None:
            s += 100.0 + self.task_age + 25.0 * (self.task.priority - 1) \
                 + 2.0 * self.blocked_ticks
            if self.state == RobotState.MOVING_TO_DROPOFF:
                s += 10.0            # finish what you've picked up: frees the fleet sooner
        elif self.state == RobotState.LOW_BATTERY:
            # Heading for a charger: nothing outranks it. Every tick spent being shoved
            # around by tasked robots is battery it may not have.
            s += 10_000.0 + self.blocked_ticks
        return s

    # ── distance / collision ─────────────────────────────────────────────────

    def distance_to(self, other: "Robot") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def overlaps(self, other: "Robot") -> bool:
        return self.distance_to(other) < (self.radius + other.radius)

    # ── serialization ─────────────────────────────────────────────────────────

    def broadcast_state(self) -> Dict[str, Any]:
        return {
            "id":           self.id,
            "color":        self.color,
            "pos":          [round(self.x, 3), round(self.y, 3)],
            "grid_cell":    list(self.grid_cell),
            "target_cell":  list(self.target_cell) if self.target_cell else None,
            "vel":          [round(self.vx, 3), round(self.vy, 3)],
            "battery":      round(max(0, self.battery), 1),
            "state":        self.state,
            "task_id":      self.task.id if self.task else None,
            "task_pickup":  list(self.task.pickup)  if self.task else None,
            "task_dropoff": list(self.task.dropoff) if self.task else None,
            "goal":         list(self.goal) if self.goal else None,
            "path":         [list(p) for p in self.path[:10]],
            "tasks_done":   self.tasks_completed,
            "collisions":   self.collision_count,
            "ticks_waiting":self.ticks_waiting,
            "blocked_ticks":self.blocked_ticks,
            "active":       self.active,
            "viz":          self.viz,
        }
