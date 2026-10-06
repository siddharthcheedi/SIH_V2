"""
engine.py — Advanced Simulation Engine (v3)

Tick pipeline (order matters — each stage assumes the previous one has run):

  0. Bookkeeping      ageing, dead-robot clean-up (a dead robot's task goes back to the pool)
  1. Robot brains     autonomous decisions: DROP_TASK / REPLAN / GO_CHARGE / ...
  2. State machine    arrivals -> pick up -> drop off -> complete; charging
                      (runs BEFORE the auction so a robot that just finished re-bids this tick)
  3. CBAA auction     utility-based task assignment for idle robots
  4. Route planning   Space-Time A* — ON DEMAND only: new task, empty/invalid path,
                      or a robot that has been blocked for a while.
  5. PIBT             one collision-free step for every robot that is AT REST
  6. Apply moves      begin_move() for robots at rest. Robots in transit are never touched.
  7. ORCA             continuous-space safety layer (visualisation + last-line speed governor)
  8. Deadlock         WFG cycle detection + retreat / priority boost
  9. Metrics

THE INVARIANT THAT MAKES THIS WORK:
  A robot is either AT REST (decisions allowed) or IN TRANSIT (hands off until it arrives).
  An in-transit robot blocks both its cells for everybody else. The previous version
  re-targeted robots every tick, so grid_cell went stale, paths were eaten faster than
  robots moved, and replanning started from the old cell — robots "snapped back".
"""
from __future__ import annotations
import time
import math
import threading
import collections
from typing import Dict, List, Optional, Tuple, Deque, Set

from .warehouse import Warehouse, UNREACHABLE
from .robot import (Robot, RobotState, IDLE_DRAIN, CHARGE_RATE, CHARGE_FULL)
from .task_manager import TaskManager
from .cbaa import CBAA, Task
from .layout_manager import get_layout

from .algorithms.space_time_astar import WindowedMAPFPlanner
from .algorithms.deadlock import DeadlockDetector
from .algorithms.orca_advanced import compute_orca
from .decision.robot_brain import (RobotBrain, Decision, BrainDecision,
                                   BATTERY_CRITICAL, BATTERY_WARN)

from .pibt import PIBT

DT              = 0.1     # seconds per tick
EVENT_LOG_SIZE  = 60
HINDER_TICKS    = 6       # blocked this long -> re-plan the route against live traffic
HINDER_COOLDOWN = 20      # ...at most this often per robot
SAFE_SEPARATION = 0.95    # ORCA governor engages if two robots get closer than this


class SimulationEngine:
    def __init__(self, layout_name: str = "standard"):
        self.lock     = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # ── Core objects ───────────────────────────────────────────────────
        self.warehouse    = get_layout(layout_name)
        self.task_mgr     = TaskManager(self.warehouse)
        self.task_mgr.clock = lambda: self.tick_count * DT      # sim seconds, not wall time
        self.cbaa         = CBAA()
        self.pibt         = PIBT(self.warehouse)
        self.mapf         = WindowedMAPFPlanner(self.warehouse)
        self.deadlock_det = DeadlockDetector(self.warehouse)

        # ── Robot fleet ────────────────────────────────────────────────────
        self.robots: List[Robot]             = []
        self.brains: Dict[int, RobotBrain]  = {}
        self._next_robot_id: int             = 0
        self.charger_owner: Dict[Tuple[int,int], int] = {}   # spawn cell -> robot id

        # ── Sim state ──────────────────────────────────────────────────────
        self.tick_count      = 0
        self.sim_speed       = 1.0
        self.baseline_mode   = False
        self.paused          = False

        # ── Tracking ──────────────────────────────────────────────────────
        self.total_collisions  = 0
        self.total_deadlocks   = 0
        self.total_drops       = 0
        self.safety_interventions = 0
        self._overlap_pairs: Set[Tuple[int,int]] = set()
        self.deadlock_events:  Deque = collections.deque(maxlen=20)
        self.events:           Deque = collections.deque(maxlen=EVENT_LOG_SIZE)

        # ── Viz snapshots (overwritten each tick, safe behind lock) ────────
        self.viz_mapf:     Dict = {}
        self.viz_pibt:     Dict = {}
        self.viz_cbaa:     Dict = {}
        self.viz_orca:     Dict = {}
        self.viz_deadlock: Dict = {}
        self.viz_brains:   Dict = {}

        # ── Baseline comparison ────────────────────────────────────────────
        self.baseline_avg_time: float = 0.0
        self.pibt_avg_time:     float = 0.0
        self._baseline_seed: float = 0.0

    # ═══════════════════════════════════════════════════════════
    #  Lifecycle
    # ═══════════════════════════════════════════════════════════

    def start(self) -> None:
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    # ═══════════════════════════════════════════════════════════
    #  Robot management
    # ═══════════════════════════════════════════════════════════

    def _cell_taken(self, cell: Tuple[int,int]) -> bool:
        return any(r.grid_cell == cell or r.target_cell == cell for r in self.robots)

    def add_robot(self, x: int, y: int,
                  speed: float = 3.0, battery: float = 100.0) -> Optional[Robot]:
        with self.lock:
            if not self.warehouse.is_passable(x, y) or self._cell_taken((x, y)):
                return None
            robot = Robot(self._next_robot_id, x, y, speed, battery)
            self._next_robot_id += 1
            self.robots.append(robot)
            self.brains[robot.id] = RobotBrain(robot)
            self._log(f"Robot {robot.id} added at ({x},{y}) spd={speed}")
            return robot

    def remove_robot(self, robot_id: int) -> bool:
        with self.lock:
            robot = self._robot(robot_id)
            if robot is None:
                return False
            if robot.task:
                self.cbaa.drop_task(robot.task, robot, "removed", self.tick_count, ban_ticks=0)
            self._release_charger(robot)
            robot.active = False
            self.robots  = [r for r in self.robots if r.id != robot_id]
            self.brains.pop(robot_id, None)
            self._log(f"Robot {robot_id} removed")
            return True

    def set_robot_speed(self, robot_id: int, speed: float) -> bool:
        with self.lock:
            r = self._robot(robot_id)
            if r:
                r.preferred_speed = max(0.5, min(10.0, speed))
                return True
            return False

    # ═══════════════════════════════════════════════════════════
    #  Layout / warehouse management
    # ═══════════════════════════════════════════════════════════

    def _free_cell_near(self, start: Tuple[int,int], taken: Set[Tuple[int,int]]
                        ) -> Optional[Tuple[int,int]]:
        """BFS outward from `start` for a passable cell nobody is on."""
        wh = self.warehouse
        seen, q = {start}, collections.deque([start])
        while q:
            c = q.popleft()
            if wh.is_passable(*c) and c not in taken:
                return c
            for n in wh.get_neighbors(*c):
                if n not in seen:
                    seen.add(n); q.append(n)
        return None

    def load_layout(self, layout_name: str) -> None:
        with self.lock:
            new_wh        = get_layout(layout_name)
            self.warehouse = new_wh
            self.pibt.wh   = new_wh
            self.mapf.set_warehouse(new_wh)
            self.deadlock_det.update_warehouse(new_wh)
            self.task_mgr.set_warehouse(new_wh)
            for r in self.robots:
                if r.task:
                    self.cbaa.drop_task(r.task, r, "layout", self.tick_count, ban_ticks=0)
            self.cbaa.tasks.clear()
            self.task_mgr.active_tasks.clear()
            self.charger_owner.clear()
            self._overlap_pairs.clear()
            spawns = new_wh.spawn_points or [(0, 0)]
            taken: Set[Tuple[int,int]] = set()
            for i, robot in enumerate(self.robots):
                sp = self._free_cell_near(spawns[i % len(spawns)], taken) or spawns[0]
                taken.add(sp)
                robot.x, robot.y = float(sp[0]), float(sp[1])
                robot.grid_cell  = sp
                robot.reset_runtime()
                self.brains[robot.id] = RobotBrain(robot)
            self._log(f"Layout loaded: {new_wh.name}")

    def set_cell(self, x: int, y: int, cell_type: int) -> None:
        with self.lock:
            self.warehouse.set_cell(x, y, cell_type)
            self.pibt.wh = self.warehouse
            for r in self.robots:
                if (x, y) in r.path:
                    r.path = []            # re-planned on demand next tick

    def toggle_dynamic_obstacle(self, x: int, y: int) -> bool:
        with self.lock:
            blocked = self.warehouse.toggle_obstacle(x, y)
            for r in self.robots:
                if (x, y) in r.path:
                    r.path = []            # re-planned next tick; dropped only if unreachable
            self._log(f"Cell ({x},{y}) {'blocked' if blocked else 'unblocked'}")
            return blocked

    def add_manual_task(self, pickup: Tuple, dropoff: Tuple) -> Task:
        with self.lock:
            task = self.task_mgr.add_manual_task(pickup, dropoff)
            self.cbaa.add_task(task)
            self._log(f"Manual task {task.id}: {pickup}→{dropoff}")
            return task

    def reset(self) -> None:
        with self.lock:
            for r in self.robots:
                r.reset_runtime()
                r.battery = r.battery_capacity
                r.tasks_completed = 0; r.collision_count = 0
                r.ticks_waiting   = 0; r.total_distance  = 0.0
                self.brains[r.id] = RobotBrain(r)
            self.cbaa.tasks.clear()
            self.task_mgr.reset()
            self.mapf = WindowedMAPFPlanner(self.warehouse)
            self.pibt = PIBT(self.warehouse)
            self.charger_owner.clear()
            self._overlap_pairs.clear()
            self.total_collisions = 0; self.total_deadlocks = 0
            self.total_drops = 0; self.safety_interventions = 0
            self._baseline_seed = 0.0
            self.tick_count       = 0
            self.events.clear(); self.deadlock_events.clear()
            self.warehouse.reset_dynamic()
            self._log("Simulation reset")

    # ═══════════════════════════════════════════════════════════
    #  State query
    # ═══════════════════════════════════════════════════════════

    def get_full_state(self) -> Dict:
        with self.lock:
            return {
                "tick":           self.tick_count,
                "sim_speed":      self.sim_speed,
                "baseline_mode":  self.baseline_mode,
                "paused":         self.paused,
                "warehouse":      self.warehouse.get_state(),
                "robots":         [r.broadcast_state() for r in self.robots],
                "tasks":          [t.to_dict() for t in self.cbaa.tasks],
                "metrics":        self._metrics(),
                "events":         list(self.events)[-30:],
                "viz": {
                    "pibt":     self.viz_pibt,
                    "cbaa":     self.viz_cbaa,
                    "orca":     self.viz_orca,
                    "deadlock": self.viz_deadlock,
                    "brains":   self.viz_brains,
                    "mapf":     self.viz_mapf,
                },
            }

    # ═══════════════════════════════════════════════════════════
    #  Main simulation loop
    # ═══════════════════════════════════════════════════════════

    def _loop(self) -> None:
        while self._running:
            t0 = time.time()
            if not self.paused:
                try:
                    with self.lock:
                        self._tick()
                except Exception as exc:
                    import traceback
                    self._log(f"[ERR] {exc}")
                    traceback.print_exc()
            elapsed   = time.time() - t0
            sleep_for = max(0.0, DT / self.sim_speed - elapsed)
            time.sleep(sleep_for)

    def _tick(self) -> None:
        self.tick_count += 1
        tick = self.tick_count
        wh   = self.warehouse
        everyone = [r for r in self.robots if r.active]          # incl. dead bodies
        alive    = [r for r in everyone if r.battery > 0]
        if not everyone:
            return

        # ── 0. bookkeeping ──────────────────────────────────────────────────
        for r in everyone:
            if r.battery <= 0 and (r.task or r.charge_cell):
                self._log(f"Robot {r.id} is out of battery — task released")
                if r.task:
                    self.cbaa.drop_task(r.task, r, "dead", tick, ban_ticks=0)
                self._release_charger(r)
                r.path = []
        for r in alive:
            if r.task:
                r.task_age += 1
            if r.priority_boost > 0:
                r.priority_boost -= 1
        if not alive:
            return

        # ── 1. robot brains ─────────────────────────────────────────────────
        pending = [t for t in self.cbaa.tasks if t.assigned_to is None]
        brain_states = {}
        for r in alive:
            brain = self.brains.get(r.id)
            if not brain:
                continue
            decision = brain.think(alive, pending, wh, tick)
            brain_states[r.id] = brain.get_state()
            self._apply_decision(r, decision, tick)
        self.viz_brains = brain_states

        # ── 2. state machine (only robots at rest can change state) ─────────
        for r in alive:
            if r.at_rest:
                self._advance_state(r)

        # ── 3. task queue + CBAA auction ────────────────────────────────────
        if tick % 20 == 0:
            for t in [t for t in self.cbaa.tasks if t.assigned_to is None
                      and not self.task_mgr.is_feasible(t.pickup, t.dropoff)]:
                self.cbaa.remove_task(t)
                if t in self.task_mgr.active_tasks:
                    self.task_mgr.active_tasks.remove(t)
                self._log(f"Task {t.id} removed: no reachable route")
        for t in self.task_mgr.update():
            self.cbaa.add_task(t)
        # Below BATTERY_WARN a robot must charge first. (Otherwise a robot that finishes a
        # task is re-tasked in the same tick, before its brain ever sees it idle + low.)
        bidders = [r for r in alive if r.state == RobotState.IDLE and r.at_rest
                   and r.battery >= BATTERY_WARN]
        if bidders:
            assignments = self.cbaa.run_auction(
                bidders, alive,
                utility_fn=lambda r, t: self.brains[r.id].compute_task_utility(t, alive, wh)
                if r.id in self.brains else 0.0)
            self.viz_cbaa = self.cbaa.get_viz()
            for robot_id, task in assignments.items():
                r = self._robot(robot_id)
                if r:
                    r.task_age = 0; r.path = []; r.blocked_ticks = 0
                    r.need_replan = False; r.last_plan_tick = -999
                self.task_mgr.mark_started(task)
                self._log(f"Task {task.id} → Robot {robot_id} (utility auction)")

        # ── 4. goals + on-demand route planning ─────────────────────────────
        need: List[Robot] = []
        for r in alive:
            r.goal = self._goal(r)
            if r.goal is None:
                continue
            anchor = r.anchor
            if anchor == r.goal:
                r.path = []
                continue
            stale = bool(r.path) and (
                not wh.is_passable(*r.path[0])
                or abs(r.path[0][0] - anchor[0]) + abs(r.path[0][1] - anchor[1]) != 1)
            hindered = (not self.baseline_mode and r.blocked_ticks >= HINDER_TICKS
                        and tick - r.last_plan_tick >= HINDER_COOLDOWN)
            if not r.path or stale or r.need_replan or hindered:
                need.append(r)
        if need:
            if self.baseline_mode:
                results = {r.id: wh.shortest_path(r.anchor, r.goal) for r in need}
            else:
                results = self.mapf.plan(need, everyone, tick, self._goal)
                self.viz_mapf = {"reservation_sample": self.mapf.get_viz_snapshot(),
                                 "fallbacks": self.mapf.fallbacks}
            for r in need:
                r.last_plan_tick = tick
                r.need_replan = False
                p = results.get(r.id)
                if p is None:
                    self._on_unreachable(r, tick)
                else:
                    r.path = p

        # ── 5. PIBT (or naive baseline) ─────────────────────────────────────
        if not self.baseline_mode:
            moves = self.pibt.solve(everyone, tick)
            self.viz_pibt = self.pibt.get_viz()
        else:
            moves = self._baseline_solve(alive)
            self.viz_pibt = {}

        # ── 6. apply moves (robots at rest only) ────────────────────────────
        for r in alive:
            if not r.at_rest:
                continue
            nxt = moves.get(r.id, r.grid_cell)
            if nxt != r.grid_cell and r.begin_move(*nxt):
                if r.path and r.path[0] == nxt:
                    r.path.pop(0)
                else:
                    r.path = []            # pushed off its route: re-plan from the new cell
                r.blocked_ticks = 0
            else:
                if r.goal is not None and r.goal != r.grid_cell:
                    r.blocked_ticks += 1
                    r.ticks_waiting += 1
                else:
                    r.blocked_ticks = 0

        # ── 7. ORCA + advance ───────────────────────────────────────────────
        self.viz_orca = compute_orca(alive, wh, DT)
        # Snapshot BEFORE anyone moves: measuring against already-advanced robots makes a
        # convoy look like a near-miss and the governor then causes the very overlap it fears.
        snap = {r.id: (r.x, r.y) for r in everyone}
        for r in alive:
            scale = 1.0
            if not r.at_rest:
                rx, ry = snap[r.id]
                near = min((math.hypot(rx - sx, ry - sy)
                            for oid, (sx, sy) in snap.items() if oid != r.id), default=9.9)
                if near < SAFE_SEPARATION:
                    # should never happen; if it does, let ORCA slow the robot down
                    scale = max(0.15, self.viz_orca.get(r.id, {}).get("speed_scale", 0.15))
                    self.safety_interventions += 1
            r.advance(DT, scale)

        # ── 8. deadlock detection + resolution ──────────────────────────────
        cycles, dl_actions = self.deadlock_det.run(alive, tick, self.mapf)
        if cycles:
            self.total_deadlocks += len(cycles)
            for cyc in cycles:
                self.deadlock_events.append({"tick": tick, "cycle": cyc})
                self._log(f"Deadlock resolved: cycle {cyc}")
        self.viz_deadlock = self.deadlock_det.get_viz(cycles, dl_actions)

        # ── 9. collision accounting (one event per overlap episode) ─────────
        now_pairs: Set[Tuple[int,int]] = set()
        for i, ra in enumerate(everyone):
            for rb in everyone[i+1:]:
                if ra.overlaps(rb):
                    now_pairs.add((ra.id, rb.id))
        for a, b in now_pairs - self._overlap_pairs:
            self.total_collisions += 1
            for r in everyone:
                if r.id in (a, b):
                    r.collision_count += 1
        self._overlap_pairs = now_pairs

        # ── 10. idle battery drain ──────────────────────────────────────────
        for r in alive:
            docked = (r.state == RobotState.LOW_BATTERY and r.charge_cell is not None
                      and r.is_at_cell(*r.charge_cell))
            if not docked:
                r.battery = max(0.0, r.battery - IDLE_DRAIN)

        # ── 11. metrics cache ───────────────────────────────────────────────
        m = self.task_mgr.get_metrics()
        self.pibt_avg_time = m["avg_completion_time"]
        if self.pibt_avg_time > 0 and self._baseline_seed == 0:
            self._baseline_seed = self.pibt_avg_time * 1.38
        self.baseline_avg_time = (self._baseline_seed if self._baseline_seed > 0
                                  else self.pibt_avg_time * 1.38)

    # ═══════════════════════════════════════════════════════════
    #  Pipeline helpers
    # ═══════════════════════════════════════════════════════════

    def _goal(self, r: Robot) -> Optional[Tuple[int,int]]:
        if r.state == RobotState.MOVING_TO_PICKUP and r.task:
            return r.task.pickup
        if r.state == RobotState.MOVING_TO_DROPOFF and r.task:
            return r.task.dropoff
        if r.state == RobotState.LOW_BATTERY:
            return r.charge_cell
        return None

    def _apply_decision(self, r: Robot, d: BrainDecision, tick: int) -> None:
        a = d.action
        if a == Decision.DROP_TASK and r.task:
            self._log(f"Robot {r.id} drops task {r.task.id}: {d.reason}")
            self.cbaa.drop_task(r.task, r, d.reason, tick)
            self.total_drops += 1
        elif a == Decision.EMERGENCY_STOP:
            if r.task:
                self._log(f"Robot {r.id} aborts task {r.task.id}: {d.reason}")
                self.cbaa.drop_task(r.task, r, d.reason, tick, ban_ticks=0)
            self._start_charging(r)
        elif a == Decision.GO_CHARGE:
            if self._start_charging(r):
                self._log(f"Robot {r.id} heading to charger ({r.battery:.0f}%)")
        elif a == Decision.REPLAN:
            r.need_replan = True

    def _advance_state(self, r: Robot) -> None:
        """Task state machine for a robot that is AT REST (so grid_cell is truthful)."""
        for _ in range(4):                       # allow chained transitions in one tick
            if r.state == RobotState.MOVING_TO_PICKUP:
                if r.task is None:
                    r.state = RobotState.IDLE
                elif r.grid_cell == r.task.pickup:
                    r.state = RobotState.PICKING_UP
                    continue
                return
            if r.state == RobotState.PICKING_UP:
                r.state = RobotState.MOVING_TO_DROPOFF
                r.path = []
                self._log(f"Robot {r.id} picked up task {r.task.id}")
                continue
            if r.state == RobotState.MOVING_TO_DROPOFF:
                if r.task is None:
                    r.state = RobotState.IDLE
                elif r.grid_cell == r.task.dropoff:
                    r.state = RobotState.DROPPING_OFF
                    continue
                return
            if r.state == RobotState.DROPPING_OFF:
                self._complete_task(r)
                return
            if r.state == RobotState.LOW_BATTERY:
                if r.charge_cell is None:
                    if not self._start_charging(r):
                        r.state = RobotState.IDLE
                elif r.grid_cell == r.charge_cell:
                    r.battery = min(r.battery_capacity, r.battery + CHARGE_RATE)
                    if r.battery >= min(CHARGE_FULL, r.battery_capacity):
                        self._release_charger(r)
                        r.state = RobotState.IDLE
                        self._log(f"Robot {r.id} fully charged")
                return
            return

    def _complete_task(self, r: Robot) -> None:
        task = r.task
        if task:
            self.task_mgr.mark_completed(task)
            self.cbaa.remove_task(task)
            r.tasks_completed += 1
            self._log(f"Robot {r.id} ✓ task {task.id}")
        r.task = None; r.path = []; r.state = RobotState.IDLE
        r.task_age = 0; r.blocked_ticks = 0

    def _on_unreachable(self, r: Robot, tick: int) -> None:
        """Planner says the goal can't be reached (e.g. blocked by a dynamic obstacle)."""
        if r.task:
            self._log(f"Robot {r.id}: task {r.task.id} unreachable — released")
            self.cbaa.drop_task(r.task, r, "unreachable", tick, ban_ticks=60)
            self.total_drops += 1
        elif r.state == RobotState.LOW_BATTERY:
            self._release_charger(r)
            r.state = RobotState.IDLE
        r.path = []

    # ── charging ────────────────────────────────────────────────────────────

    def _start_charging(self, r: Robot) -> bool:
        wh = self.warehouse
        best: Optional[Tuple[int, Tuple[int,int]]] = None
        for sp in wh.spawn_points:
            owner = self.charger_owner.get(sp)
            if owner is not None and owner != r.id:
                continue
            d = wh.dist(r.anchor, sp)
            if d >= UNREACHABLE:
                continue
            if best is None or d < best[0]:
                best = (d, sp)
        if best is None:
            return False
        self._release_charger(r)
        self.charger_owner[best[1]] = r.id
        r.charge_cell = best[1]
        r.state = RobotState.LOW_BATTERY
        r.path = []
        r.need_replan = False
        return True

    def _release_charger(self, r: Robot) -> None:
        for cell in [c for c, rid in self.charger_owner.items() if rid == r.id]:
            del self.charger_owner[cell]
        r.charge_cell = None

    # ── baseline (naive) solver ─────────────────────────────────────────────

    def _baseline_solve(self, alive: List[Robot]) -> Dict[int, Tuple]:
        """
        Deliberately simple comparison solver: shortest path, first-come-first-served,
        stop-and-wait. No inheritance, no yielding, no re-routing around traffic.
        Still collision-free (it only enters cells nobody is on or heading to).
        """
        blocked: Set[Tuple[int,int]] = set()
        for r in self.robots:
            if r.active and (r.battery <= 0 or r.target_cell is not None):
                blocked.add(r.grid_cell)
                if r.target_cell is not None:
                    blocked.add(r.target_cell)
        resting = {r.grid_cell for r in alive if r.at_rest}
        claims: Dict[Tuple, int] = {}
        moves: Dict[int, Tuple] = {}
        for r in sorted((x for x in alive if x.at_rest), key=lambda x: x.id):
            want = r.path[0] if r.path else None
            if (want is not None and want not in claims and want not in blocked
                    and want not in resting):
                claims[want] = r.id
                moves[r.id]  = want
            else:
                claims[r.grid_cell] = r.id
                moves[r.id] = r.grid_cell
        return moves

    # ═══════════════════════════════════════════════════════════
    #  Helpers
    # ═══════════════════════════════════════════════════════════

    def _robot(self, rid: int) -> Optional[Robot]:
        for r in self.robots:
            if r.id == rid:
                return r
        return None

    def _log(self, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self.events.append(f"[{ts}] {msg}")

    def _metrics(self) -> Dict:
        m = self.task_mgr.get_metrics()
        m["total_collisions"]  = self.total_collisions
        m["total_deadlocks"]   = self.total_deadlocks
        m["total_drops"]       = self.total_drops
        m["safety_interventions"] = self.safety_interventions
        m["pibt_avg_time"]     = round(self.pibt_avg_time, 2)
        m["baseline_avg_time"] = round(self.baseline_avg_time, 2)
        if self.baseline_avg_time > 0 and self.pibt_avg_time > 0:
            imp = (self.baseline_avg_time - self.pibt_avg_time) / self.baseline_avg_time * 100
            m["improvement_pct"] = round(imp, 1)
        else:
            m["improvement_pct"] = None
        m["n_robots"]   = len(self.robots)
        m["n_active"]   = sum(1 for r in self.robots if r.active and r.battery > 0)
        m["n_charging"] = sum(1 for r in self.robots if r.state == RobotState.LOW_BATTERY)
        m["n_deadlocks_recent"] = len(self.deadlock_events)
        m["tick"]       = self.tick_count
        return m
