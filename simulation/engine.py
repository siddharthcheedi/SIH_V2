"""
engine.py — the PHYSICAL WORLD for a decentralised fleet (v4).

The engine no longer plans, assigns tasks or arbitrates movement. It only simulates what
the world does:
    * carries radio messages (comms.MessageBus: range, delay, loss)
    * announces orders and records pick-ups / deliveries (the order system, or "task board")
    * moves bodies (Robot.advance), drains / charges batteries
    * measures collisions and other metrics

Every decision is taken inside each robot's own Agent (agent.py). The engine hands an agent
exactly three things per tick: its radio inbox, the list of open orders, and nothing else.
Dashboard "viz" data is assembled by READING the agents afterwards (observer only).

Tick:  deliver radio -> sync order board -> every Agent.step() -> ORCA safety layer ->
       move bodies -> collisions / battery -> metrics
"""
from __future__ import annotations
import time
import math
import threading
import collections
from typing import Dict, List, Optional, Tuple, Deque, Set

from .warehouse import Warehouse
from .robot import Robot, RobotState, IDLE_DRAIN, CHARGE_RATE
from .task_manager import TaskManager
from .cbaa import Task
from .layout_manager import get_layout
from .comms import MessageBus, CommsConfig
from .agent import Agent
from .algorithms.orca_advanced import compute_orca

DT              = 0.1     # seconds per tick
EVENT_LOG_SIZE  = 60
SAFE_SEPARATION = 0.95    # ORCA governor engages if two bodies get closer than this


class SimulationEngine:
    def __init__(self, layout_name: str = "standard", comms: Optional[CommsConfig] = None):
        self.lock     = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self.warehouse = get_layout(layout_name)
        self.task_mgr  = TaskManager(self.warehouse)
        self.task_mgr.clock = lambda: self.tick_count * DT
        self.bus       = MessageBus(comms or CommsConfig())

        self.robots: List[Robot]      = []
        self.agents: Dict[int, Agent] = {}
        self._next_robot_id = 0

        self.tick_count    = 0
        self.sim_speed     = 1.0
        self.baseline_mode = False       # True = robots do not cooperate (no yielding / probes / shared routes)
        self.paused        = False

        self.total_collisions = 0
        self.total_deadlocks  = 0
        self.total_drops      = 0
        self.total_outbid     = 0
        self.safety_interventions = 0
        self._overlap_pairs: Set[Tuple[int,int]] = set()
        self.deadlock_events: Deque = collections.deque(maxlen=20)
        self.events:          Deque = collections.deque(maxlen=EVENT_LOG_SIZE)
        self._seen_cycles: Set[Tuple] = set()

        self.viz_pibt: Dict = {}; self.viz_cbaa: Dict = {}; self.viz_orca: Dict = {}
        self.viz_deadlock: Dict = {}; self.viz_brains: Dict = {}; self.viz_mapf: Dict = {}

        # honest A/B bookkeeping: execution times recorded per mode while that mode was active
        self.mode_times: Dict[str, List[float]] = {"cooperative": [], "baseline": []}

    # ── lifecycle ───────────────────────────────────────────────────────────
    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    # ── radio configuration ────────────────────────────────────────────────
    def set_comms(self, comm_range: Optional[float] = None, delay: Optional[int] = None,
                  loss: Optional[float] = None) -> Dict:
        with self.lock:
            cfg = self.bus.cfg
            if comm_range is not None: cfg.comm_range = max(0.0, float(comm_range))
            if delay is not None:      cfg.delay = max(1, int(delay))
            if loss is not None:       cfg.loss = min(0.9, max(0.0, float(loss)))
            for r in self.robots:      # each radio knows its own delay setting
                self.agents[r.id] = self._make_agent(r, keep=self.agents.get(r.id))
            self._log(f"Radio: range={cfg.comm_range} delay={cfg.delay} loss={cfg.loss}")
            return self.bus.stats()

    # ── robots ──────────────────────────────────────────────────────────────
    def _robot(self, rid: int) -> Optional[Robot]:
        return next((r for r in self.robots if r.id == rid), None)

    def _make_agent(self, robot: Robot, keep: Optional[Agent] = None) -> Agent:
        loss_margin = 2 if self.bus.cfg.loss > 0 else 0
        a = Agent(robot, self.warehouse, self._uplink, self.bus.sender_for(robot.id),
                  delay=self.bus.cfg.delay, loss_margin=loss_margin)
        if keep is not None:           # preserve the robot's memory when only the radio changes
            for k in ("brain", "cb", "probes", "nbr", "dead_cells", "requests", "claim",
                      "claim_since", "push_since", "ban_until", "retreat", "stats"):
                setattr(a, k, getattr(keep, k))
        return a

    def _cell_taken(self, cell) -> bool:
        return any(r.grid_cell == cell or r.target_cell == cell for r in self.robots)

    def add_robot(self, x: int, y: int, speed: float = 3.0, battery: float = 100.0) -> Optional[Robot]:
        with self.lock:
            if not self.warehouse.is_passable(x, y) or self._cell_taken((x, y)):
                return None
            robot = Robot(self._next_robot_id, x, y, speed, battery)
            self._next_robot_id += 1
            self.robots.append(robot)
            self.agents[robot.id] = self._make_agent(robot)
            self._log(f"Robot {robot.id} added at ({x},{y}) spd={speed}")
            return robot

    def remove_robot(self, robot_id: int) -> bool:
        with self.lock:
            robot = self._robot(robot_id)
            if robot is None:
                return False
            robot.active = False
            self.robots = [r for r in self.robots if r.id != robot_id]
            self.agents.pop(robot_id, None)      # its held task is NOT released by anyone:
            self._log(f"Robot {robot_id} removed")   # the others notice it went quiet
            return True

    def set_robot_speed(self, robot_id: int, speed: float) -> bool:
        with self.lock:
            r = self._robot(robot_id)
            if r:
                r.preferred_speed = max(0.5, min(10.0, speed))
                return True
            return False

    # ── layout / world edits ────────────────────────────────────────────────
    def _free_cell_near(self, start, taken):
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
            wh = get_layout(layout_name)
            self.warehouse = wh
            self.task_mgr.set_warehouse(wh)
            self.task_mgr.active_tasks.clear()
            self.bus = MessageBus(self.bus.cfg)
            self._overlap_pairs.clear()
            spawns = wh.spawn_points or [(0, 0)]
            taken: Set[Tuple[int,int]] = set()
            for i, robot in enumerate(self.robots):
                sp = self._free_cell_near(spawns[i % len(spawns)], taken) or spawns[0]
                taken.add(sp)
                robot.x, robot.y, robot.grid_cell = float(sp[0]), float(sp[1]), sp
                robot.reset_runtime()
                self.agents[robot.id] = self._make_agent(robot)
            self._log(f"Layout loaded: {wh.name}")

    def set_cell(self, x: int, y: int, cell_type: int) -> None:
        with self.lock:
            self.warehouse.set_cell(x, y, cell_type)

    def toggle_dynamic_obstacle(self, x: int, y: int) -> bool:
        with self.lock:
            blocked = self.warehouse.toggle_obstacle(x, y)
            # Robots notice a blocked cell on their own (their next step finds their path
            # invalid and re-plans); nothing is pushed to them from here.
            self._log(f"Cell ({x},{y}) {'blocked' if blocked else 'unblocked'}")
            return blocked

    def add_manual_task(self, pickup, dropoff) -> Task:
        with self.lock:
            task = self.task_mgr.add_manual_task(pickup, dropoff)
            self._log(f"Manual task {task.id}: {pickup}→{dropoff}")
            return task

    def reset(self) -> None:
        with self.lock:
            self.bus = MessageBus(self.bus.cfg)
            for r in self.robots:
                r.reset_runtime()
                r.battery = r.battery_capacity
                r.tasks_completed = 0; r.collision_count = 0
                r.ticks_waiting = 0; r.total_distance = 0.0
                self.agents[r.id] = self._make_agent(r)
            self.task_mgr.reset()
            self._overlap_pairs.clear(); self._seen_cycles.clear()
            self.total_collisions = self.total_deadlocks = self.total_drops = 0
            self.safety_interventions = 0
            self.mode_times = {"cooperative": [], "baseline": []}
            self.tick_count = 0
            self.events.clear(); self.deadlock_events.clear()
            self.warehouse.reset_dynamic()
            self._log("Simulation reset")

    # ── order system (task board) ───────────────────────────────────────────
    def _uplink(self, event: str, task_id: int, robot_id: int) -> bool:
        """Robots report to the order system. The item is physical: it can be picked once."""
        t = next((x for x in self.task_mgr.active_tasks if x.id == task_id), None)
        if event == "pickup":
            if t is None or t.picked:
                return False
            t.picked = True
            t.assigned_to = robot_id
            return True
        if event == "complete":
            if t is not None:
                self.task_mgr.mark_completed(t)
                mode = "baseline" if self.baseline_mode else "cooperative"
                if t.started_at > 0 and t.completed_at > t.started_at:
                    self.mode_times[mode].append(t.completed_at - t.started_at)
            return True
        return False

    def _sync_order_board(self, tick: int) -> None:
        """Observer-side: keep order queue topped up; mirror who holds what for the dashboard."""
        holders = {a.cb.mine: rid for rid, a in self.agents.items() if a.cb.mine is not None}
        for t in self.task_mgr.active_tasks:
            if not t.picked:
                t.assigned_to = holders.get(t.id)
                if t.assigned_to is not None:
                    self.task_mgr.mark_started(t)
        if tick % 20 == 0:
            for t in [t for t in self.task_mgr.active_tasks if not t.picked
                      and not self.task_mgr.is_feasible(t.pickup, t.dropoff)]:
                self.task_mgr.active_tasks.remove(t)
                self._log(f"Task {t.id} removed: no reachable route")
        self.task_mgr.update()

    # ── state for the dashboard ─────────────────────────────────────────────
    def get_full_state(self) -> Dict:
        with self.lock:
            return {
                "tick": self.tick_count, "sim_speed": self.sim_speed,
                "baseline_mode": self.baseline_mode, "paused": self.paused,
                "warehouse": self.warehouse.get_state(),
                "robots": [r.broadcast_state() for r in self.robots],
                "tasks": [t.to_dict() for t in self.task_mgr.active_tasks],
                "metrics": self._metrics(),
                "events": list(self.events)[-30:],
                "viz": {"pibt": self.viz_pibt, "cbaa": self.viz_cbaa, "orca": self.viz_orca,
                        "deadlock": self.viz_deadlock, "brains": self.viz_brains,
                        "mapf": self.viz_mapf, "comms": self.bus.stats()},
            }

    # ── main loop ───────────────────────────────────────────────────────────
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
            time.sleep(max(0.0, DT / self.sim_speed - (time.time() - t0)))

    def _tick(self) -> None:
        self.tick_count += 1
        tick = self.tick_count
        everyone = [r for r in self.robots if r.active]
        if not everyone:
            return

        # radio: deliver what was sent `delay` ticks ago
        self.bus.begin_tick(tick, {r.id: (r.x, r.y) for r in everyone})
        inboxes = self.bus.deliver(tick)

        # order system
        self._sync_order_board(tick)
        open_infos = [t.info() for t in self.task_mgr.active_tasks if not t.picked]

        # every robot thinks for itself
        for r in everyone:
            agent = self.agents[r.id]
            agent.cooperative = not self.baseline_mode
            agent.step(tick, inboxes.get(r.id, []), open_infos)
            for e in agent.drain_events():
                self._log(e)
        alive = [r for r in everyone if r.battery > 0]

        # onboard collision-avoidance layer (ORCA). Each robot's calculation uses only bodies
        # within its sensing radius; it is a safety net, not the coordination mechanism.
        self.viz_orca = compute_orca(alive, self.warehouse, DT)
        snap = {r.id: (r.x, r.y) for r in everyone}
        for r in alive:
            scale = 1.0
            if not r.at_rest:
                rx, ry = snap[r.id]
                near = min((math.hypot(rx - sx, ry - sy) for oid, (sx, sy) in snap.items()
                            if oid != r.id), default=9.9)
                if near < SAFE_SEPARATION:
                    scale = max(0.15, self.viz_orca.get(r.id, {}).get("speed_scale", 0.15))
                    self.safety_interventions += 1
            r.advance(DT, scale)

        # collisions: one event per overlap episode
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

        # batteries (physics): drain, or charge when docked
        for r in alive:
            docked = (r.state == RobotState.LOW_BATTERY and r.charge_cell is not None
                      and r.is_at_cell(*r.charge_cell))
            if docked:
                r.battery = min(r.battery_capacity, r.battery + CHARGE_RATE)
            else:
                r.battery = max(0.0, r.battery - IDLE_DRAIN)

        self._build_viz(everyone)

    # ── observer-side visualisation ─────────────────────────────────────────
    def _build_viz(self, everyone: List[Robot]) -> None:
        ags = [self.agents[r.id] for r in everyone]
        order = sorted(ags, key=lambda a: (-a.eff_prio, a.id))
        for rank, a in enumerate(order):
            a.robot.viz["priority"] = rank
            a.robot.viz["claimed_cell"] = list(a.claim) if a.claim else None
            a.robot.viz["in_conflict"] = a.in_conflict
        self.viz_pibt = {
            "priorities": {a.id: i for i, a in enumerate(order)},
            "claimed_cells": [[a.claim[0], a.claim[1], a.id] for a in ags if a.claim],
            "conflicts": [{"from_id": a.id, "to_id": a.waiting_on, "cell": list(a.robot.desired_cell or a.robot.grid_cell)}
                          for a in ags if a.waiting_on is not None],
            "waiting": [a.id for a in ags if a.waiting_on is not None],
        }
        bids: Dict[str, Dict[str, float]] = {}
        for a in ags:
            for tid, b in a.robot.viz.get("bid_values", {}).items():
                bids.setdefault(str(tid), {})[str(a.id)] = b
        self.viz_cbaa = {"bids": bids,
                         "assignments": {str(a.id): a.cb.mine for a in ags if a.cb.mine is not None},
                         "events": []}
        self.viz_brains = {a.id: a.brain.get_state() for a in ags}
        self.viz_mapf = {"fallbacks": sum(a.planner.fallbacks for a in ags)}
        self.total_drops = sum(a.stats["drops"] for a in ags)
        self.total_outbid = sum(a.stats["outbid"] for a in ags)
        for a in ags:
            for d in a.probes.detected:
                key = (d["tick"], tuple(d["cycle"]))
                if key not in self._seen_cycles:
                    self._seen_cycles.add(key)
                    self.total_deadlocks += 1
                    self.deadlock_events.append({"tick": d["tick"], "cycle": d["cycle"]})
                    self._log(f"Deadlock {d['cycle']} found by probe, robot {d['victim']} retreats")
        recent = [d for d in self.deadlock_events if self.tick_count - d["tick"] < 20]
        self.viz_deadlock = {"cycles": [d["cycle"] for d in recent], "actions": [],
                             "n_deadlocks": len(recent)}

    # ── helpers ─────────────────────────────────────────────────────────────
    def _log(self, msg: str) -> None:
        self.events.append(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def _metrics(self) -> Dict:
        m = self.task_mgr.get_metrics()
        n = max(1, len(self.robots))
        m["total_collisions"] = self.total_collisions
        m["total_deadlocks"]  = self.total_deadlocks
        m["total_drops"]      = self.total_drops
        m["consensus_conflicts"] = self.total_outbid
        m["safety_interventions"] = self.safety_interventions
        coop, base = self.mode_times["cooperative"], self.mode_times["baseline"]
        avg = lambda xs: round(sum(xs) / len(xs), 2) if xs else 0.0
        m["pibt_avg_time"], m["baseline_avg_time"] = avg(coop), avg(base)
        # MEASURED, not assumed: only available once both modes have completed >=5 tasks
        m["improvement_pct"] = (round((avg(base) - avg(coop)) / avg(base) * 100, 1)
                                if len(coop) >= 5 and len(base) >= 5 and avg(base) > 0 else None)
        m["n_robots"]   = len(self.robots)
        m["n_active"]   = sum(1 for r in self.robots if r.active and r.battery > 0)
        m["n_charging"] = sum(1 for r in self.robots if r.state == RobotState.LOW_BATTERY)
        m["n_deadlocks_recent"] = len(self.deadlock_events)
        m["tick"] = self.tick_count
        s = self.bus.stats()
        m["msgs_sent"] = s["sent"]
        m["msgs_per_robot_per_tick"] = round(s["sent"] / n / max(1, self.tick_count), 2)
        return m
