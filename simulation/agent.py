"""
agent.py — the onboard controller of ONE robot. Fully decentralised.

What an Agent may use
  * its own Robot (state + actuators: begin_move)
  * the warehouse map (a static prior; every robot carries the floor plan)
  * its radio inbox (messages from robots that were within range, `delay` ticks ago)
  * the order announcements (TaskInfo list) and an uplink to report pickup / completion
  * `send(msg, to=None)` — its radio transmitter
What it can NOT touch: the engine, other robots' objects, the message bus, any global list.
(tests/test_decentralised.py enforces this by scanning the source.)

Per tick, in order
  ingest radio -> consensus on tasks (CBAA) -> brain -> task state machine -> pick a task
  -> plan own route -> negotiate next move -> deadlock probes -> broadcast beacon

MOVEMENT PROTOCOL (replaces the central PIBT solver)
  1. CLAIM.   To enter cell v a robot broadcasts claim(v) and keeps it on the air.
  2. ORDER.   Claims are ordered by (claim_since, -priority, id). The earliest claim wins.
  3. GO.      A robot may enter v only when
                 - its claim is at least `delay` ticks old   (so every competitor in range
                   that claimed earlier has been heard — the later claimant ALWAYS sees the
                   earlier one before moving, whatever the delay), and
                 - no neighbour is on / entering v, and no earlier claim on v exists.
              Exception: it may FOLLOW a robot that is already leaving v in the same
              straight direction (a convoy), never around a corner (0.71 < 0.8 body size).
  4. YIELD.   If v is occupied by a robot at rest, ask it to move (YIELD message carrying
              my priority). The occupant adopts the highest priority asking it — priority
              INHERITANCE — and, if that outranks its own, vacates, asking whoever is in ITS
              way in turn. Chains form by message passing, not by a solver that sees everyone.
  5. PROBE.   If blocked a while, chase the wait-for chain with probe messages; a probe that
              returns to its sender is a deadlock (algorithms/deadlock.py).
"""
from __future__ import annotations
import random
from collections import Counter
from typing import Any, Callable, Dict, List, Optional, Tuple

from .robot import Robot, RobotState
from .warehouse import Warehouse, UNREACHABLE, FREE, SPAWN, DROP, PICKUP
from .cbaa import CBAAState, TaskInfo
from .decision.robot_brain import (RobotBrain, Decision, BATTERY_CRITICAL, BATTERY_WARN)
from .algorithms.space_time_astar import LocalRoutePlanner
from .algorithms.deadlock import DeadlockProbes, RETREAT_HOLD, BOOST

Cell = Tuple[int, int]

PATIENCE        = 4     # ticks to wait for an occupant to respond to a yield request
BAN_TICKS       = 10    # then stop trying that cell for a while
REQ_TTL         = 3     # a yield request is honoured this long after it was last heard
HINDER_TICKS    = 6     # blocked this long -> re-plan the route against what I can hear
HINDER_COOLDOWN = 20
PRECLAIM_AT     = 0.3   # start claiming the NEXT cell once this far into the current move
UPLINK_PICKUP, UPLINK_COMPLETE = "pickup", "complete"


class NeighborInfo:
    """What I know about another robot: its last beacon, nothing more."""
    __slots__ = ("id", "x", "y", "grid_cell", "target_cell", "preferred_speed", "state",
                 "battery", "goal", "prio", "path", "claim", "claim_since", "claim_prio", "charge_cell",
                 "task_id", "dead", "rx_tick", "active")

    def __init__(self, m: Dict[str, Any], rx_tick: int):
        self.id = m["src"]
        self.x, self.y = m["x"], m["y"]
        self.grid_cell = tuple(m["cell"])
        self.target_cell = tuple(m["target"]) if m["target"] is not None else None
        self.preferred_speed = m["speed"]
        self.state = m["state"]
        self.battery = m["battery"]
        self.goal = tuple(m["goal"]) if m["goal"] is not None else None
        self.prio = m["prio"]
        self.path = [tuple(c) for c in m["path"]]
        self.claim = tuple(m["claim"]) if m["claim"] is not None else None
        self.claim_since = m["claim_since"]
        self.claim_prio = m["claim_prio"]
        self.charge_cell = tuple(m["charge"]) if m["charge"] is not None else None
        self.task_id = m["task"]
        self.dead = m["dead"]
        self.rx_tick = rx_tick
        self.active = True

    @property
    def at_rest(self) -> bool:
        return self.target_cell is None


class Agent:
    def __init__(self, robot: Robot, warehouse: Warehouse,
                 uplink: Callable[[str, int, int], bool],
                 send: Callable[..., None], delay: int = 1, loss_margin: int = 0):
        self.robot   = robot
        self.wh      = warehouse
        self._uplink = uplink
        self._send   = send
        self.D       = max(1, delay) + loss_margin       # claim must be this old before moving
        self.nbr_ttl = 3 + self.D

        self.brain   = RobotBrain(robot)
        self.cb      = CBAAState(robot.id)
        self.planner = LocalRoutePlanner(warehouse)
        self.probes  = DeadlockProbes(robot.id)
        self.rng     = random.Random(robot.id * 7919 + 13)
        self.cooperative = True            # False = "baseline": no yielding, no probes, no route sharing

        self.nbr: Dict[int, NeighborInfo] = {}
        self.dead_cells: Dict[Cell, int] = {}
        self.requests: Dict[int, Tuple] = {}          # asker -> (prio, asker_cell, dir, rx_tick)
        self.claim: Optional[Cell] = None
        self.claim_since = 0
        self.claim_prio = 0.0      # priority FROZEN when the claim was made
        self.push_since: Dict[Cell, int] = {}
        self.ban_until:  Dict[Cell, int] = {}
        self.retreat: Optional[Tuple[Cell, int]] = None
        self._charger_excl: Dict[Cell, int] = {}
        self._bids_in: List[Dict] = []
        self.prio = 0.0
        self.eff_prio = 0.0
        self.waiting_on: Optional[int] = None
        self.tick = 0
        self.stats: Counter = Counter()
        self._events: List[str] = []
        self.in_conflict = False

    # ═════════════════════════════════════════════════════════════════════════
    #  Public
    # ═════════════════════════════════════════════════════════════════════════

    def set_warehouse(self, wh: Warehouse) -> None:
        self.wh = wh
        self.planner = LocalRoutePlanner(wh)

    def drain_events(self) -> List[str]:
        ev, self._events = self._events, []
        return ev

    def step(self, tick: int, inbox: List[Dict], open_tasks: List[TaskInfo]) -> None:
        r = self.robot
        self.tick = tick
        self._ingest(inbox)

        if r.battery <= 0:                       # dead: only a passive transponder keeps talking
            self._send(self._beacon(dead=True))
            return

        # bookkeeping on own state
        if r.task:
            r.task_age += 1
        if r.priority_boost > 0:
            r.priority_boost -= 1
        self.prio = r.priority_score()
        self._expire()
        self.eff_prio = max([self.prio] + [rq[0] for rq in self.requests.values()])

        nbrs = list(self.nbr.values())
        self._consensus(open_tasks)
        decision = self.brain.think(nbrs, open_tasks, self.wh, tick)
        self._apply_decision(decision)
        if r.at_rest:
            self._advance_state()
        self._select_task(open_tasks, nbrs)
        r.goal = self._goal()
        self._plan_if_needed(nbrs)
        self._movement(tick, nbrs)
        self._deadlock_probes()
        self._send(self._beacon())

    def snapshot(self) -> Dict[str, Any]:
        """Read-only view for the dashboard (observer only; never fed back to algorithms)."""
        return {"claim": self.claim, "waiting_on": self.waiting_on, "neighbors": sorted(self.nbr),
                "requests": sorted(self.requests), "retreat": self.retreat,
                "held_task": self.cb.mine, "prio": round(self.eff_prio, 1)}

    # ═════════════════════════════════════════════════════════════════════════
    #  Radio in / out
    # ═════════════════════════════════════════════════════════════════════════

    def _ingest(self, inbox: List[Dict]) -> None:
        r = self.robot
        for m in inbox:
            t = m["t"]
            if t == "B":
                n = NeighborInfo(m, self.tick)
                self.nbr[n.id] = n
                if n.dead:
                    for c in (n.grid_cell, n.target_cell):
                        if c is not None:
                            self.dead_cells[c] = self.tick
                else:
                    self._bids_in.append(m["bids"])
            elif t == "YIELD":
                self.requests[m["src"]] = (m["prio"], tuple(m["cell"]), tuple(m["dir"]), self.tick)
            elif t == "PROBE" and r.battery > 0:
                fwd, brk = self.probes.on_probe(m, self.eff_prio, self.tick)
                if fwd:
                    self._send(fwd, to=fwd["to"])
                if brk:
                    self._send(brk)
                    self._on_break(brk)
            elif t == "BREAK":
                self._on_break(m)

    def _on_break(self, m: Dict) -> None:
        r = self.robot
        if self.id not in m["cycle"]:
            return
        self.stats["deadlocks"] += 1
        if m["victim"] == self.id:
            cycle_cells = {self.nbr[i].grid_cell for i in m["cycle"] if i in self.nbr}
            cycle_cells.add(r.grid_cell)
            for c in self.wh.get_neighbors(*r.grid_cell):
                if c not in cycle_cells and self._cell_status(c, self.tick)[0] == "free":
                    self.retreat = (c, self.tick + RETREAT_HOLD + 6)
                    self._events.append(f"Robot {self.id} retreats to {c} (deadlock {m['cycle']})")
                    break
        else:
            r.priority_boost = max(r.priority_boost, BOOST)

    @property
    def id(self) -> int:
        return self.robot.id

    def _beacon(self, dead: bool = False) -> Dict[str, Any]:
        r = self.robot
        return {"t": "B", "src": r.id, "tick": self.tick, "x": r.x, "y": r.y,
                "cell": r.grid_cell, "target": r.target_cell, "speed": r.preferred_speed,
                "state": r.state, "battery": r.battery, "goal": r.goal,
                "prio": self.eff_prio, "path": list(r.path[:8]),
                "claim": self.claim, "claim_since": self.claim_since, "claim_prio": self.claim_prio,
                "charge": r.charge_cell, "task": r.task.id if r.task else None,
                "bids": self.cb.export() if not dead else {}, "dead": dead}

    def _expire(self) -> None:
        t = self.tick
        for rid in [i for i, n in self.nbr.items() if t - n.rx_tick > self.nbr_ttl]:
            del self.nbr[rid]
        for s in [s for s, rq in self.requests.items() if t - rq[3] > REQ_TTL]:
            del self.requests[s]
        for c in [c for c, u in self.ban_until.items() if u <= t]:
            del self.ban_until[c]
        for c in [c for c, s in self.push_since.items() if t - s > 30]:
            del self.push_since[c]
        for c in [c for c, s in self.dead_cells.items() if t - s > 600]:
            del self.dead_cells[c]

    # ═════════════════════════════════════════════════════════════════════════
    #  Tasks: consensus, selection, state machine
    # ═════════════════════════════════════════════════════════════════════════

    def _consensus(self, open_tasks: List[TaskInfo]) -> None:
        r = self.robot
        open_ids = {t.id for t in open_tasks}
        self.cb.sync(open_ids, self.tick)
        for bids in self._bids_in:
            self.cb.merge(bids, open_ids, self.tick)
        self._bids_in = []
        self.cb.refresh(self.tick)
        if self.cb.lost_flag:
            self.cb.lost_flag = False
            if r.task and r.state == RobotState.MOVING_TO_PICKUP:
                self._events.append(f"Robot {r.id} outbid/overtaken on task {r.task.id}")
                self._drop_task("outbid", ban=40, release=False)
            elif r.task is None:
                self.cb.mine = None

    def _select_task(self, open_tasks: List[TaskInfo], nbrs: List[NeighborInfo]) -> None:
        r = self.robot
        if not (r.state == RobotState.IDLE and r.at_rest and r.task is None
                and self.cb.mine is None and r.battery >= BATTERY_WARN):
            return
        bids: Dict[int, float] = {}

        def utility(t: TaskInfo) -> float:
            u = self.brain.compute_task_utility(t, nbrs, self.wh)
            bids[t.id] = round(u, 3)
            return u

        chosen = self.cb.select(open_tasks, utility, self.tick)
        r.viz["bid_values"] = bids
        if chosen is not None:
            r.task = chosen
            r.state = RobotState.MOVING_TO_PICKUP
            r.task_age = 0; r.path = []; r.blocked_ticks = 0
            r.need_replan = False; r.last_plan_tick = -999
            self.claim = None

    def _drop_task(self, reason: str, ban: int, release: bool) -> None:
        r = self.robot
        if r.task:
            self.brain.ban(r.task.id, ban)
            if release:
                self.cb.release(self.tick)
            else:
                self.cb.mine = None
            # losing a consensus race is normal CBAA behaviour, not a failure
            self.stats["outbid" if reason == "outbid" else "drops"] += 1
        self.cb.lost_flag = False
        r.task = None; r.path = []; r.state = RobotState.IDLE
        r.task_age = 0; self.claim = None

    def _apply_decision(self, d) -> None:
        r, a = self.robot, d.action
        if a == Decision.DROP_TASK and r.task:
            self._events.append(f"Robot {r.id} drops task {r.task.id}: {d.reason}")
            self._drop_task(d.reason, ban=120, release=True)
        elif a == Decision.EMERGENCY_STOP:
            if r.task:
                self._events.append(f"Robot {r.id} aborts task {r.task.id}: {d.reason}")
                self._drop_task(d.reason, ban=0, release=True)
            self._start_charging()
        elif a == Decision.GO_CHARGE:
            if self._start_charging():
                self._events.append(f"Robot {r.id} heading to charger ({r.battery:.0f}%)")
        elif a == Decision.REPLAN:
            r.need_replan = True

    def _advance_state(self) -> None:
        r = self.robot
        for _ in range(4):
            if r.state == RobotState.MOVING_TO_PICKUP:
                if r.task is None:
                    r.state = RobotState.IDLE
                elif r.grid_cell == r.task.pickup:
                    r.state = RobotState.PICKING_UP
                    continue
                return
            if r.state == RobotState.PICKING_UP:
                if not self._uplink(UPLINK_PICKUP, r.task.id, r.id):
                    self._events.append(f"Robot {r.id}: item for task {r.task.id} already taken")
                    self._drop_task("item gone", ban=300, release=False)
                    return
                self.cb.y.pop(r.task.id, None)
                self.cb.mine = None
                r.state = RobotState.MOVING_TO_DROPOFF
                r.path = []
                self._events.append(f"Robot {r.id} picked up task {r.task.id}")
                continue
            if r.state == RobotState.MOVING_TO_DROPOFF:
                if r.task is None:
                    r.state = RobotState.IDLE
                elif r.grid_cell == r.task.dropoff:
                    r.state = RobotState.DROPPING_OFF
                    continue
                return
            if r.state == RobotState.DROPPING_OFF:
                if r.task:
                    self._uplink(UPLINK_COMPLETE, r.task.id, r.id)
                    r.tasks_completed += 1
                    self._events.append(f"Robot {r.id} ✓ task {r.task.id}")
                r.task = None; r.path = []; r.state = RobotState.IDLE
                r.task_age = 0; r.blocked_ticks = 0
                return
            if r.state == RobotState.LOW_BATTERY:
                self._charger_check()
                if r.charge_cell is not None and r.grid_cell == r.charge_cell \
                        and r.battery >= min(95.0, r.battery_capacity):
                    r.charge_cell = None
                    r.state = RobotState.IDLE
                    self._events.append(f"Robot {r.id} fully charged")
                return
            return

    def _goal(self) -> Optional[Cell]:
        r = self.robot
        if r.state == RobotState.MOVING_TO_PICKUP and r.task:
            return r.task.pickup
        if r.state == RobotState.MOVING_TO_DROPOFF and r.task:
            return r.task.dropoff
        if r.state == RobotState.LOW_BATTERY:
            return r.charge_cell
        return None

    # ── charging (chargers = spawn cells; conflicts settled by the robots themselves) ──

    def _start_charging(self) -> bool:
        r, t = self.robot, self.tick
        best: Optional[Tuple[int, Cell]] = None
        for sp in self.wh.spawn_points:
            if self._charger_excl.get(sp, 0) > t:
                continue
            if any(n.charge_cell == sp and n.state == RobotState.LOW_BATTERY and
                   (n.grid_cell == sp or (n.battery, n.id) < (r.battery, r.id))
                   for n in self.nbr.values()):
                continue
            d = self.wh.dist(r.anchor, sp)
            if d < UNREACHABLE and (best is None or d < best[0]):
                best = (d, sp)
        if best is None:
            return False
        r.charge_cell = best[1]
        r.state = RobotState.LOW_BATTERY
        r.path = []; r.need_replan = False; self.claim = None
        return True

    def _charger_check(self) -> None:
        r = self.robot
        if r.charge_cell is None:
            if not self._start_charging():
                r.state = RobotState.IDLE
            return
        if r.grid_cell == r.charge_cell:
            return
        for n in self.nbr.values():
            if n.charge_cell == r.charge_cell and n.state == RobotState.LOW_BATTERY and \
                    (n.grid_cell == r.charge_cell or (n.battery, n.id) < (r.battery, r.id)):
                self._charger_excl[r.charge_cell] = self.tick + 60
                self._start_charging()
                return

    # ═════════════════════════════════════════════════════════════════════════
    #  Route planning (own route, from own knowledge)
    # ═════════════════════════════════════════════════════════════════════════

    def _plan_if_needed(self, nbrs: List[NeighborInfo]) -> None:
        r, wh, t = self.robot, self.wh, self.tick
        if r.goal is None:
            return
        anchor = r.anchor
        if anchor == r.goal:
            r.path = []
            return
        stale = bool(r.path) and (
            not wh.is_passable(*r.path[0])
            or abs(r.path[0][0] - anchor[0]) + abs(r.path[0][1] - anchor[1]) != 1)
        hindered = (self.cooperative and r.blocked_ticks >= HINDER_TICKS
                    and t - r.last_plan_tick >= HINDER_COOLDOWN)
        if r.path and not (stale or r.need_replan or hindered):
            return
        r.last_plan_tick = t
        r.need_replan = False
        if self.cooperative:
            p = self.planner.plan(r.id, anchor, 1 if r.target_cell is not None else 0,
                                  r.goal, nbrs, self.dead_cells)
        else:
            p = wh.shortest_path(anchor, r.goal)
        if p is None:
            self._on_unreachable()
        else:
            r.path = p

    def _on_unreachable(self) -> None:
        r = self.robot
        if r.task:
            self._events.append(f"Robot {r.id}: task {r.task.id} unreachable — released")
            self._drop_task("unreachable", ban=60, release=True)
        elif r.state == RobotState.LOW_BATTERY and r.charge_cell is not None:
            self._charger_excl[r.charge_cell] = self.tick + 60
            r.charge_cell = None
            if not self._start_charging():
                r.state = RobotState.IDLE
        r.path = []

    # ═════════════════════════════════════════════════════════════════════════
    #  Movement negotiation
    # ═════════════════════════════════════════════════════════════════════════

    def _set_claim(self, v: Cell, tick: int) -> None:
        if self.claim != v:
            self.claim = v
            self.claim_since = tick
            # Frozen on purpose: every observer must compute the SAME order for this claim,
            # so the tie-break value cannot change while the claim is on the air.
            self.claim_prio = self.eff_prio

    def _cell_status(self, v: Cell, tick: int) -> Tuple[str, Optional[int]]:
        """
        'wall' | 'rest' (a robot sits on v) | 'busy' (someone on/entering v, or an earlier
        claim) | 'free'.  Everything is judged from beacons — i.e. `delay` ticks old.
        """
        r, cur = self.robot, self.robot.grid_cell
        if not self.wh.is_passable(*v) or v in self.dead_cells:
            return "wall", None
        for n in self.nbr.values():
            if n.dead:
                continue
            if n.grid_cell == v or n.target_cell == v:
                if n.target_cell is None:
                    return "rest", n.id
                if n.grid_cell == v and n.target_cell != cur:
                    w = n.target_cell
                    same_dir = (w[0] - v[0], w[1] - v[1]) == (v[0] - cur[0], v[1] - cur[1])
                    if same_dir and n.preferred_speed + 1e-9 >= r.preferred_speed:
                        continue                       # convoy: follow it out of v
                return "busy", n.id
        if self.claim == v:
            mine = (self.claim_since, -self.claim_prio, r.id)
        else:
            mine = (tick, -self.eff_prio, r.id)          # the claim I would make now
        for n in self.nbr.values():
            if n.claim == v and not n.dead and (n.claim_since, -n.claim_prio, n.id) < mine:
                return "busy", n.id
        return "free", None

    def _traffic(self, nbrs: List[NeighborInfo]) -> Counter:
        c: Counter = Counter()
        for n in nbrs:
            for cell in n.path[:3]:
                c[cell] += 1
        return c

    def _candidates(self, tick: int, nbrs: List[NeighborInfo]) -> List[Cell]:
        r, wh = self.robot, self.wh
        cur = r.grid_cell
        adj = wh.get_neighbors(*cur)
        traffic = self._traffic(nbrs)
        rnd = self.rng.random

        if self.retreat:
            cell, until = self.retreat
            if tick >= until:
                self.retreat = None
            else:
                return [cell] if cur != cell else []

        asked_by = [(rq[0], s) for s, rq in self.requests.items()]
        must_vacate = self.cooperative and any((p, -s) > (self.prio, -r.id) for p, s in asked_by)
        goal = r.goal

        if must_vacate:
            req_cells = {rq[1] for rq in self.requests.values()}
            straight = {(cur[0] + rq[2][0], cur[1] + rq[2][1]) for rq in self.requests.values()}
            pool = [c for c in adj if c not in req_cells]
            if goal is not None and goal != cur:
                dm = wh.dist_map(goal)
                pool.sort(key=lambda c: (dm[c[1]][c[0]], c not in straight, traffic[c], rnd()))
            else:
                pool.sort(key=lambda c: (c not in straight, traffic[c], rnd()))
            r.desired_cell = None
            return pool

        if goal is not None and goal != cur:
            dm = wh.dist_map(goal)
            dcur = dm[cur[1]][cur[0]]
            ranked = sorted([c for c in adj if dm[c[1]][c[0]] <= dcur],
                            key=lambda c: (dm[c[1]][c[0]], traffic[c], rnd()))
            if r.path and r.path[0] in adj:
                if r.path[0] in ranked:
                    ranked.remove(r.path[0])
                ranked.insert(0, r.path[0])
            r.desired_cell = ranked[0] if ranked else None
            return ranked

        # idle / at goal: stay put, except step off DROP / PICKUP cells onto ordinary floor
        r.desired_cell = None
        if self.cooperative and wh.cell(*cur) in (DROP, PICKUP):
            return sorted([c for c in adj if wh.cell(*c) in (FREE, SPAWN)],
                          key=lambda c: (traffic[c], rnd()))
        return []

    def _preclaim(self, tick: int) -> None:
        r = self.robot
        if r.path and r._lerp_t >= PRECLAIM_AT:
            self._set_claim(r.path[0], tick)

    def _movement(self, tick: int, nbrs: List[NeighborInfo]) -> None:
        r = self.robot
        self.waiting_on = None
        self.in_conflict = False
        if not r.at_rest:
            self._preclaim(tick)
            r.blocked_ticks = 0
            return

        cur = r.grid_cell
        blocker: Optional[int] = None
        moved_to: Optional[Cell] = None
        holding = False

        for v in self._candidates(tick, nbrs):
            st, who = self._cell_status(v, tick)
            if st == "wall":
                continue
            if st == "busy":
                blocker = blocker if blocker is not None else who
                self.in_conflict = True
                continue
            if st == "rest":
                if not self.cooperative or self.ban_until.get(v, -1) > tick:
                    blocker = blocker if blocker is not None else who
                    continue
                n = self.nbr.get(who)
                self.push_since.setdefault(v, tick)
                if n is not None and (n.claim is not None or not n.at_rest):
                    self.push_since[v] = tick            # it is responding: keep waiting
                if tick - self.push_since[v] > PATIENCE:
                    self.ban_until[v] = tick + BAN_TICKS
                    self.push_since.pop(v, None)
                    blocker = blocker if blocker is not None else who
                    continue
                self._set_claim(v, tick)
                self._send({"t": "YIELD", "src": r.id, "to": who, "prio": self.eff_prio,
                            "cell": cur, "dir": (v[0] - cur[0], v[1] - cur[1])}, to=who)
                self.stats["yield_req"] += 1
                self.waiting_on = who
                self.in_conflict = True
                holding = True
                break
            # free
            self._set_claim(v, tick)
            holding = True
            if tick - self.claim_since >= self.D:
                if r.begin_move(*v):
                    moved_to = v
            break

        if moved_to is not None:
            if r.path and r.path[0] == moved_to:
                r.path.pop(0)
            else:
                r.path = []
            r.blocked_ticks = 0
            self.claim = None
            self.push_since.pop(moved_to, None)
            return

        if not holding:
            self.claim = None
            self.waiting_on = blocker
        if r.goal is not None and r.goal != cur:
            r.blocked_ticks += 1
            r.ticks_waiting += 1
        else:
            r.blocked_ticks = 0

    def _deadlock_probes(self) -> None:
        if not self.cooperative:
            return
        self.probes.set_waiting(self.waiting_on, self.tick)
        p = self.probes.initiate(self.tick, self.eff_prio)
        if p:
            self._send(p, to=p["to"])
            self.stats["probes"] += 1
