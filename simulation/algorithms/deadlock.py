"""
deadlock.py — DISTRIBUTED deadlock detection and resolution (Chandy–Misra–Haas edge chasing).

There is no global wait-for graph. Each robot knows only ONE thing: which robot it is
currently waiting for (`waiting_on`, the occupant / earlier claimant of the cell it wants).

Detection
  A robot that has been blocked for PROBE_AFTER ticks sends  PROBE(init=me, path=[me])  to the
  robot it waits on. A robot that receives a probe and is itself blocked forwards it to the
  robot it waits on, appending itself to the path. A robot that is NOT blocked drops it
  (the chain ends: no deadlock). If the probe returns to its initiator, the path IS the
  cycle: a deadlock, discovered purely by message passing.

Resolution
  The initiator broadcasts BREAK(cycle, victim). The victim is the lowest-priority member.
  The victim steps to a free neighbouring cell that is not a cycle member's cell and holds
  there for RETREAT_HOLD ticks; the others get a temporary priority boost so they use the
  space the victim just freed.
"""
from __future__ import annotations
from typing import Dict, List, Optional, Tuple

PROBE_AFTER   = 6      # blocked this long before probing
PROBE_PERIOD  = 4      # probe at most this often
MAX_HOPS      = 12
BREAK_COOLDOWN = 25
RETREAT_HOLD  = 10
BOOST         = 40


def _key(prio: float, rid: int) -> Tuple[float, int]:
    return (prio, -rid)


class DeadlockProbes:
    """Per-robot state for edge chasing. Lives inside one Agent."""

    def __init__(self, robot_id: int):
        self.id = robot_id
        self.waiting_on: Optional[int] = None
        self.blocked_since: Optional[int] = None
        self._last_probe = -999
        self._cooldown_until = 0
        self.detected: List[Dict] = []       # for the dashboard: cycles this robot found

    def set_waiting(self, other: Optional[int], tick: int) -> None:
        if other is None:
            self.waiting_on = None
            self.blocked_since = None
        else:
            if self.waiting_on is None:
                self.blocked_since = tick
            self.waiting_on = other

    def initiate(self, tick: int, prio: float) -> Optional[Dict]:
        if (self.waiting_on is None or self.blocked_since is None
                or tick < self._cooldown_until
                or tick - self.blocked_since < PROBE_AFTER
                or tick - self._last_probe < PROBE_PERIOD):
            return None
        self._last_probe = tick
        return {"t": "PROBE", "to": self.waiting_on, "init": self.id,
                "path": [(self.id, prio)]}

    def on_probe(self, msg: Dict, my_prio: float, tick: int
                 ) -> Tuple[Optional[Dict], Optional[Dict]]:
        """Returns (probe_to_forward, break_to_broadcast)."""
        if msg.get("to") != self.id or self.waiting_on is None:
            return None, None                      # not blocked -> chain ends, no deadlock
        path = [tuple(p) for p in msg["path"]]
        if msg["init"] == self.id:                 # came back around: DEADLOCK
            if tick < self._cooldown_until:
                return None, None
            self._cooldown_until = tick + BREAK_COOLDOWN
            victim = min(path, key=lambda p: _key(p[1], p[0]))[0]
            cycle = [p[0] for p in path]
            self.detected.append({"tick": tick, "cycle": cycle, "victim": victim})
            self.detected = self.detected[-10:]
            return None, {"t": "BREAK", "cycle": cycle, "victim": victim}
        if len(path) >= MAX_HOPS or any(p[0] == self.id for p in path):
            return None, None                      # loop that doesn't include the initiator
        return ({"t": "PROBE", "to": self.waiting_on, "init": msg["init"],
                 "path": path + [(self.id, my_prio)]}, None)
