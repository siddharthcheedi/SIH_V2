"""
comms.py — the radio. The ONLY channel through which robots learn about each other.

Model (all parameters are knobs, see CommsConfig):
  * range  : a message reaches a robot only if it is within `comm_range` cells of the
             sender at the moment of sending.
  * delay  : delivery happens `delay` ticks after sending (1 tick = 0.1 s).
  * loss   : each (message, receiver) pair is independently dropped with this probability.

The bus belongs to the physical world (like air), not to any robot. Algorithms never get
a reference to it: an Agent only receives its own inbox and a `send()` function.
"""
from __future__ import annotations
import math
import random
from collections import defaultdict
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

Message = Dict[str, Any]


@dataclass
class CommsConfig:
    comm_range: float = 6.0     # cells
    delay: int        = 1       # ticks between send and receive (>= 1)
    loss: float       = 0.0     # per-receiver drop probability
    seed: int         = 1234


class MessageBus:
    def __init__(self, cfg: Optional[CommsConfig] = None):
        self.cfg = cfg or CommsConfig()
        self._rng = random.Random(self.cfg.seed)
        self._pos: Dict[int, Tuple[float, float]] = {}
        self._queue: Dict[int, List[Tuple[int, Message]]] = defaultdict(list)  # deliver_tick -> [(rid, msg)]
        self._tick = 0
        # statistics
        self.sent = 0
        self.delivered = 0
        self.dropped = 0
        self.by_kind: Dict[str, int] = defaultdict(int)

    # ── physical-layer bookkeeping (engine only) ────────────────────────────
    def begin_tick(self, tick: int, positions: Dict[int, Tuple[float, float]]) -> None:
        self._tick = tick
        self._pos = positions

    def deliver(self, tick: int) -> Dict[int, List[Message]]:
        """Messages arriving at `tick`, grouped by receiver id."""
        inboxes: Dict[int, List[Message]] = defaultdict(list)
        for rid, msg in self._queue.pop(tick, []):
            inboxes[rid].append(msg)
            self.delivered += 1
        return inboxes

    # ── what a robot's radio does ───────────────────────────────────────────
    def send(self, src: int, msg: Message, to: Optional[int] = None) -> None:
        """Broadcast (to=None) or unicast. Reaches only robots currently in range."""
        sp = self._pos.get(src)
        if sp is None:
            return
        self.sent += 1
        self.by_kind[msg.get("t", "?")] += 1
        due = self._tick + max(1, self.cfg.delay)
        for rid, p in self._pos.items():
            if rid == src or (to is not None and rid != to):
                continue
            if math.hypot(p[0] - sp[0], p[1] - sp[1]) > self.cfg.comm_range:
                continue
            if self.cfg.loss > 0 and self._rng.random() < self.cfg.loss:
                self.dropped += 1
                continue
            self._queue[due].append((rid, msg))

    def sender_for(self, src: int):
        """Bound send function handed to exactly one agent."""
        return lambda msg, to=None: self.send(src, msg, to)

    def stats(self) -> Dict[str, Any]:
        return {"sent": self.sent, "delivered": self.delivered, "dropped": self.dropped,
                "by_kind": dict(self.by_kind), "range": self.cfg.comm_range,
                "delay": self.cfg.delay, "loss": self.cfg.loss}
