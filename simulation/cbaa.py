"""
cbaa.py — Consensus-Based Auction Algorithm (Choi, Brunet & How, 2009), message-passing form.

There is NO auctioneer. Every robot owns a CBAAState with
    y[task]  = (bid, winner, stamp)   best bid it has HEARD of for that task
    mine     = task id it currently holds (at most one)
Robots exchange their y tables with whoever is in radio range (inside their beacons) and
merge them with a fixed rule, so the tables converge on connected parts of the network:

    same winner      -> keep the newer stamp         (lets a winner refresh / release)
    different winner -> higher bid wins, ties to the lower robot id

A robot that is outbid for the task it holds releases it. A release is just an entry
(bid=0, winner=me, newer stamp), so it spreads like any other update. Entries whose winner
has been silent for ENTRY_TTL ticks expire — that is how a dead robot's task becomes
available again, with nobody having to detect the failure centrally.

Task / TaskInfo
    Task      — the order as held by the (external) order system / task board.
    TaskInfo  — the read-only copy robots receive when an order is announced.
"""
from __future__ import annotations
from typing import Callable, Dict, List, Optional, Tuple

ENTRY_TTL = 60            # ticks a winner's entry survives without being refreshed by it
BID_EPS   = 1e-9

Entry = Tuple[float, int, int]      # (bid, winner_id, stamp_tick)


class Task:
    """An order, as held by the order system (task board). Robots never see this object."""
    _next_id = 0

    def __init__(self, pickup: Tuple[int,int], dropoff: Tuple[int,int], priority: int = 1):
        self.id          = Task._next_id
        Task._next_id   += 1
        self.pickup      = pickup
        self.dropoff     = dropoff
        self.priority    = priority
        self.assigned_to: Optional[int] = None    # observer-side display only
        self.created_at:   float = 0.0
        self.started_at:   float = 0.0
        self.completed_at: float = 0.0
        self.picked = False                        # item physically taken from the shelf

    def to_dict(self) -> Dict:
        return {"id": self.id, "pickup": list(self.pickup), "dropoff": list(self.dropoff),
                "priority": self.priority, "assigned_to": self.assigned_to}

    def info(self) -> "TaskInfo":
        return TaskInfo(self.id, self.pickup, self.dropoff, self.priority)

    @classmethod
    def reset_ids(cls):
        cls._next_id = 0


class TaskInfo:
    """What a robot knows about an order."""
    __slots__ = ("id", "pickup", "dropoff", "priority")

    def __init__(self, tid, pickup, dropoff, priority):
        self.id, self.pickup, self.dropoff, self.priority = tid, pickup, dropoff, priority


def beats(a: Entry, b: Entry) -> bool:
    """Does entry `a` beat entry `b` for the same task (different winners)?"""
    if a[0] > b[0] + BID_EPS:
        return True
    if abs(a[0] - b[0]) <= BID_EPS:
        return a[1] < b[1]
    return False


class CBAAState:
    def __init__(self, robot_id: int):
        self.id   = robot_id
        self.y: Dict[int, Entry] = {}
        self.mine: Optional[int] = None
        self.lost_flag = False

    # ── consensus phase ─────────────────────────────────────────────────────
    def merge(self, entries: Dict[int, Entry], open_ids, now: int) -> None:
        for tid, e in entries.items():
            if tid not in open_ids:
                continue
            e = (float(e[0]), int(e[1]), int(e[2]))
            if now - e[2] > ENTRY_TTL:
                continue
            cur = self.y.get(tid)
            if cur is None:
                self.y[tid] = e
            elif e[1] == cur[1]:
                if e[2] > cur[2]:
                    self.y[tid] = e
            elif beats(e, cur):
                self.y[tid] = e
        self._after_merge()

    def _after_merge(self) -> None:
        if self.mine is not None:
            cur = self.y.get(self.mine)
            if cur is None or cur[1] != self.id:
                self.lost_flag = True     # somebody else holds it now

    def sync(self, open_ids, now: int) -> None:
        """Forget tasks that are no longer open and entries whose winner went quiet."""
        for tid in list(self.y):
            if tid not in open_ids or now - self.y[tid][2] > ENTRY_TTL:
                del self.y[tid]
        if self.mine is not None and self.mine not in open_ids:
            self.lost_flag = True

    def refresh(self, now: int) -> None:
        """The holder re-stamps its entry so it never expires while it is alive."""
        if self.mine is not None and self.mine in self.y and self.y[self.mine][1] == self.id:
            b = self.y[self.mine][0]
            self.y[self.mine] = (b, self.id, now)

    # ── selection phase ─────────────────────────────────────────────────────
    def select(self, open_tasks: List[TaskInfo], utility: Callable[[TaskInfo], float],
               now: int) -> Optional[TaskInfo]:
        """Idle robot picks the open task with the best bid that BEATS the best known bid."""
        best, best_entry = None, None
        for t in open_tasks:
            c = utility(t)
            if c <= 0:
                continue
            mine = (c, self.id, now)
            cur = self.y.get(t.id)
            if cur is not None and cur[2] + ENTRY_TTL >= now and cur[1] != self.id and not beats(mine, cur):
                continue
            if best_entry is None or c > best_entry[0]:
                best, best_entry = t, mine
        if best is not None:
            self.y[best.id] = best_entry
            self.mine = best.id
            self.lost_flag = False
        return best

    def release(self, now: int) -> None:
        if self.mine is not None:
            self.y[self.mine] = (0.0, self.id, now)      # a stamped zero bid propagates
        self.mine = None
        self.lost_flag = False

    def export(self) -> Dict[int, Entry]:
        return dict(self.y)
