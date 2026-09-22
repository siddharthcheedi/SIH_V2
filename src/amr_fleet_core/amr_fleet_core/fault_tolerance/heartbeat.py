"""
heartbeat.py — Pure-Python heartbeat monitoring logic.

Each robot broadcasts a heartbeat every 1 second. If a robot misses
3 consecutive heartbeats (3-second timeout), it is declared DEAD and:
  1. Its reservations are released from the local table.
  2. Its in-progress task is re-announced for bidding via CNP.

Zero ROS2 dependencies — pure Python, tested with plain pytest.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Dict, List, Optional


class PeerState(Enum):
    """Observed state of a peer robot."""
    ALIVE = auto()
    DEAD = auto()


@dataclass
class PeerRecord:
    """Tracking record for a single peer robot."""
    robot_id: str
    last_heartbeat_time: float
    state: PeerState = PeerState.ALIVE
    missed_count: int = 0


@dataclass
class HeartbeatEvent:
    """A logged heartbeat timeout event."""
    timestamp: float
    robot_id: str
    reason: str


class HeartbeatMonitor:
    """
    Monitors heartbeat broadcasts from all peer robots.

    Operates on simulation time (not wall time) so behavior is
    deterministic and reproducible across trials.
    """

    def __init__(
        self,
        heartbeat_interval: float = 1.0,
        timeout_factor: int = 3,
        on_peer_dead: Optional[Callable[[str, float], None]] = None,
    ) -> None:
        """
        Parameters
        ----------
        heartbeat_interval : float
            Expected interval between heartbeats (seconds).
        timeout_factor : int
            Number of missed heartbeats before declaring DEAD.
        on_peer_dead : callable, optional
            Callback invoked with (robot_id, timestamp) when a peer is
            declared dead.
        """
        self.heartbeat_interval = heartbeat_interval
        self.timeout = heartbeat_interval * timeout_factor
        self.on_peer_dead = on_peer_dead
        self._peers: Dict[str, PeerRecord] = {}
        self._events: List[HeartbeatEvent] = []

    def on_heartbeat(self, robot_id: str, timestamp: float) -> None:
        """
        Record a received heartbeat from a peer.

        If the peer was previously DEAD and starts heartbeating again,
        it transitions back to ALIVE (allows for recovery).
        """
        if robot_id not in self._peers:
            self._peers[robot_id] = PeerRecord(
                robot_id=robot_id,
                last_heartbeat_time=timestamp,
            )
        else:
            peer = self._peers[robot_id]
            peer.last_heartbeat_time = timestamp
            peer.missed_count = 0
            if peer.state == PeerState.DEAD:
                peer.state = PeerState.ALIVE  # recovery

    def check_timeouts(self, current_time: float) -> List[str]:
        """
        Check all peers for heartbeat timeout.

        Returns a list of robot_ids that were newly declared DEAD in
        this check.
        """
        newly_dead: List[str] = []

        for robot_id, peer in self._peers.items():
            if peer.state == PeerState.DEAD:
                continue

            elapsed = current_time - peer.last_heartbeat_time
            if elapsed > self.timeout:
                peer.state = PeerState.DEAD
                peer.missed_count = int(elapsed / self.heartbeat_interval)
                newly_dead.append(robot_id)

                event = HeartbeatEvent(
                    timestamp=current_time,
                    robot_id=robot_id,
                    reason=(
                        f"Heartbeat timeout: {robot_id} last seen at "
                        f"t={peer.last_heartbeat_time:.1f}, "
                        f"now t={current_time:.1f} "
                        f"({elapsed:.1f}s > {self.timeout:.1f}s threshold)"
                    ),
                )
                self._events.append(event)

                if self.on_peer_dead:
                    self.on_peer_dead(robot_id, current_time)

        return newly_dead

    def is_alive(self, robot_id: str) -> bool:
        """Check if a peer is currently considered ALIVE."""
        peer = self._peers.get(robot_id)
        return peer is not None and peer.state == PeerState.ALIVE

    def get_alive_peers(self) -> List[str]:
        """Return list of all ALIVE peer robot_ids."""
        return [
            rid for rid, peer in self._peers.items()
            if peer.state == PeerState.ALIVE
        ]

    def get_dead_peers(self) -> List[str]:
        """Return list of all DEAD peer robot_ids."""
        return [
            rid for rid, peer in self._peers.items()
            if peer.state == PeerState.DEAD
        ]

    def get_events(self) -> List[HeartbeatEvent]:
        """Return all logged heartbeat timeout events."""
        return list(self._events)

    def peer_count(self) -> int:
        return len(self._peers)
