"""Tests for amr_fleet_core.fault_tolerance.heartbeat."""

from amr_fleet_core.fault_tolerance.heartbeat import (
    HeartbeatMonitor,
    PeerState,
)


class TestHeartbeatMonitor:
    """Heartbeat monitoring and timeout detection."""

    def test_new_peer_is_alive(self):
        mon = HeartbeatMonitor()
        mon.on_heartbeat('amr_1', timestamp=0.0)
        assert mon.is_alive('amr_1')

    def test_no_timeout_within_window(self):
        mon = HeartbeatMonitor(heartbeat_interval=1.0, timeout_factor=3)
        mon.on_heartbeat('amr_1', timestamp=0.0)
        newly_dead = mon.check_timeouts(current_time=2.5)  # 2.5s < 3s
        assert newly_dead == []
        assert mon.is_alive('amr_1')

    def test_timeout_after_threshold(self):
        mon = HeartbeatMonitor(heartbeat_interval=1.0, timeout_factor=3)
        mon.on_heartbeat('amr_1', timestamp=0.0)
        newly_dead = mon.check_timeouts(current_time=4.0)  # 4s > 3s
        assert 'amr_1' in newly_dead
        assert not mon.is_alive('amr_1')

    def test_heartbeat_resets_timer(self):
        mon = HeartbeatMonitor(heartbeat_interval=1.0, timeout_factor=3)
        mon.on_heartbeat('amr_1', timestamp=0.0)
        mon.on_heartbeat('amr_1', timestamp=2.5)  # reset at 2.5s
        newly_dead = mon.check_timeouts(current_time=4.0)  # 1.5s since last
        assert newly_dead == []
        assert mon.is_alive('amr_1')

    def test_dead_peer_not_double_reported(self):
        mon = HeartbeatMonitor(heartbeat_interval=1.0, timeout_factor=3)
        mon.on_heartbeat('amr_1', timestamp=0.0)
        mon.check_timeouts(current_time=4.0)  # first detection
        newly_dead = mon.check_timeouts(current_time=5.0)  # second check
        assert newly_dead == []  # already dead, not newly dead

    def test_recovery_after_death(self):
        mon = HeartbeatMonitor(heartbeat_interval=1.0, timeout_factor=3)
        mon.on_heartbeat('amr_1', timestamp=0.0)
        mon.check_timeouts(current_time=4.0)  # declared dead
        assert not mon.is_alive('amr_1')

        # Robot comes back
        mon.on_heartbeat('amr_1', timestamp=5.0)
        assert mon.is_alive('amr_1')

    def test_multiple_peers(self):
        mon = HeartbeatMonitor(heartbeat_interval=1.0, timeout_factor=3)
        mon.on_heartbeat('amr_1', timestamp=0.0)
        mon.on_heartbeat('amr_2', timestamp=0.0)
        mon.on_heartbeat('amr_3', timestamp=0.0)

        # Only amr_2 keeps heartbeating
        mon.on_heartbeat('amr_2', timestamp=2.0)

        newly_dead = mon.check_timeouts(current_time=4.0)
        assert 'amr_1' in newly_dead
        assert 'amr_3' in newly_dead
        assert 'amr_2' not in newly_dead

    def test_callback_invoked(self):
        dead_list = []
        def on_dead(robot_id, ts):
            dead_list.append((robot_id, ts))

        mon = HeartbeatMonitor(heartbeat_interval=1.0, timeout_factor=3,
                               on_peer_dead=on_dead)
        mon.on_heartbeat('amr_1', timestamp=0.0)
        mon.check_timeouts(current_time=4.0)
        assert len(dead_list) == 1
        assert dead_list[0][0] == 'amr_1'

    def test_events_logged(self):
        mon = HeartbeatMonitor(heartbeat_interval=1.0, timeout_factor=3)
        mon.on_heartbeat('amr_1', timestamp=0.0)
        mon.check_timeouts(current_time=4.0)
        events = mon.get_events()
        assert len(events) == 1
        assert events[0].robot_id == 'amr_1'
        assert 'timeout' in events[0].reason.lower()

    def test_get_alive_and_dead_peers(self):
        mon = HeartbeatMonitor(heartbeat_interval=1.0, timeout_factor=3)
        mon.on_heartbeat('amr_1', timestamp=0.0)
        mon.on_heartbeat('amr_2', timestamp=0.0)
        mon.on_heartbeat('amr_2', timestamp=2.0)
        mon.check_timeouts(current_time=4.0)

        assert mon.get_alive_peers() == ['amr_2']
        assert mon.get_dead_peers() == ['amr_1']

    def test_unknown_peer_not_alive(self):
        mon = HeartbeatMonitor()
        assert not mon.is_alive('amr_99')
