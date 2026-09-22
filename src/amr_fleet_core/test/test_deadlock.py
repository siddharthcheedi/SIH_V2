"""
Tests for amr_fleet_core.negotiation.deadlock — WaitForGraph.

Covers all §6 required deadlock test cases:
  - Correct cycle detection on a genuine cycle
  - No false positive when multiple robots contend without forming a cycle
  - Correct backoff robot selection (lowest priority in cycle)
"""

from amr_fleet_core.negotiation.deadlock import WaitForGraph
from amr_fleet_core.negotiation.priority import make_priority


class TestCycleDetection:
    """WaitForGraph cycle detection."""

    def test_two_robot_cycle(self):
        """A→B→A is a basic deadlock cycle."""
        wfg = WaitForGraph()
        wfg.update('amr_1', yielding_to='amr_2', priority=make_priority(1.0, 'amr_1'))
        wfg.update('amr_2', yielding_to='amr_1', priority=make_priority(2.0, 'amr_2'))

        cycles = wfg.detect_cycles()
        assert len(cycles) >= 1
        # Both robots should be in the cycle
        cycle_members = set(cycles[0])
        assert 'amr_1' in cycle_members
        assert 'amr_2' in cycle_members

    def test_three_robot_cycle(self):
        """A→B→C→A is an N-robot cycle."""
        wfg = WaitForGraph()
        wfg.update('amr_1', yielding_to='amr_2', priority=make_priority(1.0, 'amr_1'))
        wfg.update('amr_2', yielding_to='amr_3', priority=make_priority(2.0, 'amr_2'))
        wfg.update('amr_3', yielding_to='amr_1', priority=make_priority(3.0, 'amr_3'))

        cycles = wfg.detect_cycles()
        assert len(cycles) >= 1
        cycle_set = set(cycles[0])
        assert cycle_set == {'amr_1', 'amr_2', 'amr_3'}

    def test_no_false_positive_on_contention_without_cycle(self):
        """
        Multiple robots yielding to the SAME robot is NOT a cycle — it's
        just contention. This is an easy, real bug to introduce (§6).
        
        A→C, B→C — no cycle, just C is popular.
        """
        wfg = WaitForGraph()
        wfg.update('amr_1', yielding_to='amr_3', priority=make_priority(1.0, 'amr_1'))
        wfg.update('amr_2', yielding_to='amr_3', priority=make_priority(2.0, 'amr_2'))
        wfg.update('amr_3', yielding_to=None, priority=make_priority(0.5, 'amr_3'))

        cycles = wfg.detect_cycles()
        assert cycles == [], f"False positive: detected cycle {cycles} in contention scenario"

    def test_no_cycle_in_chain(self):
        """A→B→C (no back-edge) is NOT a cycle."""
        wfg = WaitForGraph()
        wfg.update('amr_1', yielding_to='amr_2', priority=make_priority(1.0, 'amr_1'))
        wfg.update('amr_2', yielding_to='amr_3', priority=make_priority(2.0, 'amr_2'))
        wfg.update('amr_3', yielding_to=None, priority=make_priority(0.5, 'amr_3'))

        cycles = wfg.detect_cycles()
        assert cycles == []

    def test_robot_not_yielding_has_no_edges(self):
        """A robot with yielding_to=None has no outgoing edges."""
        wfg = WaitForGraph()
        wfg.update('amr_1', yielding_to=None, priority=make_priority(1.0, 'amr_1'))
        cycles = wfg.detect_cycles()
        assert cycles == []


class TestBackoffSelection:
    """Correct backoff robot selection — lowest priority in cycle."""

    def test_lowest_priority_backs_off(self):
        """In a cycle, the robot with the worst priority backs off."""
        wfg = WaitForGraph()
        wfg.update('amr_1', yielding_to='amr_2',
                    priority=make_priority(1.0, 'amr_1'))  # best priority
        wfg.update('amr_2', yielding_to='amr_3',
                    priority=make_priority(2.0, 'amr_2'))
        wfg.update('amr_3', yielding_to='amr_1',
                    priority=make_priority(3.0, 'amr_3'))  # worst priority

        cycle = ['amr_1', 'amr_2', 'amr_3']
        backoff = wfg.select_backoff_robot(cycle)
        assert backoff == 'amr_3', f"Expected amr_3 (worst priority), got {backoff}"

    def test_tiebreak_by_robot_id(self):
        """Same timestamp → tiebreak by robot_id (higher string = worse)."""
        wfg = WaitForGraph()
        wfg.update('amr_1', yielding_to='amr_2',
                    priority=make_priority(1.0, 'amr_1'))
        wfg.update('amr_2', yielding_to='amr_1',
                    priority=make_priority(1.0, 'amr_2'))  # same ts, higher id

        cycle = ['amr_1', 'amr_2']
        backoff = wfg.select_backoff_robot(cycle)
        assert backoff == 'amr_2'

    def test_empty_cycle_returns_none(self):
        wfg = WaitForGraph()
        assert wfg.select_backoff_robot([]) is None


class TestDetectAndResolve:
    """Integration: detect_and_resolve logs events correctly."""

    def test_event_logged_on_cycle(self):
        wfg = WaitForGraph()
        wfg.update('amr_1', yielding_to='amr_2', priority=make_priority(1.0, 'amr_1'))
        wfg.update('amr_2', yielding_to='amr_1', priority=make_priority(2.0, 'amr_2'))

        events = wfg.detect_and_resolve(timestamp=10.0)
        assert len(events) >= 1
        ev = events[0]
        assert ev.timestamp == 10.0
        assert ev.backoff_robot == 'amr_2'
        assert 'amr_1' in ev.cycle
        assert 'amr_2' in ev.cycle

    def test_events_accumulated(self):
        wfg = WaitForGraph()
        wfg.update('amr_1', yielding_to='amr_2', priority=make_priority(1.0, 'amr_1'))
        wfg.update('amr_2', yielding_to='amr_1', priority=make_priority(2.0, 'amr_2'))

        wfg.detect_and_resolve(timestamp=1.0)
        wfg.detect_and_resolve(timestamp=2.0)

        all_events = wfg.get_all_events()
        assert len(all_events) >= 2


class TestRemove:
    """Removing a robot from the WFG."""

    def test_remove_breaks_cycle(self):
        wfg = WaitForGraph()
        wfg.update('amr_1', yielding_to='amr_2', priority=make_priority(1.0, 'amr_1'))
        wfg.update('amr_2', yielding_to='amr_1', priority=make_priority(2.0, 'amr_2'))

        # Removing one robot should break the cycle
        wfg.remove('amr_2')
        cycles = wfg.detect_cycles()
        assert cycles == []
