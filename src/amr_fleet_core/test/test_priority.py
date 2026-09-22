"""Tests for amr_fleet_core.negotiation.priority."""

from amr_fleet_core.negotiation.priority import (
    has_priority_over,
    higher_priority,
    make_priority,
    order_zones_for_acquisition,
)


class TestPriorityKey:
    """Verify priority ordering and tie-breaking."""

    def test_earlier_timestamp_wins(self):
        p1 = make_priority(1.0, 'amr_1')
        p2 = make_priority(2.0, 'amr_2')
        assert has_priority_over(p1, p2)
        assert not has_priority_over(p2, p1)

    def test_same_timestamp_tiebreak_by_robot_id(self):
        """When timestamps are equal, lower robot_id string wins."""
        p1 = make_priority(1.0, 'amr_1')
        p2 = make_priority(1.0, 'amr_2')
        assert has_priority_over(p1, p2)
        assert not has_priority_over(p2, p1)

    def test_equal_priorities_not_strictly_higher(self):
        p = make_priority(1.0, 'amr_1')
        assert not has_priority_over(p, p)

    def test_higher_priority_function(self):
        p1 = make_priority(1.0, 'amr_1')
        p2 = make_priority(2.0, 'amr_2')
        assert higher_priority(p1, p2) == p1
        assert higher_priority(p2, p1) == p1


class TestResourceOrdering:
    """Verify the resource ordering helper for structural deadlock prevention."""

    def test_already_sorted_unchanged(self):
        assert order_zones_for_acquisition([0, 1, 2]) == [0, 1, 2]

    def test_unsorted_gets_sorted(self):
        assert order_zones_for_acquisition([2, 0, 1]) == [0, 1, 2]

    def test_deduplicates(self):
        assert order_zones_for_acquisition([1, 1, 2, 2, 0]) == [0, 1, 2]

    def test_empty_input(self):
        assert order_zones_for_acquisition([]) == []

    def test_single_zone(self):
        assert order_zones_for_acquisition([3]) == [3]

    def test_deterministic_across_calls(self):
        """Same input must always produce the same output — determinism is key."""
        for _ in range(100):
            result = order_zones_for_acquisition([5, 3, 1, 4, 2, 0])
            assert result == [0, 1, 2, 3, 4, 5]
