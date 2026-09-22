"""Tests for amr_fleet_core.task_allocation.cnp."""

from amr_fleet_core.task_allocation.cnp import (
    Bid,
    TaskAllocator,
    TaskAnnouncement,
    TaskStatus,
    compute_bid_cost,
    evaluate_bids,
)


def _task(task_id='t1', px=0.0, py=0.0, dx=5.0, dy=0.0) -> TaskAnnouncement:
    return TaskAnnouncement(
        task_id=task_id, pickup_x=px, pickup_y=py,
        dropoff_x=dx, dropoff_y=dy, announced_at=1.0,
    )


class TestBidCost:
    """Bid cost computation."""

    def test_cost_is_positive(self):
        task = _task()
        cost = compute_bid_cost(0.0, 0.0, task)
        assert cost > 0

    def test_closer_robot_has_lower_cost(self):
        task = _task(px=5.0, py=0.0, dx=10.0, dy=0.0)
        cost_close = compute_bid_cost(4.0, 0.0, task)  # 1m to pickup
        cost_far = compute_bid_cost(0.0, 0.0, task)    # 5m to pickup
        assert cost_close < cost_far

    def test_cost_includes_transport(self):
        task = _task(px=0.0, py=0.0, dx=10.0, dy=0.0)
        cost = compute_bid_cost(0.0, 0.0, task)
        # Should be at least the transport distance (10m)
        assert cost >= 10.0


class TestEvaluateBids:
    """Deterministic bid evaluation."""

    def test_lowest_cost_wins(self):
        bids = [
            Bid('amr_1', 't1', estimated_cost=5.0, bid_priority=0.5, submitted_at=1.0),
            Bid('amr_2', 't1', estimated_cost=3.0, bid_priority=0.5, submitted_at=1.0),
            Bid('amr_3', 't1', estimated_cost=7.0, bid_priority=0.5, submitted_at=1.0),
        ]
        assert evaluate_bids(bids) == 'amr_2'

    def test_tiebreak_by_priority(self):
        bids = [
            Bid('amr_1', 't1', estimated_cost=5.0, bid_priority=1.0, submitted_at=1.0),
            Bid('amr_2', 't1', estimated_cost=5.0, bid_priority=0.0, submitted_at=1.0),
        ]
        assert evaluate_bids(bids) == 'amr_2'

    def test_tiebreak_by_robot_id(self):
        bids = [
            Bid('amr_2', 't1', estimated_cost=5.0, bid_priority=0.5, submitted_at=1.0),
            Bid('amr_1', 't1', estimated_cost=5.0, bid_priority=0.5, submitted_at=1.0),
        ]
        assert evaluate_bids(bids) == 'amr_1'

    def test_no_bids_returns_none(self):
        assert evaluate_bids([]) is None

    def test_single_bid_wins(self):
        bids = [Bid('amr_1', 't1', 5.0, 0.5, 1.0)]
        assert evaluate_bids(bids) == 'amr_1'


class TestTaskAllocator:
    """Task lifecycle management."""

    def test_full_lifecycle(self):
        alloc = TaskAllocator()
        task = _task()
        alloc.announce_task(task)

        # Submit bids
        alloc.submit_bid(Bid('amr_1', 't1', 5.0, 0.5, 1.0))
        alloc.submit_bid(Bid('amr_2', 't1', 3.0, 0.5, 1.0))

        # Close bidding
        winner = alloc.close_bidding('t1')
        assert winner == 'amr_2'

        record = alloc.get_task_record('t1')
        assert record.status == TaskStatus.AWARDED
        assert record.awarded_to == 'amr_2'

        # Progress and complete
        alloc.mark_in_progress('t1', timestamp=2.0)
        assert alloc.get_task_record('t1').status == TaskStatus.IN_PROGRESS

        alloc.mark_completed('t1', timestamp=10.0)
        assert alloc.get_task_record('t1').status == TaskStatus.COMPLETED

    def test_bid_on_nonexistent_task(self):
        alloc = TaskAllocator()
        result = alloc.submit_bid(Bid('amr_1', 'nope', 5.0, 0.5, 1.0))
        assert result is False

    def test_bid_on_already_awarded_task(self):
        alloc = TaskAllocator()
        alloc.announce_task(_task())
        alloc.submit_bid(Bid('amr_1', 't1', 5.0, 0.5, 1.0))
        alloc.close_bidding('t1')

        # Late bid should be rejected
        result = alloc.submit_bid(Bid('amr_2', 't1', 3.0, 0.5, 2.0))
        assert result is False

    def test_failed_task_can_be_reannounced(self):
        alloc = TaskAllocator()
        alloc.announce_task(_task())
        alloc.submit_bid(Bid('amr_1', 't1', 5.0, 0.5, 1.0))
        alloc.close_bidding('t1')
        alloc.mark_failed('t1')

        record = alloc.get_task_record('t1')
        assert record.status == TaskStatus.FAILED
        assert record.awarded_to is None

    def test_get_pending_tasks(self):
        alloc = TaskAllocator()
        alloc.announce_task(_task('t1'))
        alloc.announce_task(_task('t2'))
        alloc.submit_bid(Bid('amr_1', 't1', 5.0, 0.5, 1.0))
        alloc.close_bidding('t1')

        pending = alloc.get_pending_tasks()
        assert len(pending) == 1
        assert pending[0].task_id == 't2'
