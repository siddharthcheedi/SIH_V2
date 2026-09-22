"""Tests for amr_fleet_core.negotiation.reservation — ReservationTable."""

from amr_fleet_core.negotiation.reservation import (
    Conflict,
    Reservation,
    ReservationTable,
)
from amr_fleet_core.negotiation.grid import CellReservation, CellCoord, cell_id


def _make_reservation(robot_id: str, cid: int,
                       t_start: float, t_end: float,
                       priority: tuple = (0.0, 'x')) -> Reservation:
    return Reservation(
        robot_id=robot_id,
        cell_id=cid,
        t_start=t_start,
        t_end=t_end,
        priority_key=priority,
    )


class TestReservationTable:
    """Core ReservationTable functionality."""

    def test_no_conflict_disjoint_cells(self):
        """Reservations on different cells should never conflict."""
        table = ReservationTable()
        r1 = [_make_reservation('amr_1', cid=0, t_start=0, t_end=5)]
        r2 = [_make_reservation('amr_2', cid=1, t_start=0, t_end=5)]
        table.update_robot('amr_1', r1)
        table.update_robot('amr_2', r2)

        conflicts = table.find_conflicts(r1, exclude_robot='amr_1')
        assert conflicts == []

    def test_no_conflict_disjoint_times(self):
        """Same cell but non-overlapping times should not conflict."""
        table = ReservationTable()
        r1 = [_make_reservation('amr_1', cid=5, t_start=0, t_end=3)]
        r2 = [_make_reservation('amr_2', cid=5, t_start=4, t_end=7)]
        table.update_robot('amr_1', r1)
        table.update_robot('amr_2', r2)

        conflicts = table.find_conflicts(r1, exclude_robot='amr_1')
        assert conflicts == []

    def test_conflict_on_overlapping_cell_and_time(self):
        """Same cell with overlapping times must produce a conflict."""
        table = ReservationTable()
        r1 = [_make_reservation('amr_1', cid=5, t_start=0, t_end=5,
                                 priority=(1.0, 'amr_1'))]
        r2 = [_make_reservation('amr_2', cid=5, t_start=3, t_end=8,
                                 priority=(0.5, 'amr_2'))]
        table.update_robot('amr_2', r2)

        conflicts = table.find_conflicts(r1, exclude_robot='amr_1')
        assert len(conflicts) == 1
        assert conflicts[0].other_robot_id == 'amr_2'

    def test_conflict_boundary_overlap(self):
        """Reservations touching at boundary (t_end == t_start) should conflict."""
        table = ReservationTable()
        r1 = [_make_reservation('amr_1', cid=5, t_start=0, t_end=5)]
        r2 = [_make_reservation('amr_2', cid=5, t_start=5, t_end=10)]
        table.update_robot('amr_2', r2)

        conflicts = table.find_conflicts(r1, exclude_robot='amr_1')
        # Inclusive boundary → overlap at t=5
        assert len(conflicts) == 1

    def test_update_supersedes_not_appends(self):
        """
        A fresh update_robot() must REPLACE the previous set entirely.
        This is critical — if we appended, stale reservations from a
        previous (now-obsolete) plan would linger and cause false conflicts.
        """
        table = ReservationTable()

        # First plan: amr_2 at cell 5, t=0..5
        old = [_make_reservation('amr_2', cid=5, t_start=0, t_end=5)]
        table.update_robot('amr_2', old)

        # Second plan: amr_2 moved to cell 10, t=0..5 (cell 5 no longer needed)
        new = [_make_reservation('amr_2', cid=10, t_start=0, t_end=5)]
        table.update_robot('amr_2', new)

        # amr_1 wants cell 5 at t=0..5 — should NOT conflict with amr_2's
        # superseded reservation
        r1 = [_make_reservation('amr_1', cid=5, t_start=0, t_end=5)]
        conflicts = table.find_conflicts(r1, exclude_robot='amr_1')
        assert conflicts == [], (
            "Stale reservations from superseded plan caused false conflict"
        )

    def test_clear_robot(self):
        """Clearing a robot removes all its reservations."""
        table = ReservationTable()
        table.update_robot('amr_1', [
            _make_reservation('amr_1', cid=5, t_start=0, t_end=5)
        ])
        table.clear_robot('amr_1')

        assert table.get_robot_reservations('amr_1') == []
        assert 'amr_1' not in table.get_all_robots()

    def test_clear_nonexistent_robot_is_noop(self):
        """Clearing a robot that doesn't exist should not raise."""
        table = ReservationTable()
        table.clear_robot('amr_99')  # should not raise

    def test_self_exclusion(self):
        """find_conflicts should skip self-comparisons."""
        table = ReservationTable()
        r1 = [_make_reservation('amr_1', cid=5, t_start=0, t_end=5)]
        table.update_robot('amr_1', r1)

        conflicts = table.find_conflicts(r1, exclude_robot='amr_1')
        assert conflicts == []

    def test_multiple_peer_conflicts(self):
        """Should detect conflicts with multiple peers independently."""
        table = ReservationTable()
        table.update_robot('amr_2', [
            _make_reservation('amr_2', cid=5, t_start=0, t_end=5)
        ])
        table.update_robot('amr_3', [
            _make_reservation('amr_3', cid=5, t_start=2, t_end=7)
        ])

        r1 = [_make_reservation('amr_1', cid=5, t_start=1, t_end=4)]
        conflicts = table.find_conflicts(r1, exclude_robot='amr_1')
        peer_ids = {c.other_robot_id for c in conflicts}
        assert peer_ids == {'amr_2', 'amr_3'}


class TestConflictPriority:
    """Verify priority comparison on Conflict objects."""

    def test_other_has_priority_when_lower_tuple(self):
        r1 = _make_reservation('amr_1', 5, 0, 5, priority=(2.0, 'amr_1'))
        r2 = _make_reservation('amr_2', 5, 0, 5, priority=(1.0, 'amr_2'))
        c = Conflict(
            own_reservation=r1,
            other_reservation=r2,
            other_robot_id='amr_2',
            other_priority=(1.0, 'amr_2'),
        )
        assert c.other_has_priority is True

    def test_other_does_not_have_priority_when_higher_tuple(self):
        r1 = _make_reservation('amr_1', 5, 0, 5, priority=(1.0, 'amr_1'))
        r2 = _make_reservation('amr_2', 5, 0, 5, priority=(2.0, 'amr_2'))
        c = Conflict(
            own_reservation=r1,
            other_reservation=r2,
            other_robot_id='amr_2',
            other_priority=(2.0, 'amr_2'),
        )
        assert c.other_has_priority is False

    def test_equal_priority_means_other_has_priority(self):
        """Equal priority → other_has_priority is True (conservative)."""
        r1 = _make_reservation('amr_1', 5, 0, 5, priority=(1.0, 'amr_1'))
        r2 = _make_reservation('amr_2', 5, 0, 5, priority=(1.0, 'amr_1'))
        c = Conflict(
            own_reservation=r1,
            other_reservation=r2,
            other_robot_id='amr_2',
            other_priority=(1.0, 'amr_1'),
        )
        assert c.other_has_priority is True


class TestBuildFromCellReservations:
    """Verify the CellReservation → Reservation bridge."""

    def test_build_preserves_fields(self):
        table = ReservationTable()
        cell_res = [
            CellReservation(
                cell=CellCoord(row=5, col=10),
                t_start=0.0,
                t_end=3.0,
            ),
        ]
        priority = (1.0, 'amr_1')
        reservations = table.build_reservations_from_cells(
            'amr_1', cell_res, priority
        )
        assert len(reservations) == 1
        r = reservations[0]
        assert r.robot_id == 'amr_1'
        assert r.t_start == 0.0
        assert r.t_end == 3.0
        assert r.priority_key == priority
