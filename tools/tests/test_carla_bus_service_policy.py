import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carla_bus_service_policy import ServicePolicy, acceptance_distance, route_progress


def test_bell_latches_and_repeats_do_not_queue_another_stop():
    p = ServicePolicy()
    assert p.ring(60, 2) == 'accepted'
    assert p.ring(1, 2) == 'already_requested'
    for t in (0, 1, 6, 20):
        assert not p.observe(t, 0, 20, 0, True)
    assert p.requested  # traffic stops cannot consume the bell


def test_only_committed_correct_lane_stop_can_start_service():
    p = ServicePolicy(requested=True)
    assert not p.observe(0, 0, 0, 0, True)
    p.committed = True
    assert not p.observe(5, 0, 0, 0, False)
    assert not p.observe(10, 0, 0, .8, True)
    assert p.stopped_since is None
    assert not p.observe(11, 0, .5, .1, True)
    assert not p.observe(15.99, 0, .5, .1, True)
    assert p.observe(16, 0, .5, .1, True)
    p.complete()
    assert p.completed and not p.requested


def test_rolling_or_position_drift_restarts_full_five_seconds():
    p = ServicePolicy(requested=True, committed=True)
    p.observe(0, 0, 0, 0, True)
    p.observe(4, .1, 0, 0, True)
    assert p.stopped_since is None
    assert not p.observe(5, 0, 0, 0, True)
    assert not p.observe(9, 0, 2, 0, True)
    assert not p.observe(10, 0, 0, 0, True)
    assert p.observe(15, 0, 0, 0, True)


def test_clock_pause_does_not_complete_and_reversal_fails():
    p = ServicePolicy(requested=True, committed=True)
    for _ in range(100):
        assert not p.observe(10, 0, 0, 0, True)
    with pytest.raises(ValueError):
        p.observe(9, 0, 0, 0, True)
    with pytest.raises(ValueError):
        p.complete()


def test_late_or_closed_request_does_not_latch():
    p = ServicePolicy()
    assert p.ring(25, 0) == 'too_late'
    assert p.ring(60, 0, accepting=False) == 'too_late'
    assert not p.requested
    assert acceptance_distance(8) > acceptance_distance(2)


def test_no_bell_never_serves_even_if_stopped_at_station():
    p = ServicePolicy(committed=True)
    assert not p.observe(0, 0, 0, 0, True)
    assert not p.observe(100, 0, 0, 0, True)
    assert not p.completed


def test_progress_follows_route_direction_and_turns():
    points = [dict(carla=dict(x=x, y=y), route_distance_m=s)
              for x, y, s in [(0, 0, 0), (10, 0, 10), (10, 10, 20)]]
    assert route_progress(points, 5, .4) == pytest.approx(5)
    assert route_progress(points, 10.4, 7) == pytest.approx(17)
