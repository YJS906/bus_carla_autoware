import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carla_bus_route_policy import station_exit_preferences


def fixture():
    # Outer station lane 10 leads to a terminal outer approach 20; the turn 30
    # is connected only from inner lane 11 through inner approach 21.
    sections = [dict(lanes=[10, 11], preferred=10),
                dict(lanes=[20, 21], preferred=21),
                dict(lanes=[30], preferred=30)]
    endpoints = {10: ((1, 2), (3, 4)), 11: ((0, 1), (5, 3)),
                 20: ((3, 4), (6, 7)), 21: ((5, 3), (8, 6)),
                 30: ((8, 6), (9, 10))}
    return sections, endpoints


def test_release_passed_station_to_connected_exit():
    sections, endpoints = fixture()
    assert station_exit_preferences(sections, endpoints, 10) == {0: 11}
    assert sections[0]['preferred'] == 10  # no mutation of route evidence


def test_no_change_when_current_lane_already_reaches_exit():
    sections, endpoints = fixture()
    sections[0]['preferred'] = 11
    assert station_exit_preferences(sections, endpoints, 11) == {}


def test_preserve_unpassed_station_and_unknown_connectivity():
    sections, endpoints = fixture()
    assert station_exit_preferences(sections, endpoints, 10, protected={20}) == {}
    endpoints[30] = ((99, 100), (101, 102))
    assert station_exit_preferences(sections, endpoints, 10) == {}


def test_no_forced_exit_or_missing_station_does_not_rewrite_route():
    sections, endpoints = fixture()
    assert station_exit_preferences(sections[:2], endpoints, 10) == {}
    assert station_exit_preferences(sections, endpoints, 99) == {}


def test_urban_merge_cap_starts_before_station_exit_and_spans_junction():
    from carla_bus_route_policy import CentralUrbanMergePolicy
    p = CentralUrbanMergePolicy()
    assert p.observe(5, -2, 0, 0, 0) is None
    assert p.observe(1, 2, 0, 0, 1) == 10
    assert p.observe(1438, 1, 0, 0, 2) == 10
    assert p.observe(0, 2, 0, 0, 3) == 10


def test_urban_merge_cap_releases_only_after_stable_alignment():
    from carla_bus_route_policy import CentralUrbanMergePolicy
    p = CentralUrbanMergePolicy()
    p.observe(1, 2, 0, 0, 0)
    assert p.observe(0, 1, .8, 0, 1, station_passed=True) == 10
    assert p.observe(0, 1, .2, .2, 2, station_passed=True) == 10
    assert p.observe(0, 1, .2, .02, 3, station_passed=True) == 10
    assert p.observe(0, 1, .2, .02, 3.5, station_passed=True) == 10
    assert p.observe(0, 1, .2, .02, 4.1, station_passed=True) is None
    assert p.observe(37, -3, .2, 0, 100) is None


def test_urban_merge_cap_alignment_timer_resets():
    from carla_bus_route_policy import CentralUrbanMergePolicy
    p = CentralUrbanMergePolicy()
    p.observe(1, 2, 0, 0, 0)
    p.observe(0, 1, .2, .02, 1, station_passed=True)
    p.observe(0, 2, .2, .02, 1.5)
    assert p.observe(0, 1, .2, .02, 2, station_passed=True) == 10
    assert p.observe(0, 1, .2, .02, 3.1, station_passed=True) is None


def test_urban_merge_cap_does_not_release_before_outer_station_visit():
    from carla_bus_route_policy import CentralUrbanMergePolicy
    p = CentralUrbanMergePolicy()
    assert p.observe(1, 1, 0, 0, 0) == 10
    assert p.observe(1, 1, 0, 0, 3) == 10
    assert p.observe(1, 2, 0, 0, 5, station_passed=True) == 10
