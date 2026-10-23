import sys
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carla_bus_following import following_speed_limit


def box(x, y, length=4., width=2.):
    return [(x-length/2, y-width/2), (x-length/2, y+width/2),
            (x+length/2, y+width/2), (x+length/2, y-width/2)]


def cap(bus, heading=(1, 0), velocity=(0, 0)):
    return following_speed_limit(box(0, 0), bus, (1, 0), heading, velocity, 20/3.6)


def test_slow_bus_occupying_forward_corridor_requires_braking():
    assert cap(box(12, 0, 10, 4)) == 0
    assert 0 < cap(box(20, 0, 10, 4)) < 20/3.6
    assert cap(box(60, 0, 10, 4)) == 20/3.6


def test_adjacent_bus_only_affects_car_after_body_enters_corridor():
    assert cap(box(15, 3.5, 10, 4)) == 20/3.6
    assert cap(box(15, 2.5, 10, 4)) < 20/3.6


def test_oncoming_crossing_and_bus_behind_do_not_get_following_priority():
    for heading in [(-1, 0), (0, 1)]:
        assert cap(box(12, 0, 10, 4), heading) == 20/3.6
    assert cap(box(-12, 0, 10, 4)) == 20/3.6


def test_moving_leader_and_rotation():
    assert cap(box(20, 0, 10, 4), velocity=(5, 0)) > cap(box(20, 0, 10, 4))
    rotate = lambda p: [(-y, x) for x, y in p]
    assert following_speed_limit(rotate(box(0, 0)), rotate(box(20, 0, 10, 4)),
                                 (0, 1), (0, 1), (0, 0), 20/3.6) == cap(box(20, 0, 10, 4))


def test_angled_adjacent_car_is_not_a_follower_of_the_long_bus():
    # Reproduce the S05/ring-entry deadlock: the adjacent car was 3.93 m
    # laterally separated but pointed 15 degrees toward the bus on a bend.
    angle = math.radians(-15)
    c, s = math.cos(angle), math.sin(angle)
    car = [(-3.5+c*x-s*y, 3.93+s*x+c*y) for x, y in box(0, 0, 3.705, 1.789)]
    assert following_speed_limit(car, box(0, 0, 10.273, 4.04),
                                 (c, s), (1, 0), (0, 0), 20/3.6) == 20/3.6
    # Moving that same car into the bus corridor must restore braking.
    intruding = [(x, y-2.0) for x, y in car]
    assert following_speed_limit(intruding, box(0, 0, 10.273, 4.04),
                                 (c, s), (1, 0), (0, 0), 20/3.6) == 0
