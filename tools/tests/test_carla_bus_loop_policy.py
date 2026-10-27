import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carla_bus_loop_policy import LoopProgress, landmarks_between, random_bell_schedule


def square():
    return [dict(carla=dict(x=x, y=y), route_distance_m=s)
            for x,y,s in [(0,0,0),(100,0,100),(100,100,200),(0,100,300),(0,0,400)]]


def test_progress_crosses_lap_boundary_without_reset_or_false_early_wrap():
    p=LoopProgress(square(),400)
    assert p.observe(1,0)==pytest.approx(1)
    for x,y in [(40,0),(80,0),(100,20),(100,60),(100,100),(60,100),(20,100),(0,80),(0,40),(0,2)]:
        p.observe(x,y)
    assert p.observe(5,0)==pytest.approx(405)
    assert p.observe(3,0)==pytest.approx(405)  # no backwards progress


def test_far_position_is_not_treated_as_next_lap():
    p=LoopProgress(square(),400)
    with pytest.raises(RuntimeError):p.observe(100,100)


def test_landmarks_preserve_every_anchor_across_s01():
    a=[dict(id='S01',route_distance_m=0),dict(id='S02',route_distance_m=100),dict(id='S10',route_distance_m=350)]
    assert [(s,x['id']) for s,x in landmarks_between(a,400,340,501)]==[(350,'S10'),(400,'S01'),(500,'S02')]


def test_random_plan_reproduces_both_cases_and_acceptance_margin():
    a=random_bell_schedule(26026,2,1947.284,2032.064)
    assert a==random_bell_schedule(26026,2,1947.284,2032.064)
    assert {r['request'] for r in a}=={True,False}
    for r in a:
        if r['request']:assert 100 <= r['progress_m']-(r['lap']-1)*2032.064 <= 1727.284
    with pytest.raises(ValueError):random_bell_schedule(1,1,1947.284,2032.064)


def test_saved_town05_reference_can_be_followed_for_two_laps():
    route=json.loads((Path(__file__).resolve().parents[2]/'scenarios/bus/town05_bus_route.json').read_text())
    points=[p for segment in route['segments'] for p in segment['points']]
    tracker=LoopProgress(points,route['length_m'])
    for _ in range(2):
        for p in tracker.points:
            tracker.observe(p['carla']['x'],p['carla']['y'])
    assert tracker.value==pytest.approx(2*route['length_m'],abs=.1)
