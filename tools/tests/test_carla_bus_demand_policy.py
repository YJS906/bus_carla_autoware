from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carla_bus_demand_policy import StopDemand, approach_cap, bell_target, comparison_plan, passenger_counts, rolling_horizon


@pytest.mark.parametrize('people,bell,required', [(0,False,False),(2,False,True),(0,True,True),(2,True,True)])
def test_four_service_combinations(people,bell,required):
    stop=StopDemand('0:S02','S02',60,waiting=people,bell=bell)
    assert stop.required is required
    if not required:
        with pytest.raises(ValueError):stop.commit()
        return
    stop.commit()
    assert not stop.dwell.observe(1,0,0,0,True)
    assert stop.dwell.observe(6,0,0,0,True)
    assert len(stop.complete()) == int(people>0)+int(bell)
    assert not stop.required and not stop.bell and stop.waiting==0


def test_passenger_commit_survives_transient_count_loss_and_red_light_stop():
    stop=StopDemand('0:S02','S02',60,waiting=1)
    stop.commit();stop.waiting=0
    assert stop.required
    assert not stop.dwell.observe(10,0,30,0,True)
    assert not stop.dwell.observe(20,0,0,0,False)
    assert not stop.dwell.observe(30,0,0,0,True)
    assert stop.dwell.observe(35,0,0,0,True)
    assert stop.complete()==['waiting_passengers']


def test_late_bell_is_latched_to_following_stop_and_duplicates_keep_target():
    stops=[StopDemand('0:S02','S02',60),StopDemand('0:S03','S03',115)]
    target,result=bell_target(stops,45,2.5)
    assert (target.station,result)==('S03','carried_forward')
    target.bell=True
    assert bell_target(stops,46,2.5)==(target,'already_requested')


def test_served_stop_is_not_selected_again():
    stops=[StopDemand('0:S02','S02',60),StopDemand('0:S03','S03',115)]
    stops[0].dwell.completed=True
    assert bell_target(stops,59,0)[0].station=='S03'
    assert bell_target(stops,110,0)==(None,'closed')


def test_bell_during_passenger_dwell_is_served_at_current_stop():
    stops=[StopDemand('0:S02','S02',60,waiting=1),StopDemand('0:S03','S03',115)]
    stops[0].commit()
    assert bell_target(stops,60,0,dwelling=True)==(stops[0],'accepted')


def test_duplicate_during_dwell_does_not_create_second_request():
    stops=[StopDemand('0:S02','S02',60,waiting=1),StopDemand('0:S03','S03',115,bell=True)]
    stops[0].commit()
    assert bell_target(stops,60,0,dwelling=True)==(stops[1],'already_requested')
    assert not stops[0].bell


def test_comparison_reproducible_covers_four_cases_and_every_stop_in_two_circuits():
    ids=[f'S{i:02}' for i in range(2,11)]+['S01']
    counts,rows=comparison_plan(26027,ids,2)
    assert (counts,rows)==comparison_plan(26027,ids,2)
    assert {(r['expected_waiting']>0,r['bell']) for r in rows[:10]}=={(False,False),(True,False),(False,True),(True,True)}
    assert all(r['expected_waiting']==0 for r in rows[10:])
    assert all(counts[name] or any(r['bell'] for r in rows if r['station']==name) for name in ids)


def test_slow_frontage_includes_braking_distance_and_clears_after_pass():
    assert approach_cap(60,30/3.6)==10
    assert approach_cap(100,50/3.6)==10
    assert approach_cap(10,2)==10
    assert approach_cap(-15,2)==10
    assert approach_cap(-19,2) is None
    assert approach_cap(200,30/3.6) is None


def test_explicit_passenger_configuration_validation():
    assert passenger_counts('S02:2,S03:0',['S02','S03'])=={'S02':2,'S03':0}
    for text in ('S04:1','S02:-1','S02:4','S02:1,S02:2'):
        with pytest.raises(ValueError):passenger_counts(text,['S02','S03'])


def test_finite_horizon_stays_at_terminal_across_last_S01_boundary():
    assert rolling_horizon(1800,2000,1700,2)==('R02',1)
    assert rolling_horizon(2001,2000,1700,2)==('S10',1)
    for progress in (3700,3994,4000,4002,4012):
        assert rolling_horizon(progress,2000,1700,2)==('TEST_END',2)


def test_unbounded_horizon_keeps_advancing_after_S01():
    assert rolling_horizon(3994,2000,1700,0)==('R02',2)
    assert rolling_horizon(4000,2000,1700,0)==('S10',2)
