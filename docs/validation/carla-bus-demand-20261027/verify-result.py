"""Independently check a full two-lap report against its recorded demand plan."""
import collections
import json
import sys
from pathlib import Path

report = json.loads(Path(sys.argv[1]).read_text())
checks = {}


def check(name, condition):
    checks[name] = bool(condition)


plan = report['comparison_plan']
services = report['service_events']
passes = report['station_passes']
restarts = report['restart_events']
expected = {r['key'] for r in plan if r['bell'] or r['expected_waiting']}
actual = {r['key'] for r in services}
pass_by_key = {r['key']: r for r in passes}
samples = report['samples']
crossings = report['signal_crossings']
lap_crossings = report['lap_crossings']
check('runtime_success', report.get('success') is True)
check('full_two_laps', report.get('full_course') and report.get('completed_laps') == 2
      and len(lap_crossings) == 2 and report.get('travel_distance_m', 0) > 3900)
check('each_lap_travel', len(lap_crossings) == 2
      and lap_crossings[0]['travel_distance_m'] > 1900
      and lap_crossings[1]['travel_distance_m']-lap_crossings[0]['travel_distance_m'] > 1900)
check('all_twenty_passes', len(plan) == len(passes) == len(pass_by_key) == 20
      and set(pass_by_key) == {r['key'] for r in plan})
check('only_required_stops_served', actual == expected and len(services) == len(expected))
check('pass_results_match_plan', all(r['key'] in pass_by_key and
      pass_by_key[r['key']]['served'] == (r['key'] in expected) for r in plan))
check('all_service_restarts', {r['key'] for r in restarts} == expected
      and len(restarts) == len(expected))
check('all_ten_stations_served', {r['station'] for r in services} ==
      {f'S{i:02d}' for i in range(1, 11)})
check('five_second_stops', all(r['hold_sim_s'] >= 5 and r['speed_mps'] < .05 for r in services))
check('station_position_tolerance', all(abs(r['longitudinal_error_m']) < 1.5
      and abs(r['lateral_error_m']) < .65 for r in services))
check('all_four_cases', {(bool(r['expected_waiting']), bool(r['bell']))
      for r in plan if r['circuit'] == 1} == {(False, False), (False, True), (True, False), (True, True)})
check('all_passengers_boarded', sum(r['boarded'] for r in services) == len(report['passengers']) == 5
      and all(r['state'] == 'boarded' for r in report['passengers']))
check('no_collision', not report['collisions'])
check('no_signal_violation', bool(crossings) and not any(r['violation'] for r in crossings))
check('no_reverse', all(r['signed_speed_mps'] >= -.2 for r in samples))
check('road_speed_caps', all(r['speed_mps'] <= (50 if r['road'] in (36, 37) else 30)/3.6+.5 for r in samples))
check('station_speed_caps', all(r['speed_mps'] <= 10/3.6+.5 for r in samples
      if any(abs(r['progress_m']-d['distance_m']) < 10 for d in plan)))
check('no_pending_bell', report.get('bell_pending') is False)
cleanup = report['cleanup']
check('cleanup_verified', all(cleanup.get(k) for k in
      ('stop_verified', 'route_cleared', 'waiting_passengers_removed')) and not cleanup.get('errors'))
check('profile_restored', report['station_maneuver_profile']['restored'])
emergency_episodes = 0
previous = False
for event in report['events']:
    if event.get('event') == 'hazard_status':
        active = bool(event['emergency'])
        emergency_episodes += active and not previous
        previous = active
summary = dict(passed=all(checks.values()), checks=checks,
    travel_distance_m=report.get('travel_distance_m'), completed_laps=report.get('completed_laps'),
    service_count=len(services), slow_pass_count=sum(not r['served'] for r in passes),
    boarded_passengers=sum(r['boarded'] for r in services),
    signal_crossings=dict(collections.Counter(r['state'] for r in crossings)),
    collision_count=len(report['collisions']), emergency_episodes=emergency_episodes,
    minimum_service_hold_s=min((r['hold_sim_s'] for r in services), default=None),
    maximum_abs_longitudinal_error_m=max((abs(r['longitudinal_error_m']) for r in services), default=None),
    maximum_abs_lateral_error_m=max((abs(r['lateral_error_m']) for r in services), default=None),
    service_events=services, station_passes=passes)
print(json.dumps(summary, ensure_ascii=False, indent=2))
sys.exit(0 if summary['passed'] else 1)
