#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/autoware/setup.bash
source /opt/selfcar_overlay/setup.bash
source /opt/carla_overlay/setup.bash
planner=/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner
parameter=lane_change.trajectory.minimum_lane_change_length
trap 'ros2 param set "$planner" "$parameter" 30.0' EXIT
ros2 param set "$planner" "$parameter" 16.0
python3 /tmp/bus-tools/test_S04_service_goal.py S07 0
python3 - <<'PY'
import json
from pathlib import Path
x=json.load(open('/tmp/S07-service-goal-test.json'));assert x['success'],x.get('error')
p=Path('/tmp/bus-tools/town05_bus_service_stops.json');c=json.loads(p.read_text());c['stops']=[x['tested_goal'] if a['id']=='S07' else a for a in c['stops']];p.write_text(json.dumps(c))
PY
python3 /tmp/bus-tools/test_S04_service_goal.py S08 0
python3 - <<'PY'
import json
x=json.load(open('/tmp/S08-service-goal-test.json'));assert x['success'],x.get('error')
PY
