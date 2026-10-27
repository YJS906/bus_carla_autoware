import argparse,json,math,sys
from pathlib import Path
import rclpy
from rclpy.signals import SignalHandlerOptions
from carla_bus_lap import BusLap
from carla_bus_service_profile import StationManeuverProfile

target=sys.argv[1] if len(sys.argv)>1 else 'S04'
previous={'S01':'S10','S02':'S01','S03':'S02','S04':'S03','S05':'S04','S06':'R06','S07':'S06','S08':'S07','S09':'S08','S10':'S09'}[target]
offset=float(sys.argv[2]) if len(sys.argv)>2 else 14.
args=argparse.Namespace(route=Path('/tmp/bus-tools/town05_bus_route.json'),
 map_dir=Path('/home/aw/autoware_data/maps/Town05Bus'),case='bus_lap',speed_kmh=10.,
 minimum_peak_kmh=0.,cruise_hold_s=0.,host='127.0.0.1',port=2000,input_mode='ground_truth',
 start_anchor=previous,end_anchor=target,resume_current_pose=False,stop_at_all_stops=True,
 preflight=False,with_traffic=False,interrupted=False,duration=240)
rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO);n=None;profile=None
try:
 n=BusLap(args)
 service=json.loads(Path('/tmp/bus-tools/town05_bus_service_stops.json').read_text())
 start=next((a for a in service['stops'] if a['id']==previous),n.course['anchors'][n.start_index])
 n.course['anchors'][n.start_index]=start
 goal=next(a for a in n.course['anchors'] if a['id']==target).copy()
 wp=n.wmap.get_waypoint_xodr(goal['road_id'],goal['lane_id'],goal['s_m'])
 if offset:wp=(wp.next(offset) if offset>0 else wp.previous(-offset))[0]
 t=wp.transform
 goal.update(s_m=wp.s,route_distance_m=goal['route_distance_m']+offset,
  carla=dict(x=t.location.x,y=t.location.y,z=t.location.z,yaw_deg=t.rotation.yaw),
  ros_map=dict(x=t.location.x,y=-t.location.y,z=t.location.z,yaw_rad=-math.radians(t.rotation.yaw)))
 n.result['tested_goal']=goal
 n.initialise();profile=StationManeuverProfile(n);profile.activate();n.drive_leg([goal])
 n.result.update(success=True,result=target+'_service_goal_stop')
except Exception as exc:
 print(repr(exc),flush=True)
 if n:n.result.update(success=False,error=repr(exc))
finally:
 if n:
  n.cleanup()
  if profile:profile.restore()
  Path('/tmp/'+target+'-service-goal-test.json').write_text(json.dumps(n.result,indent=2)+'\n');n.destroy_node()
 rclpy.shutdown()
