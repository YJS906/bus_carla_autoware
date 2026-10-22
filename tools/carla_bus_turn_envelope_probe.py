#!/usr/bin/env python3
"""Short empty-road turning measurements; direct actuation, not Autoware validation."""
import argparse
import json
import math
from pathlib import Path
import signal
import time
import carla
import numpy as np


def xyz(v):return [v.x,v.y,v.z]


def run(output):
    c=carla.Client('localhost',2000);c.set_timeout(10);w=c.get_world();w.wait_for_tick(5)
    original=w.get_settings()
    if original.synchronous_mode or w.get_actors().filter('vehicle.*') or w.get_actors().filter('walker.pedestrian.*'):
        raise RuntimeError('Requires empty asynchronous world')
    if w.get_map().name.rsplit('/',1)[-1] not in ('Town05','Town05_Opt'):raise RuntimeError('Requires Town05')
    result={'purpose':'direct_actuation_turn_geometry_not_autoware_validation','cases':[],'complete':False}
    bus=sensor=None;collisions=[]
    try:
        s=w.get_settings();s.synchronous_mode=True;s.fixed_delta_seconds=.05;w.apply_settings(s)
        origin=w.get_map().get_waypoint_xodr(37,-2,330).transform;origin.location.z+=.3
        bp=w.get_blueprint_library().find('vehicle.mitsubishi.fusorosa');bp.set_attribute('role_name','bus_turn_geometry_probe')
        bus=w.spawn_actor(bp,origin);bus.apply_control(carla.VehicleControl(brake=1,hand_brake=True))
        for _ in range(60):w.tick()
        origin=bus.get_transform();inv=np.array(origin.get_inverse_matrix())
        sensor=w.spawn_actor(w.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=bus)
        sensor.listen(lambda e:collisions.append({'frame':e.frame,'other':e.other_actor.type_id,'impulse':xyz(e.normal_impulse)}))
        for steer in [-1.,1.]:
            bus.set_transform(origin);bus.set_target_velocity(carla.Vector3D());bus.set_target_angular_velocity(carla.Vector3D())
            bus.apply_control(carla.VehicleControl(brake=1,hand_brake=True,steer=steer))
            for _ in range(40):w.tick()
            collisions.clear();start_yaw=bus.get_transform().rotation.yaw
            rows=[];braking=False;done=False
            for _ in range(1200):
                speed=bus.get_velocity().length()
                throttle=max(0,min(.45,.25+.3*(1.0-speed))) if not braking else 0.
                bus.apply_control(carla.VehicleControl(steer=steer,throttle=throttle,brake=1. if braking else 0.,hand_brake=False))
                frame=w.tick();t=bus.get_transform();v=bus.get_velocity().length()
                yaw=math.degrees(math.atan2(math.sin(math.radians(t.rotation.yaw-start_yaw)),math.cos(math.radians(t.rotation.yaw-start_yaw))))
                corners=[(inv@np.array([*xyz(p),1]))[:3].tolist() for p in bus.bounding_box.get_world_vertices(t)]
                pos=(inv@np.array([*xyz(t.location),1]))[:3].tolist()
                rows.append({'frame':frame,'sim_s':len(rows)*.05,'actor_local_xyz':pos,'yaw_change_deg':yaw,
                             'speed_mps':v,'bbox_vertices_in_start_frame':corners,
                             'front_angles_deg':[bus.get_wheel_steer_angle(k) for k in [carla.VehicleWheelLocation.FL_Wheel,carla.VehicleWheelLocation.FR_Wheel]]})
                if v>1.8:raise RuntimeError('Probe exceeded 6.5km/h')
                if abs(yaw)>=25:braking=True
                if collisions:raise RuntimeError('Environment collision during short turn')
                if braking and v<.02:
                    for _ in range(10):w.tick()
                    done=True;break
            if not done:raise RuntimeError('Turn or stop did not complete')
            ys=[p[1] for r in rows for p in r['bbox_vertices_in_start_frame']]
            case={'steer_input':steer,'samples':rows,'collisions':list(collisions),
                  'swept_bbox_lateral_bounds_m':[min(ys),max(ys)],'final_yaw_deg':rows[-1]['yaw_change_deg'],
                  'peak_speed_mps':max(r['speed_mps'] for r in rows),'final_speed_mps':rows[-1]['speed_mps']}
            result['cases'].append(case);output.write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps({k:v for k,v in case.items() if k!='samples'}),flush=True)
        result['complete']=True
    finally:
        if sensor:sensor.stop();sensor.destroy()
        if bus:bus.destroy()
        w.apply_settings(original)
        result['cleanup']={'actors_destroyed':True,'settings_restored':True}
        output.write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    def interrupt(*_):raise RuntimeError('Probe interrupted')
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    run(a.output)
