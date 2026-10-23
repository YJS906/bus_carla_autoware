#!/usr/bin/env python3
"""Open-loop bus pedal calibration in an otherwise empty CARLA world.

This is actuator measurement, not Autoware driving validation. Temporarily
owns the synchronous tick only when no ego/other vehicles or pedestrians exist.
Restores world settings and destroys its own actor/sensor in finally.
"""
import argparse
import json
import math
from pathlib import Path
import statistics

import carla


def run(output, speeds=(2.78,5.56), acceleration_pedal=.65, extended=False):
    client=carla.Client('127.0.0.1',2000);client.set_timeout(10)
    world=client.get_world()
    if world.get_map().name.rsplit('/',1)[-1] not in ('Town05','Town05_Opt'):
        raise RuntimeError('Town05 is required')
    actors=world.get_actors()
    if actors.filter('vehicle.*') or actors.filter('walker.pedestrian.*'):
        raise RuntimeError('Calibration requires an empty world; no actors will be removed')
    original=world.get_settings()
    if original.synchronous_mode:
        raise RuntimeError('Existing synchronous tick owner; refusing calibration')
    bus=sensor=None;collisions=[];active=[False]
    result={'purpose':'open_loop_pedal_calibration_not_autoware_validation','cases':[]}
    try:
        settings=world.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=.05
        world.apply_settings(settings)
        native=world.get_map();wp=native.get_waypoint_xodr(37,-2,330)
        origin=wp.transform;origin.location.z+=.4
        bp=world.get_blueprint_library().find('vehicle.mitsubishi.fusorosa')
        bp.set_attribute('role_name','bus_actuation_probe')
        bus=world.spawn_actor(bp,origin)
        sensor=world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=bus)
        sensor.listen(lambda e:collisions.append({'frame':e.frame,'actor':e.other_actor.type_id}) if active[0] else None)
        forward=wp.transform.get_forward_vector()
        world.get_spectator().set_transform(carla.Transform(
            origin.location+carla.Location(x=20,y=-20,z=12),carla.Rotation(pitch=-25,yaw=135)))
        def speed():
            v=bus.get_velocity();return v.x*forward.x+v.y*forward.y+v.z*forward.z
        def tick(pedal,brake=0):
            bus.apply_control(carla.VehicleControl(throttle=pedal,brake=brake))
            world.tick()
        controls=[('brake',x) for x in (0,.005,.01,.02,.04,.06,.1,.2,.4,.8)]+[('throttle',x) for x in (.1,.2,.3,.4,.5,.6)]
        if extended: controls += [('throttle',x) for x in (.7,.8,.9,1.)]
        for target in speeds:
            for kind,pedal in controls:
                active[0]=False
                for _ in range(30):tick(0,1)
                bus.set_transform(origin)
                bus.set_target_velocity(carla.Vector3D())
                bus.set_target_angular_velocity(carla.Vector3D())
                for _ in range(30):tick(0,1)
                reached=False
                for _ in range(400):
                    tick(acceleration_pedal)
                    if speed()>=target:
                        reached=True;break
                if not reached:raise RuntimeError('Bus failed to accelerate to calibration speed')
                active[0]=True
                before=len(collisions);samples=[]
                for i in range(21):
                    snap=world.get_snapshot()
                    a=bus.get_acceleration();v=speed()
                    samples.append({'t':snap.timestamp.elapsed_seconds,'speed_mps':v,
                        'acceleration_mps2':a.x*forward.x+a.y*forward.y+a.z*forward.z,
                        'gear':bus.get_control().gear,'x':bus.get_location().x,'y':bus.get_location().y})
                    tick(pedal if kind=='throttle' else 0,pedal if kind=='brake' else 0)
                active[0]=False
                # Skip the first 0.1s of actuator transition. Exclude stopped
                # samples: zero speed cannot reveal additional brake torque.
                values=[(b['speed_mps']-a['speed_mps'])/(b['t']-a['t'])
                        for a,b in zip(samples[2:10],samples[3:11]) if b['speed_mps']>.5]
                record={'kind':kind,'pedal':pedal,'target_speed_mps':target,
                        'mean_acceleration_mps2':statistics.mean(values) if values else None,
                        'samples':samples,'collisions':collisions[before:]}
                result['cases'].append(record)
                output.write_text(json.dumps(result,indent=2)+'\n')
                print(kind,pedal,'speed',target,'accel',record['mean_acceleration_mps2'],flush=True)
                if record['collisions']:raise RuntimeError('Collision during calibration')
        result['complete']=True
    finally:
        active[0]=False
        if sensor:sensor.stop();sensor.destroy()
        if bus:bus.destroy()
        world.apply_settings(original)
        result['cleanup']={'actors_destroyed':True,'original_world_settings_restored':True}
        output.write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--speeds',type=float,nargs='+',default=[2.78,5.56])
    parser.add_argument('--acceleration-pedal',type=float,default=.65)
    parser.add_argument('--extended',action='store_true')
    args=parser.parse_args();args.output.parent.mkdir(parents=True,exist_ok=True)
    if any(not math.isfinite(v) or not 0<v<=14 for v in args.speeds):
        parser.error('Calibration speeds must be in (0,14] m/s')
    if not 0<args.acceleration_pedal<=1:parser.error('Invalid acceleration pedal')
    run(args.output,args.speeds,args.acceleration_pedal,args.extended)
