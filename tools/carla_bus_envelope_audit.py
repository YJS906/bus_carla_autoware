#!/usr/bin/env python3
"""Isolated CARLA bus geometry audit, not an Autoware driving test.

Requires an empty, asynchronous Town05 world. Owns ticks during measurement,
restores settings and removes only its own actors. Ray queries use CARLA's
overlap channel; dynamic prop contact tests independently check physical contact.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import time

import carla
import numpy as np


def xyz(p):
    return [p.x, p.y, p.z]


def record_transform(t):
    return {'xyz': xyz(t.location), 'rpy': [t.rotation.roll,t.rotation.pitch,t.rotation.yaw]}


def run(output):
    client=carla.Client('localhost',2000);client.set_timeout(10)
    world=client.get_world();world.wait_for_tick(5)
    original=world.get_settings()
    if original.synchronous_mode:
        raise RuntimeError('Stop the existing tick owner first')
    if world.get_actors().filter('vehicle.*') or world.get_actors().filter('walker.pedestrian.*'):
        raise RuntimeError('Requires an empty world')
    if world.get_map().name.rsplit('/',1)[-1] not in ('Town05','Town05_Opt'):
        raise RuntimeError('Requires Town05')
    owned=[];bus=None;contact_sensor=None;active_probe=[None];events=[]
    result={'purpose':'geometry_and_contact_audit_not_autoware_validation',
            'map':world.get_map().name,'grid_step_m':.2,'states':[], 'contacts':[],
            'complete':False}
    def save():output.write_text(json.dumps(result,indent=2)+'\n')
    def tick(n=1):
        for _ in range(n):world.tick()
    def local(t,p):return (np.array(t.get_inverse_matrix())@np.array([*xyz(p),1]))[:3].tolist()
    try:
        settings=world.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=.05
        world.apply_settings(settings)
        origin=world.get_map().get_waypoint_xodr(37,-2,330).transform;origin.location.z+=.3
        bp=world.get_blueprint_library().find('vehicle.mitsubishi.fusorosa');bp.set_attribute('role_name','bus_envelope_audit')
        bus=world.spawn_actor(bp,origin);owned.append(bus)
        bus.apply_control(carla.VehicleControl(brake=1,hand_brake=True));tick(60)
        origin=bus.get_transform()
        b=bus.bounding_box
        result['bounding_box']={'center':xyz(b.location),'extent':xyz(b.extent),'rotation':str(b.rotation)}
        result['origin']=record_transform(origin)
        physics=bus.get_physics_control()
        result['wheel_radius_m']=[w.radius/100 for w in physics.wheels]
        result['use_sweep_wheel_collision']=physics.use_sweep_wheel_collision
        contact_sensor=world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=bus)
        owned.append(contact_sensor)
        def on_contact(e):
            if e.other_actor.id==active_probe[0]:
                events.append({'frame':e.frame,'other_id':e.other_actor.id,'impulse':xyz(e.normal_impulse)})
        contact_sensor.listen(on_contact)
        camera=origin.location+carla.Location(z=20)
        world.get_spectator().set_transform(carla.Transform(camera,carla.Rotation(pitch=-90,yaw=origin.rotation.yaw)))

        for steer in [0.,-.5,.5,-1.,1.]:
            bus.apply_control(carla.VehicleControl(steer=steer,brake=1,hand_brake=True));tick(40)
            if bus.get_velocity().length()>.05:raise RuntimeError('Bus not stationary')
            t=bus.get_transform();inv=np.array(t.get_inverse_matrix())
            bones={n:record_transform(b) for n,b in zip(bus.get_bone_names(),bus.get_bone_relative_transforms())}
            state={'steer_input':steer,'front_angles_deg':[bus.get_wheel_steer_angle(k) for k in
                    [carla.VehicleWheelLocation.FL_Wheel,carla.VehicleWheelLocation.FR_Wheel]],
                   'bones':bones,'rays':[]}
            for x in np.arange(-5.6,4.81,.2):
                for z in np.arange(.2,4.41,.2):
                    for sign in [-1,1]:
                        start=t.transform(carla.Location(float(x),sign*5,float(z)))
                        end=t.transform(carla.Location(float(x),0,float(z)))
                        hits=[]
                        for h in world.cast_ray(start,end):
                            if str(h.label)=='Bus':
                                hits.append(float((inv@np.array([*xyz(h.location),1]))[1]))
                        if hits:state['rays'].append({'x':round(float(x),3),'z':round(float(z),3),'side':sign,'y_hits':hits})
            all_y=[y for r in state['rays'] for y in r['y_hits']]
            state['ray_y_bounds']=[min(all_y),max(all_y)]
            extremes=[]
            for sign in [-1,1]:
                rows=[r for r in state['rays'] if r['side']==sign]
                r=max(rows,key=lambda r:max(sign*y for y in r['y_hits']))
                extremes.append({k:r[k] for k in ['x','z','side','y_hits']})
            state['extreme_samples']=extremes
            result['states'].append(state);save()
            print(json.dumps({'steer':steer,'angles':state['front_angles_deg'],'ray_bounds':state['ray_y_bounds'],'extremes':extremes}),flush=True)

        # A moving small box tests contact response separately from overlap rays.
        boxbp=world.get_blueprint_library().find('static.prop.box01')
        probe=world.spawn_actor(boxbp,carla.Transform(origin.location+carla.Location(z=20)))
        owned.append(probe);probe.set_simulate_physics(False)
        pb=probe.bounding_box
        result['probe']={'blueprint':boxbp.id,'bbox_center':xyz(pb.location),'bbox_extent':xyz(pb.extent)}
        probe.destroy();owned.remove(probe)
        if max(pb.extent.x,pb.extent.y)>.6:raise RuntimeError('Probe too large for local contact test')
        # Multiple x/z locations include wheel centres, body and widest ray sample.
        front_x=result['states'][0]['bones']['Wheel_Front_Left']['xyz'][0]
        wheel_z=result['states'][0]['bones']['Wheel_Front_Left']['xyz'][2]
        for steer in [0.,-1.,1.]:
            state=next(s for s in result['states'] if s['steer_input']==steer)
            for side in [-1,1]:
                ex=next(s for s in state['extreme_samples'] if s['side']==side)
                for name,x,z in [('body',0.,1.8),('front_wheel',front_x,wheel_z),('widest_ray',ex['x'],ex['z'])]:
                    bus.set_transform(origin);bus.set_target_velocity(carla.Vector3D());bus.set_target_angular_velocity(carla.Vector3D())
                    bus.apply_control(carla.VehicleControl(steer=steer,brake=1,hand_brake=True));tick(40)
                    t=bus.get_transform();target_local=carla.Location(x-pb.location.x,side*2.7-pb.location.y,z-pb.location.z)
                    p=t.transform(target_local)
                    probe=world.spawn_actor(boxbp,carla.Transform(p,t.rotation));owned.append(probe)
                    probe.set_simulate_physics(True);probe.set_enable_gravity(False)
                    active_probe[0]=probe.id;events.clear()
                    right=t.get_right_vector();velocity=carla.Vector3D(-side*.4*right.x,-side*.4*right.y,0)
                    frames={};contact=None;stalled=0;free_flight=False;blocking_pose=None
                    for _ in range(120):
                        probe.set_target_velocity(velocity);frame=world.tick()
                        tr=probe.get_transform();center=tr.transform(carla.Location(pb.location.x,pb.location.y,pb.location.z))
                        v=probe.get_velocity()
                        inward_speed=-side*(v.x*right.x+v.y*right.y)
                        frames[frame]={'center_local':local(t,center),'transform':record_transform(tr),'bus_shift_m':bus.get_location().distance(t.location),
                                       'inward_speed_mps':inward_speed}
                        if inward_speed>.3:free_flight=True
                        if free_flight and inward_speed<.05:
                            stalled+=1
                            if stalled==1:blocking_pose=frames[frame]
                        else:stalled=0;blocking_pose=None
                        time.sleep(.002)  # allow the sensor callback for this frame
                        if events:
                            contact=events[0];break
                        if stalled>=10:break
                        if abs(frames[frame]['center_local'][1])<1.:break
                    event_pose=frames.get(contact['frame']) if contact else blocking_pose
                    row={'steer_input':steer,'side':side,'location':name,'x':x,'z':z,
                         'event':contact,'event_pose':event_pose,'last_pose':frames[max(frames)],
                         'contact_evidence':'collision_sensor' if contact else 'physical_blocking' if stalled>=10 else 'none',
                         'free_flight_verified':free_flight,'stalled_ticks':stalled,'frames':frames,
                         'nominal_probe_half_width_m':pb.extent.y,'approach_speed_mps':.4,
                         'tick_s':.05,'frames_elapsed':len(frames)}
                    if event_pose:
                        row['nominal_contact_bus_y']=event_pose['center_local'][1]-side*pb.extent.y
                    result['contacts'].append(row);save();print(json.dumps(row),flush=True)
                    active_probe[0]=None;probe.destroy();owned.remove(probe);tick(4)
                    if not contact and stalled<10:raise RuntimeError('Contact positive control failed: '+name)
        result['complete']=True
    finally:
        errors=[]
        if contact_sensor:
            try:contact_sensor.stop()
            except RuntimeError as e:errors.append(str(e))
        for actor in reversed(owned):
            try:actor.destroy()
            except RuntimeError as e:errors.append(str(e))
        world.apply_settings(original)
        result['cleanup']={'errors':errors,'original_settings_restored':True}
        save()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.parent.mkdir(parents=True,exist_ok=True)
    def interrupt(*_):raise RuntimeError('Audit interrupted')
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    run(args.output)
