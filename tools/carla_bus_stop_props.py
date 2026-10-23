#!/usr/bin/env python3
"""Place the ten Town05 shelters on native right-side sidewalks, without ticking CARLA."""
import argparse
import json
import math
import time
from pathlib import Path

import carla

ROOT = Path(__file__).resolve().parents[1]
ROLE_PREFIX = 'bus_mission_stop_'
# Measured static.prop.busstop local mesh box in CARLA 0.9.16.
BOX_CENTER = (.366892, .177230, 1.369453)
BOX_EXTENT = (1.938039, .946805, 1.369453)
# Small prop-only adjustments after inspecting the existing street furniture.
# Positive longitudinal follows traffic; positive lateral points away from road.
PLACEMENT_OFFSETS = {'S03': (-4.0, 0.0), 'S09': (0.0, -0.4)}


def plan(world, route):
    native = world.get_map()
    if native.name.rsplit('/',1)[-1] not in ('Town05','Town05_Opt'):
        raise ValueError('Town05 is required')
    if len(route['stops']) != 10 or len({s['id'] for s in route['stops']}) != 10:
        raise ValueError('Expected ten distinct stops')
    result = []
    for stop in route['stops']:
        lane = native.get_waypoint_xodr(stop['road_id'],stop['lane_id'],stop['s_m'])
        p = stop['carla']
        if lane is None or lane.transform.location.distance(carla.Location(p['x'],p['y'],p['z']))>.1:
            raise ValueError('Route/map mismatch at '+stop['id'])
        longitudinal, lateral = PLACEMENT_OFFSETS.get(stop['id'], (0.0, 0.0))
        if longitudinal:
            candidates = lane.next(longitudinal) if longitudinal > 0 else lane.previous(-longitudinal)
            candidates = [w for w in candidates if w.road_id == lane.road_id
                          and w.lane_id == lane.lane_id and not w.is_junction]
            if len(candidates) != 1:
                raise ValueError('Ambiguous shelter offset at '+stop['id'])
            lane = candidates[0]
        sidewalk = lane.get_right_lane()
        while sidewalk and sidewalk.lane_type != carla.LaneType.Sidewalk:
            if sidewalk.lane_type == carla.LaneType.Driving:
                raise ValueError('Stop is not in the outermost driving lane: '+stop['id'])
            sidewalk = sidewalk.get_right_lane()
        if sidewalk is None or sidewalk.lane_width/2-BOX_EXTENT[1]-abs(lateral) < .025:
            raise ValueError('No sufficiently wide right sidewalk at '+stop['id'])
        yaw = lane.transform.rotation.yaw
        f = carla.Rotation(yaw=yaw).get_forward_vector()
        r = carla.Rotation(yaw=yaw).get_right_vector()
        centre = sidewalk.transform.location + r*lateral
        hits = world.cast_ray(centre+carla.Location(z=3),centre-carla.Location(z=2))
        ground = next((h.location.z for h in hits if str(h.label)=='Sidewalks'),None)
        if ground is None:
            raise ValueError('Sidewalk surface ray missed at '+stop['id'])
        # Centre the mesh, not its offset asset origin, inside the sidewalk.
        loc = centre-f*BOX_CENTER[0]-r*BOX_CENTER[1]
        loc.z = ground+.005
        delta = centre-lane.transform.location
        road_clearance = delta.x*r.x+delta.y*r.y-BOX_EXTENT[1]-lane.lane_width/2
        if road_clearance < .3:
            raise ValueError('Shelter is too close to the driving lane: '+stop['id'])
        result.append(dict(id=stop['id'],name=stop['name'],blueprint='static.prop.busstop',
            role_name=ROLE_PREFIX+stop['id'],
            transform=dict(x=loc.x,y=loc.y,z=loc.z,yaw=yaw),
            sidewalk_width_m=sidewalk.lane_width,sidewalk_ground_z=ground,
            driving_lane_clearance_m=road_clearance,
            longitudinal_offset_m=longitudinal,lateral_offset_m=lateral,
            sidewalk_edge_margin_m=sidewalk.lane_width/2-BOX_EXTENT[1]-abs(lateral)))
    return result


def apply(world, placements):
    existing = {}
    for actor in world.get_actors():
        role = actor.attributes.get('role_name','')
        if role.startswith(ROLE_PREFIX):
            if role in existing:
                raise ValueError('Duplicate managed shelter: '+role)
            existing[role] = actor
    # A newly spawned, STOP-mode bus briefly falls onto its suspension. Wait
    # for that motion to settle, without issuing control or ticking the world.
    deadline = time.monotonic()+10
    stable_since = None
    while True:
        egos = [a for a in world.get_actors().filter('vehicle.*')
                if a.attributes.get('role_name') == 'ego_vehicle']
        stationary = all(a.get_velocity().length() < .1 for a in egos)
        now = time.monotonic()
        if stationary:
            if stable_since is None: stable_since = now
            if now-stable_since >= .5: break
        else:
            stable_since = None
        if now >= deadline: raise RuntimeError('Stop the bus before placing shelters')
        world.wait_for_tick(5)
    output=[]
    for p in placements:
        t=p['transform'];transform=carla.Transform(carla.Location(t['x'],t['y'],t['z']),carla.Rotation(yaw=t['yaw']))
        actor=existing.get(p['role_name'])
        action='unchanged'
        if actor:
            if actor.type_id != p['blueprint']:
                raise ValueError('Unexpected actor type for '+p['role_name'])
            before=actor.get_transform()
            yaw_delta=(before.rotation.yaw-t['yaw']+180)%360-180
            if before.location.distance(transform.location)>.01 or abs(yaw_delta)>.05:
                actor.set_transform(transform);action='updated'
        else:
            bp=world.get_blueprint_library().find(p['blueprint'])
            bp.set_attribute('role_name',p['role_name'])
            actor=world.try_spawn_actor(bp,transform)
            if actor is None:raise RuntimeError('Cannot spawn '+p['id'])
            actor.set_simulate_physics(False);action='created'
        output.append(dict(**p,actor_id=actor.id,action=action))
    world.wait_for_tick(10)  # Observe the bridge-owned tick; never tick the world here.
    for p in output:
        actor=world.get_actor(p['actor_id']);t=p['transform']
        if actor is None or actor.get_location().distance(carla.Location(t['x'],t['y'],t['z']))>.02:
            raise RuntimeError('Placement did not persist at '+p['id'])
    return output


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route',type=Path,default=ROOT/'scenarios/bus/town05_bus_route.json')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=2000)
    args=parser.parse_args()
    client=carla.Client(args.host,args.port);client.set_timeout(10)
    world=client.get_world();world.wait_for_tick(10)
    placements=plan(world,json.loads(args.route.read_text()))
    result=dict(map=world.get_map().name,stop_count=len(placements),world_ticks=0,
                vehicle_controls=0,dry_run=args.dry_run,
                shelters=placements if args.dry_run else apply(world,placements))
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
