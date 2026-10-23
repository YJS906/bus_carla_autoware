#!/usr/bin/env python3
"""Draw saved bus stops/reference route in an already running Town05 world.

Temporary visual markers only: no actors, world ticks, map edits or vehicle
control. Run again after markers expire or after restarting CARLA.
"""
import argparse
import json
import math
from pathlib import Path

import carla


def preview(client, route, duration):
    world = client.get_world()
    native = world.get_map()
    if native.name.rsplit('/', 1)[-1] not in ('Town05', 'Town05_Opt'):
        raise ValueError(f'Town05 required; current map: {native.name}')
    if len(route['stops']) != 10:
        raise ValueError('Expected the ten-stop bus route')
    # Check every marker before drawing any of them.
    for stop in route['stops']:
        w = native.get_waypoint_xodr(stop['road_id'], stop['lane_id'], stop['s_m'])
        p = stop['carla']
        if w is None or w.transform.location.distance(carla.Location(x=p['x'], y=p['y'], z=p['z'])) > .1:
            raise ValueError(f"Map mismatch at {stop['id']}")
    def loc(p, height=.2):
        return carla.Location(x=p['x'], y=p['y'], z=p['z']+height)
    colors = {'urban':carla.Color(30,130,255), 'ring_road':carla.Color(255,150,30),
              'connector':carla.Color(40,210,150)}
    for segment in route['segments']:
        pts = segment['points'][::5]
        if pts[-1] is not segment['points'][-1]:
            pts = pts + [segment['points'][-1]]
        for a,b in zip(pts,pts[1:]):
            world.debug.draw_line(loc(a['carla']),loc(b['carla']),thickness=.12,
                                  color=colors[segment['phase']],life_time=duration)
    for stop in route['stops']:
        p = stop['carla']
        yaw = math.radians(p['yaw_deg'])
        # A visual cross-bar at the reference position. It is not a stop-line
        # regulatory element: requests decide whether this station is served.
        dx,dy=-math.sin(yaw)*1.45,math.cos(yaw)*1.45
        a,b=loc(p),loc(p)
        a.x+=dx;a.y+=dy;b.x-=dx;b.y-=dy
        world.debug.draw_line(a,b,thickness=.20,color=carla.Color(255,220,20),life_time=duration)
        # Put the sign on the outer roadside, so it cannot look like a station
        # on the neighbouring lane when viewed from the following camera.
        sign = loc(p, .1)
        side = stop['lane_width_m']/2 + .45
        sign.x += -math.sin(yaw)*side
        sign.y += math.cos(yaw)*side
        top = carla.Location(sign.x, sign.y, sign.z+2.8)
        world.debug.draw_line(sign, top, thickness=.08,
                              color=carla.Color(255,220,20),life_time=duration)
        world.debug.draw_string(top,f"BUS {stop['id']} (outer lane)",draw_shadow=True,
                                color=carla.Color(255,220,20),life_time=duration)
    return {'map':native.name,'stop_markers':len(route['stops']), 'duration_seconds':duration,
            'actors_spawned':0,'vehicle_control_commands':0,'world_ticks':0}


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route',type=Path,default=Path(__file__).resolve().parents[1]/'scenarios/bus/town05_bus_route.json')
    parser.add_argument('--duration',type=float,default=120)
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=2000)
    args=parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 3600:
        parser.error('duration must be between 1 and 3600 seconds')
    c=carla.Client(args.host,args.port);c.set_timeout(10)
    print(json.dumps(preview(c,json.loads(args.route.read_text()),args.duration),indent=2))
