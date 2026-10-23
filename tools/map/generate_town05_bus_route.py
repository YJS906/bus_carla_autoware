#!/usr/bin/env python3
"""Generate an offline bus mission reference route, not a control trajectory.

Uses CARLA's directed topology, never connects to or ticks a running world.
The output's lane changes are route requirements for Autoware to plan, not
collision-checked trajectories. Stop service is conditional, not an OSM stop sign.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import carla
from shapely.geometry import Point, Polygon

ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_polygons(path):
    root = ET.parse(path).getroot()
    def tags(e):
        return {t.get('k'): t.get('v') for t in e.findall('tag')}
    nodes = {n.get('id'): (float(tags(n)['local_x']), float(tags(n)['local_y']))
             for n in root.findall('node')}
    ways = {w.get('id'): [nodes[n.get('ref')] for n in w.findall('nd')]
            for w in root.findall('way')}
    roads, crossings = {}, []
    for rel in root.findall('relation'):
        t = tags(rel)
        if t.get('type') == 'lanelet' and t.get('subtype') == 'road':
            left = ways[rel.find("member[@role='left']").get('ref')]
            right = ways[rel.find("member[@role='right']").get('ref')]
            if math.dist(left[0], right[0]) > math.dist(left[0], right[-1]):
                right = right[::-1]
            roads[int(rel.get('id'))] = Polygon(left + right[::-1])
        if t.get('type') == 'regulatory_element' and t.get('subtype') == 'crosswalk':
            crossings.append(Polygon(ways[rel.find("member[@role='crosswalk_polygon']").get('ref')]))
    if not roads or not crossings:
        raise ValueError('Road and crosswalk geometry are required')
    return roads, crossings


def pose(w):
    t = w.transform
    return {'road_id': w.road_id, 'section_id': w.section_id, 'lane_id': w.lane_id,
            's_m': round(w.s, 6), 'lane_width_m': round(w.lane_width, 4),
            'junction': w.is_junction,
            'carla': {'x': t.location.x, 'y': t.location.y, 'z': t.location.z,
                      'yaw_deg': t.rotation.yaw},
            'ros_map': {'x': t.location.x, 'y': -t.location.y, 'z': t.location.z,
                        'yaw_rad': math.atan2(math.sin(math.radians(-t.rotation.yaw)),
                                              math.cos(math.radians(-t.rotation.yaw)))}}


def validate_stop(w, roads, crossings):
    if w.is_junction or w.lane_type != carla.LaneType.Driving:
        raise ValueError('Stop must be on a non-junction driving lane')
    right = w.get_right_lane()
    if right and right.lane_type == carla.LaneType.Driving and right.lane_id * w.lane_id > 0:
        raise ValueError('Stop must use the outermost right driving lane')
    xy = Point(w.transform.location.x, -w.transform.location.y)
    lanelets = [rid for rid, poly in roads.items() if poly.covers(xy)]
    if len(lanelets) != 1:
        raise ValueError(f'Expected exactly one Lanelet2 road at stop, found {lanelets}')
    crosswalk_distance = min(p.distance(xy) for p in crossings)
    if crosswalk_distance < 10:
        raise ValueError(f'Stop reference is too near a crosswalk: {crosswalk_distance:.2f}m')
    # Require only a short road section, not a pull-out or parking bay.
    for distance in (3, 6, 9):
        for neighbours in (w.next(distance), w.previous(distance)):
            if len(neighbours) != 1 or neighbours[0].is_junction:
                raise ValueError('Stop reference lies within 9m of a junction')
    return {'lanelet_id': lanelets[0], 'non_junction': True, 'outermost_driving_lane': True,
            'crosswalk_distance_m': round(crosswalk_distance, 3),
            'junction_free_front_and_rear_m': 9,
            'bus_swept_envelope_verified': False, 'closed_loop_driving_verified': False}


def generate(args):
    sys.path.insert(0, str(args.carla_root / 'PythonAPI/carla'))
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    source = json.loads(args.spec.read_text())
    ids = [a['id'] for a in source['anchors']]
    assert len(ids) == len(set(ids))
    assert sum(a['kind'] == 'stop' for a in source['anchors']) == 10
    native = carla.Map('Town05', args.xodr.read_text())
    planner = GlobalRoutePlanner(native, 1.0)
    roads, crossings = read_polygons(args.lanelet_map)
    anchors = []
    for a in source['anchors']:
        w = native.get_waypoint_xodr(a['road_id'], a['lane_id'], a['s'])
        if w is None:
            raise ValueError(f'Unknown native waypoint {a}')
        record = {**a, **pose(w)}
        xy = Point(w.transform.location.x, -w.transform.location.y)
        containing = [rid for rid, poly in roads.items() if poly.covers(xy)]
        if len(containing) != 1:
            raise ValueError(f'Expected one lanelet at anchor {a["id"]}: {containing}')
        record['lanelet_id'] = containing[0]
        if a['kind'] == 'stop':
            record['validation'] = validate_stop(w, roads, crossings)
            record['stop_reference'] = 'road_centreline_reference; calibrate base_link offset with bus profile'
            record['service_rule'] = 'alighting_request OR waiting_passenger'
        anchors.append((record, w))

    segments = []
    distance = 0.0
    max_step = 0.0
    for i, (a, start) in enumerate(anchors):
        b, goal = anchors[(i + 1) % len(anchors)]
        a['route_distance_m'] = round(distance, 3)
        if start.road_id == goal.road_id == 37 and start.s < goal.s:
            # This is a visual/topological reference, not the lateral trajectory.
            # Put graph lane transitions on the same straights used by the runner.
            trace = []
            lane = start.lane_id
            transitions = [(80.,-3),(700.,-2),(745.,-1)]
            samples_s = sorted(set([start.s,goal.s] +
                list(range(math.ceil(start.s),math.floor(goal.s)+1,2)) +
                [s for s,_ in transitions if start.s < s < goal.s]))
            for s in samples_s:
                trace.append((native.get_waypoint_xodr(37,lane,s), 'LANEFOLLOW'))
                for change_s,target in transitions:
                    if s == change_s and start.s < s < goal.s and lane != target:
                        wp = native.get_waypoint_xodr(37,target,s)
                        if abs(target-lane) != 1:
                            raise ValueError('Ring reference must change one lane at a time')
                        trace.append((wp, 'CHANGELANERIGHT' if target < lane else 'CHANGELANELEFT'))
                        lane = target
            if lane != goal.lane_id:
                raise ValueError('Ring reference does not reach the requested lane')
        else:
            trace = planner.trace_route(start.transform.location, goal.transform.location)
        if not trace:
            raise ValueError(f"No directed route: {a['id']} to {b['id']}")
        # Native planner sampling can start just behind the requested origin.
        # Clip only that same-lane prefix, never a topology transition.
        forward = start.transform.get_forward_vector()
        while trace:
            first = trace[0][0]
            delta = first.transform.location - start.transform.location
            if ((first.road_id, first.lane_id) == (start.road_id, start.lane_id)
                    and delta.x * forward.x + delta.y * forward.y < -0.01):
                trace.pop(0)
            else:
                break
        if not trace:
            raise ValueError('Native route contains only samples behind its start')
        # The reference includes exact anchors at both ends. This does not
        # convert graph lane-change edges into drivable lateral trajectories.
        samples = [(start, 'ANCHOR')] + [(w, getattr(option,'name',option)) for w, option in trace] + [(goal, 'ANCHOR')]
        points = []
        for w, option in samples:
            value = pose(w)
            if points:
                prev = points[-1]['carla']
                now = value['carla']
                gap = math.dist([prev[k] for k in ('x','y','z')], [now[k] for k in ('x','y','z')])
                if gap < 0.05:
                    continue
                yaw = math.radians(prev['yaw_deg'])
                along = (now['x']-prev['x'])*math.cos(yaw)+(now['y']-prev['y'])*math.sin(yaw)
                if along < -0.1:
                    raise ValueError(f"Backward reference sample in {a['id']} -> {b['id']}")
                max_step = max(max_step, gap)
                distance += gap
            value.update(route_distance_m=round(distance, 3), road_option=option)
            points.append(value)
        gap_to_goal = trace[-1][0].transform.location.distance(goal.transform.location)
        if gap_to_goal > 4:
            raise ValueError(f"Planner did not reach anchor {b['id']}: {gap_to_goal:.2f}m")
        segments.append({'from': a['id'], 'to': b['id'],
                         'phase': 'ring_road' if a['id'] in ('R02','R03','R04','R05') else
                                  'connector' if a['kind'] == 'via' or b['kind'] == 'via' else 'urban',
                         'length_m': round(distance - a['route_distance_m'], 3),
                         'requires_lane_change': any('CHANGELANE' in p['road_option'] for p in points),
                         'points': points})
    if max_step > 20:
        raise ValueError(f'Unexpected topology discontinuity: {max_step:.2f}m')
    stops = [a for a, _ in anchors if a['kind'] == 'stop']
    for i, stop in enumerate(stops):
        stop['distance_to_next_stop_m'] = round(
            (stops[(i+1) % len(stops)]['route_distance_m'] - stop['route_distance_m']) % distance, 3)

    result = {'schema': 1, 'name': source['name'], 'map': source['map'],
              'closed_loop': True, 'status': 'offline_topology_validated_not_driving_validated',
              'coordinate_transform': 'ROS map=(CARLA x, -CARLA y, CARLA z); yaw_ros=-yaw_carla',
              'route_reference_only': True, 'length_m': round(distance, 3),
              'ring_lane_policy': source.get('ring_lane_policy'),
              'source_sha256': {'spec': digest(args.spec), 'xodr': digest(args.xodr),
                                'lanelet_map': digest(args.lanelet_map)},
              'stops': stops, 'anchors': [a for a, _ in anchors], 'segments': segments,
              'validation': {'stop_count': len(stops), 'directed_segments': len(segments),
                             'max_reference_sample_gap_m': round(max_step, 3),
                             'all_stops_in_urban_roads': all(s['road_id'] not in (12,34,35,36,37,38) for s in stops),
                             'no_stop_signs_added_to_map': True,
                             'autoware_route_handler_validated': False,
                             'live_driving_validated': False}}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'town05_bus_route.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    with (args.output/'town05_bus_stops.csv').open('w', newline='') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(['id','name','road_id','lane_id','lanelet_id','carla_x','carla_y','carla_z','carla_yaw_deg',
                         'ros_x','ros_y','ros_yaw_rad','route_distance_m','next_stop_distance_m'])
        for s in stops:
            c, r = s['carla'], s['ros_map']
            writer.writerow([s['id'],s['name'],s['road_id'],s['lane_id'],s['validation']['lanelet_id'],
                             c['x'],c['y'],c['z'],c['yaw_deg'],r['x'],r['y'],r['yaw_rad'],
                             s['route_distance_m'],s['distance_to_next_stop_m']])
    draw(native, result, args.output/'town05_bus_route.png')
    print(json.dumps({'length_m': result['length_m'], 'validation':result['validation'],
                      'stops': [{'id':s['id'],'name':s['name'],**s['validation']} for s in stops]},
                     ensure_ascii=False, indent=2))


def draw(native, result, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = 'Noto Sans CJK KR'
    plt.rcParams['axes.unicode_minus'] = False
    fig, ax = plt.subplots(figsize=(14, 11), facecolor='#f7f9fc')
    ax.set_facecolor('#f7f9fc')
    ws = native.generate_waypoints(2)
    ax.scatter([w.transform.location.x for w in ws],[-w.transform.location.y for w in ws],
               s=1.8,c='#d5dce5',zorder=1)
    colors = {'urban':'#176ac1', 'ring_road':'#e88026', 'connector':'#269d81'}
    for segment in result['segments']:
        pts = segment['points']
        xs, ys = [p['ros_map']['x'] for p in pts], [p['ros_map']['y'] for p in pts]
        color = colors[segment['phase']]
        ax.plot(xs,ys,c=color,lw=2.8,zorder=2)
        if len(xs)>8:
            i = len(xs)//2
            ax.annotate('',xy=(xs[i+4],ys[i+4]),xytext=(xs[i],ys[i]),
                        arrowprops={'arrowstyle':'->','color':color,'lw':2.5},zorder=3)
    for s in result['stops']:
        x,y=s['ros_map']['x'],s['ros_map']['y']
        ax.scatter(x,y,s=200,c='white',edgecolors='#16324f',linewidths=1.6,zorder=5)
        ax.text(x,y,s['id'][1:],ha='center',va='center',fontsize=9,fontweight='bold',zorder=6)
        ax.annotate(s['name'],(x,y),xytext=(12,8),textcoords='offset points',fontsize=11,
                    color='#16324f',zorder=6,
                    bbox={'facecolor':'#f7f9fc','edgecolor':'none','pad':1.5,'alpha':.9})
    from matplotlib.lines import Line2D
    handles=[Line2D([0],[0],color=c,lw=3,label=t) for c,t in
             [('#176ac1','도심: 정류장 10개'),('#e88026','서부 순환로: 정류장 없음'),('#269d81','도심 ↔ 순환로 연결')]]
    ax.legend(handles=handles,loc='upper right',frameon=True,fontsize=11)
    ax.set_title(f"Town05 도심·서부 순환 버스 | 정류장 10개 · 약 {result['length_m']/1000:.2f} km",fontsize=18,pad=20)
    ax.set_xlabel('ROS map X (m)',fontsize=11);ax.set_ylabel('ROS map Y (m)',fontsize=11)
    ax.set_aspect('equal');ax.grid(alpha=.16)
    fig.text(.5,.02,'01 → 05 도심 운행 → 북쪽 진입 · 서부 순환로 · 남쪽 복귀 → 06 → 10 → 01\n'
             '정류장은 바깥 차로 중앙에 정차 · 지도/차로 기반 배치 검증 · 버스 실제 주행은 후속 검증',
             ha='center',fontsize=11,color='#42566b')
    fig.subplots_adjust(bottom=.10)
    fig.savefig(output,dpi=150,bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--carla-root',type=Path,default=Path('/home/a/CARLA/0.9.16'))
    parser.add_argument('--xodr',type=Path)
    parser.add_argument('--lanelet-map',type=Path,default=Path('/home/a/autoware_data/maps/Town05/lanelet2_map.osm'))
    parser.add_argument('--spec',type=Path,default=ROOT/'config/carla/bus/town05_route_spec.json')
    parser.add_argument('--output',type=Path,default=ROOT/'scenarios/bus')
    args=parser.parse_args()
    if args.xodr is None:
        args.xodr=args.carla_root/'CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr'
    generate(args)
