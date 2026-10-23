#!/usr/bin/env python3
"""Measure paved straight-edge room in the live Town05 mesh; add metadata only.

Lane coordinates, widths, routing permissions and curve boundaries are unchanged.
The optional consumer uses this room for a wide vehicle's body, never as a lane.
"""
import argparse
from collections import defaultdict, Counter
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import carla
from repair_town05_lane_changes import tags, match_lane

SOURCE = 'carla_paved_straight_v2'


def project_lane(native, location, keys=None):
    # get_waypoint uses a sampled road index (up to ~0.2m along-road error).
    # Refine s before comparing mesh/map points; never relax lateral matching.
    wp = native.get_waypoint(location, lane_type=carla.LaneType.Driving)
    if wp and keys and (wp.road_id,wp.section_id,wp.lane_id) not in keys:
        key=next((k for k in keys if k[:2]==(wp.road_id,wp.section_id)),None)
        if key is None:return None
        wp=native.get_waypoint_xodr(key[0],key[2],wp.s)
    for _ in range(2):
        if wp is None: return None
        f = wp.transform.get_forward_vector()
        d = location-wp.transform.location
        ds = (d.x*f.x+d.y*f.y)*(1 if wp.lane_id < 0 else -1)
        refined = native.get_waypoint_xodr(wp.road_id,wp.lane_id,wp.s+ds)
        if refined is None: return wp
        wp = refined
    return wp


def straight_room(world, native, wp, side, road_lengths, allow_painted_divider=False):
    """Body clearance over verified pavement, never a new routing lane."""
    adjacent = getattr(wp, 'get_'+side+'_lane')()
    painted = bool(allow_painted_divider and side=='left' and adjacent and
                   adjacent.lane_type==carla.LaneType.Driving and wp.lane_id*adjacent.lane_id<0)
    if not adjacent or (adjacent.lane_type != carla.LaneType.Shoulder and not painted):
        return 0.
    # Native shoulder can contain grass/raised structures: mesh rays are mandatory.
    width = .5 if painted else min(.65, adjacent.lane_width-.05)
    if width < .1 or wp.is_junction:
        return 0.
    yaw = wp.transform.rotation.yaw
    # A short urban straight is not a curve merely because its road ID ends.
    # Measure geometry within the road; taper measured margins before bends.
    for distance in (2., 4., 8.):
        for sign in (-1,1):
            s=max(.01,min(road_lengths[wp.road_id]-.01,wp.s+sign*distance))
            if abs(s-wp.s)<.1:continue
            near=native.get_waypoint_xodr(wp.road_id,wp.lane_id,s)
            if near is None or near.is_junction:return 0.
            delta = abs((near.transform.rotation.yaw-yaw+180)%360-180)
            if math.radians(delta)/abs(s-wp.s) > .0015:
                return 0.
    sign = 1 if side == 'right' else -1
    r = wp.transform.get_right_vector()
    for offset in [width*i/7 for i in range(8)]:
        point = wp.transform.location+r*(sign*(wp.lane_width/2+offset))
        hits = world.cast_ray(point+carla.Location(z=1), point-carla.Location(z=.2))
        if not hits or str(hits[0].label) not in ('Roads', 'RoadLines') or abs(hits[0].location.z-point.z) > .05:
            return 0.
    return width


def build(map_path, output, report_path):
    text=map_path.read_text();root=ET.fromstring(text)
    client=carla.Client('127.0.0.1',2000);client.set_timeout(10)
    world=client.get_world();native=world.get_map()
    road_lengths={int(r.get('id')):float(r.get('length')) for r in ET.fromstring(native.to_opendrive()).findall('road')}
    if native.name.rsplit('/',1)[-1] not in ('Town05','Town05_Opt'):
        raise ValueError('Live Town05 mesh is required')
    nodes={n.get('id'):n for n in root.findall('node')}
    xyz={i:tuple(float(tags(n).get(k,0)) for k in ('local_x','local_y','ele')) for i,n in nodes.items()}
    ways={w.get('id'):w for w in root.findall('way')};uses=defaultdict(list)
    for rel in root.findall('relation'):
        if tags(rel).get('subtype')!='road':continue
        bounds={m.get('role'):m.get('ref') for m in rel.findall('member') if m.get('role') in ('left','right')}
        if set(bounds)!={'left','right'}:continue
        for side,i in bounds.items():uses[i].append((side,bounds))
    proof=[];skipped=Counter()
    for way_id,records in uses.items():
        attr=tags(ways[way_id])
        painted=(attr.get('type')=='line_thin' and attr.get('subtype')=='solid' and
                 attr.get('carla:bus_boundary_source')=='native_nonjunction_edges_v1' and
                 all(side=='left' for side,_ in records))
        if not painted and (len(records)!=1 or attr.get('type')!='road_border'):continue
        side,bounds=records[0]
        points=[[xyz[n.get('ref')] for n in ways[bounds[s]].findall('nd')] for s in ('left','right')]
        keys,reason,_=match_lane(*points,native)
        if reason or not keys:skipped['unmatched']+=1;continue
        refs=[n.get('ref') for n in ways[way_id].findall('nd')]
        margins=[];orientation=None
        for j,node_id in enumerate(refs):
            x,y,z=xyz[node_id]
            wp=project_lane(native,carla.Location(x,-y,z),keys)
            if not wp or (wp.road_id,wp.section_id,wp.lane_id) not in keys:
                margins.append(0.);continue
            r=wp.transform.get_right_vector();sign=1 if side=='right' else -1
            edge=wp.transform.location+r*(sign*wp.lane_width/2)
            if math.hypot(edge.x-x,edge.y+y)>.12 or abs(edge.z-z)>.1:
                margins.append(0.);continue
            a,b=xyz[refs[max(0,j-1)]],xyz[refs[min(len(refs)-1,j+1)]]
            # Positive sign is the LEFT normal of the stored ROS linestring.
            dot=-(b[1]-a[1])*(r.x*sign)+(b[0]-a[0])*(-r.y*sign)
            this_sign=1 if dot>0 else -1
            if orientation is not None and orientation!=this_sign:raise ValueError('Reversed boundary')
            orientation=this_sign
            room=straight_room(world,native,wp,side,road_lengths,painted)
            if painted and room:
                room=min(room,straight_room(world,native,wp.get_left_lane(),'left',road_lengths,True))
            margins.append(room)
        # Also verify each interval at <=0.5m spacing; zero either endpoint if
        # there is a non-paved patch or a curve between map vertices.
        for j in range(len(refs)-1):
            if not min(margins[j:j+2]):continue
            a,b=xyz[refs[j]],xyz[refs[j+1]];count=max(1,math.ceil(math.dist(a,b)/.5))
            for k in range(1,count):
                t=k/count;x,y,z=[a[h]+t*(b[h]-a[h]) for h in range(3)]
                wp=project_lane(native,carla.Location(x,-y,z),keys)
                room=straight_room(world,native,wp,side,road_lengths,painted) if wp else 0.
                if painted and room:room=min(room,straight_room(world,native,wp.get_left_lane(),'left',road_lengths,True))
                if not wp or (wp.road_id,wp.section_id,wp.lane_id) not in keys or room<min(margins[j:j+2]):
                    margins[j]=margins[j+1]=0.;break
        # Taper into/out of the verified strip rather than a sharp bound step.
        for indices in (range(1,len(refs)),range(len(refs)-2,-1,-1)):
            step=-1 if isinstance(indices,range) and indices.step==1 else 1
            for j in indices:
                margins[j]=min(margins[j],margins[j+step]+.04*math.dist(xyz[refs[j]],xyz[refs[j+step]]))
        if max(margins,default=0)<.1:continue
        def set_tag(element,key,value):
            old=element.find(f"tag[@k='{key}']")
            if old is None:ET.SubElement(element,'tag',k=key,v=str(value))
            else:old.set('v',str(value))
        set_tag(ways[way_id],'bus:margin_source',SOURCE)
        set_tag(ways[way_id],'bus:outward_sign',0 if painted else orientation)
        for i,value in zip(refs,margins):set_tag(nodes[i],'bus:margin:'+way_id,f'{value:.6f}')
        proof.append(dict(way_id=int(way_id),native=keys,side=side,
                          max_margin_m=max(margins),verified_nodes=sum(v>0 for v in margins),
                          outward_sign=0 if painted else orientation,boundary_kind='painted_divider' if painted else 'road_edge',
                          vertices=[dict(id=int(i),xyz=xyz[i],margin_m=round(v,6)) for i,v in zip(refs,margins)]))
    ET.ElementTree(root).write(output,encoding='UTF-8',xml_declaration=True)
    report=dict(source_sha256=hashlib.sha256(text.encode()).hexdigest(),
                map_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
                xodr_sha256=hashlib.sha256(native.to_opendrive().encode()).hexdigest(),
                geometry_modified=False,routing_permissions_modified=False,
                mesh_labels=['Roads','RoadLines'],max_height_error_m=.05,max_margin_m=.65,
                straight_context_m=8,painted_divider_body_margin_max_m=.5,
                source=SOURCE,proof=proof,skipped=dict(skipped))
    report_path.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'boundaries':len(proof),'report':str(report_path)}))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--map',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--report',type=Path,required=True)
    a=p.parse_args()
    if a.map.resolve()==a.output.resolve():p.error('Use a separate candidate output')
    build(a.map,a.output,a.report)
