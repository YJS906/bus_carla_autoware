#!/usr/bin/env python3
"""Recover proven shared dashed boundaries missed by whole-lane matching.

Town05 boundaries 6522/2087 separate road 25/8 lanes +1/+2. The imported lanelet
polygons end at slightly different native road sections. Match the shared
boundary itself, verify BOTH lanes and every native section's full marking.
No geometry or existing boundary permissions are changed.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

import carla
from shapely.geometry import LineString
from repair_town05_lane_changes import constant_broken_both, native_marking_lane, tags

SOURCE = 'native_shared_boundary_seam_v1'
BOUNDARIES = {6522: (25, 0, 1, 2), 2087: (8, 0, 1, 2)}


def reviewed_marking(marking, road_id):
    if constant_broken_both(marking):
        return True
    # Road 8 has an explicitly laneChange=both unpainted 13.84 cm seam at s=0.
    # It is not a solid/prohibited marking. All sampled interior points must
    # still be dashed; never generalise this exception to another road/length.
    marks = marking.findall('roadMark') if marking is not None else []
    return (road_id == 8 and len(marks) == 2 and
            marks[0].get('type') == 'none' and marks[0].get('laneChange') == 'both' and
            float(marks[0].get('sOffset', '-1')) == 0 and
            marks[1].get('type') == 'broken' and marks[1].get('laneChange') == 'both' and
            0 < float(marks[1].get('sOffset', '-1')) <= .15)


def candidate(text, xodr_text):
    root, source = ET.fromstring(text), ET.fromstring(xodr_text)
    native = carla.Map('Town05', xodr_text)
    nodes = {n.get('id'): (float(tags(n)['local_x']), -float(tags(n)['local_y']),
                           float(tags(n).get('ele', 0.))) for n in root.findall('node')}
    roads = {int(r.get('id')):r for r in source.findall('road')}
    proof = []
    for way_id in BOUNDARIES:
        way = root.find(f"way[@id='{way_id}']")
        users = [(r, m.get('role')) for r in root.findall('relation')
                 if tags(r).get('subtype')=='road' for m in r.findall('member')
                 if m.get('type')=='way' and m.get('ref')==str(way_id)]
        if way is None or len(users)!=2 or {role for _,role in users}!={'left','right'}:
            raise ValueError('Expected a shared same-direction road boundary')
        existing = tags(way)
        expected = {'type':'line_thin','subtype':'dashed','lane_change':'yes',
                    'carla:lane_marking_source':SOURCE}
        if existing and existing!=expected:
            raise ValueError(f'Preserve existing boundary {way_id} tags: {existing}')
        line = LineString([nodes[n.get('ref')] for n in way.findall('nd')])
        if line.length<1.:
            raise ValueError('Boundary too short for a reliable seam match')
        keys, max_error = set(), 0.
        count = math.ceil((line.length-.5)/.25)
        for i in range(count+1):
            s = .25+(line.length-.5)*i/count
            p = line.interpolate(s)
            a,b = line.interpolate(s-.05),line.interpolate(s+.05)
            dx,dy = b.x-a.x,b.y-a.y
            norm = math.hypot(dx,dy)
            nx,ny = -dy/norm,dx/norm
            lanes = [native.get_waypoint(carla.Location(p.x+side*1.75*nx,
                     p.y+side*1.75*ny,p.z),lane_type=carla.LaneType.Driving)
                     for side in (-1,1)]
            if any(w is None or w.is_junction for w in lanes):
                raise ValueError('Unmatched/junction shared boundary')
            l,r = sorted(lanes,key=lambda w:abs(w.lane_id))
            if ((l.road_id,l.section_id)!=(r.road_id,r.section_id) or
                    l.lane_id*r.lane_id<=0 or abs(l.lane_id-r.lane_id)!=1):
                raise ValueError('Boundary is not between native same-direction adjacent lanes')
            section = roads[l.road_id].findall('./lanes/laneSection')[l.section_id]
            marking = native_marking_lane(section,l.lane_id,r.lane_id)
            if not reviewed_marking(marking, l.road_id):
                raise ValueError('Entire native section must explicitly allow dashed/both')
            if (l.right_lane_marking.type != carla.LaneMarkingType.Broken or
                    l.right_lane_marking.lane_change != carla.LaneChange.Both):
                raise ValueError('Sampled native boundary must be dashed and allow both directions')
            edge = l.transform.location+l.transform.get_right_vector()*(l.lane_width/2)
            error = math.hypot(edge.x-p.x,edge.y-p.y)
            if error>.25 or abs(edge.z-p.z)>.25:
                raise ValueError('Shared boundary does not match native lane edge')
            max_error=max(max_error,error)
            keys.add((l.road_id,l.section_id,l.lane_id,r.lane_id))
        # Endpoint exclusion covers the map import seam, not a marking exemption:
        # the ENTIRE matched native lane sections have already been checked.
        if keys!={BOUNDARIES[way_id]}:
            raise ValueError(f'Unexpected native source for reviewed boundary: {keys}')
        proof.append(dict(way_id=way_id,native_sections=sorted(keys),samples=count+1,
                          max_boundary_error_m=max_error,endpoint_inset_m=.25,
                          native_unpainted_both_seam_m=.13841272397161219 if way_id==2087 else 0.))
        if not existing:
            pattern = rf'<way\b(?=[^>]*\bid="{way_id}")[^>]*>.*?</way>'
            replacement = ''.join(f'<tag k="{k}" v="{v}" />' for k,v in expected.items())
            text,n = re.subn(pattern,lambda m:m.group(0).replace('</way>',replacement+'</way>'),text,flags=re.S)
            if n!=1:raise ValueError('Expected one boundary block')
    return text,proof


def install(directory,xodr):
    path=directory/'lanelet2_map.osm'
    original=path.read_text()
    updated,proof=candidate(original,xodr.read_text())
    if original==updated:
        print('Proven shared lane seam permissions already present')
        return
    for name in ('selfcar-carla-autoware','selfcar-carla-signals'):
        status=subprocess.run(['docker','inspect','-f','{{.State.Running}}',name],capture_output=True,text=True)
        if status.returncode==0 and status.stdout.strip()=='true':
            raise RuntimeError('Stop runtime before changing map permissions')
    names=('lanelet2_map.osm','carla_traffic_signals.json','carla_map_report.json')
    backup=directory/'backups'/('bus-lane-seams-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    backup.mkdir(parents=True)
    for name in names:shutil.copy2(directory/name,backup/name)
    sha=hashlib.sha256(updated.encode()).hexdigest()
    try:
        path.write_text(updated)
        for name in names[1:]:
            data=json.loads((directory/name).read_text());data['map_sha256']=sha
            (directory/name).write_text(json.dumps(data,indent=2)+'\n')
    except BaseException:
        for name in names:shutil.copy2(backup/name,directory/name)
        raise
    report=dict(source=SOURCE,map_sha256=sha,xodr_sha256=hashlib.sha256(xodr.read_bytes()).hexdigest(),
                geometry_modified=False,boundaries=proof,backup=str(backup))
    (directory/'carla_bus_lane_seam_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map-dir',type=Path,required=True);p.add_argument('--xodr',type=Path,required=True)
    a=p.parse_args();install(a.map_dir,a.xodr)
