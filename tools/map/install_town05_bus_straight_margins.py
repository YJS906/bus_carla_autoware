#!/usr/bin/env python3
"""Install measured straight shoulder metadata without moving any map point."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
SOURCE='carla_paved_straight_v2'

def tags(e):return {t.get('k'):t.get('v') for t in e.findall('tag')}
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def candidate(text,proof):
    root=ET.fromstring(text);nodes={n.get('id'):n for n in root.findall('node')}
    if proof['geometry_modified'] or proof['routing_permissions_modified']:
        raise ValueError('Expected metadata-only proof')
    # Remove only metadata owned by this installer, including withdrawn proof.
    for way in root.findall('way'):
        if tags(way).get('bus:margin_source') not in (SOURCE,'carla_paved_straight_v1'):continue
        key='bus:margin:'+way.get('id')
        for ref in way.findall('nd'):
            node=nodes[ref.get('ref')]
            for tag in list(node.findall('tag')):
                if tag.get('k')==key:node.remove(tag)
        for tag in list(way.findall('tag')):
            if tag.get('k') in ('bus:margin_source','bus:outward_sign'):way.remove(tag)
    def set_tag(element,key,value):
        old=element.find(f"tag[@k='{key}']")
        if old is None:ET.SubElement(element,'tag',k=key,v=str(value))
        else:old.set('v',str(value))
    for record in proof['proof']:
        wid=str(record['way_id']);way=root.find(f"way[@id='{wid}']")
        if way is None:raise ValueError('Missing boundary: '+wid)
        attr=tags(way);painted=record.get('boundary_kind')=='painted_divider'
        if painted:
            if (attr.get('type'),attr.get('subtype'),attr.get('carla:bus_boundary_source'))!=('line_thin','solid','native_nonjunction_edges_v1'):
                raise ValueError('Painted divider changed: '+wid)
        elif attr.get('type')!='road_border':raise ValueError('Boundary changed: '+wid)
        vertices=record['vertices'];ids=[str(v['id']) for v in vertices]
        if ids!=[n.get('ref') for n in way.findall('nd')]:raise ValueError('Boundary geometry changed: '+wid)
        if record['outward_sign'] not in ((0,) if painted else (-1,1)):raise ValueError('Invalid outward normal')
        for v in vertices:
            node=nodes[str(v['id'])];t=tags(node)
            xyz=[float(t.get(k,0)) for k in ('local_x','local_y','ele')]
            if any(abs(a-b)>1e-6 for a,b in zip(xyz,v['xyz'])):raise ValueError('Map point changed')
            if not 0<=v['margin_m']<=(.5 if painted else .65):raise ValueError('Unverified margin')
            set_tag(node,'bus:margin:'+wid,f"{v['margin_m']:.6f}")
        set_tag(way,'bus:margin_source',SOURCE)
        set_tag(way,'bus:outward_sign',record['outward_sign'])
    return ET.tostring(root,encoding='unicode')


def install(directory,proof_path):
    path=directory/'lanelet2_map.osm';text=path.read_text();proof=json.loads(proof_path.read_text())
    updated=candidate(text,proof)
    # Compare parsed XML because an XML declaration is not a map change.
    if ET.tostring(ET.fromstring(text))==ET.tostring(ET.fromstring(updated)):
        print('Verified straight paved margins already present');return
    for name in ('selfcar-carla-autoware','selfcar-carla-signals'):
        r=subprocess.run(['docker','inspect','-f','{{.State.Running}}',name],capture_output=True,text=True)
        if r.returncode==0 and r.stdout.strip()=='true':raise RuntimeError('Stop runtime before installing map metadata')
    names=('lanelet2_map.osm','carla_traffic_signals.json','carla_map_report.json')
    backup=directory/'backups'/('bus-straight-margins-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    backup.mkdir(parents=True)
    for name in names:shutil.copy2(directory/name,backup/name)
    try:
        path.write_text(updated)
        for name in names[1:]:
            data=json.loads((directory/name).read_text());data['map_sha256']=digest(path)
            (directory/name).write_text(json.dumps(data,indent=2)+'\n')
    except BaseException:
        for name in names:shutil.copy2(backup/name,directory/name)
        raise
    report=dict(source=SOURCE,map_sha256=digest(path),proof_sha256=digest(proof_path),
                geometry_modified=False,routing_permissions_modified=False,
                boundary_count=len(proof['proof']),backup=str(backup))
    (directory/'carla_bus_straight_margin_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--map-dir',type=Path,required=True)
    p.add_argument('--proof',type=Path,default=ROOT/'config/carla/bus/town05_straight_margins.json')
    a=p.parse_args();install(a.map_dir,a.proof)
