#!/usr/bin/env python3
"""Add proven non-junction driving-road edges to a separate candidate bus map.

Never classify an internal same-direction lane boundary as a road edge. Native
matching rejects junctions, mismatched geometry and mixed/disconnected roads.
Existing boundary classifications are preserved. This does not classify the
physical perimeter inside junctions or grant routing lane-change permissions.
"""
import argparse
from collections import defaultdict, Counter
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import carla
from repair_town05_lane_changes import tags, sha, match_lane, connected_native_sequence


def classify(keys, role, roads):
    if not connected_native_sequence(keys, roads):
        return None
    classes = []
    for road_id, section_id, lane_id in keys:
        section = roads[road_id].findall('./lanes/laneSection')[section_id]
        lanes = {int(l.get('id')): l for l in section.findall('./*/lane')}
        sign = 1 if lane_id>0 else -1
        neighbor_id = lane_id+sign if role=='right' else lane_id-sign
        if neighbor_id == 0:
            neighbor_id = -sign
        neighbor = lanes.get(neighbor_id)
        if neighbor is not None and neighbor.get('type') == 'driving':
            # Opposite traffic is a protected divider; internal same-direction
            # lanes remain available for bus swept-body borrowing.
            classes.append('divider' if neighbor_id*lane_id<0 else 'internal')
        else:
            classes.append('edge')
    return classes[0] if len(set(classes))==1 else None


def build(map_path, xodr, output, manifest, output_manifest):
    if map_path.resolve()==output.resolve():
        raise ValueError('Candidate output must differ from the input map')
    tree=ET.parse(map_path); root=tree.getroot()
    roads={int(r.get('id')):r for r in ET.parse(xodr).getroot().findall('road')}
    native=carla.Map('Town05',xodr.read_text())
    nodes={n.get('id'):tuple(float(tags(n).get(k,0)) for k in ('local_x','local_y','ele'))
           for n in root.findall('node')}
    ways={w.get('id'):w for w in root.findall('way')}
    uses=defaultdict(list)
    for rel in root.findall('relation'):
        if tags(rel).get('type')!='lanelet' or tags(rel).get('subtype')!='road':
            continue
        bounds={m.get('role'):m.get('ref') for m in rel.findall('member') if m.get('role') in ('left','right')}
        points=[[nodes[n.get('ref')] for n in ways[bounds[side]].findall('nd')] for side in ('left','right')]
        keys,reason,error=match_lane(*points,native)
        for side,way in bounds.items():
            kind=classify(keys,side,roads) if keys and not reason else None
            uses[way].append({'lanelet':int(rel.get('id')),'side':side,'kind':kind,
                              'native':keys,'match_error_m':error,'rejection':reason})
    added=[]; skipped=Counter()
    for way,records in uses.items():
        kinds={r['kind'] for r in records}
        if kinds not in ({'edge'},{'divider'}):
            skipped['internal_or_unproven']+=1; continue
        current=tags(ways[way])
        if any(k in current for k in ('type','subtype','lane_change','lane_change:left','lane_change:right')):
            skipped['existing_classification']+=1; continue
        kind=next(iter(kinds))
        new={'type':'road_border'} if kind=='edge' else {'type':'line_thin','subtype':'solid','lane_change':'no'}
        new['carla:bus_boundary_source']='native_nonjunction_edges_v1'
        for k,v in new.items(): ET.SubElement(ways[way],'tag',k=k,v=v)
        added.append({'way':int(way),'kind':kind,'proof':records})
    tree.write(output,encoding='UTF-8',xml_declaration=True)
    data=json.loads(manifest.read_text())
    if data['map_sha256']!=sha(map_path): raise ValueError('Input map/signal manifest hash mismatch')
    data['map_sha256']=sha(output)
    output_manifest.write_text(json.dumps(data,indent=2)+'\n')
    return {'source_sha256':sha(map_path),'output_sha256':sha(output),'xodr_sha256':sha(xodr),
            'geometry_modified':False,'added':added,'skipped':dict(skipped)}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('map','xodr','output','manifest','output-manifest','report'):
        p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args(); result=build(a.map,a.xodr,a.output,a.manifest,a.output_manifest)
    a.report.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'added':len(result['added']),'skipped':result['skipped']}))
