#!/usr/bin/env python3
"""Prepare proven bus road-edge tags, with stopped-runtime promotion and backup."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from mark_town05_bus_road_edges import build, sha


def install(directory, xodr):
    names=('lanelet2_map.osm','carla_traffic_signals.json')
    report_name='carla_map_report.json'
    if (directory/report_name).exists():
        names += (report_name,)
    initial={name:sha(directory/name) for name in names}
    with tempfile.TemporaryDirectory(prefix='.bus-edge-',dir=directory) as tmp:
        work=Path(tmp)
        result=build(directory/names[0],xodr,work/names[0],directory/names[1],work/names[1])
        if not result['added']:
            print('Proven bus road-edge classifications already present; no map changes.')
            return
        if report_name in names:
            report=json.loads((directory/report_name).read_text())
            report['map_sha256']=result['output_sha256']
            report['bus_road_edge_classification']={
                'source_sha256':result['source_sha256'],
                'report':'carla_bus_road_edge_report.json',
                'added_count':len(result['added']),
                'geometry_modified':False}
            (work/report_name).write_text(json.dumps(report,indent=2)+'\n')
        for name in ('selfcar-carla-autoware','selfcar-carla-signals'):
            status=subprocess.run(['docker','inspect','-f','{{.State.Running}}',name],capture_output=True,text=True)
            if status.returncode==0 and status.stdout.strip()=='true':
                raise RuntimeError('Stop the CARLA Autoware runtime before promoting bus map edges')
        if initial!={name:sha(directory/name) for name in names}:
            raise RuntimeError('Map bundle changed during candidate generation')
        backup=directory/'backups'/('bus-road-edges-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
        backup.mkdir(parents=True)
        (backup/'hashes.json').write_text(json.dumps(initial,indent=2)+'\n')
        for name in names: shutil.copy2(directory/name,backup/name)
        promoted=[]
        try:
            for name in names:
                os.replace(work/name,directory/name);promoted.append(name)
        except BaseException:
            for name in promoted:
                shutil.copy2(backup/name,work/name);os.replace(work/name,directory/name)
            raise
        (directory/'carla_bus_road_edge_report.json').write_text(json.dumps(result,indent=2)+'\n')
        print(f"Added {len(result['added'])} bus road-edge/divider classifications; backup: {backup}")


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map-dir',type=Path,required=True);p.add_argument('--xodr',type=Path,required=True)
    a=p.parse_args();install(a.map_dir,a.xodr)
