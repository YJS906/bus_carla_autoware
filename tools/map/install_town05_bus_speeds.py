#!/usr/bin/env python3
"""Install urban 30 / western ring 50 km/h tags without changing map geometry."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET


def patch(text):
    counts = {'urban_30': 0, 'ring_50': 0}
    def relation(match):
        block = match.group(0)
        node = ET.fromstring(block)
        tags = {t.get('k'): t.get('v') for t in node.findall('tag')}
        if tags.get('type') != 'lanelet' or tags.get('subtype') != 'road':
            return block
        # Town05's native road 36 joins 37 in the long western ring lanelets.
        ring = tags.get('source:opendrive:road_id') in ('36', '37')
        value = '50' if ring else '30'
        counts['ring_50' if ring else 'urban_30'] += 1
        if tags.get('speed_limit') == value:
            return block
        block = re.sub(r'\s*<tag\s+k="speed_limit"\s+v="[^"]*"\s*/>', '', block)
        return block.replace('</relation>', f'  <tag k="speed_limit" v="{value}" />\n  </relation>')
    result = re.sub(r'<relation\b[^>]*>.*?</relation>', relation, text, flags=re.S)
    if not all(counts.values()):
        raise ValueError('Expected both urban and native western ring lanelets')
    return result, counts


def install(directory):
    path = directory / 'lanelet2_map.osm'
    original = path.read_text()
    updated, counts = patch(original)
    if updated == original:
        print('Bus urban/ring speed tags already present:', counts)
        return
    for name in ('selfcar-carla-autoware', 'selfcar-carla-signals'):
        p = subprocess.run(['docker', 'inspect', '-f', '{{.State.Running}}', name],
                           capture_output=True, text=True)
        if p.returncode == 0 and p.stdout.strip() == 'true':
            raise RuntimeError('Stop CARLA Autoware before installing map speed tags')
    names = ['lanelet2_map.osm', 'carla_traffic_signals.json', 'carla_map_report.json']
    backup = directory / 'backups' / ('bus-speeds-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    backup.mkdir(parents=True)
    for name in names:
        shutil.copy2(directory / name, backup / name)
    sha = hashlib.sha256(updated.encode()).hexdigest()
    try:
        path.write_text(updated)
        for name in names[1:]:
            p = directory / name
            data = json.loads(p.read_text())
            data['map_sha256'] = sha
            p.write_text(json.dumps(data, indent=2) + '\n')
    except BaseException:
        for name in names:
            shutil.copy2(backup / name, directory / name)
        raise
    report = dict(counts, map_sha256=sha, geometry_modified=False,
                  ring_native_road_ids=[36, 37], backup=str(backup))
    (directory / 'carla_bus_speed_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map-dir', type=Path, required=True)
    install(p.parse_args().map_dir)
