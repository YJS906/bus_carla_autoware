#!/usr/bin/env python3
"""Fit monotone pedal maps to recorded Fuso Rosa calibration runs.

Measured speed bands are 2.78 and 5.56m/s. Outside them values are clamped;
low-speed/standing behaviour still requires separate closed-loop validation.
"""
import csv
import json
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def isotonic(values,increasing=True):
    sign=1 if increasing else -1
    blocks=[]
    for value in values:
        blocks.append([sign*value,1])
        while len(blocks)>1 and blocks[-2][0]/blocks[-2][1]>blocks[-1][0]/blocks[-1][1]:
            b=blocks.pop();blocks[-1][0]+=b[0];blocks[-1][1]+=b[1]
    return [sign*total/count for total,count in blocks for _ in range(count)]


def generate():
    source=ROOT/'docs/validation/carla-bus-control-20261022/pedal-calibration.json'
    data=json.loads(source.read_text())
    if not data.get('complete'):raise ValueError('Complete calibration is required')
    out=ROOT/'config/carla/vehicle_profiles/fusorosa/calibration_maps';out.mkdir(exist_ok=True)
    speeds=sorted({c['target_speed_mps'] for c in data['cases']})
    columns=[0,1.39,2.78,4.17,5.56,8.33,13.89]
    report={'source':str(source.relative_to(ROOT)),'measured_speed_bands_mps':speeds,
            'outside_measured_bands':'clamped, not measured','adjustments':[]}
    coast={v:next(c['mean_acceleration_mps2'] for c in data['cases']
                 if c['target_speed_mps']==v and c['kind']=='brake' and c['pedal']==0) for v in speeds}
    for kind,name in [('throttle','accel_map.csv'),('brake','brake_map.csv')]:
        pedals=sorted({0}|{c['pedal'] for c in data['cases'] if c['kind']==kind})
        fitted={}
        for v in speeds:
            raw=[coast[v] if p==0 else next(c['mean_acceleration_mps2'] for c in data['cases']
                 if c['kind']==kind and c['pedal']==p and c['target_speed_mps']==v) for p in pedals]
            if any(x is None for x in raw):raise ValueError('Missing moving sample')
            fit=isotonic(raw,increasing=kind=='throttle')
            # Keep the identical zero-pedal coast row in both maps.
            fit[0]=coast[v]
            if kind=='brake':fit=np.minimum.accumulate(fit).tolist()
            else:fit=np.maximum.accumulate(fit).tolist()
            fitted[v]=fit
            report['adjustments'].append({'kind':kind,'speed_mps':v,'pedals':pedals,'raw':raw,'fitted':fit})
        with (out/name).open('w',newline='') as stream:
            writer=csv.writer(stream,lineterminator='\n');writer.writerow(['default',*columns])
            for i,p in enumerate(pedals):
                writer.writerow([f'{p:.6f}',*[f'{np.interp(v,speeds,[fitted[s][i] for s in speeds]):.6f}' for v in columns]])
    (ROOT/'docs/validation/carla-bus-control-20261022/map-fit.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':generate()
