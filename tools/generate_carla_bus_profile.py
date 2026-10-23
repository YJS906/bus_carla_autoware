#!/usr/bin/env python3
"""Generate Fuso Rosa geometry/sensor configuration from recorded CARLA physics."""
import json
import math
from pathlib import Path

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]


def generate():
    source=ROOT/'docs/validation/carla-bus-route-20261022/bus-geometry.json'
    out=ROOT/'config/carla/vehicle_profiles/fusorosa'
    (out/'sensors').mkdir(parents=True,exist_ok=True)
    d=json.loads(source.read_text())
    inv=np.array(d['actor_inverse_transform'])
    ws=np.array([(inv@np.array([*(np.array(w['position_cm'])/100),1]))[:3] for w in d['wheels']])
    front,rear=ws[:2].mean(axis=0),ws[2:].mean(axis=0)
    centre,extent=np.array(d['bounding_box']['location']),np.array(d['bounding_box']['extent'])
    tread=abs(ws[0,1]-ws[1,1]);base=np.array([rear[0],rear[1],centre[2]-extent[2]])
    physics_source=ROOT/'docs/validation/carla-bus-envelope-20261022/physics-asset.json'
    bodies=json.loads(physics_source.read_text())['bodies']
    radii={b['bone']:next((p['value']/100 for p in b['properties'] if p['name']=='Radius'),0) for b in bodies}
    wheel_names=['Wheel_Front_Left','Wheel_Front_Right','Wheel_Rear_Left','Wheel_Rear_Right']
    min_y=min(centre[1]-extent[1],*[p[1]-radii[n] for p,n in zip(ws,wheel_names)])
    max_y=max(centre[1]+extent[1],*[p[1]+radii[n] for p,n in zip(ws,wheel_names)])
    profile=dict(wheel_radius=.63,wheel_width=.30,wheel_base=float(front[0]-rear[0]),
        wheel_tread=float(tread),front_overhang=float(centre[0]+extent[0]-front[0]),
        rear_overhang=float(rear[0]-centre[0]+extent[0]),
        left_overhang=float(-min_y+rear[1]-tread/2),
        right_overhang=float(max_y-rear[1]-tread/2),vehicle_height=float(2*extent[2]),
        max_steer_angle=math.radians(max(w['max_steer_angle_deg'] for w in d['wheels'])))
    (out/'vehicle_info.param.yaml').write_text(
        '# CARLA 0.9.16 Fuso Rosa, measured 2026-10-22. Rear axle ground base_link.\n'
        '# wheel_width is descriptive (not exposed by CARLA); envelope dimensions are measured.\n'
        +yaml.safe_dump({'/**':{'ros__parameters':profile}},sort_keys=False))
    mounts={};roof=2*extent[2]+.20
    for name,x,y,yaw in [
        ('CAM_FRONT',centre[0]+extent[0]+.1,0,0),
        ('CAM_FRONT_LEFT',centre[0]+extent[0]-.3,-extent[1]-.1,55),
        ('CAM_FRONT_RIGHT',centre[0]+extent[0]-.3,extent[1]+.1,-55),
        ('CAM_BACK',centre[0]-extent[0]-.1,0,180),
        ('CAM_BACK_LEFT',centre[0]-extent[0]+.3,-extent[1]-.1,125),
        ('CAM_BACK_RIGHT',centre[0]-extent[0]+.3,extent[1]+.1,-125)]:
        mounts[name+'/camera_link']={'x':float(x-base[0]),'y':float(-y+base[1]),
            'z':float(roof),'roll':0.,'pitch':0.,'yaw':math.radians(yaw)}
    for name in ['velodyne_top_base_link','gnss_link','tamagawa/imu_link']:
        mounts[name]={'x':float(-base[0]) if name=='velodyne_top_base_link' else 0.,'y':0.,
            'z':float(roof) if name=='velodyne_top_base_link' else .05,'roll':0.,'pitch':0.,'yaw':0.}
    (out/'sensors/sensor_kit_calibration.yaml').write_text(
        '# ROS extrinsics relative to bus rear-axle ground base_link; lidar clears the roof.\n'
        +yaml.safe_dump({'sensor_kit_base_link':mounts},sort_keys=False))
    (out/'sensors/sensors_calibration.yaml').write_text((ROOT/'config/carla/sensors/sensors_calibration.yaml').read_text())
    manifest={'profile':'fusorosa','blueprint':'vehicle.mitsubishi.fusorosa',
        'measurement':str(source.relative_to(ROOT)),'base_offset_carla_m':base.tolist(),
        'collision_measurement':str(physics_source.relative_to(ROOT)),
        'lateral_envelope':'union of reported mesh bounding box and measured wheel collision spheres',
        'initial_validation_speed_cap_kmh':10,'operating_speed_cap_kmh':50,
        'urban_speed_cap_kmh':30}
    (out/'profile.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(profile,indent=2))


if __name__=='__main__':generate()
