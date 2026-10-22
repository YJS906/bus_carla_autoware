#!/usr/bin/env python3
"""Read-only, format-guarded inspection of the installed cooked Fuso physics asset.

This deliberately supports the measured UE4 cooked asset layout only. It does
not rewrite assets or infer that a queried shape equals a render mesh.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct


def inspect(path):
    data=path.read_bytes();exports=path.with_suffix('.uexp').read_bytes()
    if struct.unpack_from('<Ii',data)!=(0x9E2A83C1,-7):raise ValueError('Unsupported package header')
    if data[32:37]!=b'None\0':raise ValueError('Unexpected cooked header layout')
    header_size=struct.unpack_from('<i',data,24)[0]
    count,pos=struct.unpack_from('<ii',data,41)
    n_export,export_offset=struct.unpack_from('<ii',data,57)
    if header_size!=len(data) or not 0<count<1000 or not 0<n_export<100:
        raise ValueError('Unexpected package summary')
    names=[]
    for _ in range(count):
        size=struct.unpack_from('<i',data,pos)[0];pos+=4
        if not 0<size<1024:raise ValueError('Unsupported name encoding')
        names.append(data[pos:pos+size-1].decode());pos+=size+4
    wanted={'Radius':'FloatProperty','BoneName':'NameProperty','PhysicsType':'ByteProperty',
            'CollisionEnabled':'ByteProperty'}
    result={'uasset':str(path),'sha256':hashlib.sha256(data).hexdigest(),
            'uexp_sha256':hashlib.sha256(exports).hexdigest(),'bodies':[]}
    for idx in range(n_export):
        p=export_offset+104*idx
        name_i=struct.unpack_from('<i',data,p+16)[0]
        size,absolute=struct.unpack_from('<qq',data,p+28);start=absolute-header_size
        if start<0 or start+size>len(exports):raise ValueError('Unexpected export extent')
        if names[name_i]!='SkeletalBodySetup':continue
        raw=exports[start:start+size];properties=[]
        for i in range(len(raw)-41):
            ni,num,ti,tn,s,arr=struct.unpack_from('<6i',raw,i)
            if not (0<=ni<len(names) and 0<=ti<len(names) and num==tn==arr==0):continue
            name,kind=names[ni],names[ti]
            if wanted.get(name)!=kind or s!=(4 if kind=='FloatProperty' else 8):continue
            value_pos=i+24+(8 if kind=='ByteProperty' else 0)
            if raw[value_pos]!=0:continue
            value_pos+=1
            if kind=='FloatProperty':value=struct.unpack_from('<f',raw,value_pos)[0]
            else:
                value_i,num=struct.unpack_from('<ii',raw,value_pos)
                if not 0<=value_i<len(names) or num!=0:continue
                value=names[value_i]
            properties.append({'name':name,'value':value,'uexp_offset':start+i})
        bone=[p['value'] for p in properties if p['name']=='BoneName']
        if len(bone)!=1:raise ValueError('Missing or ambiguous body bone')
        result['bodies'].append({'bone':bone[0],'properties':properties})
    if len(result['bodies'])!=5:raise ValueError('Expected base and four wheel bodies')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--asset',type=Path,default=Path('/home/a/CARLA/0.9.16/CarlaUE4/Content/Carla/Static/Bus/Mitsubishi_FusoRosa/Phys_FusoRosa.uasset'))
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.write_text(json.dumps(inspect(a.asset),indent=2)+'\n')
