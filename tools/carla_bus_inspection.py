#!/usr/bin/env python3
"""Park a preview bus and provide camera buttons for the ten saved stops.

Requires a running Town05 world. Owns only the bus role bus_mission_preview;
does not tick the world, start Autoware, engage AUTO or move an existing ego.
"""
import json
import math
from pathlib import Path
import tkinter as tk
from tkinter import ttk

import carla

from carla_bus_route_preview import preview

ROOT = Path(__file__).resolve().parents[1]
STATE = Path.home()/'.local/state/carla_pp/bus-preview'
ROLE = 'bus_mission_preview'


class Inspection:
    def __init__(self, root):
        self.root = root
        self.client = carla.Client('127.0.0.1',2000)
        self.client.set_timeout(5)
        self.world = self.client.get_world()
        if self.world.get_map().name.rsplit('/',1)[-1] not in ('Town05','Town05_Opt'):
            raise RuntimeError('Town05 must be loaded first')
        self.route = json.loads((ROOT/'scenarios/bus/town05_bus_route.json').read_text())
        self.bus = None
        self.selected = 0
        self.alive = True
        root.title('Town05 버스 · 정류장 확인')
        panel_x = max(20, min(root.winfo_screenwidth()-440, 2650))
        panel_y = max(60, (root.winfo_screenheight()-780)//2)
        root.geometry(f'420x780+{panel_x}+{panel_y}')
        root.attributes('-topmost',True)
        outer = ttk.Frame(root,padding=16);outer.pack(fill='both',expand=True)
        ttk.Label(outer,text='Town05 버스 미리보기',font=('',16,'bold')).pack(anchor='w')
        ttk.Label(outer,text='정지 버스 1대 · 도심 정류장 10개\n버튼은 카메라만 이동합니다.',padding=(0,10)).pack(anchor='w')
        ttk.Button(outer,text='버스 보기 · 1번 정류장',command=self.bus_view).pack(fill='x',pady=4)
        ttk.Button(outer,text='노선 전체 내려다보기',command=self.overview).pack(fill='x',pady=4)
        ttk.Separator(outer).pack(fill='x',pady=10)
        for i,s in enumerate(self.route['stops']):
            ttk.Button(outer,text=f"{s['id'][1:]}  {s['name']}",command=lambda i=i:self.stop_view(i)).pack(fill='x',pady=2)
        self.status = tk.StringVar(value='연결 중…')
        ttk.Label(outer,textvariable=self.status,wraplength=315,padding=(0,12)).pack(anchor='w')
        ttk.Label(outer,text='노란 표시: 정류장 / 파란 선: 도심 노선\n주황 선: 순환로 / 초록 선: 연결로',wraplength=315).pack(anchor='w')
        root.protocol('WM_DELETE_WINDOW',self.close)
        self.spawn_bus()
        # CARLA's actor registry may lag a newly spawned actor by a frame.
        self.root.after(1000,self.bus_view)
        self.refresh()

    def spawn_bus(self):
        existing=[a for a in self.world.get_actors().filter('vehicle.*') if a.attributes.get('role_name')==ROLE]
        if len(existing)>1:
            raise RuntimeError('Multiple preview buses found; refusing ambiguous ownership')
        if existing:
            self.bus=existing[0]
        else:
            p=self.route['stops'][0]['carla']
            bp=self.world.get_blueprint_library().find('vehicle.mitsubishi.fusorosa')
            bp.set_attribute('role_name',ROLE)
            self.bus=self.world.try_spawn_actor(bp,carla.Transform(
                carla.Location(x=p['x'],y=p['y'],z=p['z']+.5),carla.Rotation(yaw=p['yaw_deg'])))
            if self.bus is None:
                raise RuntimeError('S01 is occupied; no existing actors were moved')
        self.bus.apply_control(carla.VehicleControl(brake=1,hand_brake=True))
        STATE.mkdir(parents=True,exist_ok=True)
        (STATE/'bus.json').write_text(json.dumps({'world_id':self.world.id,'actor_id':self.bus.id,
                                                'role_name':ROLE,'type_id':self.bus.type_id},indent=2)+'\n')

    def camera(self,p,offset_x,offset_y,height):
        yaw=math.radians(p['yaw_deg'])
        dx=offset_x*math.cos(yaw)-offset_y*math.sin(yaw)
        dy=offset_x*math.sin(yaw)+offset_y*math.cos(yaw)
        self.world.get_spectator().set_transform(carla.Transform(
            carla.Location(x=p['x']+dx,y=p['y']+dy,z=p['z']+height),
            carla.Rotation(pitch=-math.degrees(math.atan2(height-1.5,math.hypot(dx,dy))),
                           yaw=math.degrees(math.atan2(-dy,-dx)))))

    def bus_view(self):
        if not self.bus or not self.bus.is_alive:
            self.status.set('미리보기 버스가 없습니다.');return
        t=self.bus.get_transform()
        self.camera({'x':t.location.x,'y':t.location.y,'z':t.location.z,'yaw_deg':t.rotation.yaw},13,-15,10)
        self.status.set('1번 정류장 · 버스 정지 상태\n자율주행은 실행하지 않았습니다.')

    def stop_view(self,i):
        stop=self.route['stops'][i]
        self.camera(stop['carla'],-16,-10,18)
        self.status.set(f"{stop['id']} · {stop['name']}\n노란 선이 차로 중앙 정차 기준점입니다.")

    def overview(self):
        self.world.get_spectator().set_transform(carla.Transform(
            carla.Location(x=-25,y=0,z=460),carla.Rotation(pitch=-90,yaw=-90)))
        self.status.set('노선 전체 보기\n도심 01~05 → 서부 순환로 → 06~10 → 01')

    def refresh(self):
        if not self.alive:return
        try:
            if self.client.get_world().id != self.world.id:
                raise RuntimeError('CARLA world changed')
            preview(self.client,self.route,22)
        except RuntimeError as exc:
            self.status.set(f'연결 확인 필요: {exc}')
            self.alive=False
            return
        self.root.after(20000,self.refresh)

    def close(self):
        self.alive=False
        try:
            if (self.bus and self.bus.is_alive and self.client.get_world().id==self.world.id
                    and self.bus.attributes.get('role_name')==ROLE):
                self.bus.destroy()
        finally:
            self.root.destroy()


if __name__=='__main__':
    root=tk.Tk()
    app=Inspection(root)
    root.mainloop()
