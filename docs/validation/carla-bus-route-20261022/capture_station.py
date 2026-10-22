import carla,json,queue,time,math
from pathlib import Path
c=carla.Client('127.0.0.1',2000);c.set_timeout(20)
w=c.get_world();data=json.loads(Path('scenarios/bus/town05_bus_route.json').read_text());s=data['stops'][0];p=s['carla'];actors=[]
try:
 bp=w.get_blueprint_library().find('vehicle.mitsubishi.fusorosa');bp.set_attribute('role_name','bus_station_preview')
 bus=w.spawn_actor(bp,carla.Transform(carla.Location(x=p['x'],y=p['y'],z=p['z']+.5),carla.Rotation(yaw=p['yaw_deg'])))
 actors.append(bus);bus.apply_control(carla.VehicleControl(brake=1,hand_brake=True))
 camera_bp=w.get_blueprint_library().find('sensor.camera.rgb')
 camera_bp.set_attribute('image_size_x','1280');camera_bp.set_attribute('image_size_y','720');camera_bp.set_attribute('fov','80');camera_bp.set_attribute('sensor_tick','0.2')
 camera=w.spawn_actor(camera_bp,carla.Transform(carla.Location(x=13,y=-15,z=10),carla.Rotation(pitch=-26,yaw=131)),attach_to=bus)
 actors.append(camera);q=queue.Queue(maxsize=2)
 def receive(im):
  if not q.full():q.put(im)
 camera.listen(receive)
 for _ in range(12):im=q.get(timeout=20)
 im.save_to_disk('docs/validation/carla-bus-route-20261022/stop01-bus-preview.png')
 physics=bus.get_physics_control();bbox=bus.bounding_box;transform=bus.get_transform()
 result={'map':w.get_map().name,'blueprint':bus.type_id,'stop':'S01','bounding_box':{'extent':[bbox.extent.x,bbox.extent.y,bbox.extent.z],'location':[bbox.location.x,bbox.location.y,bbox.location.z]},'wheels':[{'position_cm':[x.position.x,x.position.y,x.position.z],'max_steer_angle_deg':x.max_steer_angle,'radius_cm':x.radius} for x in physics.wheels],'actor_inverse_transform':transform.get_inverse_matrix(),'mass_kg':physics.mass,'steering_curve':[[v.x,v.y] for v in physics.steering_curve],'speed_mps':bus.get_velocity().length(),'autoware_running':False,'driving_validated':False}
 Path('docs/validation/carla-bus-route-20261022/bus-geometry.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({'map':result['map'],'image_saved':True,'speed_mps':result['speed_mps']}))
finally:
 for a in actors[::-1]:
  if isinstance(a,carla.Sensor):a.stop()
  a.destroy()
 print('temporary actors cleaned')
