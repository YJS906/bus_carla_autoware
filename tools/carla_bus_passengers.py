"""Owned, stationary waiting passengers on native sidewalks beside bus shelters."""
import time

import carla

from carla_bus_stop_props import plan as shelter_plan


class WaitingPassengers:
    def __init__(self, world, course, counts, event):
        self.world = world
        self.event = event
        self.counts = counts
        self.actors = {}
        self.records = []
        self.boarded = 0
        self.placements = {row['id']: row for row in shelter_plan(world, course)}
        self.stops = {row['id']: row for row in course['stops']}
        self.token = str(time.time_ns())

    def spawn(self):
        if self.world.get_actors().filter('walker.pedestrian.*'):
            raise RuntimeError('Existing pedestrians present; refusing to overwrite a scenario')
        library = sorted((bp for bp in self.world.get_blueprint_library().filter('walker.pedestrian.*')
                          if bp.has_attribute('age') and bp.get_attribute('age').as_str()=='adult'),
                         key=lambda b:b.id)
        if not library:
            raise RuntimeError('No pedestrian blueprints available')
        try:
            for name, count in self.counts.items():
                row = self.placements[name]
                stop = self.stops[name]
                lane = self.world.get_map().get_waypoint_xodr(stop['road_id'], stop['lane_id'], stop['s_m'])
                right = lane.transform.get_right_vector()
                forward = lane.transform.get_forward_vector()
                sidewalk = lane.get_right_lane()
                while sidewalk and sidewalk.lane_type != carla.LaneType.Sidewalk:
                    if sidewalk.lane_type == carla.LaneType.Driving:
                        raise RuntimeError('Passenger area crosses another driving lane')
                    sidewalk = sidewalk.get_right_lane()
                if sidewalk is None:
                    raise RuntimeError('No sidewalk for '+name)
                centre = (sidewalk.transform.location + forward*row['longitudinal_offset_m']+
                          right*row['lateral_offset_m'])
                for index in range(count):
                    # Stand beyond the shelter ends, not in its bench/glass box.
                    along = [-3.1, 3.1, -4.1][index]
                    location = centre + forward*along
                    hits = self.world.cast_ray(location+carla.Location(z=3), location-carla.Location(z=2))
                    ground = next((h.location.z for h in hits if str(h.label)=='Sidewalks'), None)
                    if ground is None:
                        raise RuntimeError('Passenger spawn is not on a sidewalk: '+name)
                    location.z = ground+1.
                    bp = library[(int(name[1:])*3+index)%len(library)]
                    role = f'bus_waiting_{self.token}_{name}_{index}'
                    bp.set_attribute('role_name', role)
                    if bp.has_attribute('is_invincible'):
                        bp.set_attribute('is_invincible', 'false')
                    actor = self.world.try_spawn_actor(bp, carla.Transform(location,
                        carla.Rotation(yaw=lane.transform.rotation.yaw-90.)))
                    if actor is None:
                        raise RuntimeError('Passenger placement collision: '+name)
                    self.actors[actor.id] = dict(actor=actor,station=name,role=role,origin=None)
                    self.world.wait_for_tick(5)  # Actor transform cache is empty until its first snapshot.
                    actor.apply_control(carla.WalkerControl(speed=0.))
                    actor.set_simulate_physics(False)
                    box = actor.bounding_box
                    t = actor.get_transform()
                    t.location.z = ground+.025-box.location.z+box.extent.z
                    actor.set_transform(t)
                    self.actors[actor.id]['origin'] = t.location
                    clearance = ((t.location.x-lane.transform.location.x)*right.x+
                                 (t.location.y-lane.transform.location.y)*right.y-lane.lane_width/2-box.extent.y)
                    if clearance < .4:
                        raise RuntimeError(f'Passenger is too close to the driving lane: {name}, {clearance:.3f} m')
                    record = dict(station=name,actor_id=actor.id,role=role,blueprint=actor.type_id,
                                  x=t.location.x,y=t.location.y,z=t.location.z,
                                  road_clearance_m=clearance,state='waiting')
                    self.records.append(record)
                    self.event('waiting_passenger_spawned', **record)
            self.world.wait_for_tick(5)
            self.sample()
        except BaseException:
            self.cleanup()
            raise

    def sample(self):
        counts = dict.fromkeys(self.counts, 0)
        for item in self.actors.values():
            actor = item['actor']
            if not actor.is_alive or actor.attributes.get('role_name') != item['role']:
                raise RuntimeError('Waiting passenger unexpectedly disappeared')
            if actor.get_location().distance(item['origin']) > .2:
                raise RuntimeError('Waiting passenger left its designated area')
            counts[item['station']] += 1
        return counts

    def board(self, station):
        count = 0
        for actor_id, item in list(self.actors.items()):
            if item['station'] != station:
                continue
            if not item['actor'].destroy():
                raise RuntimeError('Cannot complete passenger boarding')
            del self.actors[actor_id]
            next(r for r in self.records if r['actor_id']==actor_id)['state'] = 'boarded'
            count += 1
        self.boarded += count
        self.event('passengers_boarded',station=station,count=count,total=self.boarded)
        return count

    def cleanup(self):
        # Newly spawned actors can report is_alive=False until a world snapshot.
        # Refresh before deciding that an owned actor has already disappeared.
        self.world.wait_for_tick(5)
        errors = []
        for actor_id, item in list(self.actors.items()):
            actor = item['actor']
            if actor.is_alive and actor.attributes.get('role_name') == item['role']:
                if not actor.destroy():
                    errors.append(actor_id)
                    continue
            for record in self.records:
                if record['actor_id']==actor_id and record['state']=='waiting':
                    record['state']='removed_on_cleanup'
            del self.actors[actor_id]
        if errors:
            raise RuntimeError('Passenger cleanup failed: '+str(errors))
        self.world.wait_for_tick(5)
        if any(a.attributes.get('role_name','').startswith('bus_waiting_'+self.token+'_')
               for a in self.world.get_actors().filter('walker.pedestrian.*')):
            raise RuntimeError('Owned waiting passengers remain after cleanup')
