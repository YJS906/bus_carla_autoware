"""Owned, signal-obeying traffic for the bus course; the bridge owns all ticks."""
import math

import carla

from carla_closed_loop_validation import polygon, polygon_distance, speed, xyz
from carla_bus_following import following_speed_limit


# (road, same-direction lane/s, opposite-direction lane/s). Native legal lanes.
PLACEMENTS = (
    (44, -2, 6., 1, 55.), (5, -2, 40., 1, 15.),
    (1, 2, 15., -1, 40.), (16, 2, 12., -1, 36.),
    (29, -2, 15., 1, 40.), (0, -2, 15., 1, 40.),
    (40, -2, 10., 1, 30.), (37, -3, 350., 2, 450.),
)


class CourseTraffic:
    def __init__(self, node):
        self.node = node
        self.actors = []
        self.caps = {}
        self.sensors = []
        self.last_sample = -math.inf
        self.tm = node.carla.get_trafficmanager(8000)
        self.report = dict(spawned=[], skipped=[], samples=[], encounters={}, collisions=[],
                           following_cap_samples=0, unexpected_actor_loss=[], nonfinite_controls=[], cleanup_removed_ids=[])
        node.result['course_traffic'] = self.report

    def spawn(self):
        n = self.node
        library = n.world.get_blueprint_library()
        layout = n.args.traffic_layout
        placements = PLACEMENTS if layout == 'course' else PLACEMENTS[:1]
        for road, same_lane, same_s, opposite_lane, opposite_s in placements:
            for direction, lane, s in (
                ('same', same_lane, same_s), ('opposite', opposite_lane, opposite_s)
            ):
                if layout == 'following' and direction != 'same':
                    continue
                if layout == 'opposing' and direction != 'opposite':
                    continue
                wp = n.wmap.get_waypoint_xodr(road, lane, s)
                if not wp or wp.is_junction or wp.lane_type != carla.LaneType.Driving:
                    raise RuntimeError(f'Invalid traffic placement {road}/{lane}/{s}')
                t = wp.transform
                if t.location.distance(n.ego.get_location()) < 18:
                    if n.start_index != 0:
                        # Section diagnostics start at a different station. Keep the same
                        # exclusion distance and explicitly report omitted spawn positions.
                        row = dict(road=road, lane=lane, native_s=s, reason='near_section_start')
                        self.report['skipped'].append(row)
                        n.event('course_traffic_spawn_skipped', **row)
                        continue
                    raise RuntimeError('Traffic spawn is too close to the stopped bus')
                # Newly spawned actors can report a zero transform until the next snapshot.
                if any(t.location.distance(carla.Location(*r['initial_xyz'])) < 9
                       for r in self.report['spawned']):
                    raise RuntimeError('Traffic spawn spacing is too small')
                t.location.z += .25
                bp = library.find('vehicle.tesla.model3' if direction == 'same' else 'vehicle.audi.a2')
                bp.set_attribute('role_name', 'bus_course_' + direction)
                if bp.has_attribute('color'):
                    bp.set_attribute('color', '30,100,220' if direction == 'same' else '210,50,40')
                actor = n.world.try_spawn_actor(bp, t)
                if actor is None:
                    raise RuntimeError(f'Traffic spawn blocked at {road}/{lane}/{s}')
                self.actors.append(actor)  # record ownership before any fallible configuration
                sensor = n.world.spawn_actor(library.find('sensor.other.collision'),
                                             carla.Transform(), attach_to=actor)
                self.sensors.append(sensor)
                sensor.listen(lambda e, actor_id=actor.id: self.report['collisions'].append(dict(
                    actor_id=actor_id, other_id=e.other_actor.id,
                    other_type=e.other_actor.type_id, frame=e.frame,
                    impulse=xyz(e.normal_impulse))))
                self.tm.ignore_lights_percentage(actor, 0)
                self.tm.ignore_signs_percentage(actor, 0)
                self.tm.ignore_vehicles_percentage(actor, 0)
                self.tm.ignore_walkers_percentage(actor, 0)
                self.tm.collision_detection(actor, n.ego, True)
                self.tm.distance_to_leading_vehicle(actor, 4.)
                self.tm.auto_lane_change(actor, False)
                cap = 40. if road == 37 else 20.
                self.tm.set_desired_speed(actor, cap)
                self.caps[actor.id] = cap
                actor.set_autopilot(True, 8000)
                self.report['spawned'].append(dict(
                    id=actor.id, type=actor.type_id, initial_direction=direction,
                    road=road, lane=lane, native_s=s, initial_xyz=xyz(t.location)))
        n.event('course_traffic_spawned', count=len(self.actors), layout=layout,
                same=sum(r['initial_direction']=='same' for r in self.report['spawned']),
                opposite=sum(r['initial_direction']=='opposite' for r in self.report['spawned']))

    def sample(self):
        n = self.node
        now = n.get_clock().now().nanoseconds / 1e9
        if now - self.last_sample < .2:
            return
        self.last_sample = now
        ego_t = n.ego.get_transform()
        ef = ego_t.get_forward_vector()
        ego_poly = polygon(n.ego)
        rows = []
        for actor in self.actors:
            if not actor.is_alive:
                row = dict(actor_id=actor.id, sim_time=now)
                self.report['unexpected_actor_loss'].append(row)
                n.event('course_traffic_actor_lost', actor_id=actor.id)
                raise RuntimeError(f'Owned traffic actor {actor.id} disappeared during validation')
            t = actor.get_transform()
            wp = n.wmap.get_waypoint(t.location)
            nominal_cap = 40. if wp.road_id == 37 else 20.
            f = t.get_forward_vector()
            bus_velocity = n.ego.get_velocity()
            cap = nominal_cap if abs(t.location.z - ego_t.location.z) >= 2 else 3.6 * following_speed_limit(
                polygon(actor), ego_poly, (f.x, f.y), (ef.x, ef.y),
                (bus_velocity.x, bus_velocity.y), nominal_cap / 3.6)
            if cap < nominal_cap - .1:
                self.report['following_cap_samples'] += 1
            # CARLA 0.9.16 normalizes PID error by target velocity. A literal
            # zero desired speed can produce NaN braking (0/0) at standstill.
            # 0.01 km/h is a finite near-zero target; native collision/light
            # braking remains active. The 6 m following reserve is unchanged.
            applied_cap = max(.01, cap)
            if self.caps[actor.id] != applied_cap:
                self.tm.set_desired_speed(actor, applied_cap)
                self.caps[actor.id] = applied_cap
            distance = t.location.distance(ego_t.location)
            row = dict(id=actor.id, xyz=xyz(t.location), speed_kmh=speed(actor)*3.6,
                       road=wp.road_id, lane=wp.lane_id, distance_to_bus_m=distance,
                       desired_speed_kmh=applied_cap, following_ceiling_kmh=cap, nominal_speed_kmh=nominal_cap)
            control = actor.get_control()
            row.update(throttle=control.throttle, brake=control.brake, steer=control.steer,
                       yaw_deg=t.rotation.yaw)
            if not all(math.isfinite(v) for v in (control.throttle, control.brake, control.steer)):
                self.report['nonfinite_controls'].append(dict(sim_time=now, **row))
                n.event('course_traffic_nonfinite_control', actor_id=actor.id)
                raise RuntimeError(f'Non-finite control on traffic actor {actor.id}')
            if distance < 35 and abs(t.location.z - ego_t.location.z) < 2:
                f = t.get_forward_vector()
                dot = f.x*ef.x + f.y*ef.y
                direction = 'same' if dot > .8 else 'opposite' if dot < -.8 else 'crossing'
                gap = polygon_distance(ego_poly, polygon(actor))
                row.update(relative_direction=direction, bbox_clearance_m=gap)
                if gap < .05:
                    row.update(ego_polygon=ego_poly, actor_polygon=polygon(actor))
                key = f'{actor.id}:{direction}'
                record = self.report['encounters'].get(key)
                if record is None:
                    record = dict(actor_id=actor.id, direction=direction,
                                  first_sim_time=now, minimum_bbox_clearance_m=gap,
                                  maximum_npc_speed_kmh=row['speed_kmh'])
                    self.report['encounters'][key] = record
                    n.event('traffic_encounter', **record)
                record['minimum_bbox_clearance_m'] = min(record['minimum_bbox_clearance_m'], gap)
                record['maximum_npc_speed_kmh'] = max(record['maximum_npc_speed_kmh'], row['speed_kmh'])
                record['last_sim_time'] = now
            rows.append(row)
        objects = n.last.get('objects')
        self.report['samples'].append(dict(sim_time=now, vehicles=rows,
            perceived_objects=len(objects.objects) if objects else 0))
        if any(c['other_id'] == n.ego.id for c in self.report['collisions']):
            raise RuntimeError('NPC collision sensor reported contact with bus')
        return rows

    def cleanup(self):
        errors = []
        for sensor in self.sensors:
            try:
                if sensor.is_alive:
                    sensor.stop()
                    if not sensor.destroy():
                        raise RuntimeError(f'Could not remove traffic collision sensor {sensor.id}')
            except RuntimeError as exc:
                errors.append(str(exc))
        for actor in self.actors:
            try:
                if actor.is_alive:
                    actor.set_autopilot(False, 8000)
                    if not actor.destroy():
                        raise RuntimeError(f'Could not remove owned traffic actor {actor.id}')
                self.report['cleanup_removed_ids'].append(actor.id)
            except RuntimeError as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError('; '.join(errors))
