#!/usr/bin/env python3
"""Run the saved ten-stop course through Autoware, without service-stop logic.

Rolling route replacement avoids Autoware's repeated-lanelet loop prohibition.
Only initial placement uses CARLA teleportation. Service stops are optional.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import time
import xml.etree.ElementTree as ET

import carla
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Pose
from autoware_planning_msgs.msg import Path as PlanningPath, Trajectory
from autoware_planning_msgs.srv import SetPreferredPrimitive
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray
from tier4_planning_msgs.msg import RerouteAvailability
from autoware_system_msgs.msg import HazardStatusStamped
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode, ClearRoute, SetRoutePoints
from autoware_internal_planning_msgs.msg import PlanningFactorArray, VelocityLimit
from rosidl_runtime_py.convert import message_to_ordereddict

from carla_bus_control_validation import BusValidation
from carla_closed_loop_validation import speed, xyz
from carla_bus_signal_audit import SignalAudit
from carla_bus_ring_policy import RingPolicy
from carla_bus_route_policy import CentralUrbanMergePolicy, station_exit_preferences
from carla_bus_course_traffic import CourseTraffic


class BusLap(BusValidation):
    def __init__(self, args):
        super().__init__(args)
        self.course_traffic = None
        self.course = json.loads(args.route.read_text())
        anchors = self.course['anchors']
        self.start_index = next((i for i, a in enumerate(anchors) if a['id'] == args.start_anchor), None)
        if self.start_index is None:
            raise ValueError('Unknown start anchor')
        if args.resume_current_pose and self.start_index == 0:
            raise ValueError('A full lap must use the standard S01 placement')
        self.result.update(course=self.course['name'], visited_stops=[], legs=[],
                           start_anchor=args.start_anchor, full_course=self.start_index == 0)
        self.result['ring_lane_policy'] = self.course.get('ring_lane_policy')
        self.result.pop('maximum_centreline_error_m', None)
        self.subs.append(self.create_subscription(PlanningFactorArray,
            '/planning/planning_factors/path_optimizer', self.optimizer_factor, self.stream))
        self.diagnostic_paths = {}
        self.latest_optimizer_factor = None
        self.last_hazard_key = None
        self.result['hazard_events'] = []
        self.subs.append(self.create_subscription(HazardStatusStamped,
            '/system/emergency/hazard_status', self.hazard_status, self.stream))
        for topic, message_type in (
            ('/planning/scenario_planning/lane_driving/behavior_planning/path', PlanningPath),
            ('/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/debug/mpt_traj', Trajectory),
        ):
            self.subs.append(self.create_subscription(message_type, topic,
                lambda message, key=topic: self.diagnostic_paths.update({key: message}), self.stream))
        for module in ('lane_change_left','lane_change_right','traffic_light','intersection','out_of_lane'):
            topic='/planning/planning_factors/'+module
            self.subs.append(self.create_subscription(PlanningFactorArray,topic,
                lambda message,key=topic:self.diagnostic_paths.update({key:message}),self.stream))
        for module in ('lane_change_left','lane_change_right'):
            topic='/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/'+module
            self.subs.append(self.create_subscription(MarkerArray,topic,
                lambda message,key=topic:self.diagnostic_paths.update({key:message}),self.stream))
        self.reset_optimizer_pub = self.create_publisher(Header, '/planning/path_optimizer/reset_previous', 1)
        self.preferred_client = self.create_client(SetPreferredPrimitive,
            '/planning/mission_planning/mission_planner/set_preferred_primitive')
        self.change_route_client = self.create_client(SetRoutePoints,
            '/api/routing/change_route_points')
        self.reroute_available = False
        self.reroute_received = 0.
        self.subs.append(self.create_subscription(RerouteAvailability,
            '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/output/is_reroute_available',
            self.on_reroute_availability, self.stream))
        self.ring_policy = None
        self.ring_request_attempt = 0.
        self.applied_cap = None
        self.urban_merge_policy = CentralUrbanMergePolicy()
        self.result['stop_at_all_stops'] = args.stop_at_all_stops
        self.return_route_set = False
        self.return_attempt = 0.
        self.pending_station_exits = []
        self.station_exit_attempt = 0.
        self.signal_audit = SignalAudit(self.world, args.map_dir)
        self.result['signal_crossings'] = []
        root = ET.parse(args.map_dir/'lanelet2_map.osm').getroot()
        ways = {w.get('id'):[n.get('ref') for n in w.findall('nd')] for w in root.findall('way')}
        self.lane_endpoints = {}
        for lane in root.findall('relation'):
            bounds = {m.get('role'):ways[m.get('ref')] for m in lane.findall('member')
                      if m.get('role') in ('left','right') and m.get('ref') in ways}
            if len(bounds)==2:
                self.lane_endpoints[int(lane.get('id'))] = (
                    (bounds['left'][0],bounds['right'][0]),
                    (bounds['left'][-1],bounds['right'][-1]))

    def spin(self, seconds):
        super().spin(seconds)
        if getattr(self.args, "interrupted", False):
            raise RuntimeError("Lap interrupted")

    def prefer_station_lanes(self, anchors):
        route = self.last['route']
        ids = {a['lanelet_id'] for a in anchors if a['kind']=='stop'}
        request = SetPreferredPrimitive.Request()
        request.uuid = route.uuid
        request.preferred_primitives = [s.preferred_primitive for s in route.segments]
        changed = []
        for i, section in enumerate(route.segments):
            station = next((p for p in section.primitives if p.id in ids), None)
            if not station:
                continue
            if i+1 < len(route.segments) and not any(
                    self.lane_endpoints[station.id][1] == self.lane_endpoints[p.id][0]
                    for p in route.segments[i+1].primitives):
                # A permanent station preference must not lead into a lane
                # terminal when the next turn is available only from another
                # lane. In the non-service lap retain the mission planner's
                # exit lane; timed station approach/return is a separate task.
                self.event('station_preference_kept_for_exit', station_lanelet=station.id,
                           retained_lanelet=section.preferred_primitive.id)
                continue
            target = station
            # Carry the station lane backward through directly connected route
            # lanes. Do not postpone a legal merge to the last short section.
            for j in range(i,-1,-1):
                if request.preferred_primitives[j].id != target.id:
                    request.preferred_primitives[j] = target
                    changed.append(target.id)
                if j==0:break
                start = self.lane_endpoints[target.id][0]
                target = next((p for p in route.segments[j-1].primitives
                               if self.lane_endpoints[p.id][1]==start),None)
                if target is None:break
        if changed:
            self.call(self.preferred_client, request, 'prefer_outer_station_lanes')
            self.event('station_lane_preferences', lanelets=changed)
            self.spin(.5)

    def update_station_exit(self, remaining):
        if not self.pending_station_exits or time.monotonic() < self.station_exit_attempt:
            return
        if not self.can_request_reroute():
            return
        route = self.last['route']
        station = self.pending_station_exits[0]
        sections = [dict(lanes=[p.id for p in s.primitives],
                         preferred=s.preferred_primitive.id) for s in route.segments]
        protected = {a['lanelet_id'] for a in remaining if a['kind'] == 'stop'}
        changes = station_exit_preferences(sections, self.lane_endpoints, station, protected)
        if not changes:
            self.pending_station_exits.pop(0)
            return
        request = SetPreferredPrimitive.Request()
        request.uuid = route.uuid
        request.preferred_primitives = [s.preferred_primitive for s in route.segments]
        for i, lane in changes.items():
            request.preferred_primitives[i] = next(p for p in route.segments[i].primitives
                                                   if p.id == lane)
        self.station_exit_attempt = time.monotonic() + 2.
        future = self.preferred_client.call_async(request)
        self.until(future.done, 10, 'station exit preference response')
        status = future.result().status
        self.event('station_exit_preference', station_lanelet=station,
                   lanelets=list(changes.values()), status=message_to_ordereddict(status))
        if status.success:
            self.pending_station_exits.pop(0)

    def on_reroute_availability(self, message):
        self.reroute_available = message.availability
        self.reroute_received = time.monotonic()

    def can_request_reroute(self):
        return self.reroute_available and time.monotonic()-self.reroute_received < 1.

    def extend_return_route(self, remaining):
        if self.return_route_set or time.monotonic()<self.return_attempt:
            return False
        if not self.can_request_reroute():
            return False
        self.return_attempt = time.monotonic()+2.
        request = SetRoutePoints.Request()
        request.header.frame_id = 'map'
        request.header.stamp = self.get_clock().now().to_msg()
        request.goal = self.pose(self.course['anchors'][0])
        request.waypoints = [self.pose(a) for a in remaining]
        requested = time.monotonic()
        future = self.change_route_client.call_async(request)
        self.until(future.done, 10, 'rolling return route response')
        status = future.result().status
        self.event('rolling_return_route', status=message_to_ordereddict(status))
        if not status.success:
            return False
        self.until(lambda: self.received.get('route', 0)>requested, 10, 'return route')
        self.return_route_set = True
        self.ring_policy = None
        self.result['legs'].append(message_to_ordereddict(self.last['route']))
        # Keep AUTO, the old trajectory and optimizer history during the handoff.
        return True

    def request_ring_lane(self, lane, initial=False):
        route = self.last['route']
        anchor_id = {-1:'R05', -2:'R02', -3:'R03'}[lane]
        lanelet_id = next(a['lanelet_id'] for a in self.course['anchors'] if a['id']==anchor_id)
        request = SetPreferredPrimitive.Request()
        request.uuid = route.uuid
        request.preferred_primitives = [s.preferred_primitive for s in route.segments]
        matched = False
        for i, section in enumerate(route.segments):
            target = next((p for p in section.primitives if p.id==lanelet_id), None)
            if target:
                request.preferred_primitives[i] = target
                matched = True
        if not matched:
            raise RuntimeError(f'Ring lane {lane} is absent from route primitives')
        future = self.preferred_client.call_async(request)
        self.until(future.done, 10, 'ring lane preference response')
        status = future.result().status
        self.event('ring_lane_request', lane=lane, initial=initial,
                   status=message_to_ordereddict(status))
        if initial and not status.success:
            raise RuntimeError('Initial ring lane preference rejected')
        if status.success:
            self.spin(.3)
        return status.success

    def update_ring_policy(self, wp, rear):
        if self.ring_policy is None:
            return
        if wp.road_id not in (36,37):
            return
        right = wp.transform.rotation.get_right_vector()
        delta = rear-wp.transform.location
        offset = delta.x*right.x+delta.y*right.y
        yaw = math.radians(self.ego.get_transform().rotation.yaw-wp.transform.rotation.yaw)
        yaw = math.atan2(math.sin(yaw),math.cos(yaw))
        if wp.road_id == 37:
            stage_before = self.ring_policy.stage
            target = self.ring_policy.observe(wp.s,wp.lane_id,offset,yaw,
                                              self.get_clock().now().nanoseconds/1e9)
            if (target is not None and self.can_request_reroute() and
                    time.monotonic() >= self.ring_request_attempt):
                self.ring_request_attempt = time.monotonic()+2.
                if self.request_ring_lane(target):
                    self.ring_policy.accepted(target)
            if stage_before != self.ring_policy.stage:
                self.event('ring_stage', stage=self.ring_policy.stage, native_s=wp.s,
                           lane=wp.lane_id, center_offset_m=offset)

    def has_passed_station(self, station):
        return station in self.result['visited_stops']

    def adjust_course_speed_cap(self, cap, reason):
        return cap, reason

    def update_speed_limit(self, wp, rear):
        right = wp.transform.rotation.get_right_vector()
        delta = rear - wp.transform.location
        offset = delta.x * right.x + delta.y * right.y
        yaw = math.radians(self.ego.get_transform().rotation.yaw - wp.transform.rotation.yaw)
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        urban_cap = self.urban_merge_policy.observe(
            wp.road_id, wp.lane_id, offset, yaw,
            self.get_clock().now().nanoseconds / 1e9,
            station_passed=self.has_passed_station('S04'))
        cap = self.args.speed_kmh
        reason = 'route_speed_limits'
        if self.ring_policy and wp.road_id in (36, 37):
            cap = min(cap, self.ring_policy.cap_kmh(wp.s if wp.road_id == 37 else 0))
            reason = 'ring_maneuver'
        if urban_cap is not None:
            cap = min(cap, urban_cap)
            reason = 'S04_exit_preparation'
        cap, reason = self.adjust_course_speed_cap(cap, reason)
        if cap != self.applied_cap:
            limit = VelocityLimit()
            limit.stamp = self.get_clock().now().to_msg()
            limit.max_velocity = cap / 3.6
            limit.sender = 'carla_closed_loop_test'
            self.limit_pub.publish(limit)
            self.applied_cap = cap
            self.event('course_speed_cap', cap_kmh=cap, reason=reason)

    def hazard_status(self, message):
        if not self.case_running: return
        status = message.status
        # Safe diagnostic faults do not trigger MRM; keep this log focused on
        # driving availability changes and their recovery.
        if status.level < 2 and not status.emergency:
            key = (0, False, ())
            if key != self.last_hazard_key:
                self.event('hazard_status', level=0, emergency=False, faults=[])
                self.last_hazard_key = key
            return
        messages = []
        for group in ('diag_safe_fault', 'diag_latent_fault', 'diag_single_point_fault'):
            messages.extend(getattr(status, group))
        key = (status.level, status.emergency,
               tuple((item.name, str(item.level), item.message) for item in messages))
        if key != self.last_hazard_key:
            faults = json.loads(json.dumps([message_to_ordereddict(item) for item in messages],
                default=lambda value: int.from_bytes(value, 'little') if isinstance(value, bytes) else list(value)))
            row = dict(level=status.level, emergency=status.emergency, faults=faults)
            self.result['hazard_events'].append(row)
            self.event('hazard_status', **row)
            self.last_hazard_key = key

    def optimizer_factor(self, message):
        self.latest_optimizer_factor = message
        if self.case_running and message.factors:
            self.result['planning_factors']['path_optimizer'] = message_to_ordereddict(message)

    @staticmethod
    def pose(anchor):
        p = anchor['ros_map']
        pose = Pose()
        # Keep the conservative 4.04 m collision envelope inside the road at
        # outer-lane endpoints, using 0.45 m of the legal inner-side corridor.
        yaw = p['yaw_rad']
        offset = .45 if anchor['kind'] == 'stop' else (
            .4 if anchor['road_id']==37 and anchor['lane_id']==-3 else
            -.4 if anchor['road_id']==37 and anchor['lane_id']==-1 else 0.)
        pose.position.x = p['x'] - math.sin(yaw) * offset
        pose.position.y = p['y'] + math.cos(yaw) * offset
        pose.position.z = p['z']
        pose.orientation.z, pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        return pose

    def initialise(self):
        self.until(lambda: 'mode' in self.last and 'odom' in self.last, 30, 'initial ROS state')
        self.stop()
        self.call(self.clear_client, ClearRoute.Request(), 'clear_old_route')
        start = self.course['anchors'][self.start_index]
        pose = self.pose(start)
        if self.args.resume_current_pose:
            expected = carla.Location(pose.position.x, -pose.position.y, pose.position.z)
            if self.rear_pose().distance(expected) > 3.0:
                raise RuntimeError('Current bus pose is not at the requested diagnostic anchor')
        else:
            t = self.wmap.get_waypoint_xodr(start['road_id'], start['lane_id'], start['s_m']).transform
            t.location = carla.Location(pose.position.x, -pose.position.y, pose.position.z+.05)
            offset = t.rotation.get_forward_vector()*self.base_offset[0]
            t.location -= carla.Location(offset.x, offset.y, offset.z)
            self.ego.set_transform(t)
            self.ego.set_target_velocity(carla.Vector3D())
            self.ego.set_target_angular_velocity(carla.Vector3D())
            self.spin(3)
            self.until(lambda: speed(self.ego)<.05, 10, 'stationary after placement')
        self.collision_sensor = self.world.spawn_actor(
            self.world.get_blueprint_library().find('sensor.other.collision'), carla.Transform(), attach_to=self.ego)
        self.collision_sensor.listen(lambda e: self.collisions.append({'frame': e.frame, 'other': e.other_actor.type_id}))
        limit = VelocityLimit()
        limit.stamp = self.get_clock().now().to_msg()
        limit.max_velocity = self.args.speed_kmh/3.6
        limit.sender = 'carla_closed_loop_test'
        self.limit_pub.publish(limit)
        self.limit_sent = True
        self.spin(.5)
        if start['kind'] == 'stop':
            self.result['visited_stops'].append(start['id'])

    def submit(self, anchors):
        request = SetRoutePoints.Request()
        request.header.frame_id = 'map'
        request.header.stamp = self.get_clock().now().to_msg()
        request.goal = self.pose(anchors[-1])
        request.waypoints = [self.pose(a) for a in anchors[:-1]]
        self.route_sent = True
        requested = time.monotonic()
        self.call(self.route_client, request, 'set_course_route')
        self.until(lambda: self.received.get('route', 0)>requested, 10, 'new route')
        self.prefer_station_lanes(anchors)
        self.ring_policy = None
        if any(a['id']=='R02' for a in anchors) or self.args.start_anchor in ('R02','R03','R04'):
            self.ring_policy = RingPolicy()
            if self.args.start_anchor in ('R03','R04'):
                self.ring_policy.stage = 'outer_cruise'
                self.request_ring_lane(-3, initial=True)
            else:
                self.request_ring_lane(-2, initial=True)
        self.result['legs'].append(message_to_ordereddict(self.last['route']))
        self.until(lambda: self.received.get('trajectory', 0)>requested and
                   len(self.last['trajectory'].points)>2, 30, 'fresh trajectory')
        # Repeated course tests can have an identical geometric goal. Discard
        # the preceding run's fixed optimization while the bus is stopped.
        self.reset_optimizer_pub.publish(Header(stamp=self.get_clock().now().to_msg(), frame_id='map'))
        self.spin(.5)
        self.until(lambda: 'velocity_limit' in self.last and
                   self.last['velocity_limit'].max_velocity<=self.args.speed_kmh/3.6+.01,
                   10, 'speed cap applied')
        if self.args.preflight:
            return
        self.until(lambda: self.last['mode'].is_autonomous_mode_available, 30, 'AUTO available')
        self.until(lambda: time.monotonic()-self.actuation_received<1, 10, 'actuation')
        if not self.last['mode'].is_autoware_control_enabled:
            self.call(self.enable_client, ChangeOperationMode.Request(), 'enable_control')
        self.case_running = True
        self.call(self.auto_client, ChangeOperationMode.Request(), 'engage_lap')
        self.until(lambda: self.last['mode'].mode==OperationModeState.AUTONOMOUS and
                   not self.last['mode'].is_in_transition, 15, 'AUTO transition')

    def drive_leg(self, anchors):
        anchors = list(anchors)
        self.submit(anchors)
        if self.args.preflight:
            return
        started = time.monotonic()
        last_print = started-10
        stopped_since = None
        blocked_drive_since = None
        next_anchor = 0
        previous = self.rear_pose()
        distance = 0.
        progress_position = previous
        last_progress = started
        goal = self.pose(anchors[-1]).position
        goal = carla.Location(goal.x, -goal.y, goal.z)
        while time.monotonic()-started < self.args.duration:
            self.spin(.1)
            if self.course_traffic:
                self.course_traffic.sample()
                if time.monotonic()-self.received.get('objects', 0)>1:
                    raise RuntimeError('Stale object input during traffic test')
            if self.carla.get_world().id != self.world_id:
                raise RuntimeError('World changed')
            if time.monotonic()-self.received.get('odom', 0)>1 or time.monotonic()-self.actuation_received>1:
                raise RuntimeError('Stale localization or actuation')
            rear = self.rear_pose()
            distance += rear.distance(previous)
            previous = rear
            if rear.distance(progress_position)>1:
                progress_position=rear
                last_progress=time.monotonic()
            if time.monotonic()-last_progress>120:
                raise RuntimeError('No progress for 120 seconds; inspect recorded planning factors')
            velocity = speed(self.ego)
            wp = self.wmap.get_waypoint(rear)
            self.update_ring_policy(wp, rear)
            self.update_speed_limit(wp, rear)
            control = self.ego.get_control()
            light = self.ego.get_traffic_light()
            for crossing in self.signal_audit.sample(self.ego):
                self.result['signal_crossings'].append(crossing)
                self.event('signal_stop_line_crossing', **crossing)
                if crossing['violation']:
                    raise RuntimeError('Red/unknown signal stop-line crossing')
            if next_anchor < len(anchors):
                a = anchors[next_anchor]
                p = a['carla']
                if wp.road_id == a['road_id'] and rear.distance(carla.Location(p['x'],p['y'],p['z']))<9:
                    self.event('anchor_passed', anchor=a['id'], road=wp.road_id, lane=wp.lane_id,
                               requested_lane=a['lane_id'], requested_lane_reached=wp.lane_id==a['lane_id'])
                    if a['kind']=='stop':
                        self.result['visited_stops'].append(a['id'])
                        if not self.args.stop_at_all_stops:
                            self.pending_station_exits.append(a['lanelet_id'])
                    next_anchor += 1
            self.update_station_exit(anchors[next_anchor:])
            if (not self.args.stop_at_all_stops and not self.return_route_set and
                    not self.args.end_anchor and
                    'S08' in self.result['visited_stops'] and anchors[-1]['id']=='S10'):
                if self.extend_return_route(anchors[next_anchor:]):
                    anchors.append(self.course['anchors'][0])
                    p = self.pose(anchors[-1]).position
                    goal = carla.Location(p.x, -p.y, p.z)
            error = rear.distance(goal)
            row = dict(sim_time=self.get_clock().now().nanoseconds/1e9,
                       leg_index=len(self.result['legs'])-1,
                       rear_xyz=xyz(rear), speed_mps=velocity, road=wp.road_id, lane=wp.lane_id,
                       native_s=wp.s, yaw_deg=self.ego.get_transform().rotation.yaw,
                       throttle=control.throttle, brake=control.brake, steer=control.steer,
                       traffic_light_id=light.id if light else None,
                       traffic_light_state=str(light.get_state()) if light else None,
                       goal_distance_m=error, distance_m=distance, mode=self.last['mode'].mode, matches=[])
            row['zone'] = 'ring_road' if wp.road_id in (36, 37) else 'urban'
            row['zone_cap_kmh'] = min(self.args.speed_kmh, 50. if row['zone']=='ring_road' else 30.)
            right = wp.transform.rotation.get_right_vector()
            delta = rear-wp.transform.location
            row['lane_center_right_offset_m'] = delta.x*right.x+delta.y*right.y
            self.result['samples'].append(row)
            if time.monotonic()-last_print >=10:
                self.event('lap_progress', **row, next_anchor=anchors[next_anchor]['id'] if next_anchor<len(anchors) else None)
                last_print = time.monotonic()
            if self.collisions: raise RuntimeError('Collision on bus course')
            # Curbs can physically block wheels without a collision-sensor event.
            # Do not keep applying drive torque until the generic 120 s watchdog.
            if velocity < .1 and control.throttle > .4 and control.brake < .05:
                if blocked_drive_since is None: blocked_drive_since = time.monotonic()
                if time.monotonic()-blocked_drive_since > 12:
                    raise RuntimeError('Drive torque blocked for 12 seconds; inspect curb/contact and tracking')
            else:
                blocked_drive_since = None
            if velocity > self.args.speed_kmh/3.6+.5: raise RuntimeError('Speed cap exceeded')
            if velocity > row['zone_cap_kmh']/3.6+.5:
                raise RuntimeError('Road-zone speed cap exceeded')
            if error<1.5 and velocity<.05:
                if stopped_since is None: stopped_since=time.monotonic()
                if time.monotonic()-stopped_since>=5:
                    if next_anchor != len(anchors): raise RuntimeError('Course anchors skipped')
                    if self.args.stop_at_all_stops and anchors[-1]['kind']=='stop':
                        if wp.lane_id != anchors[-1]['lane_id'] or wp.road_id != anchors[-1]['road_id']:
                            raise RuntimeError('Station reached in the wrong lane')
                        self.event('outer_lane_station_stop', station=anchors[-1]['id'],
                                   road=wp.road_id, lane=wp.lane_id, goal_error_m=error)
                    self.event('leg_completed', goal=anchors[-1]['id'], distance_m=distance)
                    self.stop()
                    self.call(self.clear_client, ClearRoute.Request(), 'clear_completed_leg')
                    return
            else:
                stopped_since=None
                if self.last['mode'].mode!=OperationModeState.AUTONOMOUS:
                    raise RuntimeError('Left AUTO before leg goal')
        raise RuntimeError('Lap leg timed out')

    def run(self):
        self.initialise()
        if self.args.with_traffic:
            self.course_traffic = CourseTraffic(self)
            self.course_traffic.spawn()
            self.until(lambda: 'objects' in self.last and len(self.last['objects'].objects)>0,
                       15, 'traffic visible to Autoware')
        remaining = self.course['anchors'][self.start_index+1:]
        if self.args.end_anchor:
            end = next((i for i,a in enumerate(remaining) if a['id']==self.args.end_anchor),None)
            if end is None:raise ValueError('Diagnostic end must follow start anchor')
            remaining = remaining[:end+1]
            self.result['full_course'] = False
        if self.args.stop_at_all_stops and remaining:
            leg = []
            for anchor in remaining:
                leg.append(anchor)
                if anchor['kind']=='stop':
                    self.drive_leg(leg)
                    leg = []
                    if self.args.preflight: break
        elif remaining:
            self.drive_leg(remaining)
        if (not self.args.preflight or not remaining) and not self.return_route_set and not self.args.end_anchor:
            if not self.args.stop_at_all_stops and remaining and not self.args.preflight:
                raise RuntimeError('Rolling return route was not accepted before S10')
            self.drive_leg([self.course['anchors'][0]])
        result = 'lap_completed' if self.result['full_course'] else 'section_completed'
        total_distance = sum(e['distance_m'] for e in self.result['events']
                             if e['event'] == 'leg_completed')
        self.result.update(success=True, result='route_preflight_pass' if self.args.preflight else result,
                           total_distance_m=total_distance)

    def cleanup(self):
        # Keep collision evidence and remove our sensor even if ROS cleanup fails.
        try:
            super().cleanup()
        finally:
            if self.course_traffic:
                self.course_traffic.cleanup()
            self.result['collisions'] = list(self.collisions)
            if self.collision_sensor and self.collision_sensor.is_alive:
                self.collision_sensor.stop()
                self.collision_sensor.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--map-dir', type=Path, default=Path('/home/aw/autoware_data/maps/Town05Bus'))
    parser.add_argument('--duration', type=float, default=1800)
    parser.add_argument('--preflight', action='store_true')
    parser.add_argument('--with-traffic', action='store_true',
                        help='Spawn 8 same-direction and 8 opposing cars; remove owned traffic on exit')
    parser.add_argument('--traffic-layout', choices=['course','following','opposing'], default='course',
                        help='Traffic isolation fixture; requires --with-traffic')
    parser.add_argument('--stop-at-all-stops', action='store_true',
                        help='Validate each outer-lane station as a separate stopping goal; no bell logic')
    parser.add_argument('--speed-kmh', type=float, default=50.,
                        help='Global ceiling; map speed limits and curves can lower this')
    parser.add_argument('--resume-current-pose', action='store_true',
                        help='Diagnostic section only: retain actual stopped pose at start anchor')
    parser.add_argument('--start-anchor', default='S01',
                        help='Diagnostic section start; only S01 can produce lap_completed')
    parser.add_argument('--end-anchor', help='End a diagnostic section early; never a full lap')
    args = parser.parse_args()
    if not 0 < args.speed_kmh <= 50:
        parser.error('speed-kmh must be in (0,50]')
    args.case='bus_lap'; args.minimum_peak_kmh=0.; args.cruise_hold_s=0.
    args.host='127.0.0.1'; args.port=2000; args.input_mode='ground_truth'
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    # Raise only after returning from native CARLA/rclpy callbacks. An exception
    # inside a pybind callback can mask the interruption and break STOP cleanup.
    def interrupt(*_): args.interrupted = True
    signal.signal(signal.SIGINT, interrupt); signal.signal(signal.SIGTERM, interrupt)
    node=None
    try:
        node=BusLap(args); node.run()
    except Exception as exc:
        print(repr(exc), flush=True)
        if node:
            node.result.update(success=False, result='error', error=repr(exc))
            context = {key: message_to_ordereddict(message) for key, message in node.diagnostic_paths.items()}
            if 'odom' in node.last:
                context['/localization/kinematic_state'] = message_to_ordereddict(node.last['odom'])
            if node.latest_optimizer_factor is not None:
                context['/planning/planning_factors/path_optimizer'] = message_to_ordereddict(node.latest_optimizer_factor)
            node.result['failure_context'] = context
    finally:
        args.interrupted = False
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node:
            try: node.cleanup()
            except Exception as exc: node.result.update(success=False, cleanup_error=repr(exc))
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(node.result, indent=2)+'\n')
            node.destroy_node()
        rclpy.shutdown()
    return 0 if node and node.result['success'] else 1


if __name__=='__main__':
    raise SystemExit(main())
