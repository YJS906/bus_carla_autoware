#!/usr/bin/env python3
"""Run the saved ten-stop course through Autoware, without service-stop logic.

Two routes avoid Autoware's repeated-lanelet loop prohibition. The bus stops at
S10 before the final S01 leg. Only initial placement uses CARLA teleportation.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import time

import carla
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Pose
from autoware_planning_msgs.msg import Path as PlanningPath, Trajectory
from std_msgs.msg import Header
from autoware_system_msgs.msg import HazardStatusStamped
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode, ClearRoute, SetRoutePoints
from autoware_internal_planning_msgs.msg import PlanningFactorArray, VelocityLimit
from rosidl_runtime_py.convert import message_to_ordereddict

from carla_bus_control_validation import BusValidation
from carla_closed_loop_validation import speed, xyz


class BusLap(BusValidation):
    def __init__(self, args):
        super().__init__(args)
        self.course = json.loads(args.route.read_text())
        anchors = self.course['anchors']
        self.start_index = next((i for i, a in enumerate(anchors) if a['id'] == args.start_anchor), None)
        if self.start_index is None:
            raise ValueError('Unknown start anchor')
        if args.resume_current_pose and self.start_index == 0:
            raise ValueError('A full lap must use the standard S01 placement')
        self.result.update(course=self.course['name'], visited_stops=[], legs=[],
                           start_anchor=args.start_anchor, full_course=self.start_index == 0)
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
        self.reset_optimizer_pub = self.create_publisher(Header, '/planning/path_optimizer/reset_previous', 1)

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
        offset = .45 if anchor['kind'] == 'stop' else 0.
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
        self.submit(anchors)
        if self.args.preflight:
            return
        started = time.monotonic()
        last_print = started-10
        stopped_since = None
        next_anchor = 0
        previous = self.rear_pose()
        distance = 0.
        progress_position = previous
        last_progress = started
        goal = self.pose(anchors[-1]).position
        goal = carla.Location(goal.x, -goal.y, goal.z)
        while time.monotonic()-started < self.args.duration:
            self.spin(.1)
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
            control = self.ego.get_control()
            light = self.ego.get_traffic_light()
            if next_anchor < len(anchors):
                a = anchors[next_anchor]
                p = a['carla']
                if wp.road_id == a['road_id'] and rear.distance(carla.Location(p['x'],p['y'],p['z']))<9:
                    self.event('anchor_passed', anchor=a['id'], road=wp.road_id, lane=wp.lane_id)
                    if a['kind']=='stop': self.result['visited_stops'].append(a['id'])
                    next_anchor += 1
            error = rear.distance(goal)
            row = dict(sim_time=self.get_clock().now().nanoseconds/1e9,
                       leg_index=len(self.result['legs'])-1,
                       rear_xyz=xyz(rear), speed_mps=velocity, road=wp.road_id, lane=wp.lane_id,
                       native_s=wp.s, yaw_deg=self.ego.get_transform().rotation.yaw,
                       throttle=control.throttle, brake=control.brake, steer=control.steer,
                       traffic_light_id=light.id if light else None,
                       traffic_light_state=str(light.get_state()) if light else None,
                       goal_distance_m=error, distance_m=distance, mode=self.last['mode'].mode, matches=[])
            self.result['samples'].append(row)
            if time.monotonic()-last_print >=10:
                self.event('lap_progress', **row, next_anchor=anchors[next_anchor]['id'] if next_anchor<len(anchors) else None)
                last_print = time.monotonic()
            if self.collisions: raise RuntimeError('Collision on bus course')
            if velocity > self.args.speed_kmh/3.6+.5: raise RuntimeError('Speed cap exceeded')
            if error<1.5 and velocity<.05:
                if stopped_since is None: stopped_since=time.monotonic()
                if time.monotonic()-stopped_since>=5:
                    if next_anchor != len(anchors): raise RuntimeError('Course anchors skipped')
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
        remaining = self.course['anchors'][self.start_index+1:]
        if remaining:
            self.drive_leg(remaining)
        if not self.args.preflight or not remaining:
            self.drive_leg([self.course['anchors'][0]])
        result = 'lap_completed' if self.start_index == 0 else 'section_completed'
        total_distance = sum(e['distance_m'] for e in self.result['events']
                             if e['event'] == 'leg_completed')
        self.result.update(success=True, result='route_preflight_pass' if self.args.preflight else result,
                           total_distance_m=total_distance)

    def cleanup(self):
        # Keep collision evidence and remove our sensor even if ROS cleanup fails.
        try:
            super().cleanup()
        finally:
            self.result['collisions'] = list(self.collisions)
            if self.collision_sensor and self.collision_sensor.is_alive:
                self.collision_sensor.stop()
                self.collision_sensor.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=1800)
    parser.add_argument('--preflight', action='store_true')
    parser.add_argument('--resume-current-pose', action='store_true',
                        help='Diagnostic section only: retain actual stopped pose at start anchor')
    parser.add_argument('--start-anchor', default='S01',
                        help='Diagnostic section start; only S01 can produce lap_completed')
    args = parser.parse_args()
    args.case='bus_lap'; args.speed_kmh=20.; args.minimum_peak_kmh=0.; args.cruise_hold_s=0.
    args.host='127.0.0.1'; args.port=2000; args.input_mode='ground_truth'
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*_): raise RuntimeError('Lap interrupted')
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
