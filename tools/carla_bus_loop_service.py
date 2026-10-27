"""Continuous full-course driving with optional S10 service on each circuit."""
import math
import time

import carla
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ClearRoute, SetRoutePoints
from autoware_planning_msgs.srv import SetPreferredPrimitive
from rosidl_runtime_py.convert import message_to_ordereddict

from carla_bus_service import BusService
from carla_bus_service_policy import ServicePolicy, acceptance_distance
from carla_bus_loop_policy import LoopProgress, landmarks_between, random_bell_schedule
from carla_bus_ring_policy import RingPolicy
from carla_bus_route_policy import CentralUrbanMergePolicy, station_exit_preferences
from carla_closed_loop_validation import speed


class BusLoopService(BusService):
    def __init__(self, args, channel):
        super().__init__(args, channel)
        self.anchors = self.course['anchors']
        self.by_id = {a['id']: a for a in self.anchors}
        self.length = self.course['length_m']
        self.reference = [p for segment in self.course['segments'] for p in segment['points']]
        self.tracker = LoopProgress(self.reference, self.length)
        self.progress_m = 0.
        self.start_anchor = self.by_id['S01']
        self.station_cycle = 0
        self.lap_count = 0
        self.next_sequence = 1
        self.current_station = 'S01'
        self.goal_id = 'S10'
        self.goal_cycle = 0
        self.route_attempt = 0.
        self.service_restart_pending = False
        self.random_schedule = (random_bell_schedule(args.random_bell_seed, args.laps,
                                 self.station['route_distance_m'], self.length)
                                if args.random_bell_seed is not None and not getattr(args,'demand_service',False) else [])
        self.injected_laps = set()
        # Finite validation ends 12 m AFTER S01, so S01 itself is a through point.
        wp = self.wmap.get_waypoint_xodr(44, -2, self.start_anchor['s_m']+12.)
        t = wp.transform
        self.end_anchor = dict(id='TEST_END', kind='stop', name='검증 종료 지점',
                              road_id=44, lane_id=-2, lanelet_id=self.start_anchor['lanelet_id'],
                              carla=dict(x=t.location.x,y=t.location.y,z=t.location.z,yaw_deg=t.rotation.yaw),
                              ros_map=dict(x=t.location.x,y=-t.location.y,z=t.location.z,
                                           yaw_rad=-math.radians(t.rotation.yaw)),
                              route_distance_m=12.)
        self.result.update(case='bus_loop_service', full_course=True, terminal=None,
                           requested_laps=args.laps, completed_laps=0,
                           random_bell_seed=args.random_bell_seed, random_bell_schedule=self.random_schedule,
                           station_passes=[], lap_crossings=[], route_handoffs=[], restart_events=[],
                           stop_reason='S01 is a through point; finite tests end 12 m later')

    @property
    def station_distance(self):
        return self.station_cycle*self.length+self.station['route_distance_m']

    def has_passed_station(self, station):
        return self.progress_m % self.length >= self.by_id[station]['route_distance_m']-9.

    def prefer_station_lanes(self, anchors):
        super().prefer_station_lanes(anchors)
        if not any(a['id']=='S03' for a in anchors):
            return
        # S03 is a through stop in this S10-only service. Select its connected
        # exit lane in advance, avoiding a late merge ending at the junction.
        route=self.last['route']
        sections=[dict(lanes=[p.id for p in s.primitives],preferred=s.preferred_primitive.id)
                  for s in route.segments]
        changes=station_exit_preferences(sections,self.lane_endpoints,self.by_id['S03']['lanelet_id'],
                                       {self.station['lanelet_id']})
        if not changes:
            return
        first=min(changes)
        target=changes[first]
        for i in range(first-1,-1,-1):
            candidates=[p.id for p in route.segments[i].primitives
                        if self.lane_endpoints[p.id][1]==self.lane_endpoints[target][0]]
            if not candidates:break
            preferred=sections[i]['preferred']
            target=preferred if preferred in candidates else candidates[0]
            if target!=preferred:changes[i]=target
        request=SetPreferredPrimitive.Request()
        request.uuid=route.uuid
        request.preferred_primitives=[s.preferred_primitive for s in route.segments]
        for i,lane in changes.items():
            request.preferred_primitives[i]=next(p for p in route.segments[i].primitives if p.id==lane)
        self.call(self.preferred_client,request,'S03_early_through_exit')
        self.event('S03_through_exit_lanes',lanelets=list(changes.values()))
        self.spin(.5)

    def adjust_course_speed_cap(self, cap, reason):
        if self.policy.requested and self.station_distance-self.progress_m < 160:
            return min(cap,20.), 'S10_service_approach'
        return cap, reason

    def update_progress(self, rear):
        self.progress_m = self.tracker.observe(rear.x, rear.y)

    def future_anchors(self, goal_id, cycle):
        goal = self.end_anchor if goal_id == 'TEST_END' else self.by_id[goal_id]
        target = cycle*self.length+goal['route_distance_m']
        remaining = [a for _,a in landmarks_between(self.anchors,self.length,self.progress_m+9.,target)
                     if a['id'] != goal_id or goal_id == 'TEST_END']
        return remaining+[goal]

    def goal_distance(self):
        a = self.end_anchor if self.goal_id == 'TEST_END' else self.by_id[self.goal_id]
        return self.goal_cycle*self.length+a['route_distance_m']-self.progress_m

    def next_horizon(self):
        cycle = self.station_cycle+1
        return ('TEST_END',self.args.laps) if self.args.laps and cycle >= self.args.laps else ('R02',cycle)

    def handoff(self, goal_id, cycle):
        if time.monotonic()<self.route_attempt or not self.can_request_reroute():
            return False
        self.route_attempt = time.monotonic()+2.
        anchors = self.future_anchors(goal_id,cycle)
        request = SetRoutePoints.Request()
        request.header.frame_id = 'map'
        request.header.stamp = self.get_clock().now().to_msg()
        request.goal = self.pose(anchors[-1])
        request.waypoints = [self.pose(a) for a in anchors[:-1]]
        requested = time.monotonic()
        future = self.change_route_client.call_async(request)
        self.until(future.done,10,'continuous route response')
        status = future.result().status
        row = dict(goal=goal_id,cycle=cycle+1,progress_m=self.progress_m,
                   status=message_to_ordereddict(status))
        self.event('continuous_route_handoff',**row)
        self.result['route_handoffs'].append(row)
        if not status.success:
            return False
        self.until(lambda:self.received.get('route',0)>requested,10,'continuous route')
        self.goal_id,self.goal_cycle = goal_id,cycle
        self.result['legs'].append(message_to_ordereddict(self.last['route']))
        self.prefer_station_lanes(anchors)
        if any(a['id']=='R02' for a in anchors) and self.ring_policy is None:
            self.ring_policy=RingPolicy()
            self.request_ring_lane(-2,initial=True)
        return True

    def ui(self):
        velocity=speed(self.ego)
        remaining=self.station_distance-self.progress_m
        valid_target=not self.args.laps or self.station_cycle<self.args.laps
        next_id=self.anchors[self.next_sequence%len(self.anchors)]['id']
        reason=''
        if self.phase=='DWELLING':reason='승하차 대기'
        elif self.phase=='READY':reason='출발 대기'
        elif velocity<.1 and self.phase=='DRIVING':
            reason='교통·경로 조건 대기'
            message=self.diagnostic_paths.get('/planning/planning_factors/traffic_light')
            if message:
                stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
                sim=self.get_clock().now().nanoseconds/1e9
                if 0<=sim-stamp<1. and any(0<=p.distance<25 for f in message.factors for p in f.control_points):
                    reason='신호 대기'
        connected=time.monotonic()-self.received.get('odom',0)<2 and time.monotonic()-self.actuation_received<2
        self.channel.publish(phase=self.phase,message=self.message,connected=connected,
                             requested=self.policy.requested,served=self.policy.completed,
                             accepting=valid_target and self.phase in ('READY','DRIVING') and
                                       not self.policy.completed and remaining>acceptance_distance(velocity),
                             speed_kmh=velocity*3.6,dwell_s=self.policy.remaining_s,
                             current_stop=self.current_station,next_stop=next_id,
                             lap_number=self.lap_count+1,completed_laps=self.lap_count,
                             stop_reason=reason,remaining_m=remaining)

    def service_remaining(self):
        return self.station_distance-self.progress_m

    def accepting_requests(self):
        return (self.phase in ('READY','DRIVING') and not self.policy.completed and
                (not self.args.laps or self.station_cycle<self.args.laps))

    def process_landmarks(self, rear, velocity, wp):
        while True:
            index=self.next_sequence%len(self.anchors)
            cycle=self.next_sequence//len(self.anchors)
            anchor=self.anchors[index]
            target=cycle*self.length+anchor['route_distance_m']
            margin=2. if index==0 or anchor['id']=='S10' else -8.
            if self.progress_m<target+margin:
                break
            self.event('loop_anchor_passed',anchor=anchor['id'],cycle=cycle+1,
                       road=wp.road_id,lane=wp.lane_id,progress_m=self.progress_m)
            if anchor['kind']=='stop':
                self.current_station=anchor['id']
                self.result['visited_stops'].append(anchor['id'])
                self.pending_station_exits.append(anchor['lanelet_id'])
            if index==0:
                self.lap_count=cycle
                row=dict(lap=cycle,sim_time=self.get_clock().now().nanoseconds/1e9,
                         speed_mps=velocity,mode=self.last['mode'].mode,travel_distance_m=self.travel_m)
                self.result['lap_crossings'].append(row)
                self.result['completed_laps']=cycle
                self.event('lap_completed_through_S01',**row)
                self.urban_merge_policy=CentralUrbanMergePolicy()
                self.ring_policy=None
            if anchor['id']=='S10':
                if self.policy.requested:
                    raise RuntimeError('Passed S10 with an unserved bell request')
                served=self.policy.completed
                row=dict(lap=cycle+1,served=served,speed_mps=velocity,progress_m=self.progress_m)
                self.result['station_passes'].append(row)
                self.event('S10_service_pass' if served else 'S10_pass_without_request',**row)
                self.station_cycle=cycle+1
                self.policy=ServicePolicy()
                self.message='S10 서비스 완료 · 순환 계속' if served else '요청 없음 · S10 통과 · 순환 계속'
            self.next_sequence+=1

    def run(self):
        self.initialise()
        self.phase='READY'
        self.message='전체 코스 연속 순환 · 하차벨 정차 대상은 S10입니다'
        self.event('random_bell_schedule',seed=self.args.random_bell_seed,schedule=self.random_schedule)
        if self.args.bell:self.channel.commands.put('bell')
        if self.args.auto_start:self.channel.commands.put('start')
        while True:
            self.spin(.1)
            start=self.commands()
            self.ui()
            if start:
                if time.monotonic()-self.received.get('odom',0)>1 or time.monotonic()-self.actuation_received>1:
                    raise RuntimeError('Cannot depart with stale inputs')
                break
        self.phase='DRIVING'
        self.submit(self.future_anchors('S10',0))
        started=time.monotonic()
        terminal_since=None
        blocked_since=None
        while time.monotonic()-started<self.args.duration:
            sim,rear,velocity,wp=self.observe()
            self.process_landmarks(rear,velocity,wp)
            for plan in self.random_schedule:
                if plan['request'] and plan['lap'] not in self.injected_laps and self.progress_m>=plan['progress_m']:
                    self.channel.commands.put(dict(action='bell',source='random_test'))
                    self.injected_laps.add(plan['lap'])
                    self.event('random_bell_pressed',lap=plan['lap'],progress_m=self.progress_m,seed=self.args.random_bell_seed)
            self.commands()
            if self.policy.requested and self.goal_id=='S10' and self.goal_cycle==self.station_cycle:
                self.policy.committed=True
            elif self.policy.requested and not self.policy.committed and self.station_distance-self.progress_m<350:
                if self.handoff('S10',self.station_cycle):self.policy.committed=True
                elif self.station_distance-self.progress_m<=acceptance_distance(velocity):
                    raise RuntimeError('Accepted bell could not be routed before S10 deadline')
            if self.goal_id=='S10' and not self.policy.requested and self.progress_m>=self.goal_cycle*self.length+self.by_id['S08']['route_distance_m']:
                if not self.handoff(*self.next_horizon()) and self.goal_distance()<acceptance_distance(velocity):
                    raise RuntimeError('Could not extend no-request route past S10')
            if self.goal_id=='R02' and self.progress_m>=self.goal_cycle*self.length+2.:
                if not self.handoff('S10',self.goal_cycle) and self.goal_distance()<acceptance_distance(velocity):
                    raise RuntimeError('Could not extend continuous route before temporary goal')
            self.update_ring_policy(wp,rear)
            self.update_speed_limit(wp,rear)
            zone_cap=min(self.args.speed_kmh,50 if wp.road_id in (36,37) else 30)
            if velocity>zone_cap/3.6+.5:raise RuntimeError('Road-zone speed cap exceeded')
            self.update_station_exit(self.future_anchors(self.goal_id,self.goal_cycle))
            if self.policy.requested and self.policy.committed:
                longitudinal,lateral=self.station_errors(rear)
                lane_ok=wp.road_id==43 and wp.lane_id==-2 and abs(self.station_distance-self.progress_m)<5
                complete=self.policy.observe(sim,velocity,longitudinal,lateral,lane_ok)
                self.phase='DWELLING' if self.policy.stopped_since is not None else 'DRIVING'
                if complete:
                    row=dict(lap=self.station_cycle+1,station='S10',sim_time=sim,
                             hold_sim_s=sim-self.policy.stopped_since,longitudinal_error_m=longitudinal,
                             lateral_error_m=lateral,speed_mps=velocity,road=wp.road_id,lane=wp.lane_id)
                    self.policy.complete()
                    self.result['service_events'].append(row)
                    self.event('service_completed',**row)
                    self.phase='RESTARTING'
                    self.message='S10 서비스 완료 · 요청 해제 · 다음 바퀴로 이어갑니다'
                    self.ui()
                    self.stop()
                    self.call(self.clear_client,ClearRoute.Request(),'clear_service_route')
                    self.observe()
                    self.goal_id,self.goal_cycle=self.next_horizon()
                    self.ring_policy=None
                    self.submit(self.future_anchors(self.goal_id,self.goal_cycle))
                    self.restart_origin=rear
                    self.result['restart_verified']=False
                    self.service_restart_pending=True
                    self.phase='DRIVING'
            if self.service_restart_pending and self.result['restart_verified']:
                self.result['restart_events'].append(dict(lap=self.station_cycle,sim_time=sim))
                self.service_restart_pending=False
            if self.goal_id=='TEST_END':
                p=self.pose(self.end_anchor).position
                error=rear.distance(carla.Location(p.x,-p.y,p.z))
                if error<1.5 and velocity<.05:
                    terminal_since=sim if terminal_since is None else terminal_since
                    if sim-terminal_since>=5:
                        if self.lap_count!=self.args.laps:raise RuntimeError('Validation ended before requested laps')
                        if self.service_restart_pending:raise RuntimeError('Service restart not verified')
                        self.event('validation_end_stop',completed_laps=self.lap_count,goal_error_m=error)
                        self.result.update(success=True,result='continuous_service_validation_completed',
                                           travel_distance_m=self.travel_m,service_completed=bool(self.result['service_events']),
                                           bell_pending=self.policy.requested)
                        return
                else:terminal_since=None
            mode=self.last['mode'].mode
            if mode!=OperationModeState.AUTONOMOUS and not (mode==OperationModeState.STOP and
                    (self.phase=='DWELLING' or terminal_since is not None)):
                raise RuntimeError('Left AUTO outside service or validation-end stop')
            control=self.ego.get_control()
            if velocity<.1 and control.throttle>.4 and control.brake<.05:
                blocked_since=blocked_since or time.monotonic()
                if time.monotonic()-blocked_since>12:raise RuntimeError('Drive torque blocked for 12 seconds')
            else:blocked_since=None
            self.ui()
        raise RuntimeError('Continuous service validation timed out')
