"""Full-course stop demand: waiting passengers OR a latched alighting request."""
import hashlib
import json
import math
import queue
import time

import carla
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ClearRoute
from autoware_planning_msgs.srv import SetPreferredPrimitive

from carla_bus_demand_policy import StopDemand, approach_cap, bell_target, comparison_plan, passenger_counts, rolling_horizon
from carla_bus_lap import BusLap
from carla_bus_loop_policy import landmarks_between
from carla_bus_loop_service import BusLoopService
from carla_bus_passengers import WaitingPassengers
from carla_bus_service_profile import StationManeuverProfile
from carla_bus_route_policy import CentralUrbanMergePolicy
from carla_closed_loop_validation import speed


class BusDemandService(BusLoopService):
    def __init__(self, args, channel):
        self.passengers = None
        self.maneuver_profile = None
        super().__init__(args, channel)
        config = json.loads(args.service_stops.read_text())
        if config['route_sha256'] != hashlib.sha256(args.route.read_bytes()).hexdigest():
            raise ValueError('Service goals do not match the selected route')
        self.goals = {a['id']:a for a in config['stops']}
        ids = [a['id'] for a in self.course['stops'][1:]]+['S01']
        if set(self.goals) != set(ids):
            raise ValueError('Expected one service goal for every station')
        cycles = args.laps or math.ceil(args.duration*args.speed_kmh/3.6/self.length)+2
        if args.random_bell_seed is not None:
            counts, comparison = comparison_plan(args.random_bell_seed, ids, cycles)
        else:
            counts = passenger_counts('S03:2,S06:1,S09:2', ids)
            comparison = [dict(circuit=c+1,station=name,bell=False,
                               expected_waiting=counts[name] if c==0 else 0,trigger_fraction=.1)
                          for c in range(cycles) for name in ids]
        if args.passenger_stops is not None:
            counts = passenger_counts(args.passenger_stops, ids)
            for row in comparison:
                row['expected_waiting'] = counts[row['station']] if row['circuit']==1 else 0
        self.waiting_counts = counts
        self.comparison = comparison
        self.demands = []
        previous = -6.
        for row in comparison:
            name = row['station']
            cycle = row['circuit']-1+(name=='S01')
            distance = cycle*self.length+self.goals[name]['route_distance_m']
            key = f'{cycle}:{name}'
            demand = StopDemand(key,name,distance,waiting=counts[name])
            self.demands.append(demand)
            begin = max(0.,previous+9.)
            end = distance-35.
            if end <= begin:
                raise ValueError('Stations are too close for the automatic request fixture')
            row.update(key=key,cycle=cycle,distance_m=distance,
                       trigger_progress_m=begin+(end-begin)*row['trigger_fraction'])
            previous = distance
        self.stop_index = 0
        self.injected_requests = set()
        self.current_station = 'S01'
        self.station = self.goals[self.demands[0].station]
        self.policy = self.demands[0].dwell
        self.last_passenger_sample = 0.
        self.service_restart_key = None
        self.result.update(case='bus_demand_service',service_station='all',
                           stop_policy='waiting_passengers OR bell',slow_pass_cap_kmh=10.,
                           waiting_passenger_configuration=counts,comparison_plan=comparison,
                           service_goal_configuration=config,station_passes=[],
                           random_bell_schedule=[],late_request_policy='carry to next safely reachable station',
                           passenger_input='owned stationary CARLA walkers in designated sidewalk areas (GT)')

    @property
    def current_demand(self):
        return self.demands[self.stop_index] if self.stop_index<len(self.demands) else None

    def bell_pending(self):
        return any(d.bell for d in self.demands[self.stop_index:])

    def goal_anchor(self, name):
        return self.end_anchor if name=='TEST_END' else self.goals.get(name,self.by_id.get(name))

    def future_anchors(self, goal_id, cycle):
        goal = self.goal_anchor(goal_id)
        target = cycle*self.length+goal['route_distance_m']
        remaining = [a for _,a in landmarks_between(self.anchors,self.length,self.progress_m+9.,target)
                     if a['id']!=goal_id]
        return remaining+[goal]

    def goal_distance(self):
        return self.goal_cycle*self.length+self.goal_anchor(self.goal_id)['route_distance_m']-self.progress_m

    def prefer_station_lanes(self, anchors):
        # Intermediate stations have no active demand. Keep the mission
        # planner's connected exit lane instead of forcing a curb-side merge
        # that can remain active until after the next request's turn deadline.
        BusLap.prefer_station_lanes(self,anchors[-1:])
        if anchors[-1]['id']=='S06' or not any(a['id']=='S06' for a in anchors):
            return
        # R06's curved median needs the proven outer approach. Finish that
        # approach before S06, retaining its inner exit lane when not serving.
        # Extending the outer preference through S06 delayed its left-turn
        # preparation until a route replacement could only select a reverse start.
        route=self.last['route'];station=self.by_id['S06']['lanelet_id']
        index=next((i for i,s in enumerate(route.segments)
                    if any(p.id==station for p in s.primitives)),None)
        if index is None:return
        request=SetPreferredPrimitive.Request(uuid=route.uuid,
            preferred_primitives=[s.preferred_primitive for s in route.segments])
        target=next(p for p in route.segments[index].primitives if p.id==station)
        changed=[]
        for i in range(index-1,-1,-1):
            start=self.lane_endpoints[target.id][0]
            target=next((p for p in route.segments[i].primitives
                         if self.lane_endpoints[p.id][1]==start),None)
            if target is None:break
            if request.preferred_primitives[i].id!=target.id:
                request.preferred_primitives[i]=target;changed.append(target.id)
        if changed:
            self.call(self.preferred_client,request,'S06_through_approach')
            self.event('S06_through_approach_lanes',approach_lanelets=changed,
                       station_exit_lanelet=request.preferred_primitives[index].id)
            self.spin(.5)

    def observe(self):
        observation=super().observe()
        velocity=self.ego.get_velocity()
        forward=self.ego.get_transform().get_forward_vector()
        signed=velocity.x*forward.x+velocity.y*forward.y+velocity.z*forward.z
        self.result['samples'][-1]['signed_speed_mps']=signed
        if signed < -.2:
            raise RuntimeError('Reverse motion is prohibited during bus service')
        return observation

    def has_passed_station(self, station):
        return self.progress_m%self.length >= self.by_id[station]['route_distance_m']-9.

    def adjust_course_speed_cap(self, cap, reason):
        velocity = speed(self.ego)
        for demand in self.demands[max(0,self.stop_index-1):self.stop_index+1]:
            slow = approach_cap(demand.distance_m-self.progress_m,velocity)
            if slow is not None:
                return min(cap,slow),'station_slow_approach_or_pass'
        return cap,reason

    def select_goal(self):
        if self.args.laps and self.current_demand is None:
            return 'TEST_END',self.args.laps
        for demand,row in zip(self.demands[self.stop_index:],self.comparison[self.stop_index:]):
            if demand.required and demand.distance_m-self.progress_m < self.length-80.:
                return demand.station,row['cycle']
        return rolling_horizon(self.progress_m,self.length,self.by_id['S08']['route_distance_m'],self.args.laps)

    def sample_passengers(self, force=False):
        if not force and time.monotonic()-self.last_passenger_sample<.5:
            return
        self.waiting_counts = self.passengers.sample()
        self.last_passenger_sample = time.monotonic()
        for demand in self.demands[self.stop_index:]:
            demand.waiting = self.waiting_counts[demand.station]

    def commands(self):
        start = False
        while True:
            try: command = self.channel.commands.get_nowait()
            except queue.Empty: break
            source = command.get('source','unspecified') if isinstance(command,dict) else 'unspecified'
            expected = command.get('expected_key') if isinstance(command,dict) else None
            action = command.get('action') if isinstance(command,dict) else command
            if action=='start' and self.phase=='READY':
                start=True
            elif action=='bell':
                if self.args.random_bell_seed is not None and source!='random_test':
                    self.event('manual_bell_ignored_during_random_test',source=source)
                    continue
                if self.phase not in ('READY','DRIVING','DWELLING'):
                    continue
                target,outcome=bell_target(self.demands[self.stop_index:],self.progress_m,
                                          speed(self.ego),dwelling=self.phase=='DWELLING')
                if expected and (target is None or target.key!=expected):
                    raise RuntimeError('Random bell could not reach its planned station')
                if target:
                    target.bell=True
                    self.message=f'{target.station} 하차 요청 '+('이월 접수' if outcome=='carried_forward' else '접수')
                else:self.message='검증 종료 구간 · 새 하차 요청 접수 종료'
                row=dict(outcome=outcome,source=source,station=target.station if target else None,
                         key=target.key if target else None,progress_m=self.progress_m,
                         remaining_m=target.distance_m-self.progress_m if target else None,
                         sim_time=self.get_clock().now().nanoseconds/1e9)
                self.result['bell_events'].append(row)
                self.event('bell_request',**row)
        return start

    def ui(self):
        demand=self.current_demand
        velocity=speed(self.ego)
        bell=next((d for d in self.demands[self.stop_index:] if d.bell),None)
        reason=''
        reasons=demand.reasons if demand else []
        label='승차 + 하차' if len(reasons)==2 else '승차' if reasons==['waiting_passengers'] else '하차' if reasons else '요청 없음 · 서행 통과'
        if self.phase=='DWELLING':reason=label+' · 5초 대기'
        elif self.phase=='READY':reason='출발 대기'
        elif self.phase=='RESTARTING':reason='승하차 완료 · 재출발 준비'
        elif velocity<.1:
            reason='교통·경로 조건 대기'
            message=self.diagnostic_paths.get('/planning/planning_factors/traffic_light')
            if message:
                stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
                sim=self.get_clock().now().nanoseconds/1e9
                if 0<=sim-stamp<1 and any(0<=p.distance<25 for f in message.factors for p in f.control_points):
                    reason='신호 대기'
            if self.goal_id=='TEST_END' and self.goal_distance()<15:reason='검증 종료점 정차'
        connected=time.monotonic()-self.received.get('odom',0)<2 and time.monotonic()-self.actuation_received<2
        self.channel.publish(phase=self.phase,message=self.message,connected=connected,
            requested=bell is not None,bell_target=bell.station if bell else None,
            accepting=demand is not None and self.phase in ('READY','DRIVING','DWELLING'),
            speed_kmh=velocity*3.6,dwell_s=demand.dwell.remaining_s if demand else 0,
            current_stop=self.current_station,next_stop=demand.station if demand else '검증 종료',
            waiting_passengers=demand.waiting if demand else 0,
            total_waiting_passengers=sum(self.waiting_counts.values()),
            boarded_passengers=self.passengers.boarded if self.passengers else 0,
            service_reason=label,stop_reason=reason,
            lap_number=self.lap_count+1,completed_laps=self.lap_count)

    def process_landmarks(self, rear, velocity, wp):
        while True:
            index=self.next_sequence%len(self.anchors)
            cycle=self.next_sequence//len(self.anchors)
            anchor=self.anchors[index]
            active=self.current_demand
            servicing=active and active.station==anchor['id'] and (active.required or active.dwell.completed)
            margin=2. if index==0 or servicing else -8.
            if self.progress_m<cycle*self.length+anchor['route_distance_m']+margin:break
            self.event('loop_anchor_passed',anchor=anchor['id'],cycle=cycle+1,
                       road=wp.road_id,lane=wp.lane_id,progress_m=self.progress_m)
            if anchor['kind']=='stop':
                self.result['visited_stops'].append(anchor['id'])
                self.pending_station_exits.append(anchor['lanelet_id'])
            if index==0:
                self.lap_count=cycle
                row=dict(lap=cycle,sim_time=self.get_clock().now().nanoseconds/1e9,
                         speed_mps=velocity,mode=self.last['mode'].mode,travel_distance_m=self.travel_m)
                self.result['lap_crossings'].append(row)
                self.result['completed_laps']=cycle
                self.event('lap_completed_through_S01',**row)
                self.urban_merge_policy=CentralUrbanMergePolicy();self.ring_policy=None
            self.next_sequence+=1

    def run(self):
        self.initialise()
        self.maneuver_profile=StationManeuverProfile(self)
        self.maneuver_profile.activate()
        self.passengers=WaitingPassengers(self.world,self.course,self.waiting_counts,self.event)
        self.result['passengers']=self.passengers.records
        self.passengers.spawn();self.sample_passengers(force=True)
        self.phase='READY';self.message='승객 또는 하차벨 → 정차 · 둘 다 없으면 서행 통과'
        self.event('demand_comparison_plan',seed=self.args.random_bell_seed,plan=self.comparison)
        if self.args.bell:self.channel.commands.put('bell')
        if self.args.auto_start:self.channel.commands.put('start')
        while True:
            self.spin(.1);self.sample_passengers();start=self.commands();self.ui()
            if start:break
        if time.monotonic()-self.received.get('odom',0)>1 or time.monotonic()-self.actuation_received>1:
            raise RuntimeError('Cannot depart with stale inputs')
        self.phase='DRIVING'
        self.goal_id,self.goal_cycle=self.select_goal()
        self.submit(self.future_anchors(self.goal_id,self.goal_cycle))
        started=time.monotonic();terminal_since=None;blocked_since=None
        while time.monotonic()-started<self.args.duration:
            sim,rear,velocity,wp=self.observe()
            self.sample_passengers();self.process_landmarks(rear,velocity,wp)
            current=self.current_demand
            if current and self.progress_m>current.distance_m+8.:
                if current.required:raise RuntimeError('Passed a requested station without service: '+current.key)
                row=dict(key=current.key,station=current.station,served=current.dwell.completed,
                         progress_m=self.progress_m,speed_mps=velocity,waiting=current.waiting)
                self.result['station_passes'].append(row)
                self.event('demand_station_pass',**row)
                self.current_station=current.station
                self.stop_index+=1;current=self.current_demand
            if current:
                self.station=self.goals[current.station];self.policy=current.dwell
            for row in self.comparison:
                if row['bell'] and row['key'] not in self.injected_requests and self.progress_m>=row['trigger_progress_m']:
                    self.channel.commands.put(dict(action='bell',source='random_test',expected_key=row['key']))
                    self.injected_requests.add(row['key'])
                    self.event('random_bell_pressed',station=row['station'],key=row['key'],progress_m=self.progress_m)
            self.commands()
            desired=self.select_goal()
            if desired!=(self.goal_id,self.goal_cycle):
                changed=self.handoff(*desired)
                desired_remaining=desired[1]*self.length+self.goal_anchor(desired[0])['route_distance_m']-self.progress_m
                if not changed and ((current and current.required and current.distance_m-self.progress_m<20.) or
                                    self.goal_distance()<25. or desired_remaining<25.):
                    raise RuntimeError('Could not safely change route before station deadline')
            if current and current.required and (self.goal_id,self.goal_cycle)==(current.station,self.comparison[self.stop_index]['cycle']):
                current.commit()
            self.update_ring_policy(wp,rear);self.update_speed_limit(wp,rear)
            if velocity>min(self.args.speed_kmh,50 if wp.road_id in (36,37) else 30)/3.6+.5:
                raise RuntimeError('Road-zone speed cap exceeded')
            for d in self.demands[max(0,self.stop_index-1):self.stop_index+1]:
                if abs(d.distance_m-self.progress_m)<10 and velocity>10/3.6+.5:
                    raise RuntimeError('Station slow approach/pass speed exceeded')
            self.update_station_exit(self.future_anchors(self.goal_id,self.goal_cycle))
            if current and current.required and current.dwell.committed:
                longitudinal,lateral=self.station_errors(rear)
                correct=(wp.road_id==self.station['road_id'] and wp.lane_id==self.station['lane_id'] and
                         abs(current.distance_m-self.progress_m)<5)
                complete=current.dwell.observe(sim,velocity,longitudinal,lateral,correct)
                self.phase='DWELLING' if current.dwell.stopped_since is not None else 'DRIVING'
                if self.phase=='DWELLING':
                    self.message=current.station+' 승하차 대기'
                if complete:
                    reasons=current.reasons
                    boarded=self.passengers.board(current.station)
                    event=dict(key=current.key,station=current.station,reasons=reasons,boarded=boarded,
                               sim_time=sim,hold_sim_s=sim-current.dwell.stopped_since,
                               longitudinal_error_m=longitudinal,lateral_error_m=lateral,
                               speed_mps=velocity,road=wp.road_id,lane=wp.lane_id)
                    current.complete();self.sample_passengers(force=True)
                    self.result['service_events'].append(event);self.event('service_completed',**event)
                    self.phase='RESTARTING';self.message=current.station+' 승하차 완료 · 재출발'
                    self.ui();self.stop();self.call(self.clear_client,ClearRoute.Request(),'clear_service_route')
                    self.observe()
                    self.goal_id,self.goal_cycle=self.select_goal()
                    self.submit(self.future_anchors(self.goal_id,self.goal_cycle))
                    self.restart_origin=rear;self.result['restart_verified']=False
                    self.service_restart_key=current.key;self.phase='DRIVING'
            if self.service_restart_key and self.result['restart_verified']:
                self.result['restart_events'].append(dict(key=self.service_restart_key,sim_time=sim))
                self.service_restart_key=None
            if self.goal_id=='TEST_END':
                p=self.pose(self.end_anchor).position
                error=rear.distance(carla.Location(p.x,-p.y,p.z))
                if error<1.5 and velocity<.05:
                    terminal_since=sim if terminal_since is None else terminal_since
                    if sim-terminal_since>=5:
                        if self.lap_count!=self.args.laps or self.current_demand or self.service_restart_key:
                            raise RuntimeError('Demand validation ended before all stations/restarts were checked')
                        passes={r['key']:r for r in self.result['station_passes']}
                        for row in self.comparison:
                            expected=bool(row['expected_waiting'] or row['bell'])
                            if self.args.random_bell_seed is not None and passes[row['key']]['served']!=expected:
                                raise RuntimeError('Actual station service differs from demand plan')
                        self.result.update(success=True,result='demand_service_validation_completed',
                            travel_distance_m=self.travel_m,service_completed=bool(self.result['service_events']),
                            bell_pending=self.bell_pending(),boarded_passengers=self.passengers.boarded)
                        self.event('validation_end_stop',completed_laps=self.lap_count,goal_error_m=error)
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
        raise RuntimeError('Demand service validation timed out')

    def cleanup(self):
        try:
            super().cleanup()
        finally:
            try:
                if self.passengers:
                    self.passengers.cleanup()
                    self.result['cleanup']['waiting_passengers_removed']=True
            finally:
                if self.maneuver_profile:
                    self.maneuver_profile.restore()
