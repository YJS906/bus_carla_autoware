#!/usr/bin/env python3
"""S09-S10-S01 bell demonstration, using only Autoware routes for motion.

S10 is the only service stop in this first-stage demonstration. S01 is a
diagnostic terminal. Late requests are explicitly rejected, never silently
consumed or claimed to be carried over to an unimplemented service station.
"""
import argparse
import json
import math
from pathlib import Path
import queue
import signal
import threading
import time

import carla
import rclpy
from rclpy.signals import SignalHandlerOptions
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ClearRoute, SetRoutePoints
from rosidl_runtime_py.convert import message_to_ordereddict

from carla_bus_lap import BusLap
from carla_closed_loop_validation import speed, xyz
from carla_bus_service_policy import ServicePolicy, acceptance_distance, route_progress


class Channel:
    def __init__(self):
        self.commands = queue.Queue()
        self.lock = threading.Lock()
        self.status = {'phase': 'PREPARING', 'message': '연결 및 정지 상태 확인 중'}
        self.done = threading.Event()

    def publish(self, **values):
        with self.lock:
            self.status.update(values, heartbeat=time.monotonic())

    def snapshot(self):
        with self.lock:
            return dict(self.status)


class BusService(BusLap):
    def __init__(self, args, channel):
        super().__init__(args)
        self.channel = channel
        self.policy = ServicePolicy()
        by_id = {a['id']: a for a in self.course['anchors']}
        self.start_anchor, self.station, self.terminal = (by_id[k] for k in ('S09', 'S10', 'S01'))
        if [(s['from'], s['to']) for s in self.course['segments'][-2:]] != [('S09', 'S10'), ('S10', 'S01')]:
            raise RuntimeError('Service demo requires S09-S10-S01 in route order')
        self.reference = self.course['segments'][-2]['points'] + self.course['segments'][-1]['points']
        self.phase = 'PREPARING'
        self.message = ''
        self.progress_m = self.start_anchor['route_distance_m']
        self.travel_m = 0.
        self.last_position = None
        self.last_moving_position = None
        self.last_progress_wall = time.monotonic()
        self.last_clock = None
        self.last_clock_wall = time.monotonic()
        self.last_print = 0.
        self.auto_bell_sent = False
        self.passed_station = False
        self.restart_origin = None
        self.result.update(case='bus_service', full_course=False, service_station='S10',
                           terminal='S01', service_events=[], bell_events=[], restart_verified=False,
                           service_goal_reference='rear axle / base_link, 0.45 m inward from lane centre',
                           stop_tolerance=dict(longitudinal_m=1.5, lateral_m=.65, speed_mps=.05),
                           late_request_policy='reject explicitly; next-station carry-over not implemented')

    def ui(self):
        velocity = speed(self.ego)
        remaining = self.service_remaining()
        accepting = self.accepting_requests()
        connected = (time.monotonic()-self.received.get('odom', 0) < 2 and
                     time.monotonic()-self.actuation_received < 2)
        self.channel.publish(phase=self.phase, message=self.message,
                             connected=connected,
                             requested=self.policy.requested, served=self.policy.completed,
                             accepting=accepting and remaining > acceptance_distance(velocity),
                             speed_kmh=velocity*3.6, remaining_m=remaining,
                             dwell_s=self.policy.remaining_s,
                             next_stop='S01 · 시험 종료' if self.passed_station or self.policy.completed else 'S10 · 서부 남단')

    def service_remaining(self):
        return self.station['route_distance_m'] - self.progress_m

    def accepting_requests(self):
        return self.phase in ('READY', 'DRIVING') and not self.passed_station

    def bell_pending(self):
        return self.policy.requested

    def commands(self):
        start = False
        while True:
            try:
                command = self.channel.commands.get_nowait()
            except queue.Empty:
                break
            source = command.get('source', 'unspecified') if isinstance(command, dict) else 'unspecified'
            command = command.get('action') if isinstance(command, dict) else command
            if command == 'start' and self.phase == 'READY':
                start = True
            elif command == 'bell':
                if self.args.random_bell_seed is not None and source != 'random_test':
                    self.event('manual_bell_ignored_during_random_test', source=source)
                    continue
                remaining = self.service_remaining()
                outcome = self.policy.ring(remaining, speed(self.ego),
                    accepting=self.accepting_requests())
                self.message = {'accepted': 'S10 하차 요청 접수',
                                'already_requested': 'S10 하차 요청이 이미 접수되어 있습니다',
                                'too_late': 'S10 접수 마감 · 다음 정류장 이월은 아직 지원하지 않습니다'}[outcome]
                row = dict(outcome=outcome, source=source, remaining_m=remaining, speed_mps=speed(self.ego),
                           sim_time=self.get_clock().now().nanoseconds/1e9)
                self.result['bell_events'].append(row)
                self.event('bell_request', **row)
        return start

    def observe(self):
        self.spin(.1)
        now = time.monotonic()
        sim = self.get_clock().now().nanoseconds/1e9
        if self.carla.get_world().id != self.world_id:
            raise RuntimeError('CARLA world changed')
        for key in ('odom', 'trajectory', 'objects'):
            if now-self.received.get(key, 0) > 2.:
                raise RuntimeError('Stale input: '+key)
        if now-self.actuation_received > 1.:
            raise RuntimeError('Stale actuation input')
        if self.last_clock is not None and sim < self.last_clock:
            raise RuntimeError('Simulation clock moved backwards')
        if sim != self.last_clock:
            self.last_clock_wall = now
        if now-self.last_clock_wall > 2.:
            raise RuntimeError('Simulation clock stopped')
        self.last_clock = sim
        rear = self.rear_pose()
        velocity = speed(self.ego)
        control = self.ego.get_control()
        values = (*xyz(rear), velocity, control.throttle, control.brake, control.steer)
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError('Non-finite position or control')
        if self.collisions:
            raise RuntimeError('Collision during service demonstration')
        if velocity > self.args.speed_kmh/3.6+.5:
            raise RuntimeError('Service demonstration speed cap exceeded')
        if self.last_position is not None:
            self.travel_m += rear.distance(self.last_position)
        self.last_position = rear
        self.update_progress(rear)
        if self.last_moving_position is None or rear.distance(self.last_moving_position) > 1.:
            self.last_moving_position = rear
            self.last_progress_wall = now
        if now-self.last_progress_wall > 120:
            raise RuntimeError('No progress for 120 seconds')
        for crossing in self.signal_audit.sample(self.ego):
            self.result['signal_crossings'].append(crossing)
            self.event('signal_stop_line_crossing', **crossing)
            if crossing['violation']:
                raise RuntimeError('Red/unknown signal stop-line crossing')
        wp = self.wmap.get_waypoint(rear)
        row = dict(sim_time=sim, rear_xyz=xyz(rear), speed_mps=velocity,
                   road=wp.road_id, lane=wp.lane_id, distance_m=self.travel_m,
                   progress_m=self.progress_m, phase=self.phase,
                   mode=self.last['mode'].mode, matches=[])
        self.result['samples'].append(row)
        if now-self.last_print >= 10:
            self.event('service_progress', **row)
            self.last_print = now
        if self.restart_origin is not None and not self.result['restart_verified']:
            if rear.distance(self.restart_origin) > 5 and velocity > .5:
                self.result['restart_verified'] = True
                self.event('service_restart_verified', distance_m=rear.distance(self.restart_origin))
        return sim, rear, velocity, wp

    def update_progress(self, rear):
        self.progress_m = max(self.progress_m, route_progress(self.reference, rear.x, rear.y))

    def station_errors(self, rear):
        p = self.pose(self.station).position
        yaw = self.station['carla']['yaw_deg']*math.pi/180
        dx, dy = rear.x-p.x, rear.y+p.y
        return dx*math.cos(yaw)+dy*math.sin(yaw), -dx*math.sin(yaw)+dy*math.cos(yaw)

    def commit_station_route(self):
        remaining = self.service_remaining()
        if remaining <= acceptance_distance(speed(self.ego)):
            raise RuntimeError('Bell retained but S10 route could not be committed before request deadline')
        if not self.can_request_reroute():
            self.message = 'S10 요청 유지 · 경로 변경 가능 상태를 기다리는 중'
            return False
        request = SetRoutePoints.Request()
        request.header.frame_id = 'map'
        request.header.stamp = self.get_clock().now().to_msg()
        request.goal = self.pose(self.station)
        requested = time.monotonic()
        self.call(self.change_route_client, request, 'set_service_goal')
        self.until(lambda: self.received.get('route', 0) > requested, 10, 'service route')
        self.until(lambda: self.received.get('trajectory', 0) > requested, 10, 'service trajectory')
        self.result['legs'].append(message_to_ordereddict(self.last['route']))
        self.policy.committed = True
        self.message = 'S10 정차 경로 승인 · 접근 중'
        return True

    def drive(self, service_goal):
        started = time.monotonic()
        terminal_since = None
        blocked_since = None
        while time.monotonic()-started < self.args.duration:
            sim, rear, velocity, wp = self.observe()
            if (self.args.bell_after_m is not None and not self.auto_bell_sent and
                    self.travel_m >= self.args.bell_after_m):
                self.channel.commands.put('bell')
                self.auto_bell_sent = True
            self.commands()
            if self.policy.requested and not self.policy.committed:
                if self.commit_station_route():
                    service_goal = True
            if (not service_goal and not self.policy.completed and not self.passed_station and
                    self.progress_m > self.station['route_distance_m']+2):
                self.passed_station = True
                self.result['visited_stops'].append('S10')
                self.event('station_passed_without_service', station='S10', lane=wp.lane_id, speed_mps=velocity)
                self.message = '하차 요청 없음 · S10 통과'
            if service_goal:
                longitudinal, lateral = self.station_errors(rear)
                lane_ok = wp.road_id == self.station['road_id'] and wp.lane_id == self.station['lane_id']
                complete = self.policy.observe(sim, velocity, longitudinal, lateral, lane_ok)
                dwelling = self.policy.stopped_since is not None
                new_phase = 'DWELLING' if dwelling else 'DRIVING'
                if new_phase != self.phase and dwelling:
                    self.event('service_dwell_started', station='S10', longitudinal_error_m=longitudinal,
                               lateral_error_m=lateral, road=wp.road_id, lane=wp.lane_id)
                self.phase = new_phase
                if complete:
                    row = dict(station='S10', hold_sim_s=sim-self.policy.stopped_since,
                               longitudinal_error_m=longitudinal, lateral_error_m=lateral,
                               speed_mps=velocity, road=wp.road_id, lane=wp.lane_id)
                    self.policy.complete()
                    self.result['service_events'].append(row)
                    self.result['visited_stops'].append('S10')
                    self.event('service_completed', **row)
                    self.phase = 'RESTARTING'
                    self.message = 'S10 서비스 완료 · 하차 요청 해제 · 재출발 준비'
                    self.ui()
                    self.stop()
                    self.call(self.clear_client, ClearRoute.Request(), 'clear_service_route')
                    self.restart_origin = rear
                    return 'served'
            else:
                p = self.pose(self.terminal).position
                goal_error = rear.distance(carla.Location(p.x, -p.y, p.z))
                if goal_error < 1.5 and velocity < .05:
                    if terminal_since is None:
                        terminal_since = sim
                    if sim-terminal_since >= 5:
                        if not (self.passed_station or self.policy.completed):
                            raise RuntimeError('S10 was not visited')
                        self.result['visited_stops'].append('S01')
                        self.result['terminal_goal_error_m'] = goal_error
                        self.event('terminal_stop', station='S01', goal_error_m=goal_error)
                        return 'terminal'
                else:
                    terminal_since = None
            mode = self.last['mode'].mode
            valid_goal_stop = ((self.phase == 'DWELLING' or terminal_since is not None) and
                               mode == OperationModeState.STOP)
            if mode != OperationModeState.AUTONOMOUS and not valid_goal_stop:
                raise RuntimeError('Left autonomous mode during service demonstration')
            control = self.ego.get_control()
            if velocity < .1 and control.throttle > .4 and control.brake < .05:
                blocked_since = blocked_since or time.monotonic()
                if time.monotonic()-blocked_since > 12:
                    raise RuntimeError('Drive torque blocked for 12 seconds')
            else:
                blocked_since = None
            self.ui()
        raise RuntimeError('Service leg timed out')

    def run(self):
        self.initialise()
        self.phase = 'READY'
        self.message = 'S10 하차벨을 누른 뒤 출발하거나, 출발 후 접수 가능 거리에서 누르세요'
        if self.args.bell:
            self.channel.commands.put('bell')
        if self.args.auto_start:
            self.channel.commands.put('start')
        while True:
            self.spin(.1)
            start = self.commands()
            self.ui()
            if start:
                if time.monotonic()-self.received.get('odom', 0) > 1 or time.monotonic()-self.actuation_received > 1:
                    raise RuntimeError('Cannot depart: stale localization or actuation')
                break
        self.phase = 'DRIVING'
        if self.policy.requested:
            self.submit([self.station])
            self.policy.committed = True
        else:
            self.submit([self.station, self.terminal])
        outcome = self.drive(self.policy.committed)
        if outcome == 'served':
            # Recheck fresh inputs before issuing a new route/engage command.
            self.observe()
            self.submit([self.terminal])
            self.phase = 'DEPARTED'
            self.drive(False)
            if not self.result['restart_verified']:
                raise RuntimeError('Service restart was not verified')
        self.result.update(success=True, result='service_demo_completed',
                           travel_distance_m=self.travel_m,
                           service_completed=self.policy.completed,
                           bell_pending=self.policy.requested)


def execute(args, channel):
    node = None
    try:
        if args.demand_service:
            from carla_bus_demand_service import BusDemandService
            node = BusDemandService(args, channel)
        elif args.continuous:
            from carla_bus_loop_service import BusLoopService
            node = BusLoopService(args, channel)
        else:
            node = BusService(args, channel)
        node.run()
    except Exception as exc:
        print(repr(exc), flush=True)
        channel.publish(phase='ERROR', message=str(exc))
        if node:
            node.result.update(success=False, result='interrupted' if args.interrupted else 'error',
                               error=repr(exc), bell_pending=node.bell_pending(),
                               service_completed=node.policy.completed)
            node.result['failure_context'] = {
                key: message_to_ordereddict(value) for key, value in node.diagnostic_paths.items()}
    finally:
        args.cleaning = True
        args.interrupted = False
        if node:
            node.phase = 'CLEANUP'
            channel.publish(phase='CLEANUP', message='버스 정지와 경로 해제 확인 중')
            try:
                node.cleanup()
            except Exception as exc:
                node.result.update(success=False, cleanup_error=repr(exc))
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(node.result, ensure_ascii=False, indent=2)+'\n')
            ok = node.result['success']
            channel.publish(phase='DONE' if ok else 'ERROR', requested=node.bell_pending(),
                            served=node.policy.completed, accepting=False,
                            message=(f"연속 순환 검증 완료 · {node.result.get('completed_laps',0)}바퀴" if args.continuous else
                                    '시험 완료 · S10 서비스 정차 후 재출발 확인' if node.policy.completed else
                                     '시험 완료 · S10 서비스 정차 없이 통과') if ok else
                                    node.result.get('error', '정리 실패 · 결과 파일 확인'))
            node.destroy_node()
        channel.done.set()
    return 0 if node and node.result['success'] else 1


def panel(args, channel):
    import tkinter as tk
    from tkinter import ttk
    root = tk.Tk()
    root.title('Town05 버스 · 하차벨 시범 운행')
    root.geometry('500x660+40+80' if args.demand_service else '500x600+40+80')
    root.attributes('-topmost', True)
    style = ttk.Style(root)
    style.theme_use('clam')
    style.configure('TFrame', background='#f6f8fc')
    style.configure('TLabel', background='#f6f8fc', foreground='#18243a', font=('sans', 11))
    style.configure('TButton', font=('sans', 11), padding=6)
    style.configure('Bell.TButton', background='#2458d3', foreground='white', font=('sans', 14, 'bold'))
    style.map('Bell.TButton', background=[('disabled', '#dce3ee'), ('active', '#1946ae')],
              foreground=[('disabled', '#566579')])
    frame = ttk.Frame(root, padding=22)
    frame.pack(fill='both', expand=True)
    ttk.Label(frame, text='버스 하차벨', font=('', 22, 'bold')).pack(anchor='w')
    route_label = ('전체 코스 · 대기 승객 또는 하차벨이 있으면 정차' if args.demand_service else
                   '전체 코스 연속 순환 · 하차벨 정차 대상 S10' if args.continuous else
                   'S09 출발 → S10 선택 정차 → S01 시험 종료')
    ttk.Label(frame, text=route_label, wraplength=450).pack(anchor='w', pady=10)
    text = tk.StringVar()
    ttk.Label(frame, textvariable=text, font=('', 12), wraplength=450).pack(anchor='w', pady=12)
    automatic_test = args.random_bell_seed is not None
    def send_bell(source):
        if not automatic_test:
            channel.commands.put(dict(action='bell',source=source))
    bell = ttk.Button(frame, text='하차벨  ·  Space', style='Bell.TButton', command=lambda: send_bell('button'))
    bell.pack(fill='x', ipady=15, pady=8)
    start = ttk.Button(frame, text='시범 운행 출발', command=lambda: channel.commands.put('start'))
    start.pack(fill='x', ipady=6, pady=4)
    def request_stop():
        if not args.cleaning:
            args.interrupted = True
    ttk.Button(frame, text='시험 중단 · 버스 정지', command=request_stop).pack(fill='x', pady=4)
    input_help = ('자동 시험 중에는 하차벨·Space 입력을 잠급니다.\n' if automatic_test else
                  '하차벨은 이 창에 포커스가 있을 때 Space로 누릅니다.\n')
    service_help = ('승객·벨이 없으면 10km/h 서행 통과합니다.\n'
                    '늦은 요청은 다음 정류장으로 이월합니다.\n' if args.demand_service else
                    '현재는 S10에서만 서비스 정차하며, 늦은 요청은 거부합니다.\n')
    ttk.Label(frame, text=input_help+service_help+
                         '신호 대기는 승하차 대기와 별도로 처리합니다.', wraplength=450).pack(anchor='w', pady=12)
    key_down = False
    release_job = None
    def press(_):
        nonlocal key_down, release_job
        if release_job is not None:
            root.after_cancel(release_job)
            release_job = None
        if not key_down and not bell.instate(['disabled']):
            send_bell('keyboard_space')
        key_down = True
        return 'break'
    def release(_):
        nonlocal release_job
        def reset():
            nonlocal key_down, release_job
            key_down = False
            release_job = None
        release_job = root.after(30, reset)
        return 'break'
    root.bind('<KeyPress-space>', press)
    root.bind('<KeyRelease-space>', release)
    def close():
        request_stop()
        if channel.done.is_set():
            root.destroy()
        else:
            root.after(100, close)
    root.protocol('WM_DELETE_WINDOW', close)
    labels = dict(PREPARING='준비 중', READY='출발 대기', DRIVING='주행 / 정류장 접근',
                  DWELLING='승하차 대기', RESTARTING='재출발 준비', DEPARTED='S01로 이동',
                  CLEANUP='정지 확인 중', DONE='시험 완료', ERROR='중단 / 오류')
    def refresh():
        s = channel.snapshot()
        stale = time.monotonic()-s.get('heartbeat', 0) > 3 or not s.get('connected', False)
        phase = s['phase']
        flashing = s.get('requested') and phase in ('READY', 'DRIVING', 'DWELLING')
        if flashing:
            colour = '#cf2338' if int(time.monotonic()*2) % 2 == 0 else '#8f1525'
            style.configure('Bell.TButton', background=colour)
            style.map('Bell.TButton', background=[('disabled', colour if automatic_test else '#dce3ee'), ('active', colour)],
                      foreground=[('disabled','white' if automatic_test else '#566579')])
            bell.configure(text='하차 요청 접수 · '+(s.get('bell_target') or 'S10'))
        else:
            style.configure('Bell.TButton', background='#2458d3')
            style.map('Bell.TButton', background=[('disabled', '#dce3ee'), ('active', '#1946ae')])
            style.map('Bell.TButton',foreground=[('disabled','#566579')])
            bell.configure(text='자동 시험 · 수동 입력 잠금' if automatic_test else '하차벨  ·  Space')
        bell.state(['!disabled'] if not automatic_test and not stale and (s.get('accepting') or s.get('requested')) and
                   phase in ('READY', 'DRIVING', 'DWELLING') else ['disabled'])
        start.state(['!disabled'] if phase == 'READY' and not stale else ['disabled'])
        finished_loop = args.continuous and phase == 'DONE'
        next_stop = '검증 종료' if finished_loop else s.get('next_stop', 'S10 · 서부 남단')
        lines = [f"상태: {labels.get(phase, phase)}", f"다음: {next_stop}",
                 f"속도: {s.get('speed_kmh', 0):.1f} km/h",
                 '하차 요청: '+('접수됨' if s.get('requested') else '없음')]
        if args.continuous:
            lap_label = (f"{s.get('completed_laps',0)}바퀴 완료" if finished_loop else
                         f"{s.get('lap_number',1)}바퀴째")
            lines.insert(1, lap_label+f" · 최근 통과 {s.get('current_stop','S01')}")
            if finished_loop:lines.append('정지 이유: 지정한 바퀴 수 검증 완료')
            elif s.get('stop_reason'):lines.append('정지 이유: '+s['stop_reason'])
        if args.demand_service:
            lines.append(f"다음 정류장 대기: {s.get('waiting_passengers',0)}명 · 전체 대기: {s.get('total_waiting_passengers',0)}명")
            lines.append(f"탑승 완료: {s.get('boarded_passengers',0)}명")
            if not finished_loop:lines.append('정류장 처리: '+s.get('service_reason','확인 중'))
        if phase == 'DWELLING':
            lines.append(f"승하차 대기: {s.get('dwell_s', 5):.1f}초")
        lines.append(s.get('message', ''))
        if stale and not channel.done.is_set():
            lines.append('연결 상태 확인 중 · 출발/벨 입력 잠금')
        text.set('\n'.join(lines))
        root.after(100, refresh)
    result = []
    worker = threading.Thread(target=lambda: result.append(execute(args, channel)), daemon=False)
    worker.start()
    refresh()
    root.mainloop()
    worker.join()
    return result[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--map-dir', type=Path, default=Path('/home/aw/autoware_data/maps/Town05Bus'))
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--auto-start', action='store_true')
    parser.add_argument('--bell', action='store_true', help='Inject a bell request before departure')
    parser.add_argument('--bell-after-m', type=float, help='Inject one bell after measured travel distance')
    parser.add_argument('--continuous', action='store_true', help='Full course with rolling route handoffs and S10 service')
    parser.add_argument('--demand-service', action='store_true', help='All stations: waiting passengers OR bell; implies continuous')
    parser.add_argument('--service-stops', type=Path)
    parser.add_argument('--passenger-stops', help='Waiting passengers, e.g. S03:2,S06:1; empty string creates none')
    parser.add_argument('--laps', type=int, default=0, help='Continuous validation laps; 0 runs until stopped')
    parser.add_argument('--random-bell-seed', type=int, help='Reproducible request/no-request comparison, requires --laps >= 2')
    parser.add_argument('--speed-kmh', type=float)
    parser.add_argument('--duration', type=float)
    args = parser.parse_args()
    if args.demand_service:
        args.continuous = True
        args.service_stops = args.service_stops or args.route.with_name('town05_bus_service_stops.json')
    elif args.service_stops is not None or args.passenger_stops is not None:
        parser.error('Passenger and service-stop options require --demand-service')
    args.speed_kmh = args.speed_kmh if args.speed_kmh is not None else (50 if args.continuous else 20)
    args.duration = args.duration if args.duration is not None else (5400 if args.continuous else 240)
    if not 0 < args.speed_kmh <= (50 if args.continuous else 20):
        parser.error('Speed limit: 20 km/h for short demo, 50 km/h for full course')
    if args.laps < 0 or (args.laps and not args.continuous):
        parser.error('--laps requires --continuous and must be nonnegative')
    if args.random_bell_seed is not None and (not args.continuous or args.laps < (1 if args.demand_service else 2) or args.bell):
        parser.error('Random testing requires finite laps (demand >= 1, S10 loop >= 2), without --bell')
    if args.continuous and args.bell_after_m is not None:
        parser.error('Use --random-bell-seed for continuous bell injection')
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('duration must be finite and positive')
    if args.headless and not args.auto_start:
        parser.error('--headless requires --auto-start')
    if args.bell_after_m is not None and (not math.isfinite(args.bell_after_m) or args.bell_after_m < 0):
        parser.error('bell-after-m must be a nonnegative finite distance')
    if args.bell and args.bell_after_m is not None:
        parser.error('Choose one bell injection mode')
    args.case='bus_service'; args.minimum_peak_kmh=0.; args.cruise_hold_s=0.
    args.host='127.0.0.1'; args.port=2000; args.input_mode='ground_truth'
    args.start_anchor='S01' if args.continuous else 'S09'
    args.end_anchor='S01'; args.resume_current_pose=False
    args.stop_at_all_stops=False; args.preflight=False; args.with_traffic=False
    args.interrupted=False; args.cleaning=False
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*_):
        if not args.cleaning:
            args.interrupted = True
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        channel = Channel()
        return execute(args, channel) if args.headless else panel(args, channel)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
