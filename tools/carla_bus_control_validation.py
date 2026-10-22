#!/usr/bin/env python3
"""Basic bus tracking/stopping through full Autoware ADAPI in empty Town05.

The active bridge owns world ticks. Teleport is used only while verified STOP
to initialise each case; all subsequent motion is from Autoware commands.
"""
import argparse
import json
import math
import signal
from pathlib import Path
import time

import carla
import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode, ClearRoute, SetRoutePoints
from autoware_internal_planning_msgs.msg import VelocityLimit
from tier4_vehicle_msgs.msg import ActuationCommandStamped
from rosidl_runtime_py.convert import message_to_ordereddict

from carla_closed_loop_validation import Validation, speed, xyz


class BusValidation(Validation):
    def __init__(self,args):
        super().__init__(args)
        if self.ego.type_id!='vehicle.mitsubishi.fusorosa':raise RuntimeError('Expected Fuso Rosa ego')
        if len(self.world.get_actors().filter('vehicle.*'))!=1 or self.world.get_actors().filter('walker.pedestrian.*'):
            raise RuntimeError('Basic control test requires no background vehicles or pedestrians')
        self.collision_sensor=None;self.collisions=[]
        self.actuation_subscription=self.create_subscription(ActuationCommandStamped,
            '/control/command/actuation_cmd',self.on_actuation,10)
        self.actuation_received=0.
        self.result.update(required_stop_hold_s=3.0,maximum_goal_error_m=1.5,
                           maximum_centreline_error_m=1.0)
        self.base_offset=(-2.6150478087440625,-.000007316637322674069,.0009262561798095703)

    def on_actuation(self,message):
        self.actuation_received=time.monotonic()

    def rear_pose(self):
        point=self.ego.get_transform().transform(carla.Location(*self.base_offset))
        return point

    def prepare(self):
        self.until(lambda:'mode' in self.last and 'odom' in self.last,30,'mode and localization')
        self.stop()
        self.call(self.clear_client,ClearRoute.Request(),'clear_old_route')
        s0,s1=(330.,390.) if self.args.case=='straight' else (130.,300.)
        self.start_wp=self.wmap.get_waypoint_xodr(37,-2,s0)
        self.goal_wp=self.wmap.get_waypoint_xodr(37,-2,s1)
        t=self.start_wp.transform
        offset=t.rotation.get_forward_vector()*self.base_offset[0]
        t.location-=carla.Location(offset.x,offset.y,offset.z)
        t.location.z+=.05
        self.ego.set_transform(t)
        self.ego.set_target_velocity(carla.Vector3D())
        self.ego.set_target_angular_velocity(carla.Vector3D())
        self.spin(3)
        self.until(lambda:speed(self.ego)<.05,10,'stationary after setup')
        self.collision_sensor=self.world.spawn_actor(
            self.world.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=self.ego)
        self.collision_sensor.listen(lambda e:self.collisions.append({'frame':e.frame,'other':e.other_actor.type_id}))
        limit=VelocityLimit();limit.stamp=self.get_clock().now().to_msg()
        limit.max_velocity=self.args.speed_kmh/3.6;limit.use_constraints=False;limit.sender='carla_closed_loop_test'
        self.limit_pub.publish(limit);self.limit_sent=True;self.spin(.5)
        goal=self.goal_wp.transform
        request=SetRoutePoints.Request();request.header.frame_id='map';request.header.stamp=self.get_clock().now().to_msg()
        request.option.allow_goal_modification=False
        request.goal.position.x,request.goal.position.y,request.goal.position.z=goal.location.x,-goal.location.y,goal.location.z
        angle=math.radians(-goal.rotation.yaw)
        request.goal.orientation.z,request.goal.orientation.w=math.sin(angle/2),math.cos(angle/2)
        self.route_sent=True;requested=time.monotonic()
        self.call(self.route_client,request,'set_bus_test_route')
        self.until(lambda:self.received.get('route',0)>=requested and len(self.last['route'].segments)>0,10,'route')
        self.result['lanelet_route']=message_to_ordereddict(self.last['route'])
        self.until(lambda:'trajectory' in self.last and len(self.last['trajectory'].points)>2
                   and time.monotonic()-self.received['trajectory']<1,30,'fresh trajectory')
        self.until(lambda:'velocity_limit' in self.last and self.last['velocity_limit'].max_velocity<=self.args.speed_kmh/3.6+.01,10,'speed cap applied')
        self.until(lambda:self.last['mode'].is_autonomous_mode_available,30,'AUTO available')
        self.until(lambda:time.monotonic()-self.actuation_received<1,10,'fresh converted actuation commands')
        if not self.last['mode'].is_autoware_control_enabled:
            self.call(self.enable_client,ChangeOperationMode.Request(),'enable_autoware_control')
        self.result['native_start']={'road':37,'lane':-2,'s':s0}
        self.result['native_goal']={'road':37,'lane':-2,'s':s1}
        self.result['goal']=message_to_ordereddict(request.goal)
        self.case_start_frame=self.world.get_snapshot().frame
        self.case_running=True
        self.call(self.auto_client,ChangeOperationMode.Request(),'engage_bus')
        self.until(lambda:self.last['mode'].mode==OperationModeState.AUTONOMOUS
                   and not self.last['mode'].is_in_transition,15,'AUTO transition')

    def run(self):
        self.prepare()
        started=time.monotonic();sim_start=self.world.get_snapshot().timestamp.elapsed_seconds
        stopped_since=None;last_print=-5;last_position=self.rear_pose();distance=0.;peak=0.;max_error=0.
        start_yaw=self.ego.get_transform().rotation.yaw
        while time.monotonic()-started<self.args.duration:
            self.spin(.10)
            if self.carla.get_world().id!=self.world_id:raise RuntimeError('World changed')
            if time.monotonic()-self.received.get('odom',0)>1:raise RuntimeError('Stale localization')
            if time.monotonic()-self.actuation_received>1:raise RuntimeError('Stale actuation commands')
            now=self.world.get_snapshot();rear=self.rear_pose();v=speed(self.ego)
            distance+=rear.distance(last_position);last_position=rear
            goal_distance=rear.distance(self.goal_wp.transform.location)
            native=self.wmap.get_waypoint(rear)
            ref=self.wmap.get_waypoint_xodr(37,-2,native.s) if native.road_id==37 else None
            error=rear.distance(ref.transform.location) if ref else 999.
            max_error=max(max_error,error);peak=max(peak,v)
            control=self.ego.get_control();ros=self.last['odom'];pos=ros.pose.pose.position
            odom_error=math.dist((pos.x,-pos.y,pos.z),(rear.x,rear.y,rear.z))
            yaw=self.ego.get_transform().rotation.yaw
            turn=math.degrees(math.atan2(math.sin(math.radians(yaw-start_yaw)),math.cos(math.radians(yaw-start_yaw))))
            row={'sim_s':now.timestamp.elapsed_seconds-sim_start,'frame':now.frame,'speed_mps':v,
                 'rear_xyz':xyz(rear),'goal_distance_m':goal_distance,'distance_m':distance,
                 'centreline_error_m':error,'yaw_change_deg':turn,'odometry_position_error_m':odom_error,
                 'matches':[],
                 'mode':self.last['mode'].mode,'throttle':control.throttle,'brake':control.brake,'steer':control.steer,
                 'ros_control':message_to_ordereddict(self.last['control']) if 'control' in self.last else None}
            self.result['samples'].append(row)
            if row['sim_s']-last_print>=5:
                self.event('bus_progress',speed_kmh=v*3.6,distance_m=distance,goal_distance_m=goal_distance,error_m=error)
                last_print=row['sim_s']
            if self.collisions:raise RuntimeError('Collision during test')
            if v>self.args.speed_kmh/3.6+.5:raise RuntimeError('Speed cap exceeded')
            if error>1:raise RuntimeError('Centreline tracking error exceeded 1m')
            if goal_distance<1.5 and v<.05 and peak>2.0:
                if stopped_since is None:stopped_since=row['sim_s']
                if row['sim_s']-stopped_since>=3:
                    self.result.update(success=True,result='goal_stop_pass',max_speed_mps=peak,
                        max_centreline_error_m=max_error,travel_distance_m=distance,final_goal_error_m=goal_distance,
                        yaw_change_deg=turn,stop_hold_s=row['sim_s']-stopped_since)
                    break
            else:
                stopped_since=None
                if self.last['mode'].mode!=OperationModeState.AUTONOMOUS:raise RuntimeError('Left AUTO before reaching goal')
        else:raise RuntimeError('Timeout without goal stop')

    def cleanup(self):
        super().cleanup()
        if self.collision_sensor:
            self.collision_sensor.stop();self.collision_sensor.destroy()
        self.result['collisions']=self.collisions


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=['straight','curve'],required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--speed-kmh',type=float,default=10)
    parser.add_argument('--duration',type=float,default=180)
    args=parser.parse_args();args.host='127.0.0.1';args.port=2000;args.input_mode='ground_truth'
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*_):raise RuntimeError('Test interrupted')
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    node=None;error=None
    try:
        node=BusValidation(args);node.run()
    except Exception as exc:
        error=repr(exc);print(error,flush=True)
        if node:node.result.update(success=False,result='error',error=error)
    finally:
        if node:
            try:node.cleanup()
            except Exception as exc:node.result['cleanup_error']=repr(exc);node.result['success']=False
            args.output.write_text(json.dumps(node.result,indent=2)+'\n')
            node.destroy_node()
        rclpy.shutdown()
    if error or not node or not node.result['success']:raise SystemExit(1)
