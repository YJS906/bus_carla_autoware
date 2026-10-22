#!/usr/bin/env python3
"""Replay a captured bus path/odometry into an isolated real optimizer node.

No CARLA connection or vehicle commands. ROS_DOMAIN_ID must be 211 to keep
fixture messages away from the driving runtime.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from autoware_planning_msgs.msg import Path as PathMessage, Trajectory
from autoware_internal_planning_msgs.msg import PlanningFactorArray
from nav_msgs.msg import Odometry
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fixture',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--preserve-bounds',choices=['true','false'],required=True)
    p.add_argument('--circle-method',choices=['fitting_uniform_circle','uniform_circle'],default='fitting_uniform_circle')
    p.add_argument('--collision-weight',type=float,default=100.)
    p.add_argument('--clearance',type=float,default=.1)
    p.add_argument('--param',action='append',default=[],help='Additional ROS parameter override name:=value')
    p.add_argument('--refinements',type=int,default=1)
    p.add_argument('--debug',action='store_true')
    p.add_argument('--center-offset',type=float)
    a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='211': raise RuntimeError('Use isolated ROS_DOMAIN_ID=211')
    captured=json.loads(a.fixture.read_text())
    path=PathMessage();odom=Odometry()
    set_message_fields(path,captured['/planning/scenario_planning/lane_driving/behavior_planning/path'])
    set_message_fields(odom,captured['/localization/kinematic_state'])
    a.output.parent.mkdir(parents=True,exist_ok=True)
    args=['/opt/carla_overlay/lib/autoware_path_optimizer/path_optimizer_node','--ros-args',
          '--params-file','/opt/carla-config/vehicle_profiles/fusorosa/vehicle_info.param.yaml',
          '--params-file','/opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/autoware_path_optimizer/path_optimizer.param.yaml',
          '-p','ego_nearest_dist_threshold:=3.0','-p','ego_nearest_yaw_threshold:=1.046',
          '-p','mpt.avoidance.preserve_input_bounds:='+a.preserve_bounds,
          '-p','mpt.collision_free_constraints.vehicle_circles.method:='+a.circle_method,
          '-p','mpt.collision_free_constraints.vehicle_circles.uniform_circle.num:=10',
          '-p','mpt.weight.soft_collision_free_weight:='+str(a.collision_weight),
          '-p','mpt.clearance.soft_clearance_from_road:='+str(a.clearance)]
    args += ['-p','mpt.option.refinement_iterations:='+str(a.refinements)]
    if a.debug: args += ['-p','option.debug.enable_debug_info:=true']
    if a.center_offset is not None:
        args += ['-p','mpt.kinematics.optimization_center_offset:='+str(a.center_offset)]
    for value in a.param: args += ['-p', value]
    result={'parameters':{key:str(value) if isinstance(value,Path) else value for key,value in vars(a).items()},
            'preserve_input_bounds':a.preserve_bounds,'trajectory':None,'factors':None,'trajectory_history':[]}
    def record_trajectory(message):
        data=message_to_ordereddict(message)
        result['trajectory']=data
        result['trajectory_history'].append(data)
    with a.output.with_suffix('.log').open('w') as log:
        proc=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
        rclpy.init(args=[]);node=rclpy.create_node('bus_optimizer_replay')
        publishers=[node.create_publisher(PathMessage,'/path_optimizer/input/path',1),
                    node.create_publisher(Odometry,'/path_optimizer/input/odometry',1)]
        subscriptions=[node.create_subscription(Trajectory,'/path_optimizer/output/path',
            record_trajectory,1),
            node.create_subscription(PlanningFactorArray,'/planning/planning_factors/path_optimizer',
            lambda m:result.update(factors=message_to_ordereddict(m)),1),
            node.create_subscription(Trajectory,'/path_optimizer/debug/mpt_traj',
                lambda m:result.update(optimized_trajectory=message_to_ordereddict(m)),1),
            node.create_subscription(Trajectory,'/path_optimizer/debug/mpt_ref_traj',
                lambda m:result.update(reference_trajectory=message_to_ordereddict(m)),1)]
        try:
            start=time.monotonic();last=0
            while time.monotonic()-start<8:
                if proc.poll() is not None: raise RuntimeError('Optimizer exited; inspect replay log')
                if time.monotonic()-last>.2:
                    stamp=node.get_clock().now().to_msg();path.header.stamp=stamp;odom.header.stamp=stamp
                    publishers[1].publish(odom);publishers[0].publish(path);last=time.monotonic()
                rclpy.spin_once(node,timeout_sec=.03)
            if result['trajectory'] is None: raise RuntimeError('No replay trajectory')
            points=result['trajectory']['points'];pos=odom.pose.pose.position
            idx=min(range(len(points)),key=lambda i:math.hypot(points[i]['pose']['position']['x']-pos.x,points[i]['pose']['position']['y']-pos.y))
            result['nearest_velocity_mps']=points[idx]['longitudinal_velocity_mps']
            q=points[idx]['pose']['orientation'];ego_q=odom.pose.pose.orientation
            yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
            ego_yaw=math.atan2(2*(ego_q.w*ego_q.z+ego_q.x*ego_q.y),1-2*(ego_q.y**2+ego_q.z**2))
            result['nearest_yaw_error_deg']=math.degrees(math.atan2(math.sin(yaw-ego_yaw),math.cos(yaw-ego_yaw)))
            print(json.dumps({'preserve_bounds':a.preserve_bounds,'nearest_velocity_mps':result['nearest_velocity_mps'],
                              'has_optimized_trajectory':bool(result.get('optimized_trajectory',{}).get('points')),
                              'yaw_error_deg':result['nearest_yaw_error_deg'],
                              'factors':result['factors']}),flush=True)
        finally:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
            node.destroy_node();rclpy.shutdown()
            a.output.write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
