"""Replay only S10 -> S01 using second-circuit demand; not a two-lap completion."""
import argparse,json,signal
from pathlib import Path
import rclpy
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.convert import message_to_ordereddict
from carla_bus_demand_service import BusDemandService
from carla_bus_service import Channel

class TailRegression(BusDemandService):
    def __init__(self,args,channel):
        super().__init__(args,channel)
        start=self.length+self.by_id['S10']['route_distance_m']
        selected=[(d,r) for d,r in zip(self.demands,self.comparison) if d.distance_m>start]
        self.demands=[d for d,r in selected];self.comparison=[r for d,r in selected]
        self.waiting_counts=dict.fromkeys(self.waiting_counts,0)
        for d in self.demands:d.waiting=0
        self.progress_m=start;self.tracker.value=start;self.lap_count=1
        self.next_sequence=len(self.anchors)+self.start_index+1
        self.current_station='S10';self.station=self.goals[self.demands[0].station]
        self.policy=self.demands[0].dwell
        self.result.update(case='bus_demand_terminal_regression',full_course=False,
            diagnostic_scope='S10 to S01/end only, second-circuit fixture, zero waiting passengers',
            initial_logical_cycle=1,initial_progress_m=start,comparison_plan=self.comparison,
            waiting_passenger_configuration=self.waiting_counts,requested_laps=None,completed_laps=0)

args=argparse.Namespace(route=Path('/tmp/bus-tools/town05_bus_route.json'),
 service_stops=Path('/tmp/bus-tools/town05_bus_service_stops.json'),
 map_dir=Path('/home/aw/autoware_data/maps/Town05Bus'),case='bus_service',speed_kmh=50.,
 minimum_peak_kmh=0.,cruise_hold_s=0.,host='127.0.0.1',port=2000,input_mode='ground_truth',
 start_anchor='S10',end_anchor='S01',resume_current_pose=False,stop_at_all_stops=False,
 preflight=False,with_traffic=False,interrupted=False,cleaning=False,duration=900,
 continuous=True,demand_service=True,laps=2,random_bell_seed=26027,passenger_stops=None,
 auto_start=True,bell=False,bell_after_m=None)
rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO);node=None
signal.signal(signal.SIGINT,lambda *_:setattr(args,'interrupted',True))
try:
 node=TailRegression(args,Channel());node.run()
 node.result.update(result='demand_terminal_regression_completed')
except Exception as exc:
 print(repr(exc),flush=True)
 if node:
  node.result.update(success=False,error=repr(exc))
  node.result['failure_context']={k:message_to_ordereddict(v) for k,v in node.diagnostic_paths.items()}
finally:
 args.interrupted=False
 if node:
  try:node.cleanup()
  except Exception as exc:node.result.update(success=False,cleanup_error=repr(exc))
  node.result['logical_cycle_at_end']=node.lap_count
  node.result['completed_laps']=0  # This diagnostic does not cover an entire lap.
  Path('/tmp/bus-demand-terminal-regression.json').write_text(json.dumps(node.result,ensure_ascii=False,indent=2)+'\n')
  node.destroy_node()
 rclpy.shutdown()
