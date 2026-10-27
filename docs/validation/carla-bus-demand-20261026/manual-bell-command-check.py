"""ROS/CARLA imports required; command dispatch uses a fake stationary vehicle only."""
import json
import queue
from types import SimpleNamespace
import carla
from carla_bus_demand_policy import StopDemand
from carla_bus_demand_service import BusDemandService

out=[]
for pending_next in (False,True):
    node=BusDemandService.__new__(BusDemandService)
    node.args=SimpleNamespace(random_bell_seed=None)
    node.phase='DWELLING';node.progress_m=60.;node.stop_index=0
    node.demands=[StopDemand('0:S02','S02',60,waiting=1),
                  StopDemand('0:S03','S03',115,bell=pending_next)]
    node.demands[0].commit()
    node.ego=SimpleNamespace(get_velocity=lambda:carla.Vector3D())
    node.channel=SimpleNamespace(commands=queue.Queue())
    node.result={'bell_events':[]};node.event=lambda *args,**kwargs:None
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000000000))
    node.channel.commands.put(dict(action='bell',source='keyboard'))
    node.commands()
    assert node.demands[0].bell is (not pending_next)
    assert node.demands[1].bell is pending_next
    event=node.result['bell_events'][0]
    assert event['station']==('S03' if pending_next else 'S02')
    assert event['outcome']==('already_requested' if pending_next else 'accepted')
    out.append(dict(pending_next=pending_next,event=event))
print(json.dumps(dict(success=True,cases=out),ensure_ascii=False,indent=2))
