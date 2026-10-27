"""Reversible low-speed station maneuver profile; all dynamic checks stay enabled."""
import math

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically


class StationManeuverProfile:
    NODE = '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner'
    PREFIX = 'lane_change.trajectory.'
    LENGTH = PREFIX+'minimum_lane_change_length'
    CHECKS = [PREFIX+'enable_lateral_acceleration_limit', PREFIX+'enable_lateral_jerk_limit']
    BACK = ['start_planner.enable_back', 'start_planner.freespace_planner.astar.use_back']

    def __init__(self, node):
        self.node = node
        self.previous = None
        self.previous_back = None
        self.getter = node.create_client(GetParameters, self.NODE+'/get_parameters')
        self.setter = node.create_client(SetParametersAtomically, self.NODE+'/set_parameters_atomically')

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Station maneuver parameter service unavailable')
        future = client.call_async(request)
        self.node.until(future.done, 10, 'station maneuver parameter response')
        return future.result()

    def read(self):
        return self.call(self.getter, GetParameters.Request(names=[self.LENGTH]+self.CHECKS+self.BACK)).values

    def set_profile(self, value, backward):
        parameters=[Parameter(name=self.LENGTH,
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))]
        parameters += [Parameter(name=name,value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL,bool_value=enabled))
            for name,enabled in zip(self.BACK,backward)]
        request = SetParametersAtomically.Request(parameters=parameters)
        result = self.call(self.setter, request).result
        if not result.successful:
            raise RuntimeError('Station maneuver profile rejected: '+result.reason)
        values = self.read()
        if (values[0].double_value != value or not all(v.bool_value for v in values[1:3]) or
                [v.bool_value for v in values[3:]] != list(backward)):
            raise RuntimeError('Station maneuver profile verification failed')

    def activate(self):
        values = self.read()
        if (len(values)!=5 or values[0].type!=ParameterType.PARAMETER_DOUBLE or
                not math.isfinite(values[0].double_value) or values[0].double_value<0 or
                not all(v.type==ParameterType.PARAMETER_BOOL and v.bool_value for v in values[1:3]) or
                not all(v.type==ParameterType.PARAMETER_BOOL for v in values[3:])):
            raise RuntimeError('Station service requires lateral acceleration and jerk checks')
        # This is an additional comfort floor, not a substitute for the planner's
        # speed-dependent acceleration/jerk, footprint, collision and curvature checks.
        self.previous = values[0].double_value
        self.previous_back = [v.bool_value for v in values[3:]]
        self.set_profile(16., [False,False])
        self.node.result['station_maneuver_profile'] = dict(
            parameter=self.LENGTH, previous=self.previous, service_value=16.,
            lateral_acceleration_limit=True, lateral_jerk_limit=True, restored=False,
            backward_parameters=self.BACK,previous_backward=self.previous_back,backward_enabled=False)
        self.node.event('station_maneuver_profile_applied', previous=self.previous, value=16.,
                        previous_backward=self.previous_back,backward_enabled=False)

    def restore(self):
        if self.previous is None:
            return
        self.set_profile(self.previous, self.previous_back)
        self.node.result.setdefault('station_maneuver_profile', {})['restored'] = True
        self.node.event('station_maneuver_profile_restored', value=self.previous,backward=self.previous_back)
        self.previous = None
