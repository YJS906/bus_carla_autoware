"""Read-only audit of the front bumper crossing mapped signal stop lines.

Uses the same hash-checked stop lines as the signal bridge. A phase transition
between samples is recorded for review rather than labelled a definite red run.
This is a centre-of-front-bumper audit, not a proof of every traffic rule.
"""
import hashlib
import json
import xml.etree.ElementTree as ET


def crosses_segment(previous, current, start, end):
    def cross(a, b):
        return a[0]*b[1]-a[1]*b[0]
    motion = (current[0]-previous[0], current[1]-previous[1])
    line = (end[0]-start[0], end[1]-start[1])
    denominator = cross(motion, line)
    if abs(denominator)<1e-9:
        return False
    delta = (start[0]-previous[0], start[1]-previous[1])
    t, u = cross(delta, line)/denominator, cross(delta, motion)/denominator
    return 0<t<=1 and 0<=u<=1


class SignalAudit:
    def __init__(self, world, map_dir):
        data = (map_dir/'lanelet2_map.osm').read_bytes()
        manifest = json.loads((map_dir/'carla_traffic_signals.json').read_text())
        if hashlib.sha256(data).hexdigest()!=manifest['map_sha256']:
            raise ValueError('Signal audit map hash mismatch')
        wmap = world.get_map()
        if hashlib.sha256(wmap.to_opendrive().encode()).hexdigest()!=manifest['live_xodr_sha256']:
            raise ValueError('Signal audit native map mismatch')
        root = ET.fromstring(data)
        nodes = {}
        for n in root.findall('node'):
            tags = {t.get('k'):t.get('v') for t in n.findall('tag')}
            nodes[n.get('id')] = (float(tags['local_x']), -float(tags['local_y']),
                                 float(tags.get('ele', 0.)))
        ways = {int(w.get('id')):[nodes[n.get('ref')] for n in w.findall('nd')]
                for w in root.findall('way')}
        actors = {str(a.get_opendrive_id()):a for a in world.get_actors().filter('traffic.traffic_light')}
        self.lines = []
        for group in manifest['groups']:
            actor = actors[str(group['opendrive_id'])]
            points = ways[group['stop_line_id']]
            approaches = actor.get_affected_lane_waypoints()
            directions = [(w.transform.get_forward_vector().x,w.transform.get_forward_vector().y)
                          for w in approaches]
            self.lines.append((group, actor, points[0], points[-1], directions))
        self.previous = None
        self.states = {}
        self.world = world

    def sample(self, ego):
        transform = ego.get_transform()
        forward = transform.get_forward_vector()
        # Measured rear-axle-relative front 7.306 m, actor offset -2.615 m.
        front = transform.transform(type(transform.location)(4.691117, 0., 0.))
        current = (front.x, front.y)
        result = []
        states = {}
        for group, actor, start, end, directions in self.lines:
            key = group['group_id']
            state = str(actor.get_state())
            states[key] = state
            if self.previous is None or not crosses_segment(self.previous, current, start, end):
                continue
            if abs(front.z-(start[2]+end[2])/2)>2.:
                continue  # A flyover crossing in XY is not this stop line.
            if not any(forward.x*x+forward.y*y>0.5 for x,y in directions):
                continue
            previous_state = self.states[key]
            result.append(dict(group_id=key, opendrive_id=group['opendrive_id'],
                               stop_line_id=group['stop_line_id'],
                               frame=self.world.get_snapshot().frame,
                               state=state, previous_state=previous_state,
                               phase_transition=state!=previous_state,
                               violation=state==previous_state and state not in ('Green','Yellow'),
                               front_xy=list(current)))
        self.previous, self.states = current, states
        return result
