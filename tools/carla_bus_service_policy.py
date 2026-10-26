"""ROS-independent bell latch and continuous simulation-time service dwell."""
from dataclasses import dataclass
import math


def acceptance_distance(speed_mps):
    """Conservative request window; Autoware still approves every route change."""
    v = max(0., speed_mps)
    return max(25., 10. + 3. * v + v * v / (2. * .8))


def route_progress(points, x, y):
    """Project onto the directed short demonstration reference, returning along-route metres."""
    best = (math.inf, 0.)
    for a, b in zip(points, points[1:]):
        p, q = a['carla'], b['carla']
        dx, dy = q['x'] - p['x'], q['y'] - p['y']
        length2 = dx * dx + dy * dy
        if length2 == 0:
            continue
        t = max(0., min(1., ((x-p['x'])*dx + (y-p['y'])*dy) / length2))
        distance2 = (x-p['x']-t*dx)**2 + (y-p['y']-t*dy)**2
        s = a['route_distance_m'] + t * (b['route_distance_m']-a['route_distance_m'])
        if distance2 < best[0]:
            best = distance2, s
    return best[1]


@dataclass
class ServicePolicy:
    requested: bool = False
    completed: bool = False
    committed: bool = False
    stopped_since: float | None = None
    last_sim_time: float | None = None
    hold_s: float = 5.

    def ring(self, remaining_m, velocity, accepting=True):
        if self.requested:
            return 'already_requested'
        if self.completed or not accepting or remaining_m <= acceptance_distance(velocity):
            return 'too_late'
        self.requested = True
        return 'accepted'

    def observe(self, sim_time, velocity, longitudinal_error, lateral_error, correct_lane):
        if not all(math.isfinite(v) for v in
                   (sim_time, velocity, longitudinal_error, lateral_error)):
            raise ValueError('Invalid service observation')
        if self.last_sim_time is not None and sim_time < self.last_sim_time:
            raise ValueError('Simulation clock moved backwards')
        self.last_sim_time = sim_time
        qualified = (self.requested and self.committed and correct_lane and
                     abs(longitudinal_error) < 1.5 and abs(lateral_error) < .65 and
                     velocity < .05)
        if not qualified:
            self.stopped_since = None
            return False
        if self.stopped_since is None:
            self.stopped_since = sim_time
        return sim_time - self.stopped_since >= self.hold_s

    def complete(self):
        if (not self.requested or not self.committed or self.stopped_since is None or
                self.last_sim_time - self.stopped_since < self.hold_s):
            raise ValueError('Service dwell is incomplete')
        self.requested = False
        self.completed = True

    @property
    def remaining_s(self):
        if self.stopped_since is None:
            return self.hold_s
        return max(0., self.hold_s - (self.last_sim_time - self.stopped_since))
