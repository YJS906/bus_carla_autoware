"""Directed progress and reproducible bell scheduling for the Town05 loop."""
from bisect import bisect_left, bisect_right
import math
import random


class LoopProgress:
    def __init__(self, points, length):
        self.length = length
        self.points = []
        for p in points:
            if not self.points or p['route_distance_m'] > self.points[-1]['route_distance_m']:
                self.points.append(p)
        self.distances = [p['route_distance_m'] for p in self.points]
        if len(self.points) < 2 or abs(self.distances[0]) > .01 or abs(self.distances[-1]-length) > .01:
            raise ValueError('Expected a complete, ordered closed-loop reference')
        self.value = 0.

    def observe(self, x, y):
        """Project locally, avoiding false jumps to later crossing/parallel roads."""
        best = (math.inf, self.value)
        cycle = int(self.value // self.length)
        low, high = self.value-12., self.value+60.
        for lap in range(max(0, cycle-1), cycle+2):
            offset = lap*self.length
            begin = max(0, bisect_left(self.distances, low-offset)-1)
            end = min(len(self.points)-1, bisect_right(self.distances, high-offset))
            for i in range(begin, end):
                a, b = self.points[i:i+2]
                p, q = a['carla'], b['carla']
                dx, dy = q['x']-p['x'], q['y']-p['y']
                length2 = dx*dx+dy*dy
                if length2 == 0:
                    continue
                t = max(0., min(1., ((x-p['x'])*dx+(y-p['y'])*dy)/length2))
                s = offset+a['route_distance_m']+t*(b['route_distance_m']-a['route_distance_m'])
                if not low <= s <= high:
                    continue
                distance2 = (x-p['x']-t*dx)**2+(y-p['y']-t*dy)**2
                if distance2 < best[0]:
                    best = distance2, s
        if best[0] > 8.**2:
            raise RuntimeError('Bus is outside the local course reference corridor')
        self.value = max(self.value, best[1])
        return self.value


def landmarks_between(anchors, length, start, end):
    result = []
    for lap in range(max(0, int(start//length)), int(end//length)+1):
        for anchor in anchors:
            s = lap*length+anchor['route_distance_m']
            if start < s <= end+.001:
                result.append((s, anchor))
    return result


def random_bell_schedule(seed, laps, station_distance, length):
    """Randomised paired comparison: at least one request and one no-request lap."""
    if laps < 2:
        raise ValueError('Random comparison requires at least two laps')
    rng = random.Random(seed)
    requests = [True, False]+[rng.choice([True, False]) for _ in range(laps-2)]
    rng.shuffle(requests)
    return [dict(lap=i+1, request=request,
                 progress_m=i*length+rng.uniform(100., station_distance-220.) if request else None)
            for i, request in enumerate(requests)]
