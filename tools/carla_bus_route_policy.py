"""Pure route-graph policy for leaving a passed station in the non-service lap."""


def station_exit_preferences(sections, endpoints, station, protected=()):
    """Choose a connected exit chain to the next forced route segment.

    Keep unpassed stations untouched. Only the next three route sections are
    considered; this is not a broad route rewrite or a lane-change approval.
    """
    start = next((i for i, s in enumerate(sections) if station in s['lanes']), None)
    if start is None:
        return {}
    finish = None
    for i in range(start + 1, min(len(sections), start + 4)):
        if set(sections[i]['lanes']).intersection(protected):
            return {}
        if len(sections[i]['lanes']) == 1:
            finish = i
            break
    if finish is None:
        return {}
    target = sections[finish]['lanes'][0]
    selected = {}
    for i in range(finish - 1, start - 1, -1):
        candidates = [lane for lane in sections[i]['lanes']
                      if endpoints[lane][1] == endpoints[target][0]]
        if not candidates:
            return {}
        preferred = sections[i]['preferred']
        target = preferred if preferred in candidates else candidates[0]
        selected[i] = target
    return {i: lane for i, lane in selected.items() if sections[i]['preferred'] != lane}


class CentralUrbanMergePolicy:
    """Slow the short S04 exit before selecting the bus's lane-change geometry.

    Road 1 is the S04 approach; road 0 is the next straight before the mandatory
    left turn. Intermediate junction road IDs vary with the chosen lane.
    This only lowers the speed ceiling; Autoware still plans and approves motion.
    """
    def __init__(self):
        self.active = False
        self.completed = False
        self.aligned_since = None

    def observe(self, road, lane, offset, heading_error, sim_time, station_passed=False):
        if not self.completed and road == 1:
            self.active = True
        aligned = station_passed and road in (1, 0) and lane == 1 and abs(offset) < .65 and abs(heading_error) < .08
        if self.active and aligned:
            if self.aligned_since is None:
                self.aligned_since = sim_time
            elif sim_time - self.aligned_since >= 1.0:
                self.active = False
                self.completed = True
        else:
            self.aligned_since = None
        return 10.0 if self.active else None
