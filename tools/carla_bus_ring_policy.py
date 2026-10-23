"""Town05 road 37 lane requests, restricted to surveyed straight sections."""


class RingPolicy:
    def __init__(self):
        self.stage = 'entry_middle'
        self.stable_since = None
        self.stable_lane = None

    def observe(self, s, lane, offset, heading_error, sim_time):
        aligned = abs(offset) < .95 and abs(heading_error) < .05
        if not aligned or lane != self.stable_lane:
            self.stable_since = sim_time if aligned else None
            self.stable_lane = lane if aligned else None
        stable = self.stable_since is not None and sim_time-self.stable_since >= 1.0
        if self.stage == 'merging_outer' and lane == -3 and stable:
            self.stage = 'outer_cruise'
        if self.stage == 'merging_inner' and lane == -1 and stable:
            self.stage = 'exit_ready'
        if 130 <= s < 658 and self.stage in ('entry_middle', 'merging_outer'):
            raise RuntimeError('Outer merge not completed before ring curve; stop diagnostic')
        if s >= 785 and self.stage != 'exit_ready':
            raise RuntimeError('Inner merge not completed on exit straight; stop diagnostic')
        if self.stage == 'entry_middle' and 25 <= s < 100 and lane == -2 and stable:
            return -3
        if self.stage == 'outer_cruise' and 670 <= s < 725:
            return -2
        if self.stage == 'merging_middle' and 670 <= s < 765 and lane == -2 and stable:
            return -1
        return None

    def accepted(self, lane):
        self.stage = {-3:'merging_outer', -2:'merging_middle', -1:'merging_inner'}[lane]
        self.stable_since = None
        self.stable_lane = None

    def cap_kmh(self, s):
        return 50. if self.stage == 'exit_ready' or (
            self.stage == 'outer_cruise' and s < 620) else 20.
