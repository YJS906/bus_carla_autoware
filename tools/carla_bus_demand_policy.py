"""Stop demand and reproducible passenger/bell cases, independent of CARLA/ROS."""
from dataclasses import dataclass, field
import random

from carla_bus_service_policy import ServicePolicy, acceptance_distance


def passenger_counts(value, station_ids):
    counts = dict.fromkeys(station_ids, 0)
    if not value:
        return counts
    seen = set()
    for item in value.split(','):
        name, amount = item.strip().split(':')
        count = int(amount)
        if name not in counts or name in seen or not 0 <= count <= 3:
            raise ValueError('Expected distinct station:count entries, count 0..3')
        seen.add(name)
        counts[name] = count
    return counts


def comparison_plan(seed, ordered_ids, cycles):
    """At least one of each (people, bell) combination on the first circuit."""
    rng = random.Random(seed)
    cases = [(False, False), (True, False), (False, True), (True, True)]
    cases += [rng.choice(cases) for _ in range(len(ordered_ids)-4)]
    rng.shuffle(cases)
    counts = {name: rng.randint(1, 2) if people else 0
              for name, (people, _) in zip(ordered_ids, cases)}
    rows = []
    for cycle in range(cycles):
        for i, name in enumerate(ordered_ids):
            bell = (cases[i][1] if cycle == 0 else
                    (not any(cases[i]) or rng.choice([False, True])))
            rows.append(dict(circuit=cycle+1, station=name, bell=bell,
                             expected_waiting=counts[name] if cycle == 0 else 0,
                             trigger_fraction=rng.uniform(.05, .20)))
    return counts, rows


@dataclass
class StopDemand:
    key: str
    station: str
    distance_m: float
    waiting: int = 0
    bell: bool = False
    passenger_latched: bool = False
    dwell: ServicePolicy = field(default_factory=ServicePolicy)

    @property
    def required(self):
        return not self.dwell.completed and (self.bell or self.passenger_latched or self.waiting > 0)

    @property
    def reasons(self):
        return (['bell'] if self.bell else []) + (['waiting_passengers'] if self.passenger_latched or self.waiting else [])

    def commit(self):
        if not self.required:
            raise ValueError('Cannot commit an empty stop')
        self.passenger_latched |= self.waiting > 0
        self.dwell.requested = self.dwell.committed = True

    def complete(self):
        reasons = self.reasons
        self.dwell.complete()
        self.bell = False
        self.waiting = 0
        return reasons


def bell_target(stops, progress_m, velocity, dwelling=False):
    """Latch to the next safely reachable unserved stop; late requests carry on."""
    for stop in stops:
        if stop.bell:
            return stop, 'already_requested'
    if dwelling and stops and stops[0].required:
        return stops[0], 'accepted'
    for index, stop in enumerate(stops):
        if stop.dwell.completed:
            continue
        if stop.distance_m-progress_m > acceptance_distance(velocity):
            return stop, 'accepted' if index == 0 else 'carried_forward'
    return None, 'closed'


def approach_cap(remaining_m, velocity, slow_kmh=10.):
    """Brake in advance, hold the low ceiling through the whole stop frontage."""
    v = max(0., velocity)
    slow = slow_kmh/3.6
    approach = max(45., 15.+v*2.+max(0., v*v-slow*slow)/(2*.8))
    return slow_kmh if -18. <= remaining_m <= approach else None


def rolling_horizon(progress_m, length_m, last_urban_entry_m, laps):
    """Retain the finite terminal across S01 while its pass is still pending."""
    cycle = int(progress_m//length_m)
    if laps and progress_m >= (laps-1)*length_m+last_urban_entry_m:
        return 'TEST_END', laps
    if progress_m < cycle*length_m+last_urban_entry_m:
        return 'S10', cycle
    return 'R02', cycle+1
