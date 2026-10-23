"""Defensive following for TM cars approaching a wide, externally controlled bus.

Only a bus physically occupying the car's forward corridor is a lead vehicle.
This does not give the bus priority to merge into an unoccupied adjacent lane.
Units are metres and seconds; the result is a speed ceiling in m/s.
"""
import math


def following_speed_limit(car_polygon, bus_polygon, car_heading, bus_heading,
                          bus_velocity, nominal_speed):
    fx, fy = car_heading
    norm = math.hypot(fx, fy)
    fx, fy = fx / norm, fy / norm
    if fx * bus_heading[0] + fy * bus_heading[1] < math.cos(math.pi / 4):
        return nominal_speed
    def project(points):
        return ([x * fx + y * fy for x, y in points],
                [-x * fy + y * fx for x, y in points])
    car_x, car_y = project(car_polygon)
    bus_x, bus_y = project(bus_polygon)
    if (min(bus_y) > max(car_y) + .25 or max(bus_y) < min(car_y) - .25 or
            (min(bus_x) + max(bus_x)) <= (min(car_x) + max(car_x))):
        return nominal_speed
    # A car alongside a bus can point slightly toward it while following a curve.
    # Projecting the LONG bus on only the car heading then invents a leader.
    # Require overlap in the bus lateral frame as well; keep genuine followers
    # and cars already entering the bus corridor protected.
    bx, by = bus_heading
    bnorm = math.hypot(bx, by)
    bx, by = bx / bnorm, by / bnorm
    car_lateral = [-x * by + y * bx for x, y in car_polygon]
    bus_lateral = [-x * by + y * bx for x, y in bus_polygon]
    if (min(bus_lateral) > max(car_lateral) + .25 or
            max(bus_lateral) < min(car_lateral) - .25):
        return nominal_speed
    gap = min(bus_x) - max(car_x)
    # Include reaction travel and equal-deceleration stopping distance. A 6 m
    # standstill gap gives TM's speed controller room before any physical contact.
    lead_speed = max(0., bus_velocity[0] * fx + bus_velocity[1] * fy)
    decel, reaction, standstill_gap = 2., 1., 6.
    safe_speed = max(0., math.sqrt(max(0., (decel * reaction) ** 2 +
        lead_speed ** 2 + 2 * decel * (gap - standstill_gap))) - decel * reaction)
    return min(nominal_speed, safe_speed)
