# Archived investigation before the completed run 20

This is the historical diagnosis log. See [README](README.md) for the final result.

# Town05 bus course diagnosis — 2026-10-22

**Work in progress: no completed lap yet.** The best completed trial (run 16)
drove 531.74 m, passed S02–S05 and R01, then stopped at the loop-road entry.
No collision events were recorded; STOP and route cleanup succeeded. Its peak
speed was 18.09 km/h. The 20 km/h cap remains active. No service bell or passenger stop logic
is enabled, and CARLA autopilot is not used.

The runner uses S01 → S10, then S10 → S01, because a single Autoware route cannot
repeat its starting lanelet. It checks ordered anchors, collisions, fresh
localization/control and a 120-second no-progress timeout. Initial placement is
the only teleportation. A successful trial must complete both legs.

Current investigations:

- Preserve actual BPP boundaries in MPT instead of artificially widening them.
  Captured-path A/B replay changes an immediate boundary stop into a forward path.
- Maintain bus clearance through turn exits and use ten covering circles for the
  full measured collision rectangle. No footprint dimensions were reduced.
- Tag 212 proven nonjunction map edges/dividers using native CARLA geometry.
  Map coordinates and route topology remain unchanged; junction perimeters are
  not comprehensively classified by this tool.
- Preserve previous optimized poses in world coordinates across three sequential
  refinements. The stopped-pose anchor experiment was removed after convergence
  failures.
- Run 11 stopped at 23.82 m. Its left bound contains a one-point notch at the
  transition from a two-lane corridor into a turn. A physical edge endpoint's
  zero expansion limit also pins the internal endpoint of the transverse
  connector. The targeted correction passed unit tests and run 12 passed this point.

Run 10 changed parameters during the diagnostic drive (smoothing range, expansion
ceiling, clearance, extra width). It is not a clean validation of one fixed
configuration. Run 11 started with the combined settings and exposed the earlier
connector problem. Source/config edits remain experimental until a full lap is
verified. Raw failed reports are retained in compressed form.

Run 5 verified STOP but its cleanup was interrupted; its collision list is
unavailable, not zero. Subsequent restarts removed stale actors. The external
pre-mission backup and the pre-lap bus Docker image are retained.

```bash
scripts/carla/carla_autoware start-bus
scripts/carla/run_bus_lap
```

These commands start actual simulated driving. The runner is diagnostic tooling,
not a validated bus service. `tools/replay_bus_optimizer.py` instead runs only
captured-path optimization in isolated ROS domain 211, with no vehicle commands.

## Further turn diagnosis

Run 12 passed the former 23.82 m connector stop after the targeted fix and passed
S02. It stopped at 92.72 m during the next right turn. The conservative front-left
footprint corner was about 0.21 m outside the supplied corridor. No collision
sensor events were recorded; STOP and route cleanup succeeded.

The optimizer's linearized circle constraints reported zero violation even when
the full footprint check rejected the turn. On the same captured fixture, one
successful solve produced an immediate boundary stop (-0.30 m); three sequential
reference refinements produced a successful optimized path with its first boundary
stop 63.74 m ahead. Both replays had successful QPs (no fallback due to solver
failure). This is a captured-path improvement, not a completed live lap.

A rectangular circle-grid experiment did not establish a live improvement and
was removed from the active source. Large grids also caused solver convergence
failures, which can yield a fallback input path; a nonzero output velocity alone
is therefore not sufficient validation. The replay tool now records the optimizer
debug trajectory as well as its final output. Raw replay velocities are upstream
of the actual 20 km/h velocity cap.

## Current solver convergence diagnosis

Run 15 reached 162.50 m but a persistent minimum-risk stop followed low trajectory
publication rate; temporary trajectory-shift faults were also recorded. Reusing
fixed start poses in world coordinates and reducing the optimizer horizon from
100 to 60 one-metre points allowed run 16 to reach 531.74 m with the 20 km/h cap.
Run 16 eventually stopped with an `outside_drivable_area` factor approximately
0.96 m ahead. Solver maximum-iteration failures preceded the stop.

The new solver matrix includes each nonnegative slack constraint once instead of
repeating the same row for every circle with the infinity-norm penalty. The
feasible set and objective are unchanged. Isolated replays of both the run 15 and
run 16 failure fixtures produced successful QPs throughout, no optimizer-failure
fallbacks, and final optimized/output trajectories with matching timestamps.
Neither replay had a final boundary-stop factor. Live run 17 reached 319.22 m
before a different left-turn entry boundary stop; no full lap was completed.

A stored debug trajectory can be stale after a later failed solve. Validation
therefore checks timestamps and solver logs, in addition to output velocity.

Run 17 passed S04 but stopped before S05. The captured successful optimizer solve
still placed a rear corner 0.20 m outside a one-point notch in the corridor.
The lateral connector correction required virtually exact alignment with the
path projection, which does not hold at a curved approach. A small angular
tolerance is being tested while retaining nearest protected-edge distance caps.

`run_bus_lap OUTPUT --start-anchor S04` runs a diagnostic suffix from S04 through
S10 to S01. It reports `section_completed`, never `lap_completed`; a final full
validation must start at S01 with the default arguments.

## Suffix and restart validation

Run 18 started at S04 and completed the S10 leg (1691.05 m), passing the earlier
left-turn and loop-entry blockers plus every remaining route anchor. It failed
to restart for S01: the start planner interpreted the actual approximately 0.7 m
inward stop offset as a pull-out maneuver because its centerline threshold was
0.5 m. No safe pull-out path was produced for the wide bus.

The bus profile now uses a 1.0 m centerline threshold, allowing the normal path
planner to handle an in-lane restart while keeping road/object checks enabled.
Run 19 retained the actual S10 stopped pose (no teleport), set this parameter in
the running node, and completed the 85.01 m return to S01. Both runs recorded zero
collision events and successful cleanup. Neither is a full-lap result. Run 20
will start at S01 after restarting with the persisted complete profile.
