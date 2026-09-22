# Benchmark Harness — SIH26123

## Overview

The benchmark harness compares two configurations:

1. **Baseline** — Uncoordinated: all robots get NavigateToPose goals
   simultaneously, relying only on DWB's local obstacle avoidance.
2. **Negotiated** — Full P2P coordination: reservation-based negotiation,
   priority + deadlock prevention, ORCA safety net, heartbeat fault tolerance.

## What Gets Measured

| Metric | Source | File |
|---|---|---|
| Per-robot task completion time (s) | `goal_dispatcher.py` / `fleet_agent.py` timestamps | `benchmarks/raw/{baseline,negotiated}_task_times.csv` |
| Collisions (center-to-center < 0.45m) | `collision_monitor.py` via Gazebo ModelStates | `benchmarks/raw/collision_events.csv` |
| Near-misses (center-to-center < 0.85m) | `collision_monitor.py` | `benchmarks/raw/collision_events.csv` |
| Deadlock events | `deadlock.py` WaitForGraph event log | `benchmarks/raw/deadlock_events.csv` |
| Negotiation decisions (per-event) | `fleet_agent.py` explain topic | `benchmarks/raw/explain_log.csv` |

## Running a Trial

```bash
# Terminal 1: Baseline trial
ros2 launch amr_fleet_core trial_baseline.launch.py

# Terminal 2: Negotiated trial
ros2 launch amr_fleet_core trial_negotiated.launch.py
```

Each trial runs until all 9 robots reach their goals or 5 minutes elapse
(whichever comes first). Output CSVs are written to `benchmarks/raw/`.

## Analysis

```bash
python3 tools/analyze_benchmark.py \
    --baseline benchmarks/raw/baseline_task_times.csv \
    --negotiated benchmarks/raw/negotiated_task_times.csv \
    --collisions benchmarks/raw/collision_events.csv
```

This produces:
- Per-robot improvement table
- Aggregate improvement % (must be ≥20%)
- Collision count comparison
- Statistical confidence (paired t-test if ≥3 trials)

## Honest Reporting Rules

1. **No cherry-picking** — all trials are reported, not just the best ones.
2. **No self-comparison** — baseline and negotiated use identical robot
   configurations, world geometry, and goal positions.
3. **Ground-truth distances** — collision data comes from Gazebo's world-frame
   ModelStates, not per-robot odometry (which drifts and is not comparable).
4. **Simulation time** — all timestamps use `use_sim_time=true` Gazebo clock,
   not wall clock (which varies with CPU load).
5. **Measured, not asserted** — the 20% improvement target is measured from
   CSV data. If the measured improvement is less than 20%, that's what we
   report — we don't fake numbers.

## Expected Results

Based on the conflict density of the warehouse layout (6 chokepoints,
3 inter-shelf aisles with 9 robots), we expect:

- Baseline: frequent stop-start behavior at aisle intersections, occasional
  DWB-only deadlocks requiring manual recovery, ~40-60s total task time.
- Negotiated: smoother aisle traversal with proactive yielding, zero
  true deadlocks (wait-for-graph resolves them), ~25-40s total task time.
- Improvement: likely 25-40% depending on robot-goal assignment.

These are ESTIMATES — actual numbers come from measurements.
