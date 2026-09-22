# Demo Script — SIH26123 Live Presentation

## Prerequisites

- Ubuntu 22.04 with ROS2 Humble
- Gazebo Classic 11
- Nav2 (`sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`)
- Python 3.10+ with `pip install fastapi uvicorn[standard] websockets scipy Pillow`
- This repository cloned and `pip install -e src/amr_fleet_core`
- Map generated: `python3 tools/generate_warehouse_map.py`

## Pre-Demo Checklist

```bash
# 1. Source ROS2
source /opt/ros/humble/setup.bash

# 2. Build (if using colcon)
cd SIH_V2 && colcon build --packages-select amr_fleet_core
source install/setup.bash

# 3. Verify tests pass
python3 -m pytest src/amr_fleet_core/test/ -v
# Expected: 119 passed
```

## Demo Flow (15 minutes)

### Part 1: Architecture Overview (3 min)
- Show `docs/architecture.md` on screen
- Key points:
  - Fully decentralized — no central brain
  - Three-layer conflict resolution (structural → proactive → reactive)
  - RL confined to bid priority only (show the structural isolation test)

### Part 2: Tests & Correctness (2 min)
```bash
python3 -m pytest src/amr_fleet_core/test/ -v --tb=short
```
- Highlight key tests:
  - `test_deadlock.py::test_no_false_positive_on_contention_without_cycle`
  - `test_reservation.py::test_update_supersedes_not_appends`
  - `test_rl_policy.py::test_structural_isolation`
  - `test_negotiator.py::test_conflict_clears_on_plan_supersede`

### Part 3: Baseline Trial (3 min)
```bash
ros2 launch amr_fleet_core trial_baseline.launch.py
```
- Point out: robots getting stuck at intersections, DWB oscillation
- Note collision count and task completion times

### Part 4: Negotiated Trial (4 min)
```bash
ros2 launch amr_fleet_core trial_negotiated.launch.py
```
- Open dashboard: `http://localhost:8080`
- Point out:
  - Smooth aisle traversal
  - Robots yielding proactively (see "Yielding to amr_X" in explain log)
  - Zero collisions (check collision_monitor output)
  - Decision log showing real-time negotiation

### Part 5: Benchmark Comparison (2 min)
```bash
python3 tools/analyze_benchmark.py \
    --baseline benchmarks/raw/baseline_task_times.csv \
    --negotiated benchmarks/raw/negotiated_task_times.csv
```
- Show improvement percentage
- Show collision count difference
- If improvement < 20%, say so honestly and explain why

### Part 6: Fault Tolerance (1 min)
```bash
# Kill the blackboard mid-trial
ros2 lifecycle set /blackboard shutdown

# Kill one robot's fleet agent
kill -9 $(pgrep -f "amr_3_fleet_agent")
```
- Show: remaining robots detect timeout via heartbeat
- Show: amr_3's task is re-announced via CNP
- Show: fleet continues operating

## Troubleshooting

| Problem | Solution |
|---|---|
| Gazebo crashes on startup | `killall gzserver gzclient` then retry |
| Robot doesn't move | Check AMCL initial pose: `ros2 topic echo /amr_1/amcl_pose` |
| Dashboard blank | Verify FastAPI: `curl http://localhost:8080/api/status` |
| Nav2 lifecycle stuck | `ros2 lifecycle set /amr_1/controller_server activate` |
