# Build Log — SIH26123

## Build Progression

### Phase 1: Repository Scaffold + Fixed Foundations ✅
- `.gitignore`, `LICENSE`, `pytest.ini`
- ROS2 package manifest (`package.xml`, `setup.py`, `setup.cfg`)
- `scenario.py` — single source of truth for 9 robots + 6 chokepoint zones
- `amr_robot.urdf.xacro` — plain frame names, chassis_color arg
- `warehouse.world` — 4-shelf layout with `libgazebo_ros_state.so` plugin
- `generate_warehouse_map.py` — map generator matching world geometry
- `nav2_params_template.yaml` — plain frames, DWB at 0.9 m/s

### Phase 2: Pure-Python Negotiation Core + Tests ✅
- `negotiation/grid.py` — 0.5m coarse grid, world↔cell conversions, chokepoint zones
- `negotiation/reservation.py` — ReservationTable with atomic update (replace, not append)
- `negotiation/priority.py` — PriorityKey (lower tuple wins), resource ordering
- `negotiation/deadlock.py` — WaitForGraph with DFS cycle detection, lowest-priority backoff
- `negotiation/orca.py` — Simplified RVO reactive safety net (explicitly NOT full ORCA)
- `negotiation/negotiator.py` — Main evaluate() combining all modules
- 11 test files, 119 tests, all passing in 0.47s

### Phase 3: Multi-Robot Launch Infrastructure ✅
- `spawn_robots.launch.py` — staggered spawn from scenario.py
- `bringup_single.launch.py` — per-robot URDF + Nav2 stack under namespace
- `trial_baseline.launch.py` — uncoordinated baseline
- `trial_negotiated.launch.py` — full negotiated fleet
- `full_demo.launch.py` — negotiated + dashboard

### Phase 4: Custom A* + Spline Planner ✅
- `planners/astar.py` — 8-connected A* with corner-cutting prevention
- `planners/spline_smoother.py` — cubic spline (scipy fallback to linear)
- `planners/global_planner_node.py` — ROS2 ComputePathToPose wrapper
- Tests: 16 tests covering pathfinding, obstacle avoidance, smoothing

### Phase 5: Task Allocation — CNP + RL ✅
- `task_allocation/cnp.py` — Contract Net Protocol with deterministic bid evaluation
- `task_allocation/rl_policy.py` — Tabular Q-learning, ONLY outputs bid priority scalar
- `task_allocation/task_manager_node.py` — ROS2 node wrapper
- Tests: 18 tests including structural isolation verification

### Phase 6: Fault Tolerance + Fleet Agent + Support Nodes ✅
- `fault_tolerance/heartbeat.py` — pure-Python heartbeat monitor with timeout
- `fault_tolerance/heartbeat_node.py` — ROS2 wrapper
- `fleet_agent.py` — main per-robot coordination node
- `goal_dispatcher.py` — baseline uncoordinated dispatcher
- `collision_monitor.py` — ground-truth collision logger (Gazebo ModelStates)
- `blackboard_node.py` — info-only broadcaster (no control authority)
- `dashboard_bridge.py` — observer-only telemetry collector
- Tests: 11 heartbeat tests

### Phase 7: Dashboard ✅
- `dashboard/app.py` — FastAPI + WebSocket server (observer-only)
- `dashboard/static/index.html` — premium dark-mode UI
- `dashboard/static/style.css` — glassmorphic design with Inter + JetBrains Mono
- `dashboard/static/dashboard.js` — canvas warehouse renderer + live telemetry

### Phase 8: Documentation + Benchmark Harness ✅
- `benchmarks/README.md` — measurement methodology, honest reporting rules
- `docs/architecture.md` — system diagram, design decisions, module map
- `docs/demo-script.md` — 15-minute live presentation walkthrough
- `docs/build-log.md` — this file
- `README.md` — honest status table, quick start, key design decisions
- `tools/analyze_benchmark.py` — trial comparison + improvement % calculator

## Test Evidence

```
$ python3 -m pytest src/amr_fleet_core/test/ -v
119 passed in 0.47s
```

## What's Left

1. **Gazebo simulation trials** — requires Ubuntu with ROS2 Humble + Gazebo 11.
   Run `trial_baseline.launch.py` and `trial_negotiated.launch.py`, collect CSVs.
2. **Benchmark analysis** — run `analyze_benchmark.py` on the collected data.
3. **Map generation** — run `generate_warehouse_map.py` (requires Pillow).
4. **20% improvement verification** — measured from trial data, not asserted.
