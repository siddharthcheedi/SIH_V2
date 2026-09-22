# SIH26123 — Edge-AI Based Distributed Fleet Coordination for AMRs

> **Smart India Hackathon 2026** · Problem Statement SIH26123
> Organization: Bharat Electronics Limited

## What This Is

A fully decentralized, peer-to-peer multi-robot fleet coordination system
for Autonomous Mobile Robots (AMRs) in a smart warehouse. 9 robots
coordinate navigation through a shared warehouse with narrow aisles and
chokepoints — without any central coordinator.

## Honest Status

| Component | Status | Evidence |
|---|---|---|
| Negotiation core (grid, reservation, priority, deadlock, ORCA) | ✅ Complete | 119 pytest tests, all passing |
| A* + spline planner | ✅ Complete | Tests passing |
| CNP task allocation | ✅ Complete | Tests passing |
| RL bid priority policy | ✅ Complete | Tests passing, structural isolation verified |
| Heartbeat fault tolerance | ✅ Complete | Tests passing |
| Gazebo warehouse world | ✅ Complete | World file + map generator |
| Multi-robot launch infrastructure | ✅ Complete | spawn/bringup/baseline/negotiated/demo launch files |
| Dashboard (FastAPI + canvas) | ✅ Complete | WebSocket telemetry, observer-only |
| Benchmark harness | ⬜ Needs Gazebo trials | CSV logger ready, awaiting simulation runs |
| 20% improvement demonstrated | ⬜ Not yet measured | Awaiting baseline + negotiated trial comparison |
| Real hardware validation | ❌ Out of scope | Simulation only — stated honestly |

## Quick Start

```bash
# Prerequisites: Ubuntu 22.04, ROS2 Humble, Gazebo 11, Nav2

# 1. Install Python dependencies
pip install -e src/amr_fleet_core
pip install fastapi uvicorn[standard] websockets scipy Pillow

# 2. Generate the map
python3 tools/generate_warehouse_map.py

# 3. Run tests (119 tests, no ROS2 needed)
python3 -m pytest src/amr_fleet_core/test/ -v

# 4. Run full demo (requires ROS2 + Gazebo)
source /opt/ros/humble/setup.bash
ros2 launch amr_fleet_core full_demo.launch.py

# 5. Open dashboard
firefox http://localhost:8080
```

## Architecture

```
Every robot runs identical code:
  Fleet Agent → Negotiator → Reservation Table
       ↕              ↕
  Nav2 Stack    Peer Broadcasts (intent + heartbeat)
```

- **No central coordinator** — every robot negotiates directly with peers
- **Three-layer conflict resolution**: structural (resource ordering) →
  proactive (reservation negotiation) → reactive (simplified RVO)
- **RL confined to bid priority** — cannot touch motion or safety
- See [docs/architecture.md](docs/architecture.md) for full details

## Project Structure

```
SIH_V2/
├── src/amr_fleet_core/        # ROS2 package
│   ├── amr_fleet_core/        # Python source
│   │   ├── negotiation/       # Core P2P negotiation (pure Python)
│   │   ├── planners/          # A* + spline smoother
│   │   ├── task_allocation/   # CNP + RL policy
│   │   └── fault_tolerance/   # Heartbeat monitoring
│   └── test/                  # 119 pytest tests
├── launch/                    # ROS2 launch files
├── config/                    # Nav2 params template
├── worlds/                    # Gazebo warehouse
├── urdf/                      # Robot URDF (xacro)
├── maps/                      # Static map (generated)
├── dashboard/                 # FastAPI + canvas dashboard
├── benchmarks/                # Trial data + analysis
├── tools/                     # Map generator, analysis scripts
└── docs/                      # Architecture, demo script
```

## Key Design Decisions

1. **All logic in pure Python, zero `rclpy`** — grid, reservation, priority,
   deadlock, ORCA, CNP, RL are all tested with plain pytest on any OS.
   ROS2 nodes are thin wrappers.

2. **Atomic reservation updates** — `update_robot()` replaces (not appends)
   a peer's entire reservation set. This prevents stale-reservation buildup
   that causes false conflicts.

3. **Resource ordering** — zones are always acquired in ascending ID order,
   making circular-wait deadlock structurally impossible.

4. **Wait-for-graph** — catches N-robot cycles that resource ordering
   doesn't cover. Lowest-priority robot in the cycle backs off.

5. **Ground-truth collision data** — from Gazebo's world-frame ModelStates,
   not per-robot odometry (which drifts and isn't comparable across robots).

## Testing

```bash
$ python3 -m pytest src/amr_fleet_core/test/ -v
========================= 119 passed in 0.47s =========================
```

Tests cover: grid conversions, chokepoint zones, reservation conflicts,
priority ordering, deadlock detection (including no-false-positive test),
RVO avoidance, negotiation integration (proceed/yield/deadlock/stale-clear/
timeout), A* pathfinding, spline smoothing, CNP bid evaluation, RL state
discretization + structural isolation, and heartbeat timeout/recovery.

## License

MIT — see [LICENSE](LICENSE)
