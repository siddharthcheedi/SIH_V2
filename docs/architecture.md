# Architecture — SIH26123

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    Per-Robot Process (×9)                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │ Nav2 Stack   │  │ Fleet Agent  │  │ Task Manager (CNP)   │  │
│  │  • AMCL      │  │  • Negotiate │  │  • Bid evaluation    │  │
│  │  • DWB       │  │  • ORCA      │  │  • RL bid priority   │  │
│  │  • Planner   │  │  • Heartbeat │  │  • Award tracking    │  │
│  └──────┬───────┘  └──────┬───────┘  └──────────┬───────────┘  │
│         │                  │                      │              │
│         │    cmd_vel       │   intent/heartbeat   │  task msgs   │
│         ▼                  ▼                      ▼              │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │               ROS2 Namespace: /<robot_id>/              │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────┬───────────────────────────────────┘
                              │
            ┌─────────────────┼─────────────────┐
            │                 │                  │
            ▼                 ▼                  ▼
   ┌────────────────┐ ┌──────────────┐ ┌────────────────────┐
   │  Peer Robots   │ │  Blackboard  │ │ Dashboard Bridge   │
   │  (same arch)   │ │  (info only) │ │ (observer only)    │
   └────────────────┘ └──────────────┘ └────────┬───────────┘
                                                 │
                                          ┌──────▼──────┐
                                          │  Dashboard   │
                                          │  (FastAPI)   │
                                          └─────────────┘
```

## Key Design Decisions

### 1. Fully Decentralized — No Hidden Leader

Every robot runs identical code. There is no central allocator, coordinator,
or arbitrator. The "blackboard" is a convenience broadcaster of map/task
info — it cannot make decisions, and killing it has zero effect on fleet
coordination (tested by literally killing the process mid-trial).

### 2. Three-Layer Conflict Resolution

| Layer | Mechanism | When it fires |
|---|---|---|
| **Structural** | Resource ordering (ascending zone_id) | Build time — impossible to form circular wait |
| **Proactive** | Reservation table + priority negotiation | Planning time — before committing to a path |
| **Reactive** | Simplified RVO (ORCA-like) | Execution time — when reality drifts from plan |

### 3. RL Confinement

The RL policy (tabular Q-learning) outputs ONLY a bid-priority scalar.
It has zero access to cmd_vel, reservations, or collision avoidance.
This is enforced structurally — the module doesn't import any motion
interface. The structural isolation is verified by a test.

### 4. Multi-Robot TF Pattern

Standard Nav2 pattern:
- URDF uses plain frame names (`base_link`, `odom`, `map`)
- Each robot runs in its own ROS2 namespace (`/amr_1/`, `/amr_2/`, ...)
- `nav2_bringup` handles topic remapping (`/tf` → `/<ns>/tf`)
- No frame-name prefixing (which breaks costmap lookups)

### 5. Ground-Truth Collision Monitoring

Collision detection uses Gazebo's `ModelStates` message, which provides
world-frame positions for all models. We do NOT use per-robot odometry
for this, because each robot's odom frame drifts independently and the
coordinates are not comparable across robots.

## Module Map

```
src/amr_fleet_core/amr_fleet_core/
├── scenario.py                 # Single source of truth (robots, zones)
├── fleet_agent.py              # Per-robot coordination node
├── goal_dispatcher.py          # Baseline (uncoordinated) goal sender
├── collision_monitor.py        # Ground-truth collision logger
├── blackboard_node.py          # Info broadcaster (no control)
├── dashboard_bridge.py         # Observer-only telemetry bridge
├── initial_pose_publisher.py   # AMCL pose seeder
│
├── negotiation/
│   ├── grid.py                 # Coarse reservation grid + zones
│   ├── reservation.py          # Reservation table (atomic update)
│   ├── priority.py             # Priority keys + resource ordering
│   ├── deadlock.py             # Wait-for-graph cycle detection
│   ├── orca.py                 # Simplified RVO safety net
│   └── negotiator.py           # Main evaluate() entry point
│
├── planners/
│   ├── astar.py                # Custom A* on 8-connected grid
│   ├── spline_smoother.py      # Cubic spline path smoothing
│   └── global_planner_node.py  # ROS2 action server wrapper
│
├── task_allocation/
│   ├── cnp.py                  # Contract Net Protocol
│   ├── rl_policy.py            # Tabular Q-learning bid priority
│   └── task_manager_node.py    # ROS2 node wrapper
│
└── fault_tolerance/
    ├── heartbeat.py            # Pure-Python heartbeat monitor
    └── heartbeat_node.py       # ROS2 node wrapper
```

## What This Is NOT

- Not a centralized fleet manager with a single point of failure
- Not an RL-controlled collision avoidance system
- Not a production-grade warehouse management system
- Not validated on real hardware (simulation only, stated honestly)
