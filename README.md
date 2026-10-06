# AMR Fleet Coordinator (SIH26123) — decentralised build

    pip install -r requirements.txt
    python main.py                       # dashboard -> http://localhost:5001  (put your index.html in static/)
    python tests/test_decentralised.py   # 7 tests: no global access, radio-bounded knowledge, safety, failure recovery
    python benchmark.py 3000             # all layouts x 3/8/14 robots
    python benchmark.py 3000 compare     # decentralised vs the centralised reference (simulation_central/)
    python benchmark.py 3000 radio       # sensitivity to radio range / delay / packet loss

## Who decides what
| Concern | Where it lives | How it works (no central component) |
|---|---|---|
| Task allocation | `cbaa.py` (CBAAState, one per robot) | consensus: robots merge bid tables with neighbours; no auctioneer |
| Movement / collision avoidance | `agent.py` | claim -> wait `delay` ticks -> go; earliest claim wins; yield requests with priority inheritance |
| Route planning | `algorithms/space_time_astar.py` (LocalRoutePlanner) | each robot plans its own route against neighbours' broadcast paths |
| Deadlocks | `algorithms/deadlock.py` | edge-chasing probes along "who am I waiting for"; lowest priority retreats |
| Battery / charging | `decision/robot_brain.py` + `agent.py` | per-robot; charger conflicts settled between the robots |
| Radio | `comms.py` | range-limited, delayed, optionally lossy; the ONLY channel between robots |
| Engine | `engine.py` | physical world only: carries messages, moves bodies, measures. Never plans. |

## Assumptions
* Every robot carries the floor plan (static map). Obstacle changes are discovered when a planned cell turns out blocked.
* Orders are announced to all robots by an order system (task board); robots report pick-up / delivery to it. It makes no allocation decisions.
* Radio safe envelope (measured): **range >= 3.5 cells for delays up to 4 ticks (0.4 s)**; range 3 needs delay <= 2.
  10 % packet loss is tolerated; 30 % is not. With NO radio the fleet collides (that is the point of the isolation test).
* ORCA runs as an onboard safety layer using only bodies within its sensing radius; it never fired in any run.
* Chargers are the SPAWN cells of the layout.
* `simulation_central/` is the earlier centralised solver, kept ONLY as an upper-bound reference for benchmark.py.
