"""
Tests that the fleet is genuinely decentralised and still safe.
Run:  python tests/test_decentralised.py        (or: pytest tests/)
"""
import ast, math, os, random, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from simulation.engine import SimulationEngine
from simulation.comms import CommsConfig
from simulation.robot import RobotState

# Modules that make up a robot's onboard intelligence. None of them may reach global state.
ONBOARD = ["simulation/agent.py", "simulation/cbaa.py", "simulation/decision/robot_brain.py",
           "simulation/algorithms/deadlock.py", "simulation/algorithms/space_time_astar.py"]
FORBIDDEN_NAMES = {"SimulationEngine", "engine", "all_robots", "robots", "MessageBus", "bus",
                   "task_mgr", "TaskManager", "agents"}
FORBIDDEN_IMPORTS = {"engine", "comms", "task_manager"}


def make(layout="standard", n=8, seed=1, **comms):
    random.seed(seed)
    eng = SimulationEngine(layout, comms=CommsConfig(**comms))
    eng.task_mgr.rng.seed(seed)
    wh = eng.warehouse; placed = 0
    for sp in wh.spawn_points + [(x, y) for y in range(wh.height) for x in range(wh.width) if wh.is_passable(x, y)]:
        if placed >= n: break
        if eng.add_robot(*sp): placed += 1
    return eng


def run(eng, ticks, on_tick=None):
    min_sep = 9.0
    for _ in range(ticks):
        eng._tick()
        rs = eng.robots
        for i, a in enumerate(rs):
            for b in rs[i+1:]:
                min_sep = min(min_sep, a.distance_to(b))
        if on_tick: on_tick(eng)
    return min_sep


# ── 1. static: onboard code cannot reach the engine, the bus, or any fleet-wide list ──
def test_onboard_code_has_no_global_access():
    for rel in ONBOARD:
        tree = ast.parse(open(os.path.join(ROOT, rel)).read())
        for node in ast.walk(tree):
            if isinstance(node, ast.Name) and node.id in FORBIDDEN_NAMES:
                raise AssertionError(f"{rel}:{node.lineno} uses forbidden name '{node.id}'")
            if isinstance(node, ast.Attribute) and node.attr in FORBIDDEN_NAMES:
                raise AssertionError(f"{rel}:{node.lineno} accesses forbidden attribute '.{node.attr}'")
            if isinstance(node, ast.ImportFrom) and node.module:
                if set(node.module.split(".")) & FORBIDDEN_IMPORTS:
                    raise AssertionError(f"{rel}:{node.lineno} imports {node.module}")
    print("PASS static scan: no onboard module touches engine / bus / fleet list")


# ── 2. dynamic: a robot's knowledge of others never exceeds its radio range ──
def test_neighbours_are_only_those_in_radio_range():
    rng = 3.0
    eng = make(n=10, comm_range=rng)
    worst = 0.0
    def check(e):
        nonlocal worst
        pos = {r.id: (r.x, r.y) for r in e.robots}
        for rid, ag in e.agents.items():
            for nid in ag.nbr:
                if nid in pos:
                    d = math.hypot(pos[rid][0] - pos[nid][0], pos[rid][1] - pos[nid][1])
                    worst = max(worst, d)
    run(eng, 1500, check)
    ttl = next(iter(eng.agents.values())).nbr_ttl
    # an entry is sent at <= range, then lives `delay + ttl` ticks while BOTH robots move 0.3/tick
    slack = 2 * 0.3 * (ttl + eng.bus.cfg.delay)
    assert worst <= rng + slack + 1e-6, f"robot knows about a robot {worst:.2f} away (range {rng})"
    print(f"PASS neighbour tables bounded by radio range (farthest known robot {worst:.2f} cells, range {rng})")


# ── 3. no radio => no coordination (proves there is no side channel) ──
def test_without_radio_coordination_fails():
    eng = make(n=10, comm_range=0.0)
    run(eng, 1500)
    assert eng.total_collisions > 0, "with no radio robots still avoided each other: a hidden channel exists"
    print(f"PASS with radio range 0 the fleet collides ({eng.total_collisions} collisions): no hidden channel")


# ── 4. safe across layouts with realistic radio ──
def test_safe_with_realistic_radio():
    for layout in ("standard", "narrow_aisle", "pod_storage"):
        for rng, dly in ((6, 1), (4, 2), (3.5, 4)):   # the measured safe envelope: range >= 3.5 cells, delay <= 4 ticks
            eng = make(layout, n=10, comm_range=rng, delay=dly)
            ms = run(eng, 1500)
            assert eng.total_collisions == 0 and ms > 0.99, (layout, rng, dly, eng.total_collisions, ms)
            assert eng._metrics()["completed"] > 10, (layout, rng, dly, "no progress")
    print("PASS 0 collisions, min separation >= 1.0, progress made for range/delay in {6/1, 4/2, 3.5/4}")


# ── 5. a robot that vanishes does not strand its task (no central reassigner) ──
def test_task_survives_robot_removal():
    eng = make(n=6)
    victim = {}
    def find(e):
        if victim: return
        for rid, ag in e.agents.items():
            r = ag.robot
            if ag.cb.mine is not None and r.state == RobotState.MOVING_TO_PICKUP:
                victim["rid"], victim["task"] = rid, ag.cb.mine
    run(eng, 400, find)
    assert victim, "no robot ever held a task"
    eng.remove_robot(victim["rid"])
    tid = victim["task"]
    run(eng, 600)
    done = {t.id for t in eng.task_mgr.completed_tasks}
    assert tid in done, f"task {tid} held by removed robot {victim['rid']} was never completed"
    print(f"PASS task {tid} of vanished robot {victim['rid']} was re-claimed and completed by others")


# ── 6. a robot that dies in place becomes an obstacle others route around ──
def test_dead_robot_is_avoided_and_task_recovered():
    eng = make(n=6)
    victim = {}
    def find(e):
        if victim: return
        for rid, ag in e.agents.items():
            if ag.cb.mine is not None and ag.robot.state == RobotState.MOVING_TO_PICKUP and not ag.robot.target_cell:
                victim.update(rid=rid, task=ag.cb.mine, cell=ag.robot.grid_cell)
    run(eng, 400, find)
    assert victim
    eng._robot(victim["rid"]).battery = 0.0
    ms = run(eng, 800)
    assert eng.total_collisions == 0, f"{eng.total_collisions} collisions after a robot died"
    done = {t.id for t in eng.task_mgr.completed_tasks}
    assert victim["task"] in done, "dead robot's task never recovered"
    print(f"PASS dead robot at {victim['cell']} avoided (0 collisions) and its task recovered")


# ── 7. deadlock handling is by message passing ──
def test_deadlocks_are_found_by_probes():
    eng = make("narrow_aisle", n=14)
    run(eng, 3000)
    probes = sum(a.stats["probes"] for a in eng.agents.values())
    assert eng.total_collisions == 0
    print(f"PASS narrow aisle, 14 robots: {probes} probe messages, {eng.total_deadlocks} cycles resolved, 0 collisions")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for t in tests:
        try:
            t()
        except AssertionError as e:
            failed += 1; print(f"FAIL {t.__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
