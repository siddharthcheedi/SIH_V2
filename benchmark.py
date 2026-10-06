"""
benchmark.py — headless runs, no server.

    python benchmark.py                  # decentralised build, all layouts, 3/8/14 robots
    python benchmark.py 3000 compare     # decentralised vs centralised reference (simulation_central/)
    python benchmark.py 3000 radio       # sensitivity to comm range / delay / packet loss

Reports tasks done, throughput, mean end-to-end latency, collisions, minimum robot
separation (cells; bodies touch below 0.8), dropped tasks, dead robots, radio load.
"""
import sys, importlib, random, time

def load(pkg="simulation"):
    return importlib.import_module(pkg + ".engine"), importlib.import_module(pkg + ".comms") \
        if pkg == "simulation" else None

def run(layout, nrob, ticks, pkg="simulation", baseline=False, seed=1,
        comm_range=6.0, delay=1, loss=0.0):
    E, C = load(pkg)
    random.seed(seed)
    if pkg == "simulation":
        eng = E.SimulationEngine(layout, comms=C.CommsConfig(comm_range=comm_range, delay=delay, loss=loss, seed=seed))
    else:
        eng = E.SimulationEngine(layout)
    eng.baseline_mode = baseline
    eng.task_mgr.rng.seed(seed)
    wh = eng.warehouse
    placed = 0
    for sp in wh.spawn_points + [(x, y) for y in range(wh.height) for x in range(wh.width) if wh.is_passable(x, y)]:
        if placed >= nrob: break
        if eng.add_robot(*sp): placed += 1
    min_sep, worst, t0 = 9.0, 0.0, time.time()
    for _ in range(ticks):
        t1 = time.time(); eng._tick(); worst = max(worst, time.time() - t1)
        rs = eng.robots
        for i, a in enumerate(rs):
            for b in rs[i+1:]:
                d = a.distance_to(b)
                if d < min_sep: min_sep = d
    m = eng._metrics()
    return dict(layout=layout, n=nrob, done=m["completed"], per_min=round(m["completed"] / (ticks * 0.1 / 60), 1),
                lat=m["avg_latency"], exec=m["avg_completion_time"], coll=eng.total_collisions,
                min_sep=round(min_sep, 3), dl=eng.total_deadlocks, drops=eng.total_drops,
                safety=eng.safety_interventions, dead=sum(1 for r in eng.robots if r.battery <= 0),
                msgs=m.get("msgs_per_robot_per_tick", 0), worst_ms=round(worst * 1000), eng=eng)

HDR = f"{'config':26s} {'done':>5s} {'/min':>5s} {'lat':>6s} {'exec':>6s} {'coll':>4s} {'minsep':>6s} {'dl':>3s} {'drop':>4s} {'safe':>4s} {'dead':>4s} {'msg/r/t':>7s} {'ms':>4s}"
def row(label, r):
    return (f"{label:26s} {r['done']:5d} {r['per_min']:5.1f} {r['lat']:6.1f} {r['exec']:6.1f} {r['coll']:4d} "
            f"{r['min_sep']:6.3f} {r['dl']:3d} {r['drops']:4d} {r['safety']:4d} {r['dead']:4d} {r['msgs']:7} {r['worst_ms']:4d}")

if __name__ == "__main__":
    ticks = int(sys.argv[1]) if len(sys.argv) > 1 else 3000
    mode = sys.argv[2] if len(sys.argv) > 2 else "matrix"
    print(HDR)
    if mode == "matrix":
        for layout in ["standard", "narrow_aisle", "cross_shaped", "pod_storage", "open_floor"]:
            for n in (3, 8, 14):
                print(row(f"{layout} n={n}", run(layout, n, ticks)), flush=True)
    elif mode == "compare":
        for layout in ["standard", "narrow_aisle", "pod_storage"]:
            for n in (8, 14):
                print(row(f"{layout} n={n} CENTRAL", run(layout, n, ticks, pkg="simulation_central")), flush=True)
                print(row(f"{layout} n={n} DECENTR.", run(layout, n, ticks)), flush=True)
    elif mode == "radio":
        for rng_, dly, loss in [(10,1,0),(6,1,0),(4,1,0),(3,1,0),(2,1,0),(6,2,0),(6,3,0),(6,1,.1),(6,1,.3),(0,1,0)]:
            print(row(f"range={rng_} delay={dly} loss={loss}", run("standard", 10, ticks, comm_range=rng_, delay=dly, loss=loss)), flush=True)
