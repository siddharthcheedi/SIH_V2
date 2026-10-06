"""
main.py — Flask + SocketIO server for the AMR fleet. The server only hosts the PHYSICAL world
(radio, bodies, order board); every robot decides for itself (simulation/agent.py).
"""
import eventlet
eventlet.monkey_patch()

import time, threading
from flask import Flask, jsonify, request
from flask_socketio import SocketIO

from simulation.engine import SimulationEngine
from simulation.layout_manager import list_layouts, save_layout
from simulation.cbaa import Task

app = Flask(__name__, static_url_path='', static_folder='static')
app.config['SECRET_KEY'] = 'amr-sih26123-adv'
socketio = SocketIO(app, async_mode='eventlet', cors_allowed_origins='*',
                    logger=False, engineio_logger=False)

engine = SimulationEngine("standard")

# Seed with 3 robots at spawn points
_spawns = engine.warehouse.spawn_points or [(1,0),(1,4),(1,8)]
for sp in _spawns[:3]:
    engine.add_robot(sp[0], sp[1])

engine.start()


# ── Broadcast loop ─────────────────────────────────────────────────────────────
def _broadcast():
    while True:
        try:
            state = engine.get_full_state()
            socketio.emit('state_update', state)
        except Exception:
            pass
        time.sleep(0.10)

threading.Thread(target=_broadcast, daemon=True).start()


# ── Routes ──────────────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return app.send_static_file('index.html')

@app.route('/api/state')
def api_state():
    return jsonify(engine.get_full_state())

@app.route('/api/layouts')
def api_layouts():
    return jsonify(list_layouts())

@app.route('/api/layout', methods=['POST'])
def api_layout():
    name = request.json.get('name', 'standard')
    try:
        engine.load_layout(name)
        return jsonify({'ok': True, 'name': name})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 400

@app.route('/api/robot/add', methods=['POST'])
def api_add_robot():
    d = request.json
    r = engine.add_robot(int(d['x']), int(d['y']),
                         float(d.get('speed', 3.0)), float(d.get('battery', 100.0)))
    if r:
        return jsonify({'ok': True, 'id': r.id})
    return jsonify({'ok': False, 'error': 'Cell impassable or occupied'}), 400

@app.route('/api/robot/remove', methods=['POST'])
def api_remove_robot():
    ok = engine.remove_robot(int(request.json['id']))
    return jsonify({'ok': ok})

@app.route('/api/robot/speed', methods=['POST'])
def api_robot_speed():
    d = request.json
    ok = engine.set_robot_speed(int(d['id']), float(d['speed']))
    return jsonify({'ok': ok})

@app.route('/api/obstacle', methods=['POST'])
def api_obstacle():
    d = request.json
    blocked = engine.toggle_dynamic_obstacle(int(d['x']), int(d['y']))
    return jsonify({'ok': True, 'blocked': blocked})

@app.route('/api/task', methods=['POST'])
def api_task():
    d = request.json
    t = engine.add_manual_task(tuple(d['pickup']), tuple(d['dropoff']))
    return jsonify({'ok': True, 'task': t.to_dict()})

@app.route('/api/cell', methods=['POST'])
def api_cell():
    d = request.json
    engine.set_cell(int(d['x']), int(d['y']), int(d['type']))
    return jsonify({'ok': True})

@app.route('/api/layout/save', methods=['POST'])
def api_layout_save():
    name = request.json.get('name', 'custom')
    path = save_layout(engine.warehouse, name)
    return jsonify({'ok': True, 'path': path})

@app.route('/api/reset', methods=['POST'])
def api_reset():
    engine.reset()
    return jsonify({'ok': True})

@app.route('/api/speed', methods=['POST'])
def api_speed():
    spd = float(request.json.get('speed', 1.0))
    engine.sim_speed = max(0.1, min(10.0, spd))
    return jsonify({'ok': True, 'speed': engine.sim_speed})

@app.route('/api/mode', methods=['POST'])
def api_mode():
    engine.baseline_mode = bool(request.json.get('baseline', False))
    return jsonify({'ok': True})

@app.route('/api/comms', methods=['POST'])
def api_comms():
    # radio model: {"range": cells, "delay": ticks, "loss": 0..0.9}. Safe envelope: range >= 3.5, delay <= 4
    d = request.json or {}
    stats = engine.set_comms(d.get('range'), d.get('delay'), d.get('loss'))
    return jsonify({'ok': True, 'comms': stats})

@app.route('/api/pause', methods=['POST'])
def api_pause():
    engine.paused = not engine.paused
    return jsonify({'ok': True, 'paused': engine.paused})


if __name__ == '__main__':
    print("=" * 60)
    print("  AMR Fleet Coordinator — SIH26123 (Decentralised v3)")
    print("  Dashboard → http://localhost:5001")
    print("=" * 60)
    socketio.run(app, host='0.0.0.0', port=5001, debug=False)
