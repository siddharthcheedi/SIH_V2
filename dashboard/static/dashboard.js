/**
 * dashboard.js — Real-time warehouse fleet visualization.
 *
 * Connects to the FastAPI WebSocket and renders:
 *   - Warehouse floor plan (walls + shelves)
 *   - Robot positions with color-coded dots and labels
 *   - Chokepoint zones (semi-transparent)
 *   - Grid overlay
 *   - System metrics and decision log
 *
 * OBSERVER-ONLY: This script reads telemetry. It has no ability to send
 * commands to any robot or ROS2 topic.
 */

(() => {
    'use strict';

    // ─── Warehouse geometry (matches worlds/warehouse.world) ───────
    const WORLD = {
        minX: -10.5, maxX: 10.5,
        minY: -7.5,  maxY: 7.5,
    };

    const WALLS = [
        { cx: 0, cy: 7, w: 20.4, h: 0.2 },
        { cx: 0, cy: -7, w: 20.4, h: 0.2 },
        { cx: 10, cy: 0, w: 0.2, h: 14.4 },
        { cx: -10, cy: 0, w: 0.2, h: 14.4 },
    ];

    const SHELVES = [
        { cx: -6, cy: -1, w: 1.0, h: 8.0 },
        { cx: -2, cy: -1, w: 1.0, h: 8.0 },
        { cx:  2, cy: -1, w: 1.0, h: 8.0 },
        { cx:  6, cy: -1, w: 1.0, h: 8.0 },
    ];

    const CHOKE_ZONES = [
        { id: 0, cx: -4, cy: -5, r: 1.2, label: 'A1-S' },
        { id: 1, cx:  0, cy: -5, r: 1.2, label: 'A2-S' },
        { id: 2, cx:  4, cy: -5, r: 1.2, label: 'A3-S' },
        { id: 3, cx: -4, cy:  3, r: 1.2, label: 'A1-N' },
        { id: 4, cx:  0, cy:  3, r: 1.2, label: 'A2-N' },
        { id: 5, cx:  4, cy:  3, r: 1.2, label: 'A3-N' },
    ];

    const ROBOT_COLORS = {
        'amr_1': '#ef4444', 'amr_2': '#3b82f6', 'amr_3': '#22c55e',
        'amr_4': '#f59e0b', 'amr_5': '#a855f7', 'amr_6': '#06b6d4',
        'amr_7': '#ec4899', 'amr_8': '#f97316', 'amr_9': '#6366f1',
    };

    // ─── Canvas setup ──────────────────────────────────────────────
    const canvas = document.getElementById('warehouse-canvas');
    const ctx = canvas.getContext('2d');

    function worldToCanvas(x, y) {
        const px = ((x - WORLD.minX) / (WORLD.maxX - WORLD.minX)) * canvas.width;
        const py = ((WORLD.maxY - y) / (WORLD.maxY - WORLD.minY)) * canvas.height;
        return [px, py];
    }

    function worldToCanvasSize(w, h) {
        const pw = (w / (WORLD.maxX - WORLD.minX)) * canvas.width;
        const ph = (h / (WORLD.maxY - WORLD.minY)) * canvas.height;
        return [pw, ph];
    }

    // ─── Drawing functions ─────────────────────────────────────────
    function drawFloor() {
        ctx.fillStyle = '#1a1f2e';
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        // Grid lines
        ctx.strokeStyle = 'rgba(99, 102, 241, 0.06)';
        ctx.lineWidth = 0.5;
        const cellSize = 0.5;
        for (let x = WORLD.minX; x <= WORLD.maxX; x += cellSize) {
            const [px] = worldToCanvas(x, 0);
            ctx.beginPath();
            ctx.moveTo(px, 0);
            ctx.lineTo(px, canvas.height);
            ctx.stroke();
        }
        for (let y = WORLD.minY; y <= WORLD.maxY; y += cellSize) {
            const [, py] = worldToCanvas(0, y);
            ctx.beginPath();
            ctx.moveTo(0, py);
            ctx.lineTo(canvas.width, py);
            ctx.stroke();
        }
    }

    function drawWalls() {
        ctx.fillStyle = '#475569';
        ctx.strokeStyle = '#64748b';
        ctx.lineWidth = 1;
        for (const wall of WALLS) {
            const [x, y] = worldToCanvas(wall.cx - wall.w / 2, wall.cy + wall.h / 2);
            const [w, h] = worldToCanvasSize(wall.w, wall.h);
            ctx.fillRect(x, y, w, h);
            ctx.strokeRect(x, y, w, h);
        }
    }

    function drawShelves() {
        for (const shelf of SHELVES) {
            const [x, y] = worldToCanvas(shelf.cx - shelf.w / 2, shelf.cy + shelf.h / 2);
            const [w, h] = worldToCanvasSize(shelf.w, shelf.h);

            // Shelf body
            ctx.fillStyle = '#92400e';
            ctx.fillRect(x, y, w, h);

            // Shelf outline
            ctx.strokeStyle = '#b45309';
            ctx.lineWidth = 1.5;
            ctx.strokeRect(x, y, w, h);

            // Shelf pattern (horizontal lines for "shelving")
            ctx.strokeStyle = 'rgba(180, 83, 9, 0.4)';
            ctx.lineWidth = 0.5;
            const rows = 8;
            for (let i = 1; i < rows; i++) {
                const ly = y + (h / rows) * i;
                ctx.beginPath();
                ctx.moveTo(x, ly);
                ctx.lineTo(x + w, ly);
                ctx.stroke();
            }
        }
    }

    function drawChokeZones() {
        for (const zone of CHOKE_ZONES) {
            const [cx, cy] = worldToCanvas(zone.cx, zone.cy);
            const [rw] = worldToCanvasSize(zone.r * 2, 0);
            const r = rw / 2;

            // Zone glow
            ctx.beginPath();
            ctx.arc(cx, cy, r, 0, Math.PI * 2);
            ctx.fillStyle = 'rgba(99, 102, 241, 0.08)';
            ctx.fill();
            ctx.strokeStyle = 'rgba(99, 102, 241, 0.25)';
            ctx.lineWidth = 1;
            ctx.setLineDash([4, 4]);
            ctx.stroke();
            ctx.setLineDash([]);

            // Zone label
            ctx.fillStyle = 'rgba(99, 102, 241, 0.5)';
            ctx.font = '10px "JetBrains Mono", monospace';
            ctx.textAlign = 'center';
            ctx.fillText(zone.label, cx, cy + 4);
        }
    }

    function drawRobot(x, y, robotId, color, status) {
        const [cx, cy] = worldToCanvas(x, y);
        const radius = 8;

        // Glow ring
        ctx.beginPath();
        ctx.arc(cx, cy, radius + 4, 0, Math.PI * 2);
        ctx.fillStyle = color + '20';
        ctx.fill();

        // Robot body
        ctx.beginPath();
        ctx.arc(cx, cy, radius, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Label
        ctx.fillStyle = '#fff';
        ctx.font = 'bold 10px "JetBrains Mono", monospace';
        ctx.textAlign = 'center';
        ctx.fillText(robotId.replace('amr_', ''), cx, cy + 3.5);

        // Status indicator below
        if (status === 'yielding') {
            ctx.fillStyle = '#fbbf24';
            ctx.beginPath();
            ctx.arc(cx, cy + radius + 6, 3, 0, Math.PI * 2);
            ctx.fill();
        } else if (status === 'deadlock') {
            ctx.fillStyle = '#f87171';
            ctx.beginPath();
            ctx.arc(cx, cy + radius + 6, 3, 0, Math.PI * 2);
            ctx.fill();
        }
    }

    function drawFrame(robots) {
        drawFloor();
        drawChokeZones();
        drawWalls();
        drawShelves();

        // Draw robots
        for (const [rid, data] of Object.entries(robots)) {
            const color = ROBOT_COLORS[rid] || '#9ca3af';
            drawRobot(data.x, data.y, rid, color, data.status);
        }
    }

    // ─── UI update functions ───────────────────────────────────────
    function updateMetrics(system) {
        const uptime = Math.floor(system.uptime_s || 0);
        const mins = Math.floor(uptime / 60);
        const secs = uptime % 60;
        document.getElementById('metric-uptime').textContent =
            `${mins}:${secs.toString().padStart(2, '0')}`;

        document.getElementById('metric-tasks').textContent =
            `${system.completed_tasks || 0}/${system.total_tasks || 0}`;

        const collEl = document.getElementById('metric-collisions');
        collEl.textContent = system.collisions || 0;
        if (system.collisions > 0) {
            collEl.closest('.metric-card').classList.add('alert-card');
        }

        document.getElementById('metric-deadlocks').textContent =
            system.deadlocks_resolved || 0;
    }

    function updateRobotList(robots) {
        const list = document.getElementById('robot-list');
        const count = Object.keys(robots).length;
        document.getElementById('robot-count').textContent = `${count} robots`;

        if (count === 0) {
            list.innerHTML = '<div class="empty-state">Waiting for telemetry…</div>';
            return;
        }

        list.innerHTML = Object.entries(robots).map(([rid, data]) => {
            const color = ROBOT_COLORS[rid] || '#9ca3af';
            const bat = (data.battery || 1.0) * 100;
            const batClass = bat < 20 ? 'low' : bat < 50 ? 'medium' : '';
            const explain = data.explain || 'Idle';

            return `
                <div class="robot-card">
                    <div class="robot-color" style="color: ${color}; background: ${color}"></div>
                    <div class="robot-info">
                        <div class="robot-name">${rid}</div>
                        <div class="robot-status" title="${explain}">${explain}</div>
                    </div>
                    <div class="robot-battery">
                        <div class="robot-battery-fill ${batClass}" style="width: ${bat}%"></div>
                    </div>
                </div>
            `;
        }).join('');
    }

    const MAX_LOG_ENTRIES = 100;
    function addLogEntry(text) {
        const log = document.getElementById('decision-log');
        if (log.querySelector('.empty-state')) {
            log.innerHTML = '';
        }

        const type = text.includes('PROCEED') ? 'proceed' :
                     text.includes('YIELD') || text.includes('yield') ? 'yield' :
                     text.includes('DEADLOCK') || text.includes('deadlock') ? 'deadlock' :
                     text.includes('COLLISION') ? 'collision' : '';

        const entry = document.createElement('div');
        entry.className = `log-entry ${type}`;

        const now = new Date();
        const ts = `${now.getHours().toString().padStart(2,'0')}:${now.getMinutes().toString().padStart(2,'0')}:${now.getSeconds().toString().padStart(2,'0')}`;

        entry.innerHTML = `<span class="timestamp">${ts}</span> ${text}`;
        log.prepend(entry);

        // Trim old entries
        while (log.children.length > MAX_LOG_ENTRIES) {
            log.removeChild(log.lastChild);
        }
    }

    // ─── WebSocket connection ──────────────────────────────────────
    let ws = null;
    let reconnectTimer = null;

    function connect() {
        const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
        const url = `${protocol}//${location.host}/ws/telemetry`;

        ws = new WebSocket(url);

        ws.onopen = () => {
            const statusEl = document.getElementById('connection-status');
            statusEl.querySelector('.status-dot').className = 'status-dot connected';
            statusEl.querySelector('.status-text').textContent = 'Connected';
            addLogEntry('Dashboard connected to telemetry stream');
        };

        ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                if (data.robots) {
                    drawFrame(data.robots);
                    updateRobotList(data.robots);
                }
                if (data.system) {
                    updateMetrics(data.system);
                }
            } catch (e) {
                console.warn('Bad telemetry payload:', e);
            }
        };

        ws.onclose = () => {
            const statusEl = document.getElementById('connection-status');
            statusEl.querySelector('.status-dot').className = 'status-dot disconnected';
            statusEl.querySelector('.status-text').textContent = 'Disconnected';

            // Reconnect after 3s
            if (!reconnectTimer) {
                reconnectTimer = setTimeout(() => {
                    reconnectTimer = null;
                    connect();
                }, 3000);
            }
        };

        ws.onerror = () => {
            ws.close();
        };
    }

    // ─── Clear log button ──────────────────────────────────────────
    document.getElementById('clear-log').addEventListener('click', () => {
        document.getElementById('decision-log').innerHTML =
            '<div class="empty-state">Log cleared</div>';
    });

    // ─── Initial render ────────────────────────────────────────────
    drawFrame({});
    connect();

    // Fallback: if no WebSocket, poll /api/status every 2s
    setInterval(async () => {
        if (ws && ws.readyState === WebSocket.OPEN) return;
        try {
            const resp = await fetch('/api/status');
            const data = await resp.json();
            if (data.robots) {
                drawFrame(data.robots);
                updateRobotList(data.robots);
            }
            if (data.system) updateMetrics(data.system);
        } catch (e) {
            // Server not reachable — draw empty warehouse
        }
    }, 2000);

})();
