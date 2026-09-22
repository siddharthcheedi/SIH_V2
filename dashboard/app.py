"""
dashboard/app.py — FastAPI + WebSocket dashboard server.

OBSERVER-ONLY: This server receives telemetry data from the
dashboard_bridge ROS2 node and serves it to the browser. It has NO
ROS2 imports and NO ability to issue commands to any robot.

Endpoints:
  GET /              — serve static HTML/JS/CSS
  WS  /ws/telemetry  — push robot telemetry at 2 Hz
  GET /api/status    — JSON snapshot of all robot states
"""

import asyncio
import json
import os
import time
from pathlib import Path
from typing import Dict

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles

app = FastAPI(title="AMR Fleet Dashboard", version="2.0.0")

STATIC_DIR = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

# Shared telemetry state — updated by the bridge, read by WebSocket clients
_telemetry: Dict[str, dict] = {}
_system_status = {
    "uptime_s": 0.0,
    "total_tasks": 0,
    "completed_tasks": 0,
    "collisions": 0,
    "near_misses": 0,
    "deadlocks_resolved": 0,
}
_start_time = time.time()


@app.get("/")
async def index():
    return FileResponse(str(STATIC_DIR / "index.html"))


@app.get("/api/status")
async def api_status():
    return {
        "robots": _telemetry,
        "system": {
            **_system_status,
            "uptime_s": time.time() - _start_time,
        },
    }


@app.post("/api/telemetry")
async def update_telemetry(data: dict):
    """Called by dashboard_bridge to push robot state updates."""
    for robot_id, state in data.get("robots", {}).items():
        _telemetry[robot_id] = state
    for key in ["total_tasks", "completed_tasks", "collisions",
                 "near_misses", "deadlocks_resolved"]:
        if key in data:
            _system_status[key] = data[key]
    return {"status": "ok"}


@app.websocket("/ws/telemetry")
async def ws_telemetry(websocket: WebSocket):
    """Push telemetry to connected dashboard clients at 2 Hz."""
    await websocket.accept()
    try:
        while True:
            payload = {
                "robots": _telemetry,
                "system": {
                    **_system_status,
                    "uptime_s": time.time() - _start_time,
                },
            }
            await websocket.send_json(payload)
            await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        pass
