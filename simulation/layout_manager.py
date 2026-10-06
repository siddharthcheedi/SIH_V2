"""
layout_manager.py — Built-in warehouse layout library + save/load support.

Provides:
  - Several pre-built warehouse layouts (standard, narrow aisle, cross, open, pods)
  - save_layout / load_layout (JSON files in layouts/ directory)
  - list_layouts()
"""

from __future__ import annotations
import json
from pathlib import Path
from typing import Dict, List

from .warehouse import Warehouse, FREE, OBSTACLE, DROP, PICKUP, SPAWN


LAYOUTS_DIR = Path(__file__).parent.parent / "layouts"


# ── layout builders ───────────────────────────────────────────────────────────

def _make_standard(w=25, h=20) -> Warehouse:
    """Classic horizontal shelf rows with wide cross-aisles."""
    wh = Warehouse(w, h)
    wh.name = "Standard Warehouse"
    shelf_rows  = [3, 7, 11, 15]
    aisle_cols  = [6, 12, 18]    # vertical aisles through shelves
    shelf_start, shelf_end = 4, w - 2

    for row in shelf_rows:
        for col in range(shelf_start, shelf_end + 1):
            if col not in aisle_cols:
                wh.set_cell(col, row, OBSTACLE)
            # Pickup points on both sides of each shelf row
        for col in range(shelf_start, shelf_end + 1):
            if wh.cell(col, row) == FREE:
                continue
            for dy in [-1, 1]:
                ny = row + dy
                if 0 <= ny < h and wh.cell(col, ny) == FREE:
                    if col not in aisle_cols:
                        wh.set_cell(col, ny, PICKUP)

    # Drop zones: left column
    for y in range(0, h, 2):
        wh.set_cell(0, y, DROP)
        if y + 1 < h:
            wh.set_cell(0, y+1, DROP)

    # Spawn points: column 1
    for y in range(0, h, 3):
        wh.set_cell(1, y, SPAWN)

    return wh


def _make_narrow_aisle(w=28, h=22) -> Warehouse:
    """Very narrow aisles — AMR coordination stress test."""
    wh = Warehouse(w, h)
    wh.name = "Narrow Aisle Warehouse"
    shelf_rows  = [2, 5, 8, 11, 14, 17, 20]
    aisle_col   = [7, 14, 21]   # only single-cell aisle gaps
    shelf_start, shelf_end = 3, w - 2

    for row in shelf_rows:
        if row >= h:
            continue
        for col in range(shelf_start, min(shelf_end+1, w)):
            if col not in aisle_col:
                wh.set_cell(col, row, OBSTACLE)

    # Pickup adjacent to shelves
    for row in shelf_rows:
        if row >= h:
            continue
        for col in range(shelf_start, min(shelf_end+1, w)):
            if wh.cell(col, row) != OBSTACLE:
                continue
            for dy in [-1, 1]:
                ny = row + dy
                if 0 <= ny < h and wh.cell(col, ny) == FREE:
                    wh.set_cell(col, ny, PICKUP)

    for y in range(0, h, 2):
        wh.set_cell(0, y, DROP)
    for y in range(0, h, 4):
        wh.set_cell(1, y, SPAWN)

    return wh


def _make_cross_shaped(w=25, h=25) -> Warehouse:
    """Cross-shaped main corridors, four shelf quadrants."""
    wh   = Warehouse(w, h)
    wh.name = "Cross-Corridor Warehouse"
    cx, cy  = w // 2, h // 2
    corridor = 2  # half-width of main corridor

    for y in range(h):
        for x in range(w):
            in_h_corridor = abs(y - cy) <= corridor
            in_v_corridor = abs(x - cx) <= corridor
            if not in_h_corridor and not in_v_corridor:
                wh.set_cell(x, y, OBSTACLE)

    # Carve pickup aisles inside quadrants
    for qx, qy in [(0,0),(cx+corridor+1,0),(0,cy+corridor+1),(cx+corridor+1,cy+corridor+1)]:
        for step_x in range(qx + 2, min(qx + cx - 1, w), 3):
            for step_y in range(qy + 1, min(qy + cy - 1, h)):
                if wh.cell(step_x, step_y) == OBSTACLE:
                    wh.set_cell(step_x, step_y, FREE)
                    # Pickup on adjacent cells
                    for dx in [-1, 1]:
                        nx = step_x + dx
                        if 0 <= nx < w and wh.cell(nx, step_y) == OBSTACLE:
                            wh.set_cell(nx, step_y, PICKUP)

    # Drop zones at perimeter
    for x in range(w):
        if wh.is_passable(x, 0):
            wh.set_cell(x, 0, DROP)
        if wh.is_passable(x, h-1):
            wh.set_cell(x, h-1, DROP)

    # Spawns at corridor entrances
    wh.set_cell(0,  cy, SPAWN)
    wh.set_cell(w-1, cy, SPAWN)
    wh.set_cell(cx,  0, SPAWN)
    wh.set_cell(cx,  h-1, SPAWN)

    return wh


def _make_pod_storage(w=24, h=24) -> Warehouse:
    """Scattered 2×2 shelf pods (Amazon-style)."""
    wh = Warehouse(w, h)
    wh.name = "Pod Storage Warehouse"

    pod_starts_x = range(3, w-3, 4)
    pod_starts_y = range(3, h-3, 4)

    for py in pod_starts_y:
        for px in pod_starts_x:
            # 2×2 pod
            for dy in range(2):
                for dx in range(2):
                    nx, ny = px+dx, py+dy
                    if 0 <= nx < w and 0 <= ny < h:
                        wh.set_cell(nx, ny, OBSTACLE)
            # Pickup points on all 4 sides of the pod
            for adj_x, adj_y in [
                (px-1, py), (px+2, py), (px-1, py+1), (px+2, py+1),
                (px, py-1), (px+1, py-1), (px, py+2), (px+1, py+2)
            ]:
                if 0 <= adj_x < w and 0 <= adj_y < h and wh.cell(adj_x, adj_y) == FREE:
                    wh.set_cell(adj_x, adj_y, PICKUP)

    for y in range(0, h, 3):
        if wh.is_passable(0, y):
            wh.set_cell(0, y, DROP)
    for y in range(0, h, 6):
        if wh.is_passable(1, y):
            wh.set_cell(1, y, SPAWN)

    return wh


def _make_open_floor(w=20, h=20) -> Warehouse:
    """Minimal obstacles — mostly open space, easy coordination."""
    wh = Warehouse(w, h)
    wh.name = "Open Floor Plan"

    # Just a few long shelves
    for col in range(4, 16):
        wh.set_cell(col, 5, OBSTACLE)
        wh.set_cell(col, 12, OBSTACLE)

    for col in range(4, 16):
        for row in [5, 12]:
            for dy in [-1, 1]:
                ny = row + dy
                if wh.cell(col, ny) == FREE:
                    wh.set_cell(col, ny, PICKUP)

    for y in range(0, h, 2):
        wh.set_cell(0, y, DROP)
    for y in range(1, h, 5):
        wh.set_cell(1, y, SPAWN)

    return wh


# ── registry ─────────────────────────────────────────────────────────────────

BUILTIN_LAYOUTS: Dict[str, callable] = {
    "standard":    _make_standard,
    "narrow_aisle": _make_narrow_aisle,
    "cross_shaped": _make_cross_shaped,
    "pod_storage":  _make_pod_storage,
    "open_floor":   _make_open_floor,
}


def get_layout(name: str) -> Warehouse:
    if name in BUILTIN_LAYOUTS:
        return BUILTIN_LAYOUTS[name]()
    # Try loading from file
    path = LAYOUTS_DIR / f"{name}.json"
    if path.exists():
        return Warehouse.load(str(path))
    raise ValueError(f"Unknown layout: {name}")


def list_layouts() -> List[Dict]:
    layouts = []
    for key, fn in BUILTIN_LAYOUTS.items():
        wh = fn()
        layouts.append({
            "id":     key,
            "name":   wh.name,
            "width":  wh.width,
            "height": wh.height,
            "builtin": True,
        })
    # Custom saved layouts
    if LAYOUTS_DIR.exists():
        for f in LAYOUTS_DIR.glob("*.json"):
            layouts.append({
                "id":     f.stem,
                "name":   f.stem.replace("_", " ").title(),
                "width":  0, "height": 0,
                "builtin": False,
            })
    return layouts


def save_layout(wh: Warehouse, name: str) -> str:
    LAYOUTS_DIR.mkdir(parents=True, exist_ok=True)
    path = LAYOUTS_DIR / f"{name}.json"
    wh.name = name
    wh.save(str(path))
    return str(path)
