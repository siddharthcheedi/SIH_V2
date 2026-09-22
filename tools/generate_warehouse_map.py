#!/usr/bin/env python3
"""
Generates maps/warehouse_map.pgm + maps/warehouse_map.yaml for Nav2/AMCL.

The wall/shelf geometry is copy-matched to worlds/warehouse.world — one
source of truth so the static map and the simulated world can't silently
drift apart. If you change the world file, change the geometry constants
here and re-run this script.

Usage:
    python3 tools/generate_warehouse_map.py
"""

from pathlib import Path

try:
    from PIL import Image, ImageDraw
except ImportError:
    raise SystemExit(
        'Pillow is required: pip install Pillow'
    )

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT_DIR = REPO_ROOT / 'maps'

RESOLUTION = 0.05  # meters/pixel
MAP_MIN_X, MAP_MAX_X = -10.5, 10.5
MAP_MIN_Y, MAP_MAX_Y = -7.5, 7.5

FREE = 255
OCCUPIED = 0
UNKNOWN = 205

# (center_x, center_y, size_x, size_y) — matches worlds/warehouse.world
WALLS = [
    (0, 7, 20.4, 0.2),    # wall_north
    (0, -7, 20.4, 0.2),   # wall_south
    (10, 0, 0.2, 14.4),   # wall_east
    (-10, 0, 0.2, 14.4),  # wall_west
]
SHELVES = [
    (-6, -1, 1.0, 8.0),   # shelf_1
    (-2, -1, 1.0, 8.0),   # shelf_2
    (2, -1, 1.0, 8.0),    # shelf_3
    (6, -1, 1.0, 8.0),    # shelf_4
]


def world_to_px(x: float, y: float) -> tuple:
    """Convert world coordinates to pixel coordinates."""
    col = (x - MAP_MIN_X) / RESOLUTION
    row = (MAP_MAX_Y - y) / RESOLUTION
    return col, row


def draw_box(draw: ImageDraw.Draw, cx: float, cy: float,
             sx: float, sy: float, fill: int) -> None:
    """Draw a filled rectangle from world-frame center+size."""
    x0, y0 = world_to_px(cx - sx / 2, cy + sy / 2)
    x1, y1 = world_to_px(cx + sx / 2, cy - sy / 2)
    draw.rectangle([x0, y0, x1, y1], fill=fill)


def main():
    width_px = round((MAP_MAX_X - MAP_MIN_X) / RESOLUTION)
    height_px = round((MAP_MAX_Y - MAP_MIN_Y) / RESOLUTION)

    img = Image.new('L', (width_px, height_px), color=UNKNOWN)
    draw = ImageDraw.Draw(img)

    # Interior of the building footprint is free space...
    interior_x0, interior_y0 = world_to_px(-10, 7)
    interior_x1, interior_y1 = world_to_px(10, -7)
    draw.rectangle([interior_x0, interior_y0, interior_x1, interior_y1],
                   fill=FREE)

    # ...minus the walls and shelves, which are occupied.
    for cx, cy, sx, sy in WALLS:
        draw_box(draw, cx, cy, sx, sy, OCCUPIED)
    for cx, cy, sx, sy in SHELVES:
        draw_box(draw, cx, cy, sx, sy, OCCUPIED)

    OUT_DIR.mkdir(exist_ok=True)
    pgm_path = OUT_DIR / 'warehouse_map.pgm'
    img.save(pgm_path)

    yaml_path = OUT_DIR / 'warehouse_map.yaml'
    yaml_path.write_text(
        "image: warehouse_map.pgm\n"
        f"resolution: {RESOLUTION}\n"
        f"origin: [{MAP_MIN_X}, {MAP_MIN_Y}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    print(f"wrote {pgm_path} ({width_px}×{height_px}px) and {yaml_path}")


if __name__ == '__main__':
    main()
