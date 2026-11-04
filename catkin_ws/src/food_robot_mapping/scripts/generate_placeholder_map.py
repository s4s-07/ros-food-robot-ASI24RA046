#!/usr/bin/env python3
"""
generate_placeholder_map.py
============================
Generates a valid binary PGM (P5) cafeteria map for use with ROS map_server.

Map specs:
  Width  = 480 px  (24 m  at 0.05 m/px)
  Height = 400 px  (20 m  at 0.05 m/px)
  Origin = (-12, -10, 0)  matching cafeteria.yaml

Pixel values:
  205 = free space (ROS convention)
    0 = occupied / wall

Run:
  python3 generate_placeholder_map.py
  (output saved to the same directory as this script → maps/cafeteria.pgm)
"""

import os
import struct

# ── Map dimensions ───────────────────────────────────────────────────────────
WIDTH   = 480   # pixels  (24 m at 0.05 m/px)
HEIGHT  = 400   # pixels  (20 m at 0.05 m/px)
FREE    = 205   # ROS free-space value
WALL    = 0     # occupied
WALL_T  = 3     # wall thickness in pixels (~15 cm)

# ── World-to-pixel conversion ────────────────────────────────────────────────
# origin = (-12, -10)  → pixel (0, HEIGHT-1) is world (-12, -10)
def w2p(wx, wy):
    """Convert world coords (m) to pixel (col, row). Row 0 = top of image."""
    col = int((wx + 12.0) / 0.05)
    row = int(HEIGHT - 1 - (wy + 10.0) / 0.05)
    return col, row


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def fill_rect(grid, col0, row0, col1, row1, value):
    for r in range(clamp(row0, 0, HEIGHT - 1), clamp(row1 + 1, 0, HEIGHT)):
        for c in range(clamp(col0, 0, WIDTH - 1), clamp(col1 + 1, 0, WIDTH)):
            grid[r][c] = value


def draw_hwall(grid, wx0, wx1, wy, value=WALL):
    """Horizontal wall segment."""
    c0, r0 = w2p(wx0, wy + WALL_T * 0.05 / 2)
    c1, r1 = w2p(wx1, wy - WALL_T * 0.05 / 2)
    fill_rect(grid, min(c0, c1), min(r0, r1), max(c0, c1), max(r0, r1), value)


def draw_vwall(grid, wx, wy0, wy1, value=WALL):
    """Vertical wall segment."""
    c0, r0 = w2p(wx - WALL_T * 0.05 / 2, wy0)
    c1, r1 = w2p(wx + WALL_T * 0.05 / 2, wy1)
    fill_rect(grid, min(c0, c1), min(r0, r1), max(c0, c1), max(r0, r1), value)


def draw_box(grid, wx0, wy0, wx1, wy1, value=WALL):
    """Filled rectangle (for furniture obstacles)."""
    c0, r0 = w2p(wx0, wy1)   # wy1 > wy0, maps to smaller row (top)
    c1, r1 = w2p(wx1, wy0)
    fill_rect(grid, min(c0, c1), min(r0, r1), max(c0, c1), max(r0, r1), value)


# ── Build grid ───────────────────────────────────────────────────────────────
grid = [[FREE] * WIDTH for _ in range(HEIGHT)]

# Outer cafeteria walls  (room: x ∈ [-10, 10],  y ∈ [-7.5, 7.5])
draw_hwall(grid, -10.0,  10.0,  7.5)   # north wall
draw_hwall(grid, -10.0,  10.0, -7.5)   # south wall
draw_vwall(grid,  10.0,  -7.5,  7.5)   # east wall
draw_vwall(grid, -10.0,  -7.5,  7.5)   # west wall

# Tables (1.2 x 0.8 m each)
tables = [
    (3.0, 2.0), (3.0, 4.5), (3.0, 7.0),
    (7.0, 2.0), (7.0, 4.5), (7.0, 7.0),
]
for tx, ty in tables:
    draw_box(grid, tx - 0.6, ty - 0.4, tx + 0.6, ty + 0.4)

# Kitchen counter (0.6 x 5.0 m at x=9, y=0)
draw_box(grid, 8.7, -2.5, 9.3, 2.5)

# Front desk (2.0 x 0.6 m at x=-7, y=-6.5)
draw_box(grid, -8.0, -6.8, -6.0, -6.2)

# ── Write PGM P5 (binary) ────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
maps_dir   = os.path.join(script_dir, '..', 'maps')
os.makedirs(maps_dir, exist_ok=True)
out_path   = os.path.join(maps_dir, 'cafeteria.pgm')

with open(out_path, 'wb') as f:
    # PGM header
    header = f"P5\n{WIDTH} {HEIGHT}\n255\n"
    f.write(header.encode('ascii'))
    # Pixel data (row-major, top-to-bottom)
    for row in grid:
        f.write(bytes(row))

print(f"Map saved to: {out_path}")
print(f"  Size: {WIDTH} x {HEIGHT} px  ({WIDTH*0.05:.1f} x {HEIGHT*0.05:.1f} m)")
print("  Run with: rosrun map_server map_server cafeteria.yaml")
