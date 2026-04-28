"""
One-shot script: renders the maze world geometry into a grayscale PNG map.

Run from anywhere:
    python3 generate_map.py

Output:
    maze_map.png  — written next to this script
    maze_map.yaml — ROS-style map metadata (for reference / nav_msgs/OccupancyGrid)

Map convention (matching MCL node):
    255 = free space (white)
      0 = obstacle   (black)

World bounds (from maze.sdf, world-absolute coordinates):
    x: -5.535 to  4.715   (room width  ~10.25 m + 0.075 m half-wall on each side)
    y: -8.095 to  2.905   (room height ~10.85 m + 0.075 m half-wall on each side)

Resolution: 0.05 m/pixel  →  image ~204 x 220 pixels
"""

import math
import os
import struct
import zlib


RESOLUTION = 0.05   # metres per pixel

# World bounds — slightly outside the outer walls so the wall thickness is captured
WORLD_X_MIN = -5.54
WORLD_X_MAX =  4.72
WORLD_Y_MIN = -8.10
WORLD_Y_MAX =  2.91

# ── Obstacle definitions ──────────────────────────────────────────────────────
# Each wall/box is (cx, cy, half_width_x, half_width_y, angle_rad)
# angle_rad rotates the LOCAL x-axis of the obstacle

def _rotated_box(cx, cy, hx, hy, angle):
    """Return (cx, cy, hx, hy, angle) — kept as a dict for clarity."""
    return {"cx": cx, "cy": cy, "hx": hx, "hy": hy, "a": angle}


OBSTACLES = [
    # ── Outer walls (from SDF state poses, thickness = 0.075 on each side) ──
    # Wall_0: south-left vertical segment, len=3.75, rot=-90° → local x is world-y
    _rotated_box(-5.46, -6.22,  3.75/2, 0.15/2, -math.pi/2),
    # Wall_1: bottom horizontal, len=10.25
    _rotated_box(-0.41, -8.02, 10.25/2, 0.15/2,  0.0),
    # Wall_2: right vertical, len=11, rot=90°
    _rotated_box( 4.64, -2.595, 11.0/2, 0.15/2,  math.pi/2),
    # Wall_4: left lower gap vertical, len=2, rot=90°
    _rotated_box(-5.46, -3.495,  2.0/2, 0.15/2,  math.pi/2),
    # Wall_5: left upper vertical, len=5, rot=90°
    _rotated_box(-5.46, -0.145,  5.0/2, 0.15/2,  math.pi/2),
    # Wall_8: top horizontal, len=10.25
    _rotated_box(-0.41,  2.83,  10.25/2, 0.15/2,  math.pi),
    # Wall_9: top-left short vertical, len=0.7, rot=-90°
    _rotated_box(-5.46,  2.555,  0.7/2, 0.15/2, -math.pi/2),

    # ── Interior boxes (1×1 m, axis-aligned) ─────────────────────────────────
    _rotated_box(-2.599,  0.229, 0.5, 0.5, 0.0),
    _rotated_box( 1.091, -2.438, 0.5, 0.5, 0.0),
    _rotated_box(-1.383, -3.322, 0.5, 0.5, 0.0),
    _rotated_box(-3.371, -6.146, 0.5, 0.5, 0.0),
    _rotated_box( 2.467,  0.506, 0.5, 0.5, 0.0),
    _rotated_box( 2.364, -5.329, 0.5, 0.5, 0.0),
    _rotated_box(-4.104, -1.578, 0.5, 0.5, 0.0),
    _rotated_box( 0.208, -7.051, 0.5, 0.5, 0.0),
]


def _pixel_in_obstacle(px_world, py_world, obs):
    """True if world point (px_world, py_world) is inside this obstacle."""
    dx = px_world - obs["cx"]
    dy = py_world - obs["cy"]
    cos_a = math.cos(-obs["a"])
    sin_a = math.sin(-obs["a"])
    lx = dx * cos_a - dy * sin_a
    ly = dx * sin_a + dy * cos_a
    return abs(lx) <= obs["hx"] and abs(ly) <= obs["hy"]


def world_to_pixel(wx, wy, img_height):
    """Convert world (x,y) → pixel (col, row). Y is flipped (image row 0 = max world y)."""
    col = int((wx - WORLD_X_MIN) / RESOLUTION)
    row = img_height - 1 - int((wy - WORLD_Y_MIN) / RESOLUTION)
    return col, row


def pixel_to_world(col, row, img_height):
    """Convert pixel (col, row) → world (x, y) centre of pixel."""
    wx = WORLD_X_MIN + (col + 0.5) * RESOLUTION
    wy = WORLD_Y_MIN + (img_height - 1 - row + 0.5) * RESOLUTION
    return wx, wy


def render():
    width  = int(math.ceil((WORLD_X_MAX - WORLD_X_MIN) / RESOLUTION))
    height = int(math.ceil((WORLD_Y_MAX - WORLD_Y_MIN) / RESOLUTION))

    pixels = []
    for row in range(height):
        for col in range(width):
            wx, wy = pixel_to_world(col, row, height)
            occupied = any(_pixel_in_obstacle(wx, wy, o) for o in OBSTACLES)
            pixels.append(0 if occupied else 255)

    return width, height, bytes(pixels)


# ── Minimal PNG writer (no external deps) ────────────────────────────────────

def _png_chunk(chunk_type, data):
    c = chunk_type + data
    return (
        struct.pack(">I", len(data))
        + c
        + struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)
    )


def write_png(path, width, height, grayscale_bytes):
    signature = b"\x89PNG\r\n\x1a\n"

    # IHDR
    ihdr_data = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    ihdr = _png_chunk(b"IHDR", ihdr_data)

    # IDAT — filter byte 0 (None) prepended to each scanline
    raw_rows = b""
    for r in range(height):
        raw_rows += b"\x00" + grayscale_bytes[r * width: (r + 1) * width]
    idat = _png_chunk(b"IDAT", zlib.compress(raw_rows, 9))

    iend = _png_chunk(b"IEND", b"")

    with open(path, "wb") as f:
        f.write(signature + ihdr + idat + iend)


def write_yaml(yaml_path, png_path, width, height):
    with open(yaml_path, "w") as f:
        f.write(f"image: {os.path.basename(png_path)}\n")
        f.write(f"resolution: {RESOLUTION}\n")
        f.write(f"origin: [{WORLD_X_MIN}, {WORLD_Y_MIN}, 0.0]\n")
        f.write(f"width: {width}\n")
        f.write(f"height: {height}\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    png_path  = os.path.join(script_dir, "maze_map.png")
    yaml_path = os.path.join(script_dir, "maze_map.yaml")

    print("Rendering map …")
    w, h, data = render()
    write_png(png_path, w, h, data)
    write_yaml(yaml_path, png_path, w, h)
    print(f"Saved {w}×{h} px map → {png_path}")
    print(f"Saved metadata      → {yaml_path}")
    print(f"Resolution: {RESOLUTION} m/px | "
          f"World: x[{WORLD_X_MIN}, {WORLD_X_MAX}] y[{WORLD_Y_MIN}, {WORLD_Y_MAX}]")
