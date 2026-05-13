"""Generate the maze occupancy map as a PNG + YAML pair.

Run once to produce maze_map.png and maze_map.yaml that are bundled
with the puzzlebot_control package and consumed by localization_node.py.

Usage::

    python3 -m puzzlebot_control.maze_builder

Output files are written to the directory where this script lives.
"""

import math
import os
import struct
import zlib
from dataclasses import dataclass, field
from typing import List, Tuple


# ---------------------------------------------------------------------------
# World & map configuration
# ---------------------------------------------------------------------------

# World bounding box (metres)
WORLD_X_MIN, WORLD_X_MAX = -5.54,  4.72
WORLD_Y_MIN, WORLD_Y_MAX = -8.10,  2.91

RESOLUTION = 0.05   # metres per pixel
FREE_PIXEL  = 255   # white → free
OCC_PIXEL   = 0     # black → occupied


@dataclass
class Box:
    """Axis-aligned or rotated rectangular obstacle.

    All values are in world metres.
    cx, cy   : centre
    hw_x     : half-extent along the *local* x axis (before rotation)
    hw_y     : half-extent along the *local* y axis (before rotation)
    angle    : counter-clockwise rotation in radians
    """
    cx:    float
    cy:    float
    hw_x:  float
    hw_y:  float
    angle: float = 0.0


# ---------------------------------------------------------------------------
# Maze geometry
# ---------------------------------------------------------------------------

WALL_T = 0.10   # wall half-thickness (metres)

OBSTACLES: List[Box] = [
    # --- outer boundary walls ---
    Box( -0.41,  2.66, 5.13, WALL_T,  0.0),    # north
    Box( -0.41, -7.85, 5.13, WALL_T,  0.0),    # south
    Box( -5.29, -2.60, WALL_T, 5.25,  0.0),    # west
    Box(  4.47, -2.60, WALL_T, 5.25,  0.0),    # east
    Box( -0.41,  2.66, 5.13, WALL_T, math.pi), # north duplicate (SDF mirroring)
    Box(  1.05, -2.60, WALL_T, 2.60,  0.0),    # inner vertical divider
    Box( -1.87, -2.60, WALL_T, 2.60,  0.0),    # inner vertical divider 2

    # --- interior 1 × 1 m boxes ---
    Box(-4.04,  1.41, 0.50, 0.50, 0.0),
    Box(-1.54,  1.41, 0.50, 0.50, 0.0),
    Box( 0.96,  1.41, 0.50, 0.50, 0.0),
    Box( 3.46,  1.41, 0.50, 0.50, 0.0),
    Box(-4.04, -1.09, 0.50, 0.50, 0.0),
    Box(-1.54, -1.09, 0.50, 0.50, 0.0),
    Box( 0.96, -1.09, 0.50, 0.50, 0.0),
    Box( 3.46, -1.09, 0.50, 0.50, 0.0),
]


# ---------------------------------------------------------------------------
# MazeBuilder
# ---------------------------------------------------------------------------

class MazeBuilder:
    """Rasterise the obstacle list into a binary occupancy image."""

    def __init__(
        self,
        x_min: float = WORLD_X_MIN,
        x_max: float = WORLD_X_MAX,
        y_min: float = WORLD_Y_MIN,
        y_max: float = WORLD_Y_MAX,
        resolution: float = RESOLUTION,
        obstacles: List[Box] = None,
    ) -> None:
        self.x_min      = x_min
        self.x_max      = x_max
        self.y_min      = y_min
        self.y_max      = y_max
        self.resolution = resolution
        self.obstacles  = obstacles if obstacles is not None else OBSTACLES

        self.width  = math.ceil((x_max - x_min) / resolution)
        self.height = math.ceil((y_max - y_min) / resolution)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def build(self) -> List[List[int]]:
        """Return a height × width pixel grid (row 0 = top of image)."""
        grid = [[FREE_PIXEL] * self.width for _ in range(self.height)]
        for row in range(self.height):
            for col in range(self.width):
                wx, wy = self._pixel_to_world(col, row)
                if any(self._inside_box(wx, wy, obs) for obs in self.obstacles):
                    grid[row][col] = OCC_PIXEL
        return grid

    def save(self, output_dir: str | None = None) -> Tuple[str, str]:
        """Build the grid and write maze_map.png + maze_map.yaml.

        Returns the absolute paths of the two files written.
        """
        if output_dir is None:
            output_dir = os.path.dirname(os.path.abspath(__file__))

        png_path  = os.path.join(output_dir, 'maze_map.png')
        yaml_path = os.path.join(output_dir, 'maze_map.yaml')

        grid = self.build()
        _write_png(grid, self.width, self.height, png_path)
        _write_yaml(yaml_path, png_path, self.resolution, self.x_min, self.y_min)

        return png_path, yaml_path

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    def _pixel_to_world(self, col: int, row: int) -> Tuple[float, float]:
        wx = self.x_min + (col + 0.5) * self.resolution
        # row 0 is the top of the image → highest y value
        wy = self.y_max - (row + 0.5) * self.resolution
        return wx, wy

    @staticmethod
    def _inside_box(wx: float, wy: float, box: Box) -> bool:
        """Return True when (wx, wy) is inside the (possibly rotated) box."""
        # Translate to box centre
        dx = wx - box.cx
        dy = wy - box.cy
        # Rotate into box-local frame (inverse rotation = negative angle)
        cos_a =  math.cos(box.angle)
        sin_a =  math.sin(box.angle)
        lx =  dx * cos_a + dy * sin_a
        ly = -dx * sin_a + dy * cos_a
        return abs(lx) <= box.hw_x and abs(ly) <= box.hw_y


# ---------------------------------------------------------------------------
# Minimal PNG + YAML writers (no external dependencies)
# ---------------------------------------------------------------------------

def _write_png(grid: List[List[int]], width: int, height: int, path: str) -> None:
    """Write a grayscale 8-bit PNG with filter type 0 (None) per row."""
    def chunk(tag: bytes, data: bytes) -> bytes:
        c = tag + data
        return struct.pack('>I', len(data)) + c + struct.pack('>I', zlib.crc32(c) & 0xFFFFFFFF)

    ihdr_data = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    raw_rows  = b''.join(b'\x00' + bytes(row) for row in grid)
    idat_data = zlib.compress(raw_rows, level=9)

    png = (
        b'\x89PNG\r\n\x1a\n'
        + chunk(b'IHDR', ihdr_data)
        + chunk(b'IDAT', idat_data)
        + chunk(b'IEND', b'')
    )
    with open(path, 'wb') as fh:
        fh.write(png)


def _write_yaml(yaml_path: str, png_path: str,
                resolution: float, origin_x: float, origin_y: float) -> None:
    content = (
        f"image: {os.path.basename(png_path)}\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin_x}, {origin_y}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    with open(yaml_path, 'w') as fh:
        fh.write(content)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    builder = MazeBuilder()
    png, yaml = builder.save()
    print(f'Written: {png}')
    print(f'Written: {yaml}')
    print(f'Map size: {builder.width} × {builder.height} px  '
          f'({builder.width * RESOLUTION:.1f} × {builder.height * RESOLUTION:.1f} m)')


if __name__ == '__main__':
    main()
