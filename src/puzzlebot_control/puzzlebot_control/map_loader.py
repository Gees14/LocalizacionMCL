"""Occupancy map: PNG loading, free-space queries and ray casting."""

import math
import struct
import zlib
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class MapConfig:
    """Geometric parameters that relate pixel coordinates to world coordinates."""
    resolution: float   # metres per pixel
    origin_x:   float   # world-x of the bottom-left corner  (pixel col=0, row=height-1)
    origin_y:   float   # world-y of the bottom-left corner


class OccupancyMap:
    """Binary occupancy map loaded from an 8-bit grayscale PNG.

    Convention
    ----------
    pixel >= 128  →  free  (white)
    pixel <  128  →  occupied (black / grey)

    The PNG row order (top = row 0) is inverted relative to the ROS
    OccupancyGrid convention (bottom = row 0), so all coordinate
    conversions flip the row index.
    """

    def __init__(self, png_path: str, cfg: MapConfig) -> None:
        self.cfg = cfg
        self._grid, self.width, self.height = _load_png(png_path)

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    def world_to_pixel(self, wx: float, wy: float) -> Tuple[int, int]:
        col = int((wx - self.cfg.origin_x) / self.cfg.resolution)
        row = self.height - 1 - int((wy - self.cfg.origin_y) / self.cfg.resolution)
        return col, row

    def pixel_to_world(self, col: int, row: int) -> Tuple[float, float]:
        wx = self.cfg.origin_x + (col + 0.5) * self.cfg.resolution
        wy = self.cfg.origin_y + (self.height - 1 - row + 0.5) * self.cfg.resolution
        return wx, wy

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def is_free(self, wx: float, wy: float) -> bool:
        """Return True when world point (wx, wy) is inside a free cell."""
        col, row = self.world_to_pixel(wx, wy)
        if not (0 <= col < self.width and 0 <= row < self.height):
            return False
        return self._grid[row][col] >= 128

    def ray_cast(
        self,
        wx: float,
        wy: float,
        angle: float,
        max_range: float = 10.0,
        step: float = 0.025,
    ) -> float:
        """March a ray from (wx, wy) along *angle* until hitting a wall.

        Returns the travel distance in metres, capped at *max_range*.
        """
        dx = math.cos(angle) * step
        dy = math.sin(angle) * step
        cx, cy = wx, wy
        dist = 0.0
        while dist < max_range:
            cx += dx
            cy += dy
            dist += step
            col, row = self.world_to_pixel(cx, cy)
            if not (0 <= col < self.width and 0 <= row < self.height):
                return dist
            if self._grid[row][col] < 128:
                return dist
        return max_range

    def free_cells(self) -> List[Tuple[float, float]]:
        """Return world (x, y) centre of every free cell — used for particle seeding."""
        cells: List[Tuple[float, float]] = []
        for row in range(self.height):
            for col in range(self.width):
                if self._grid[row][col] >= 128:
                    cells.append(self.pixel_to_world(col, row))
        return cells

    def to_occupancy_grid_data(self) -> List[int]:
        """Flat row-major list in nav_msgs/OccupancyGrid format.

        OccupancyGrid stores rows from bottom to top, values: 0=free, 100=occupied.
        """
        data: List[int] = []
        for row in reversed(self._grid):
            for pixel in row:
                data.append(0 if pixel >= 128 else 100)
        return data


# ---------------------------------------------------------------------------
# Internal PNG decoder (no Pillow / OpenCV dependency)
# ---------------------------------------------------------------------------

def _load_png(path: str):
    """Decode an 8-bit grayscale PNG into a list-of-lists pixel grid.

    Returns (grid, width, height).  grid[row][col] is a uint8 value.
    Only filter type 0 (None) is handled — sufficient for maps written
    by maze_builder.py.
    """
    with open(path, 'rb') as fh:
        raw = fh.read()

    if raw[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError(f"Not a PNG file: {path}")

    # Collect chunk data by type (first occurrence wins)
    idat_blocks: List[bytes] = []
    chunks: dict = {}
    pos = 8
    while pos < len(raw):
        length = struct.unpack('>I', raw[pos:pos + 4])[0]
        ctype  = raw[pos + 4:pos + 8]
        data   = raw[pos + 8:pos + 8 + length]
        if ctype == b'IDAT':
            idat_blocks.append(data)
        elif ctype not in chunks:
            chunks[ctype] = data
        pos += 12 + length

    ihdr = chunks[b'IHDR']
    width, height = struct.unpack('>II', ihdr[:8])
    bit_depth, color_type = ihdr[8], ihdr[9]

    if bit_depth != 8 or color_type != 0:
        raise ValueError(
            f"Only 8-bit grayscale PNGs are supported "
            f"(got bit_depth={bit_depth}, color_type={color_type})"
        )

    raw_pixels = zlib.decompress(b''.join(idat_blocks))
    stride = width + 1   # 1 filter-type byte per row

    grid: List[List[int]] = []
    for r in range(height):
        row_bytes = raw_pixels[r * stride + 1: (r + 1) * stride]
        grid.append(list(row_bytes))

    return grid, width, height
