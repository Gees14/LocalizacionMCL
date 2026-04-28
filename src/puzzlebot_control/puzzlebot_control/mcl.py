"""
Monte Carlo Localisation (MCL) node — Activity 3.4

Algorithm (matches assignment steps D–I):
  D. Sample N particles (x, y, θ) uniformly in free space.
  E. Score each particle using the laser scan against the map bitmap.
  F. Keep the top-K particles by score (filter step).
  G. Estimate robot motion (dead reckoning) from consecutive /odom messages.
  H. Move all surviving particles by Δpose + Gaussian noise.
  I. Repeat from D (resample to N particles by duplicating top-K with noise).

Inputs
------
  /scan          sensor_msgs/LaserScan   — laser rangefinder
  /odom          nav_msgs/Odometry       — dead-reckoning odometry

Outputs
-------
  /mcl/particles geometry_msgs/PoseArray    — all current particles (RViz)
  /mcl/pose      geometry_msgs/PoseStamped — weighted mean best estimate

Parameters
----------
  map_path        str    path to the grayscale PNG map
  map_resolution  float  metres per pixel (default 0.05)
  map_origin_x    float  world x at pixel col=0 (default -5.54)
  map_origin_y    float  world y at pixel row=height-1 (default -8.10)
  num_particles   int    total particles N (default 500)
  top_k           int    survivors after filter step (default 150)
  noise_xy        float  std-dev of position noise per step [m] (default 0.05)
  noise_theta     float  std-dev of heading noise per step [rad] (default 0.05)
  score_rays      int    laser rays sampled per particle for scoring (default 36)
  ray_step        float  ray-marching step size [m] (default 0.025)
  hit_sigma       float  scan likelihood sigma [m] (default 0.20)
  map_frame       str    global map frame (default "map")
  odom_frame      str    local odometry frame (default "odom")
  laser_offset_x  float  lidar x offset from base frame [m] (default 0.0)
  laser_offset_y  float  lidar y offset from base frame [m] (default 0.0)
"""

import math
import os
import random
import struct
import zlib

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros


# ── Minimal PNG loader (no external deps) ────────────────────────────────────

def _load_png_grayscale(path):
    """Return (width, height, flat bytearray) for an 8-bit grayscale PNG."""
    with open(path, "rb") as f:
        raw = f.read()

    assert raw[:8] == b"\x89PNG\r\n\x1a\n", "Not a PNG file"

    idx = 8
    width = height = bit_depth = color_type = None
    idat_chunks = []

    while idx < len(raw):
        length = struct.unpack(">I", raw[idx:idx+4])[0]
        ctype  = raw[idx+4:idx+8]
        data   = raw[idx+8:idx+8+length]
        idx   += 12 + length

        if ctype == b"IHDR":
            width, height = struct.unpack(">II", data[:8])
            bit_depth, color_type = data[8], data[9]
        elif ctype == b"IDAT":
            idat_chunks.append(data)
        elif ctype == b"IEND":
            break

    assert bit_depth == 8 and color_type == 0, \
        f"Need 8-bit grayscale PNG (got bit_depth={bit_depth}, color_type={color_type})"

    raw_data = zlib.decompress(b"".join(idat_chunks))
    pixels   = bytearray(width * height)
    stride   = width + 1  # one filter byte per row

    for row in range(height):
        # filter byte is raw_data[row * stride]; only filter=0 (None) is used
        # by generate_map.py, so we don't need a full PNG filter decoder.
        base_in  = row * stride + 1
        base_out = row * width
        pixels[base_out:base_out + width] = raw_data[base_in:base_in + width]

    return width, height, pixels


# ── Geometry helpers ──────────────────────────────────────────────────────────

def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _rotate_2d(x: float, y: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        x * cos_yaw - y * sin_yaw,
        x * sin_yaw + y * cos_yaw,
    )


# ── MCL node ─────────────────────────────────────────────────────────────────

class MCLNode(Node):

    def __init__(self):
        super().__init__("mcl")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("map_path",
            os.path.join(os.path.dirname(__file__), "maze_map.png"))
        self.declare_parameter("map_resolution", 0.05)
        self.declare_parameter("map_origin_x",  -5.54)
        self.declare_parameter("map_origin_y",  -8.10)
        self.declare_parameter("num_particles",  500)
        self.declare_parameter("top_k",          150)
        self.declare_parameter("noise_xy",        0.05)
        self.declare_parameter("noise_theta",     0.05)
        self.declare_parameter("score_rays",      36)
        self.declare_parameter("ray_step",         0.025)
        self.declare_parameter("hit_sigma",        0.20)
        self.declare_parameter("map_frame",       "map")
        self.declare_parameter("odom_frame",      "odom")
        self.declare_parameter("laser_offset_x",   0.0)
        self.declare_parameter("laser_offset_y",   0.0)

        self._res      = self.get_parameter("map_resolution").value
        self._orig_x   = self.get_parameter("map_origin_x").value
        self._orig_y   = self.get_parameter("map_origin_y").value
        self._n        = self.get_parameter("num_particles").value
        self._k        = self.get_parameter("top_k").value
        self._noise_xy = self.get_parameter("noise_xy").value
        self._noise_th = self.get_parameter("noise_theta").value
        self._n_rays   = self.get_parameter("score_rays").value
        self._ray_step = self.get_parameter("ray_step").value
        self._hit_sigma = self.get_parameter("hit_sigma").value
        self._map_frame = self.get_parameter("map_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._laser_offset_x = self.get_parameter("laser_offset_x").value
        self._laser_offset_y = self.get_parameter("laser_offset_y").value

        # ── Load map ──────────────────────────────────────────────────────
        map_path = self.get_parameter("map_path").value
        self._map_w, self._map_h, self._map = _load_png_grayscale(map_path)
        self.get_logger().info(
            f"MCL: loaded map {self._map_w}×{self._map_h} px "
            f"from {map_path}"
        )

        # Pre-build list of free pixel world-centres for uniform sampling
        self._free_cells = []
        for row in range(self._map_h):
            for col in range(self._map_w):
                if self._map[row * self._map_w + col] > 127:
                    wx = self._orig_x + (col + 0.5) * self._res
                    wy = self._orig_y + (self._map_h - 1 - row + 0.5) * self._res
                    self._free_cells.append((wx, wy))

        # ── State ─────────────────────────────────────────────────────────
        self._particles: list[list[float]] = []   # each: [x, y, theta]
        self._prev_odom: Odometry | None   = None

        # Initialise with uniform sample (step D)
        self._initialise_particles()

        # ── Publishers ────────────────────────────────────────────────────
        self._pub_array = self.create_publisher(PoseArray,     "/mcl/particles", 10)
        self._pub_pose  = self.create_publisher(PoseStamped,   "/mcl/pose",      10)
        self._pub_map   = self.create_publisher(OccupancyGrid, "/mcl/map",
                                                rclpy.qos.QoSProfile(
                                                    depth=1,
                                                    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                                    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                                ))
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Re-publish map every 3 s for the first 30 s so RViz always gets it
        self._map_timer_count = 0
        self._map_timer = self.create_timer(3.0, self._map_timer_cb)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.create_subscription(Odometry,  "/odom", self._odom_cb, 10)

        self.get_logger().info(
            f"MCL: N={self._n} particles, top_k={self._k}, "
            f"{len(self._free_cells)} free cells available"
        )

    # ── Map publisher ─────────────────────────────────────────────────────

    def _map_timer_cb(self):
        self._publish_map()
        self._map_timer_count += 1
        if self._map_timer_count >= 10:   # stop after 30 s
            self._map_timer.cancel()

    def _publish_map(self):
        """Publish the PNG map as a latched OccupancyGrid for RViz."""
        grid = OccupancyGrid()
        # stamp 0 = always valid in RViz regardless of sim/wall time
        grid.header.stamp.sec  = 0
        grid.header.stamp.nanosec = 0
        grid.header.frame_id = self._map_frame

        grid.info.resolution = self._res
        grid.info.width      = self._map_w
        grid.info.height     = self._map_h
        grid.info.origin.position.x = self._orig_x
        grid.info.origin.position.y = self._orig_y
        grid.info.origin.orientation.w = 1.0

        # OccupancyGrid: 0=free, 100=occupied, -1=unknown
        # PNG: 255=free (white), 0=occupied (black)
        data = []
        for row in range(self._map_h):
            # OccupancyGrid row 0 = bottom of map (y_min); PNG row 0 = top
            png_row = self._map_h - 1 - row
            for col in range(self._map_w):
                pixel = self._map[png_row * self._map_w + col]
                data.append(0 if pixel > 127 else 100)

        grid.data = data
        self._pub_map.publish(grid)

    # ── D. Particle initialisation / resampling ───────────────────────────

    def _initialise_particles(self):
        """Scatter N particles uniformly across free space."""
        self._particles = []
        sample = random.choices(self._free_cells, k=self._n)
        for (wx, wy) in sample:
            theta = random.uniform(-math.pi, math.pi)
            self._particles.append([wx, wy, theta])

    def _resample(self, survivors: list[list[float]]):
        """Repopulate to N by duplicating survivors with small noise (step I)."""
        new_particles = list(survivors)  # keep exact survivors
        while len(new_particles) < self._n:
            base = random.choice(survivors)
            new_particles.append([
                base[0] + random.gauss(0.0, self._noise_xy),
                base[1] + random.gauss(0.0, self._noise_xy),
                _wrap(base[2] + random.gauss(0.0, self._noise_th)),
            ])
        self._particles = new_particles

    # ── Map helpers ───────────────────────────────────────────────────────

    def _world_to_pixel(self, wx: float, wy: float):
        col = int((wx - self._orig_x) / self._res)
        row = self._map_h - 1 - int((wy - self._orig_y) / self._res)
        return col, row

    def _map_value(self, wx: float, wy: float) -> int:
        """Return pixel value [0,255] at world point, or 0 if out of bounds."""
        col, row = self._world_to_pixel(wx, wy)
        if 0 <= col < self._map_w and 0 <= row < self._map_h:
            return self._map[row * self._map_w + col]
        return 0

    def _expected_range(self, sx: float, sy: float, angle: float,
                        range_min: float, range_max: float) -> float:
        """
        March a synthetic ray through the occupancy map until it hits an
        occupied cell. If nothing is hit, fall back to range_max.
        """
        distance = max(range_min, self._ray_step)
        while distance <= range_max:
            wx = sx + distance * math.cos(angle)
            wy = sy + distance * math.sin(angle)
            if self._map_value(wx, wy) <= 127:
                return distance
            distance += self._ray_step
        return range_max

    # ── E. Scoring ────────────────────────────────────────────────────────

    def _score_particle(self, px: float, py: float, pth: float,
                        scan: LaserScan) -> float:
        """
        Score = sum of Gaussian likelihoods comparing measured ranges to
        expected map ranges from the particle pose.
        """
        total_rays = len(scan.ranges)
        if total_rays == 0:
            return 0.0

        step = max(1, total_rays // self._n_rays)
        score = 0.0
        sensor_dx, sensor_dy = _rotate_2d(
            self._laser_offset_x, self._laser_offset_y, pth
        )
        sensor_x = px + sensor_dx
        sensor_y = py + sensor_dy

        for i in range(0, total_rays, step):
            r = scan.ranges[i]
            if not math.isfinite(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment + pth
            expected = self._expected_range(
                sensor_x, sensor_y, angle, scan.range_min, scan.range_max
            )
            error = r - expected
            score += math.exp(
                -0.5 * (error / self._hit_sigma) * (error / self._hit_sigma)
            )

        return score

    # ── G. Dead reckoning delta from /odom ───────────────────────────────

    def _compute_delta(self, prev: Odometry, curr: Odometry):
        """Return (Δx, Δy, Δθ) in the robot's previous frame."""
        x0 = prev.pose.pose.position.x
        y0 = prev.pose.pose.position.y
        h0 = _yaw_from_quaternion(prev.pose.pose.orientation)

        x1 = curr.pose.pose.position.x
        y1 = curr.pose.pose.position.y
        h1 = _yaw_from_quaternion(curr.pose.pose.orientation)

        dx_w = x1 - x0
        dy_w = y1 - y0

        # Rotate world delta into robot frame at h0
        cos0, sin0 = math.cos(-h0), math.sin(-h0)
        dx_r = dx_w * cos0 - dy_w * sin0
        dy_r = dx_w * sin0 + dy_w * cos0

        return dx_r, dy_r, _wrap(h1 - h0)

    # ── H. Move particles ─────────────────────────────────────────────────

    def _move_particles(self, dx_r: float, dy_r: float, dth: float):
        for p in self._particles:
            cos_p = math.cos(p[2])
            sin_p = math.sin(p[2])
            p[0] += dx_r * cos_p - dy_r * sin_p + random.gauss(0.0, self._noise_xy)
            p[1] += dx_r * sin_p + dy_r * cos_p + random.gauss(0.0, self._noise_xy)
            p[2]  = _wrap(p[2] + dth + random.gauss(0.0, self._noise_th))

    def _broadcast_map_to_odom(self, best_pose: list[float], stamp):
        """
        Publish the correction between the estimated robot pose in map and the
        dead-reckoned pose in odom.
        """
        if self._prev_odom is None:
            return

        odom_x = self._prev_odom.pose.pose.position.x
        odom_y = self._prev_odom.pose.pose.position.y
        odom_yaw = _yaw_from_quaternion(self._prev_odom.pose.pose.orientation)

        map_to_odom_yaw = _wrap(best_pose[2] - odom_yaw)
        rot_x, rot_y = _rotate_2d(odom_x, odom_y, map_to_odom_yaw)
        map_to_odom_x = best_pose[0] - rot_x
        map_to_odom_y = best_pose[1] - rot_y

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._map_frame
        tf.child_frame_id = self._odom_frame
        tf.transform.translation.x = map_to_odom_x
        tf.transform.translation.y = map_to_odom_y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = _quaternion_from_yaw(map_to_odom_yaw)
        self._tf_broadcaster.sendTransform(tf)

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        if self._prev_odom is None:
            self._prev_odom = msg
            return

        dx_r, dy_r, dth = self._compute_delta(self._prev_odom, msg)
        self._prev_odom = msg

        # H. Move all particles by the estimated robot motion
        if self._particles:
            self._move_particles(dx_r, dy_r, dth)

    def _scan_cb(self, scan: LaserScan):
        """Main MCL loop triggered on every new laser scan."""
        if not self._particles:
            return

        # E. Score every particle
        scored = [
            (self._score_particle(p[0], p[1], p[2], scan), p)
            for p in self._particles
        ]

        # F. Keep top-K
        scored.sort(key=lambda t: t[0], reverse=True)
        survivors = [p for _, p in scored[: self._k]]

        # I. Resample to N
        self._resample(survivors)

        # Publish
        stamp = scan.header.stamp
        self._publish_particles(stamp)
        best_pose = self._publish_best_pose(survivors, stamp)
        if best_pose is not None:
            self._broadcast_map_to_odom(best_pose, stamp)

    # ── Publishing ────────────────────────────────────────────────────────

    def _publish_particles(self, stamp):
        msg = PoseArray()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._map_frame

        for p in self._particles:
            pose = Pose()
            pose.position.x  = p[0]
            pose.position.y  = p[1]
            pose.orientation = _quaternion_from_yaw(p[2])
            msg.poses.append(pose)

        self._pub_array.publish(msg)

    def _publish_best_pose(self, survivors: list[list[float]], stamp):
        """Publish the mean of the top-K survivors as the best pose estimate."""
        if not survivors:
            return None

        mean_x = sum(p[0] for p in survivors) / len(survivors)
        mean_y = sum(p[1] for p in survivors) / len(survivors)

        # Circular mean for heading
        sin_sum = sum(math.sin(p[2]) for p in survivors)
        cos_sum = sum(math.cos(p[2]) for p in survivors)
        mean_th = math.atan2(sin_sum, cos_sum)

        msg = PoseStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._map_frame
        msg.pose.position.x  = mean_x
        msg.pose.position.y  = mean_y
        msg.pose.orientation = _quaternion_from_yaw(mean_th)

        self._pub_pose.publish(msg)
        return [mean_x, mean_y, mean_th]


def main(args=None):
    rclpy.init(args=args)
    node = MCLNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
