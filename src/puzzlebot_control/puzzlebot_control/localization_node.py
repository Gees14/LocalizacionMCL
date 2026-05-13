"""ROS 2 localization node — wires the particle filter to robot sensor topics."""

import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    Pose, PoseArray, PoseStamped,
    Quaternion, TransformStamped,
)
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros

from .map_loader import MapConfig, OccupancyMap
from .particle_filter import ParticleFilter, _wrap


# ---------------------------------------------------------------------------
# Quaternion ↔ yaw helpers
# ---------------------------------------------------------------------------

def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


def _quat_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class LocalizationNode(Node):
    """Particle-filter localization for the Puzzlebot.

    Subscriptions
    -------------
    /odom   nav_msgs/Odometry    — drives the motion model
    /scan   sensor_msgs/LaserScan — drives the sensor model

    Publications
    ------------
    /localization/particles  geometry_msgs/PoseArray   — full particle cloud
    /localization/pose       geometry_msgs/PoseStamped — best pose estimate
    /localization/map        nav_msgs/OccupancyGrid    — static map for RViz

    TF broadcast
    ------------
    map → odom  (correction transform computed from best estimate vs odometry)
    """

    def __init__(self) -> None:
        super().__init__('localization_node')
        self._declare_params()

        # Read parameters
        map_path    = self.get_parameter('map_path').value
        resolution  = self.get_parameter('map_resolution').value
        origin_x    = self.get_parameter('map_origin_x').value
        origin_y    = self.get_parameter('map_origin_y').value
        n_parts     = self.get_parameter('num_particles').value
        top_k       = self.get_parameter('top_k').value
        noise_xy    = self.get_parameter('noise_xy').value
        noise_th    = self.get_parameter('noise_theta').value
        sigma       = self.get_parameter('hit_sigma').value
        n_rays      = self.get_parameter('score_rays').value
        ray_step    = self.get_parameter('ray_step').value
        self._map_frame  = self.get_parameter('map_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value

        # Build map + filter
        cfg     = MapConfig(resolution=resolution, origin_x=origin_x, origin_y=origin_y)
        occ_map = OccupancyMap(map_path, cfg)
        self._filter = ParticleFilter(
            occ_map,
            num_particles      = n_parts,
            survivors          = top_k,
            motion_noise_xy    = noise_xy,
            motion_noise_theta = noise_th,
            sensor_sigma       = sigma,
            num_rays           = n_rays,
            ray_step           = ray_step,
        )

        # State
        self._prev_odom: Odometry | None = None
        self._last_odom: Odometry | None = None

        # TF broadcaster
        self._tf_br = tf2_ros.TransformBroadcaster(self)

        # Publishers
        self._pub_particles = self.create_publisher(PoseArray,     '/localization/particles', 10)
        self._pub_pose      = self.create_publisher(PoseStamped,   '/localization/pose',      10)
        self._pub_map       = self.create_publisher(OccupancyGrid, '/localization/map',       10)

        # Subscriptions
        self.create_subscription(Odometry,  '/odom', self._on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)

        # Publish the static map once after RViz has had time to start
        self._map_timer = self.create_timer(2.0, self._publish_map)

        self.get_logger().info(
            f'LocalizationNode started | {n_parts} particles | map: {map_path}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        """Propagate particles through the motion model using the odom delta."""
        self._last_odom = msg
        if self._prev_odom is None:
            self._prev_odom = msg
            return

        prev = self._prev_odom
        self._prev_odom = msg

        # World-frame displacement
        dx_w = msg.pose.pose.position.x - prev.pose.pose.position.x
        dy_w = msg.pose.pose.position.y - prev.pose.pose.position.y
        prev_yaw = _quat_to_yaw(prev.pose.pose.orientation)
        dtheta   = _wrap(_quat_to_yaw(msg.pose.pose.orientation) - prev_yaw)

        # Rotate displacement into the robot's previous frame
        dx =  dx_w * math.cos(prev_yaw) + dy_w * math.sin(prev_yaw)
        dy = -dx_w * math.sin(prev_yaw) + dy_w * math.cos(prev_yaw)

        self._filter.predict(dx, dy, dtheta)

    def _on_scan(self, msg: LaserScan) -> None:
        """Update particle weights with the laser scan, then publish results."""
        angles, ranges = self._subsample_scan(msg)
        if not angles:
            return

        self._filter.update(angles, ranges)

        stamp = msg.header.stamp
        self._publish_particles(stamp)
        self._publish_pose(stamp)
        self._broadcast_map_to_odom(stamp)

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------

    def _subsample_scan(self, msg: LaserScan):
        """Sub-sample the scan to ~num_rays rays, discarding invalid readings."""
        total   = len(msg.ranges)
        step    = max(1, total // self._filter.num_rays)
        angles, ranges = [], []
        for i in range(0, total, step):
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max:
                angles.append(msg.angle_min + i * msg.angle_increment)
                ranges.append(r)
        return angles, ranges

    def _publish_particles(self, stamp) -> None:
        msg = PoseArray()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._map_frame
        for p in self._filter.particles:
            pose = Pose()
            pose.position.x = p.x
            pose.position.y = p.y
            pose.orientation = _yaw_to_quat(p.theta)
            msg.poses.append(pose)
        self._pub_particles.publish(msg)

    def _publish_pose(self, stamp) -> None:
        x, y, theta = self._filter.best_estimate()
        msg = PoseStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._map_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation = _yaw_to_quat(theta)
        self._pub_pose.publish(msg)

    def _broadcast_map_to_odom(self, stamp) -> None:
        """Broadcast the map→odom correction so the TF tree stays consistent."""
        if self._last_odom is None:
            return

        est_x, est_y, est_yaw = self._filter.best_estimate()
        odom_x   = self._last_odom.pose.pose.position.x
        odom_y   = self._last_odom.pose.pose.position.y
        odom_yaw = _quat_to_yaw(self._last_odom.pose.pose.orientation)

        # Transform: map_T_odom
        tf_yaw = est_yaw - odom_yaw
        tf_x   = est_x - (odom_x * math.cos(tf_yaw) - odom_y * math.sin(tf_yaw))
        tf_y   = est_y - (odom_x * math.sin(tf_yaw) + odom_y * math.cos(tf_yaw))

        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = self._map_frame
        t.child_frame_id  = self._odom_frame
        t.transform.translation.x = tf_x
        t.transform.translation.y = tf_y
        t.transform.rotation = _yaw_to_quat(tf_yaw)
        self._tf_br.sendTransform(t)

    def _publish_map(self) -> None:
        """Publish the occupancy grid once and cancel the timer."""
        occ = self._filter.map
        msg = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.info.resolution = occ.cfg.resolution
        msg.info.width      = occ.width
        msg.info.height     = occ.height
        msg.info.origin.position.x = occ.cfg.origin_x
        msg.info.origin.position.y = occ.cfg.origin_y
        msg.data = occ.to_occupancy_grid_data()
        self._pub_map.publish(msg)
        self._map_timer.cancel()

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_params(self) -> None:
        default_map = os.path.join(os.path.dirname(__file__), 'maze_map.png')
        self.declare_parameter('map_path',       default_map)
        self.declare_parameter('map_resolution',  0.05)
        self.declare_parameter('map_origin_x',   -5.54)
        self.declare_parameter('map_origin_y',   -8.10)
        self.declare_parameter('num_particles',   500)
        self.declare_parameter('top_k',           150)
        self.declare_parameter('noise_xy',          0.05)
        self.declare_parameter('noise_theta',       0.05)
        self.declare_parameter('hit_sigma',         0.20)
        self.declare_parameter('score_rays',         36)
        self.declare_parameter('ray_step',          0.025)
        self.declare_parameter('map_frame',       'map')
        self.declare_parameter('odom_frame',      'odom')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
