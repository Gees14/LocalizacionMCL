"""Differential-drive wheel odometry node for the Puzzlebot.

Integrates wheel angular velocities into a 2-D pose estimate and
publishes it as nav_msgs/Odometry plus the odom→base_footprint TF.

Two input modes are supported (selected via the *input_source* parameter):

* ``joint_states``  — reads sensor_msgs/JointState from the Gazebo bridge.
  Timestamps come from the message header, so the integration rate
  matches the simulation rate exactly.

* ``encoders``      — reads std_msgs/Float32 topics from real-robot
  encoder drivers.  A fixed-rate timer integrates at *encoder_rate* Hz
  and stores the latest velocity sample between ticks.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import tf2_ros


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


class WheelOdometryNode(Node):
    """Integrates wheel velocities into an odometry estimate.

    Parameters
    ----------
    wheel_radius       : metres
    wheel_separation   : distance between wheel contact points (metres)
    odom_frame         : name of the odometry frame  (default: 'odom')
    base_frame         : name of the robot base frame (default: 'base_footprint')
    input_source       : 'joint_states' | 'encoders'
    encoder_rate       : integration rate when using encoder topics (Hz)
    """

    def __init__(self) -> None:
        super().__init__('wheel_odometry_node')
        self._declare_params()

        self._radius    = self.get_parameter('wheel_radius').value
        self._baseline  = self.get_parameter('wheel_separation').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        source           = self.get_parameter('input_source').value
        enc_rate         = self.get_parameter('encoder_rate').value

        # Robot pose state
        self._x:     float = 0.0
        self._y:     float = 0.0
        self._theta: float = 0.0

        # TF broadcaster
        self._tf_br = tf2_ros.TransformBroadcaster(self)

        # Publisher
        self._pub_odom = self.create_publisher(Odometry, '/odom', 10)

        # Subscriptions — choose input mode
        if source == 'joint_states':
            self._prev_stamp: Optional[float] = None
            self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
            self.get_logger().info('WheelOdometryNode — source: joint_states')
        else:
            # Real-robot encoder topics
            self._w_right: float = 0.0
            self._w_left:  float = 0.0
            self.create_subscription(Float32, '/velocity_enc_r', self._on_enc_right, 10)
            self.create_subscription(Float32, '/velocity_enc_l', self._on_enc_left,  10)
            self.create_timer(1.0 / enc_rate, self._integrate_encoders)
            self.get_logger().info(f'WheelOdometryNode — source: encoders @ {enc_rate} Hz')

    # ------------------------------------------------------------------
    # Joint-state input (simulation)
    # ------------------------------------------------------------------

    def _on_joint_states(self, msg: JointState) -> None:
        """Read wheel velocities from a JointState message and integrate."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._prev_stamp is None:
            self._prev_stamp = stamp
            return

        dt = stamp - self._prev_stamp
        self._prev_stamp = stamp
        if dt <= 0.0:
            return

        w_right, w_left = self._extract_wheel_velocities(msg)
        self._integrate(w_right, w_left, dt)
        self._publish(msg.header.stamp)

    def _extract_wheel_velocities(self, msg: JointState):
        """Return (w_right, w_left) in rad/s from a JointState message."""
        w_right = w_left = 0.0
        for name, vel in zip(msg.name, msg.velocity):
            lower = name.lower()
            if 'right' in lower or '_r' in lower:
                w_right = vel
            elif 'left' in lower or '_l' in lower:
                w_left = vel
        return w_right, w_left

    # ------------------------------------------------------------------
    # Encoder topics input (real robot)
    # ------------------------------------------------------------------

    def _on_enc_right(self, msg: Float32) -> None:
        self._w_right = msg.data

    def _on_enc_left(self, msg: Float32) -> None:
        self._w_left = msg.data

    def _integrate_encoders(self) -> None:
        dt = 1.0 / self.get_parameter('encoder_rate').value
        self._integrate(self._w_right, self._w_left, dt)
        self._publish(self.get_clock().now().to_msg())

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------

    def _integrate(self, w_right: float, w_left: float, dt: float) -> None:
        """Euler integration of differential-drive kinematics.

        v = r * (w_r + w_l) / 2
        w = r * (w_r - w_l) / L
        """
        r, L = self._radius, self._baseline
        v = r * (w_right + w_left) / 2.0
        w = r * (w_right - w_left) / L

        self._x     += v * math.cos(self._theta) * dt
        self._y     += v * math.sin(self._theta) * dt
        self._theta += w * dt
        self._theta  = (self._theta + math.pi) % (2.0 * math.pi) - math.pi

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish(self, stamp) -> None:
        q = _yaw_to_quat(self._theta)

        # Odometry message
        odom = Odometry()
        odom.header.stamp          = stamp
        odom.header.frame_id       = self._odom_frame
        odom.child_frame_id        = self._base_frame
        odom.pose.pose.position.x  = self._x
        odom.pose.pose.position.y  = self._y
        odom.pose.pose.orientation = q
        self._pub_odom.publish(odom)

        # TF: odom → base_footprint
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id  = self._base_frame
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.rotation      = q
        self._tf_br.sendTransform(t)

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_params(self) -> None:
        self.declare_parameter('wheel_radius',     0.05)
        self.declare_parameter('wheel_separation',  0.19)
        self.declare_parameter('odom_frame',       'odom')
        self.declare_parameter('base_frame',       'base_footprint')
        self.declare_parameter('input_source',     'joint_states')
        self.declare_parameter('encoder_rate',      20.0)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
