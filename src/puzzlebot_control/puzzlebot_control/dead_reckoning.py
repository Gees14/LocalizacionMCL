"""
Dead-reckoning odometry node for a differential-drive robot.

Inputs
------
  /joint_states  (sensor_msgs/JointState)
      Joint velocities from Gazebo Harmonic via ros_gz_bridge.
      Joints: wheel_l_joint, wheel_r_joint  [rad/s]

      --- Real robot equivalent ---
      When deployed to hardware, this node instead subscribes to:
        /velocity_enc_r  (std_msgs/Float32)  [rad/s]
        /velocity_enc_l  (std_msgs/Float32)  [rad/s]
      A separate launch argument selects the input source so the
      algorithm node itself does not change between sim and real.

Outputs
-------
  /odom          (nav_msgs/Odometry)    pose + twist in odom frame
  /TF            odom → base_footprint  (via tf2)

Parameters
----------
  wheel_radius     (float, default 0.05)   metres
  wheel_separation (float, default 0.19)   metres  (centre to centre)
  odom_frame       (str,   default 'odom')
  base_frame       (str,   default 'base_footprint')
  input_source     (str,   default 'joint_states')
                   'joint_states'  → sim (JointState)
                   'encoders'      → real robot (Float32 per wheel)
"""

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

import tf2_ros


def _euler_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class DeadReckoning(Node):
    def __init__(self):
        super().__init__('dead_reckoning')

        self.declare_parameter('wheel_radius',     0.05)
        self.declare_parameter('wheel_separation', 0.19)
        self.declare_parameter('odom_frame',       'odom')
        self.declare_parameter('base_frame',       'base_footprint')
        self.declare_parameter('input_source',     'joint_states')

        self._r  = self.get_parameter('wheel_radius').value
        self._l  = self.get_parameter('wheel_separation').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._source     = self.get_parameter('input_source').value

        # Pose state
        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0
        self._last_stamp = None

        # For encoder source: store latest readings
        self._wl = 0.0
        self._wr = 0.0

        # Publishers
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers — selected by input_source parameter
        if self._source == 'joint_states':
            self.create_subscription(
                JointState, '/joint_states', self._joint_states_cb, 10)
            self.get_logger().info(
                'dead_reckoning: input=joint_states (simulation mode)')
        else:
            # Real robot: two Float32 topics, integrate at fixed rate
            self.create_subscription(
                Float32, '/velocity_enc_r', self._enc_r_cb, 10)
            self.create_subscription(
                Float32, '/velocity_enc_l', self._enc_l_cb, 10)
            self.create_timer(0.05, self._encoder_timer_cb)  # 20 Hz
            self.get_logger().info(
                'dead_reckoning: input=encoders (real robot mode)')

    # ── Joint-state path (simulation) ─────────────────────────────────

    def _joint_states_cb(self, msg: JointState):
        stamp = msg.header.stamp

        if self._last_stamp is None:
            self._last_stamp = stamp
            return

        dt = (stamp.sec - self._last_stamp.sec) + \
             (stamp.nanosec - self._last_stamp.nanosec) * 1e-9
        self._last_stamp = stamp

        if dt <= 0.0 or dt > 0.5:
            return

        # Extract wheel velocities by joint name (order is not guaranteed)
        wl = wr = None
        for i, name in enumerate(msg.name):
            if name == 'wheel_l_joint' and i < len(msg.velocity):
                wl = msg.velocity[i]
            elif name == 'wheel_r_joint' and i < len(msg.velocity):
                wr = msg.velocity[i]

        if wl is None or wr is None:
            return

        self._integrate(wl, wr, dt, stamp)

    # ── Encoder path (real robot) ──────────────────────────────────────

    def _enc_r_cb(self, msg: Float32):
        self._wr = msg.data

    def _enc_l_cb(self, msg: Float32):
        self._wl = msg.data

    def _encoder_timer_cb(self):
        now = self.get_clock().now().to_msg()
        if self._last_stamp is None:
            self._last_stamp = now
            return
        dt = (now.sec - self._last_stamp.sec) + \
             (now.nanosec - self._last_stamp.nanosec) * 1e-9
        self._last_stamp = now
        if dt <= 0.0 or dt > 0.5:
            return
        self._integrate(self._wl, self._wr, dt, now)

    # ── Core kinematics ───────────────────────────────────────────────

    def _integrate(self, wl: float, wr: float, dt: float, stamp):
        """
        Differential-drive forward kinematics.
          v  = r * (wr + wl) / 2        linear velocity  [m/s]
          w  = r * (wr - wl) / L        angular velocity [rad/s]
        Integrate with Euler (fine at 20 Hz, replace with RK4 if needed).
        """
        v = self._r * (wr + wl) / 2.0
        w = self._r * (wr - wl) / self._l

        self._x   += v * math.cos(self._yaw) * dt
        self._y   += v * math.sin(self._yaw) * dt
        self._yaw += w * dt

        # Normalise yaw to [-pi, pi]
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        self._publish(v, w, stamp)

    def _publish(self, v: float, w: float, stamp):
        q = _euler_to_quaternion(self._yaw)

        # Odometry message
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id  = self._base_frame

        odom.pose.pose.position.x  = self._x
        odom.pose.pose.position.y  = self._y
        odom.pose.pose.position.z  = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w

        self._odom_pub.publish(odom)

        # TF: odom → base_footprint
        tf = TransformStamped()
        tf.header.stamp            = stamp
        tf.header.frame_id         = self._odom_frame
        tf.child_frame_id          = self._base_frame
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.translation.z = 0.0
        tf.transform.rotation      = q

        self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
