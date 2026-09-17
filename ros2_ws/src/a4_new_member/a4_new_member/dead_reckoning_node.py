"""A4.1 — dead-reckoning pose estimator (IMU-only).

Subscribes to /neil/imu (sensor_msgs/Imu) and integrates the body-frame
forward acceleration (`linear_acceleration.x`) and yaw rate
(`angular_velocity.z`) into a 2D pose on `<namespace>/odom_dr`
(nav_msgs/Odometry, frame `map`).

Purpose: show that dead-reckoning drifts without correction. The grader only
scores the first 10 s of your stream, so you can pass A4.1 by simply getting
the integration right. A4.2 is where you fix the drift with an EKF.

Trajectory contract (see README):
    - Like any real dead-reckoning system, you need a starting fix before
      you can integrate blind. This node seeds its initial (x, y, theta, v)
      from the *first* /neil/truth message it receives, then stops
      listening to truth entirely — everything after that is pure IMU
      integration. That one-time seed is not the same thing as continuous
      correction; continuous fusion is what A4.2's EKF does instead.

Run:
    ros2 launch a4_new_member dr.launch.py github_user:=<your-handle>
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


def yaw_to_quat(theta: float) -> tuple[float, float, float, float]:
    """Return (x, y, z, w) for a pure yaw rotation."""
    return (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))


class DeadReckoningNode(Node):
    def __init__(self):
        super().__init__('dead_reckoning_node')

        self.sub = self.create_subscription(
            Imu, '/neil/imu', self._on_imu, RELIABLE_QOS
        )
        self.pub = self.create_publisher(Odometry, 'odom_dr', RELIABLE_QOS)

        # Initial state — seeded from the vehicle's actual position/heading/
        # speed the moment we start tracking (see _on_truth). Not assumed
        # to be the origin: /neil/truth has been running continuously since
        # the grader booted, so "start at rest at the origin" would only be
        # literally true for whoever connects in the first instant.
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._v = 0.0
        self._seeded = False
        self._truth_sub = self.create_subscription(
            Odometry, '/neil/truth', self._on_truth, RELIABLE_QOS
        )

        # Previous IMU header stamp (seconds) for computing dt.
        self._t_prev: float | None = None

        ns = self.get_namespace()
        self.get_logger().info(
            f'Dead-reckoning from /neil/imu -> {ns}/odom_dr (map frame). '
            'Expect drift; A4.1 grader only scores the first 10 s.'
        )

    def _on_truth(self, msg: Odometry) -> None:
        """One-shot seed of our starting pose from /neil/truth. After this
        we never look at truth again — everything from here on is pure IMU
        dead-reckoning."""
        if self._seeded:
            return
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._theta = 2.0 * math.atan2(q.z, q.w)
        self._v = msg.twist.twist.linear.x
        self._seeded = True
        self.destroy_subscription(self._truth_sub)

    def _on_imu(self, msg: Imu) -> None:
        if not self._seeded:
            # Haven't got our starting fix yet — nothing to integrate from.
            return

        # Timestamp in seconds from the message header (reproducible, not wall time).
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._t_prev is None:
            self._t_prev = t
            # Publish an initial-pose Odometry so grader has a first sample.
            self._publish(msg.header.stamp)
            return

        dt = t - self._t_prev
        self._t_prev = t
        if dt <= 0.0 or dt > 1.0:
            # Skip pathological gaps (clock jumps, first-message races).
            return

        # ------------------------------------------------------------------
        # TODO(student): implement dead-reckoning integration.
        #
        # IMU gives you:
        #   a_body_x = msg.linear_acceleration.x   (forward acceleration, m/s^2)
        #   omega_z  = msg.angular_velocity.z      (yaw rate, rad/s)
        #
        # This is a unicycle model:
        #   theta_dot = omega_z
        #   v_dot     = a_body_x
        #   x_dot     = v * cos(theta)
        #   y_dot     = v * sin(theta)
        #
        # Integrate over dt using semi-implicit (symplectic) Euler:
        #   https://en.wikipedia.org/wiki/Semi-implicit_Euler_method
        #
        # Update self._x, self._y, self._theta, self._v.
        # ------------------------------------------------------------------
        self._theta = self._theta  # <-- replace this stub with the correct expression
        self._v = self._v          # <-- replace this stub with the correct expression
        self._x = self._x          # <-- replace this stub with the correct expression
        self._y = self._y          # <-- replace this stub with the correct expression

        self._publish(msg.header.stamp)

    def _publish(self, stamp) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self._x)
        odom.pose.pose.position.y = float(self._y)
        odom.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(self._theta)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(self._v)
        self.pub.publish(odom)


def main():
    rclpy.init()
    node = DeadReckoningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
