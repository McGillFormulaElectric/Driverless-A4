"""A4.1 — dead-reckoning pose estimator (IMU-only).

Subscribes to /neil/imu (sensor_msgs/Imu) and integrates the body-frame
forward acceleration (`linear_acceleration.x`) and yaw rate
(`angular_velocity.z`) into a 2D pose on `<namespace>/odom_dr`
(nav_msgs/Odometry, frame `map`).

Purpose: show that dead-reckoning drifts without correction. The grader only
scores the first 10 s of your stream, so you can pass A4.1 by simply getting
the integration right. A4.2 is where you fix the drift with an EKF.

Trajectory contract (see README):
    - The vehicle starts at rest at the origin, heading = 0.
    - Initial state is therefore (x, y, theta, v) = (0, 0, 0, 0).

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

        # Initial state — vehicle starts at rest at origin, heading 0.
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._v = 0.0

        # Previous IMU header stamp (seconds) for computing dt.
        self._t_prev: float | None = None

        ns = self.get_namespace()
        self.get_logger().info(
            f'Dead-reckoning from /neil/imu -> {ns}/odom_dr (map frame). '
            'Expect drift; A4.1 grader only scores the first 10 s.'
        )

    def _on_imu(self, msg: Imu) -> None:
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
        # Integrate (semi-implicit Euler works fine):
        #   theta_new = theta + omega_z * dt
        #   v_new     = v + a_body_x * dt
        #   x_new     = x + v_new * cos(theta_new) * dt
        #   y_new     = y + v_new * sin(theta_new) * dt
        #
        # Update self._x, self._y, self._theta, self._v.
        # ------------------------------------------------------------------
        a = float(msg.linear_acceleration.x)
        w = float(msg.angular_velocity.z)

        # <-- replace the four lines below with the correct integration.
        self._theta = self._theta  # TODO
        self._v = self._v          # TODO
        self._x = self._x          # TODO
        self._y = self._y          # TODO

        # Keep referenced so lint doesn't complain in the stub; students replace above.
        _ = (a, w)

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
