"""A4.2 — 2D Extended Kalman Filter fusing IMU (predict) + GPS (update).

State:
    x = [px, py, theta, v]^T

Predict (on every IMU message):
    px'    = px + v * cos(theta) * dt
    py'    = py + v * sin(theta) * dt
    theta' = theta + omega_z * dt
    v'     = v + a_body_x * dt

Update (on every GPS message):
    z = [px_meas, py_meas]^T,  H = [[1,0,0,0],[0,1,0,0]]

Publishes /<namespace>/odom (nav_msgs/Odometry, frame `map`).

References (see README):
    - Wikipedia: Extended Kalman filter
    - Thrun, Burgard, Fox — Probabilistic Robotics, Ch. 3

Tuning knobs (see a4_new_member/config/params.yaml — these are hints, tune away):
    initial_pos_cov, initial_yaw_cov, initial_v_cov   -> P0 diagonal
    q_accel, q_yaw_rate                               -> Q diagonal drivers
    r_gps (variance, not sigma; 0.5 m sigma -> 0.25)  -> R diagonal

Run:
    ros2 launch a4_new_member ekf.launch.py github_user:=<your-handle>
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


def yaw_to_quat(theta: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))


class EKF:
    """Plain-numpy 2D EKF. No ROS types in here so students can unit-test it.

    Tuning constants are injected via the constructor so this class stays
    ROS-agnostic. See EkfNode.__init__ for parameter wiring.
    """

    def __init__(
        self,
        initial_pos_cov: float = 1.0,
        initial_yaw_cov: float = 0.1,
        initial_v_cov: float = 0.5,
        q_accel: float = 0.1,
        q_yaw_rate: float = 0.01,
        r_gps: float = 0.25,  # NOTE: variance, not sigma. 0.5 m sigma -> 0.25.
    ):
        # State vector [x, y, theta, v]
        self.x = np.zeros(4)

        # ------------------------------------------------------------------
        # Initial covariance P (4x4). Diagonal, seeded from ROS parameters.
        # Small values because we know we start at rest at the origin.
        # ------------------------------------------------------------------
        self.P = np.diag([initial_pos_cov, initial_pos_cov,
                          initial_yaw_cov, initial_v_cov])

        # ------------------------------------------------------------------
        # Process-noise Q (4x4) and measurement-noise R (2x2).
        # Position rows of Q are driven by q_accel (they get accel noise
        # after one integration); theta by q_yaw_rate; v by q_accel.
        # Students: feel free to override these directly if you want a
        # different structure.
        # ------------------------------------------------------------------
        self.Q = np.diag([q_accel, q_accel, q_yaw_rate, q_accel])
        self.R = np.diag([r_gps, r_gps])

    def predict(self, a_body_x: float, omega_z: float, dt: float) -> None:
        """Non-linear motion model f(x, u) + EKF covariance propagation."""
        if dt <= 0.0:
            return

        px, py, theta, v = self.x

        # ------------------------------------------------------------------
        # TODO(student): implement the predict step.
        #
        # 1. Propagate the state through the non-linear motion model above.
        # 2. Build the Jacobian F = df/dx evaluated at the current state:
        #        F = [[1, 0, -v*sin(theta)*dt, cos(theta)*dt],
        #             [0, 1,  v*cos(theta)*dt, sin(theta)*dt],
        #             [0, 0,  1,               0            ],
        #             [0, 0,  0,               1            ]]
        # 3. Propagate covariance: P = F @ P @ F.T + Q
        # ------------------------------------------------------------------

        # Stub: identity propagation so the node still runs before the student fills it in.
        self.x = np.array([px, py, theta, v])  # TODO
        F = np.eye(4)                          # TODO
        self.P = F @ self.P @ F.T + self.Q

        # keep names referenced so linters don't strip them
        _ = (a_body_x, omega_z, dt)

    def update_gps(self, z_x: float, z_y: float) -> None:
        """GPS measures position directly: z = H x + v, with H = [[1,0,0,0],[0,1,0,0]]."""
        H = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0]])
        z = np.array([z_x, z_y])

        # ------------------------------------------------------------------
        # TODO(student): implement the EKF update step.
        #     y   = z - H @ x                 # innovation
        #     S   = H @ P @ H.T + R           # innovation covariance
        #     K   = P @ H.T @ inv(S)          # Kalman gain
        #     x   = x + K @ y
        #     P   = (I - K @ H) @ P
        # ------------------------------------------------------------------
        _ = (H, z)  # remove once implemented


class EkfNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # --- ROS parameters (see a4_new_member/config/params.yaml) -------------
        self.declare_parameter('initial_pos_cov', 1.0)
        self.declare_parameter('initial_yaw_cov', 0.1)
        self.declare_parameter('initial_v_cov', 0.5)
        self.declare_parameter('q_accel', 0.1)
        self.declare_parameter('q_yaw_rate', 0.01)
        self.declare_parameter('r_gps', 0.25)  # variance, not sigma

        self.ekf = EKF(
            initial_pos_cov=float(self.get_parameter('initial_pos_cov').value),
            initial_yaw_cov=float(self.get_parameter('initial_yaw_cov').value),
            initial_v_cov=float(self.get_parameter('initial_v_cov').value),
            q_accel=float(self.get_parameter('q_accel').value),
            q_yaw_rate=float(self.get_parameter('q_yaw_rate').value),
            r_gps=float(self.get_parameter('r_gps').value),
        )

        self.create_subscription(Imu, '/neil/imu', self._on_imu, RELIABLE_QOS)
        self.create_subscription(PoseStamped, '/neil/gps', self._on_gps, RELIABLE_QOS)
        self.pub = self.create_publisher(Odometry, 'odom', RELIABLE_QOS)

        self._t_prev: float | None = None

        ns = self.get_namespace()
        self.get_logger().info(
            f'EKF fusing /neil/imu + /neil/gps -> {ns}/odom (map frame).'
        )

    def _on_imu(self, msg: Imu) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t_prev is None:
            self._t_prev = t
            self._publish(msg.header.stamp)
            return
        dt = t - self._t_prev
        self._t_prev = t
        if dt <= 0.0 or dt > 1.0:
            return

        self.ekf.predict(
            a_body_x=float(msg.linear_acceleration.x),
            omega_z=float(msg.angular_velocity.z),
            dt=dt,
        )
        self._publish(msg.header.stamp)

    def _on_gps(self, msg: PoseStamped) -> None:
        self.ekf.update_gps(
            z_x=float(msg.pose.position.x),
            z_y=float(msg.pose.position.y),
        )
        self._publish(msg.header.stamp)

    def _publish(self, stamp) -> None:
        px, py, theta, v = self.ekf.x
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(px)
        odom.pose.pose.position.y = float(py)
        odom.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(float(theta))
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(v)
        self.pub.publish(odom)


def main():
    rclpy.init()
    node = EkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
