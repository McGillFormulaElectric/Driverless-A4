"""Neil sensor simulator for MFE A4.

Drives a bicycle-style vehicle around a figure-8-shaped trajectory (two lobes
of radius ~8 m) using prescribed forward-acceleration and yaw-rate profiles.
The vehicle starts at rest at the origin with heading = 0.

At every IMU tick we:
  1. Advance the ground-truth state (x, y, theta, v) with semi-implicit Euler,
     using an *analytical* forward acceleration and yaw rate.
  2. Publish /neil/truth (nav_msgs/Odometry) — noise-free.
  3. Publish /neil/imu (sensor_msgs/Imu) — accel + yaw-rate + Gaussian noise.
  4. Every N ticks (5 Hz by default), publish /neil/gps
     (geometry_msgs/PoseStamped) with x/y ground truth + Gaussian noise.

All three topics share the same simulated-time header stamp so the grader can
match user estimates to ground truth by header timestamp (not wall clock).

RNG is seeded (default 20260909) so every run is reproducible. All scenario
constants are exposed as ROS parameters — see config/params.yaml.
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


class SensorSimNode(Node):
    def __init__(self):
        super().__init__('sensor_sim_node')

        # --- ROS parameters (see a4_neil/config/params.yaml) ----------------
        self.declare_parameter('imu_hz', 50.0)
        self.declare_parameter('gps_hz', 5.0)
        self.declare_parameter('imu_accel_sigma', 0.2)
        self.declare_parameter('imu_yaw_rate_sigma', 0.02)
        self.declare_parameter('gps_pos_sigma', 0.5)
        self.declare_parameter('seed', 20260909)

        # Trajectory parameters (figure-8 shape).
        # v_ss:        steady-state forward speed [m/s]
        # tau_ramp:    speed ramp-up time constant [s]
        # omega_max:   peak yaw rate [rad/s]
        # traj_period: yaw-rate period [s]; default 2*pi^2 ~ 19.74 gives two lobes r~8m
        self.declare_parameter('v_ss', 5.0)
        self.declare_parameter('tau_ramp', 2.0)
        self.declare_parameter('omega_max', 1.0)
        self.declare_parameter('traj_period', 2.0 * math.pi ** 2)

        imu_hz = float(self.get_parameter('imu_hz').value)
        gps_hz = float(self.get_parameter('gps_hz').value)
        self._accel_sigma = float(self.get_parameter('imu_accel_sigma').value)
        self._yawrate_sigma = float(self.get_parameter('imu_yaw_rate_sigma').value)
        self._gps_sigma = float(self.get_parameter('gps_pos_sigma').value)
        seed = int(self.get_parameter('seed').value)

        self._v_ss = float(self.get_parameter('v_ss').value)
        self._tau_ramp = float(self.get_parameter('tau_ramp').value)
        self._omega_max = float(self.get_parameter('omega_max').value)
        self._traj_period = float(self.get_parameter('traj_period').value)

        if gps_hz <= 0.0 or gps_hz > imu_hz:
            raise ValueError('gps_hz must be in (0, imu_hz]')
        self._gps_every_n = max(1, int(round(imu_hz / gps_hz)))

        self.truth_pub = self.create_publisher(Odometry, '/neil/truth', RELIABLE_QOS)
        self.imu_pub = self.create_publisher(Imu, '/neil/imu', RELIABLE_QOS)
        self.gps_pub = self.create_publisher(PoseStamped, '/neil/gps', RELIABLE_QOS)

        self._rng = np.random.default_rng(seed)

        # Ground truth state; vehicle starts at rest at origin, heading 0.
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._v = 0.0

        # Simulated time; header stamps are derived from this so runs are reproducible.
        self._t_sim = 0.0
        self._tick_idx = 0
        self._dt = 1.0 / imu_hz

        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'Sensor sim @ {imu_hz:.0f} Hz IMU, {imu_hz / self._gps_every_n:.1f} Hz GPS. '
            f'V_ss={self._v_ss} m/s, Omega_max={self._omega_max} rad/s, T={self._traj_period:.2f} s. '
            f'Noise sigma: accel={self._accel_sigma}, yaw_rate={self._yawrate_sigma}, '
            f'gps={self._gps_sigma}. Seed={seed}.'
        )

    # --- analytical control profiles ---------------------------------------
    def _a_true(self, t: float) -> float:
        """Analytical body-forward acceleration = dv/dt for v(t)=v_ss*(1-exp(-t/tau))."""
        return (self._v_ss / self._tau_ramp) * math.exp(-t / self._tau_ramp)

    def _omega_true(self, t: float) -> float:
        """Analytical yaw rate."""
        return self._omega_max * math.sin(2.0 * math.pi * t / self._traj_period)

    # --- helpers ------------------------------------------------------------
    def _sim_stamp(self):
        stamp = rclpy.time.Time(seconds=self._t_sim).to_msg()
        return stamp

    # --- main tick ----------------------------------------------------------
    def _tick(self) -> None:
        # 1. Analytical true controls at current sim time.
        a = self._a_true(self._t_sim)
        w = self._omega_true(self._t_sim)

        # 2. Semi-implicit Euler on truth. This is exactly what a perfect
        #    user DR node would do, so DR error is only noise-integration.
        self._theta = self._theta + w * self._dt
        self._v = self._v + a * self._dt
        self._x = self._x + self._v * math.cos(self._theta) * self._dt
        self._y = self._y + self._v * math.sin(self._theta) * self._dt

        stamp = self._sim_stamp()

        # 3. Publish ground truth.
        truth = Odometry()
        truth.header.stamp = stamp
        truth.header.frame_id = 'map'
        truth.child_frame_id = 'base_link'
        truth.pose.pose.position.x = self._x
        truth.pose.pose.position.y = self._y
        truth.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(self._theta)
        truth.pose.pose.orientation.x = qx
        truth.pose.pose.orientation.y = qy
        truth.pose.pose.orientation.z = qz
        truth.pose.pose.orientation.w = qw
        truth.twist.twist.linear.x = self._v
        truth.twist.twist.angular.z = w
        self.truth_pub.publish(truth)

        # 4. Publish noisy IMU.
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'base_link'
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.orientation_covariance = [0.01, 0.0, 0.0,
                                      0.0, 0.01, 0.0,
                                      0.0, 0.0, 0.01]
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = w + float(self._rng.normal(0.0, self._yawrate_sigma))
        imu.angular_velocity_covariance = [0.0] * 9
        imu.angular_velocity_covariance[8] = self._yawrate_sigma ** 2
        imu.linear_acceleration.x = a + float(self._rng.normal(0.0, self._accel_sigma))
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0
        imu.linear_acceleration_covariance = [0.0] * 9
        imu.linear_acceleration_covariance[0] = self._accel_sigma ** 2
        self.imu_pub.publish(imu)

        # 5. Publish GPS every N ticks.
        if self._tick_idx % self._gps_every_n == 0:
            gps = PoseStamped()
            gps.header.stamp = stamp
            gps.header.frame_id = 'map'
            gps.pose.position.x = self._x + float(self._rng.normal(0.0, self._gps_sigma))
            gps.pose.position.y = self._y + float(self._rng.normal(0.0, self._gps_sigma))
            gps.pose.position.z = 0.0
            gps.pose.orientation.w = 1.0
            self.gps_pub.publish(gps)

        # 6. Advance sim clock.
        self._tick_idx += 1
        self._t_sim += self._dt


def main():
    rclpy.init()
    node = SensorSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
