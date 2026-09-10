"""Professor sensor simulator for MFE A4.

Drives a bicycle-style vehicle around a figure-8-shaped trajectory (two lobes
of radius ~8 m) using prescribed forward-acceleration and yaw-rate profiles.
The vehicle starts at rest at the origin with heading = 0.

At every 50 Hz tick we:
  1. Advance the ground-truth state (x, y, theta, v) with semi-implicit Euler,
     using an *analytical* forward acceleration and yaw rate.
  2. Publish /professor/truth (nav_msgs/Odometry) — noise-free.
  3. Publish /professor/imu (sensor_msgs/Imu) — accel + yaw-rate + Gaussian noise.
  4. Every 10th tick (5 Hz), publish /professor/gps (geometry_msgs/PoseStamped)
     with x/y ground truth + Gaussian noise.

All three topics share the same simulated-time header stamp so the grader can
match student estimates to ground truth by header timestamp (not wall clock).

RNG is seeded (20260909) so every run is reproducible.
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

# --- simulation constants --------------------------------------------------
IMU_HZ = 50.0
GPS_EVERY_N = 10          # -> 5 Hz GPS given 50 Hz IMU

# Motion profile — chosen so the traced curve has two lobes of radius ~8 m and
# the vehicle starts at rest.
V_SS = 5.0                # steady-state speed [m/s]
TAU_RAMP = 2.0            # speed ramp-up time constant [s]
OMEGA_MAX = 1.0           # peak yaw rate [rad/s]
T_PERIOD = 2.0 * math.pi ** 2  # yaw-rate period [s] ~ 19.74

# Sensor noise (documented in README so students can set R correctly).
ACCEL_NOISE_STD = 0.2     # m/s^2 on body-forward accel
YAWRATE_NOISE_STD = 0.02  # rad/s on omega_z
GPS_NOISE_STD = 0.5       # m on x and y

SEED = 20260909


def yaw_to_quat(theta: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))


def a_true(t: float) -> float:
    """Analytical body-forward acceleration = dv/dt for v(t)=V_SS*(1-exp(-t/tau))."""
    return (V_SS / TAU_RAMP) * math.exp(-t / TAU_RAMP)


def omega_true(t: float) -> float:
    """Analytical yaw rate."""
    return OMEGA_MAX * math.sin(2.0 * math.pi * t / T_PERIOD)


class SensorSimNode(Node):
    def __init__(self):
        super().__init__('sensor_sim_node')

        self.truth_pub = self.create_publisher(Odometry, '/professor/truth', RELIABLE_QOS)
        self.imu_pub = self.create_publisher(Imu, '/professor/imu', RELIABLE_QOS)
        self.gps_pub = self.create_publisher(PoseStamped, '/professor/gps', RELIABLE_QOS)

        self._rng = np.random.default_rng(SEED)

        # Ground truth state; vehicle starts at rest at origin, heading 0.
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._v = 0.0

        # Simulated time; header stamps are derived from this so runs are reproducible.
        self._t_sim = 0.0
        self._tick_idx = 0
        self._dt = 1.0 / IMU_HZ

        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'Sensor sim @ {IMU_HZ:.0f} Hz IMU, {IMU_HZ / GPS_EVERY_N:.1f} Hz GPS. '
            f'V_ss={V_SS} m/s, Omega_max={OMEGA_MAX} rad/s, T={T_PERIOD:.2f} s. '
            f'Noise sigma: accel={ACCEL_NOISE_STD}, yaw_rate={YAWRATE_NOISE_STD}, gps={GPS_NOISE_STD}.'
        )

    # --- helpers ------------------------------------------------------------
    def _sim_stamp(self):
        stamp = rclpy.time.Time(seconds=self._t_sim).to_msg()
        return stamp

    # --- main tick ----------------------------------------------------------
    def _tick(self) -> None:
        # 1. Analytical true controls at current sim time.
        a = a_true(self._t_sim)
        w = omega_true(self._t_sim)

        # 2. Semi-implicit Euler on truth. This is exactly what a perfect
        #    student DR node would do, so DR error is only noise-integration.
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
        # -1 indicates orientation is provided but with unknown covariance.
        imu.orientation_covariance = [0.01, 0.0, 0.0,
                                      0.0, 0.01, 0.0,
                                      0.0, 0.0, 0.01]
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = w + float(self._rng.normal(0.0, YAWRATE_NOISE_STD))
        imu.angular_velocity_covariance = [0.0] * 9
        imu.angular_velocity_covariance[8] = YAWRATE_NOISE_STD ** 2
        imu.linear_acceleration.x = a + float(self._rng.normal(0.0, ACCEL_NOISE_STD))
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0
        imu.linear_acceleration_covariance = [0.0] * 9
        imu.linear_acceleration_covariance[0] = ACCEL_NOISE_STD ** 2
        self.imu_pub.publish(imu)

        # 5. Publish GPS every N ticks.
        if self._tick_idx % GPS_EVERY_N == 0:
            gps = PoseStamped()
            gps.header.stamp = stamp
            gps.header.frame_id = 'map'
            gps.pose.position.x = self._x + float(self._rng.normal(0.0, GPS_NOISE_STD))
            gps.pose.position.y = self._y + float(self._rng.normal(0.0, GPS_NOISE_STD))
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
