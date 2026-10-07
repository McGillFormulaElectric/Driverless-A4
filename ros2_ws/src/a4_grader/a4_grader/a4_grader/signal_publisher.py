"""Publish deterministic local IMU, GPS, and ground-truth inputs for A4."""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=10)


class ScenarioPublisher(Node):
    """A smooth, reproducible vehicle trajectory used by the local grader."""
    def __init__(self):
        super().__init__('scenario_publisher')
        self.declare_parameter('scenario_hz', 20.0)
        self.declare_parameter('gps_hz', 5.0)
        self.declare_parameter('gps_noise_std', 0.15)
        self.declare_parameter('seed', 20260909)
        self.hz = float(self.get_parameter('scenario_hz').value)
        self.gps_period = max(1, round(self.hz / float(self.get_parameter('gps_hz').value)))
        self.gps_noise = float(self.get_parameter('gps_noise_std').value)
        self.rng = np.random.default_rng(int(self.get_parameter('seed').value))
        self.imu_pub = self.create_publisher(Imu, '/grader/imu', QOS)
        self.gps_pub = self.create_publisher(PoseStamped, '/grader/gps', QOS)
        self.truth_pub = self.create_publisher(Odometry, '/grader/truth', QOS)
        self.t0 = self.get_clock().now()
        self.count = 0
        self.timer = self.create_timer(1.0 / self.hz, self.tick)
        self.get_logger().info('Publishing local A4 IMU, GPS, and truth scenario')

    def tick(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        # Smooth varying speed and heading provide both acceleration and yaw.
        v = 2.0 + 0.4 * math.sin(0.35 * t)
        a = 0.14 * math.cos(0.35 * t)
        theta = 0.25 * math.sin(0.22 * t)
        omega = 0.055 * math.cos(0.22 * t)
        # Approximate the trajectory by applying the current speed and heading
        # over elapsed time. It is used only to seed/observe the local exercise.
        x = v * math.cos(theta) * t
        y = v * math.sin(theta) * t
        stamp = now.to_msg()
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'base_link'
        imu.linear_acceleration.x = a
        imu.angular_velocity.z = omega
        self.imu_pub.publish(imu)

        truth = Odometry()
        truth.header.stamp = stamp
        truth.header.frame_id = 'map'
        truth.child_frame_id = 'base_link'
        truth.pose.pose.position.x = x
        truth.pose.pose.position.y = y
        truth.pose.pose.orientation.z = math.sin(theta / 2.0)
        truth.pose.pose.orientation.w = math.cos(theta / 2.0)
        truth.twist.twist.linear.x = v
        self.truth_pub.publish(truth)

        if self.count % self.gps_period == 0:
            gps = PoseStamped()
            gps.header.stamp = stamp
            gps.header.frame_id = 'map'
            gps.pose.position.x = x + float(self.rng.normal(0.0, self.gps_noise))
            gps.pose.position.y = y + float(self.rng.normal(0.0, self.gps_noise))
            self.gps_pub.publish(gps)
        self.count += 1


def main():
    rclpy.init()
    node = ScenarioPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
