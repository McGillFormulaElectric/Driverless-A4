"""Auto-discovery grader for MFE A4.

Grades:
  - A4.1: /<user>/dr_odom (nav_msgs/Odometry) — dead reckoning
  - A4.2: /<user>/odom (nav_msgs/Odometry) — EKF fusion

Feedback published on /grader/feedback (std_msgs/String).

params: discovery_period_s, grade_period_s
"""
from __future__ import annotations

import re
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

DR_ODOM_RE = re.compile(r'^/([^/]+)/dr_odom$')
EKF_ODOM_RE = re.compile(r'^/([^/]+)/odom$')
RESERVED_USERS = {'grader'}


@dataclass
class OdomState:
    last_msg: Optional[Odometry] = None
    message_count: int = 0


class Grader(Node):
    def __init__(self):
        super().__init__('grader')

        self.declare_parameter('discovery_period_s', 2.0)
        self.declare_parameter('grade_period_s', 2.0)

        self.discovery_period_s = float(self.get_parameter('discovery_period_s').value)
        self.grade_period_s = float(self.get_parameter('grade_period_s').value)

        self.feedback_pub = self.create_publisher(String, '/grader/feedback', RELIABLE_QOS)

        self._dr_odom: dict[str, OdomState] = {}
        self._dr_odom_subs: dict[str, object] = {}
        self._ekf_odom: dict[str, OdomState] = {}
        self._ekf_odom_subs: dict[str, object] = {}

        self.create_timer(self.discovery_period_s, self._discover)
        self.create_timer(self.grade_period_s, self._grade)

        self.get_logger().info('A4 Grader running (A4.1 DR + A4.2 EKF)')

    def _discover(self) -> None:
        for name, types in self.get_topic_names_and_types():
            m = DR_ODOM_RE.match(name)
            if m and 'nav_msgs/msg/Odometry' in types:
                user = m.group(1)
                if user in RESERVED_USERS or user in self._dr_odom_subs:
                    continue
                self._dr_odom[user] = OdomState()
                self._dr_odom_subs[user] = self.create_subscription(
                    Odometry, name, self._make_dr_cb(user), RELIABLE_QOS
                )
                self.get_logger().info(f'Discovered A4.1 (DR) topic: {name}')
                continue

            m = EKF_ODOM_RE.match(name)
            if m and 'nav_msgs/msg/Odometry' in types:
                user = m.group(1)
                if user in RESERVED_USERS or user in self._ekf_odom_subs:
                    continue
                self._ekf_odom[user] = OdomState()
                self._ekf_odom_subs[user] = self.create_subscription(
                    Odometry, name, self._make_ekf_cb(user), RELIABLE_QOS
                )
                self.get_logger().info(f'Discovered A4.2 (EKF) topic: {name}')

    def _make_dr_cb(self, user: str):
        def _cb(msg: Odometry) -> None:
            state = self._dr_odom[user]
            state.last_msg = msg
            state.message_count += 1
        return _cb

    def _make_ekf_cb(self, user: str):
        def _cb(msg: Odometry) -> None:
            state = self._ekf_odom[user]
            state.last_msg = msg
            state.message_count += 1
        return _cb

    def _grade(self) -> None:
        for user, state in self._dr_odom.items():
            if state.message_count == 0:
                continue
            x = state.last_msg.pose.pose.position.x
            y = state.last_msg.pose.pose.position.y
            v = state.last_msg.twist.twist.linear.x
            self._publish_feedback(
                user,
                'A4.1',
                f'DR: pos=({x:.2f}, {y:.2f}) v={v:.2f}',
            )

        for user, state in self._ekf_odom.items():
            if state.message_count == 0:
                continue
            x = state.last_msg.pose.pose.position.x
            y = state.last_msg.pose.pose.position.y
            v = state.last_msg.twist.twist.linear.x
            self._publish_feedback(
                user,
                'A4.2',
                f'EKF: pos=({x:.2f}, {y:.2f}) v={v:.2f}',
            )

    def _publish_feedback(self, user: str, task: str, detail: str) -> None:
        text = f'{user} {task}: {detail}'
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)
        self.get_logger().info(text)


def main():
    rclpy.init()
    node = Grader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
