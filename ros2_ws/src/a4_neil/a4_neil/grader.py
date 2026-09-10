"""Auto-discovery grader for MFE A4.

Buffers /neil/truth and matches user streams by *header timestamp*:
  - /<user>/odom_dr  (nav_msgs/Odometry)  -> A4.1 (dead-reckoning warm-up)
  - /<user>/odom     (nav_msgs/Odometry)  -> A4.2 (EKF)

Rules:
  - A4.1: RMSE over samples whose header stamp is within the first
    `dr_window_s` of sim time. Threshold: RMSE < `dr_rmse_threshold` m.
    Verdict is latched once the window is full so it does not flip once
    new samples fall outside the window.
  - A4.2: RMSE over the last `match_window` samples. Threshold:
    RMSE < `ekf_rmse_threshold` m sustained.

Feedback is published on /neil/feedback (std_msgs/String):
    'Congrats <user>, the answer is correct (<metric>)'
    'Sorry <user>, the answer is incorrect (<metric>)'
Only republished on state changes.

All thresholds and periods are ROS parameters — see config/params.yaml.
"""
from __future__ import annotations

import re
from collections import deque
from dataclasses import dataclass, field
from typing import Deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

TRUTH_BUFFER = 2000            # 40 s of truth at 50 Hz
DR_MIN_SAMPLES = 100           # need >= 100 samples inside the window before grading
EKF_MIN_SAMPLES = 100

# Reserved usernames — Neil's own nodes must never be graded as a submission.
RESERVED_USERS = {'neil'}

DR_RE = re.compile(r'^/([^/]+)/odom_dr$')
EKF_RE = re.compile(r'^/([^/]+)/odom$')


def _is_reserved_user(user: str) -> bool:
    """Skip anything starting with `neil` (spec: skip `neil` / `neil_*`)."""
    return user.startswith('neil') or user in RESERVED_USERS


@dataclass
class StreamState:
    # (t_stamp_seconds, x, y)
    samples: Deque[tuple[float, float, float]] = field(default_factory=deque)
    last_verdict: str | None = None
    verdict_latched: bool = False


class Grader(Node):
    def __init__(self):
        super().__init__('grader')

        # --- ROS parameters (see a4_neil/config/params.yaml) ----------------
        self.declare_parameter('dr_rmse_threshold', 5.0)
        self.declare_parameter('dr_window_s', 10.0)
        self.declare_parameter('ekf_rmse_threshold', 0.5)
        self.declare_parameter('match_window', 200)
        self.declare_parameter('discovery_period_s', 2.0)
        self.declare_parameter('grade_period_s', 2.0)

        self._dr_threshold = float(self.get_parameter('dr_rmse_threshold').value)
        self._dr_window_s = float(self.get_parameter('dr_window_s').value)
        self._ekf_threshold = float(self.get_parameter('ekf_rmse_threshold').value)
        self._ekf_match_window = int(self.get_parameter('match_window').value)
        discovery_period_s = float(self.get_parameter('discovery_period_s').value)
        grade_period_s = float(self.get_parameter('grade_period_s').value)

        self.feedback_pub = self.create_publisher(String, '/neil/feedback', RELIABLE_QOS)

        # Truth ring buffer: sorted by header stamp seconds.
        self._truth_t: Deque[float] = deque(maxlen=TRUTH_BUFFER)
        self._truth_x: Deque[float] = deque(maxlen=TRUTH_BUFFER)
        self._truth_y: Deque[float] = deque(maxlen=TRUTH_BUFFER)
        self.create_subscription(Odometry, '/neil/truth', self._on_truth, RELIABLE_QOS)

        # Per-user streams.
        self._dr: dict[str, StreamState] = {}
        self._dr_subs: dict[str, object] = {}
        self._ekf: dict[str, StreamState] = {}
        self._ekf_subs: dict[str, object] = {}

        self.create_timer(discovery_period_s, self._discover)
        self.create_timer(grade_period_s, self._grade_all)

        self.get_logger().info(
            f'Grader running. DR_threshold={self._dr_threshold} m over first '
            f'{self._dr_window_s} s, EKF_threshold={self._ekf_threshold} m over '
            f'last {self._ekf_match_window} samples.'
        )

    # --- truth --------------------------------------------------------------
    def _on_truth(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._truth_t.append(t)
        self._truth_x.append(float(msg.pose.pose.position.x))
        self._truth_y.append(float(msg.pose.pose.position.y))

    # --- discovery ----------------------------------------------------------
    def _discover(self) -> None:
        for name, types in self.get_topic_names_and_types():
            if 'nav_msgs/msg/Odometry' not in types:
                continue

            m = DR_RE.match(name)
            if m:
                user = m.group(1)
                if _is_reserved_user(user) or user in self._dr_subs:
                    continue
                self._dr[user] = StreamState(samples=deque(maxlen=2000))
                self._dr_subs[user] = self.create_subscription(
                    Odometry, name, self._make_dr_cb(user), RELIABLE_QOS
                )
                self.get_logger().info(f'Discovered A4.1 topic: {name}')
                continue

            m = EKF_RE.match(name)
            if m:
                user = m.group(1)
                if _is_reserved_user(user) or user in self._ekf_subs:
                    continue
                self._ekf[user] = StreamState(samples=deque(maxlen=self._ekf_match_window))
                self._ekf_subs[user] = self.create_subscription(
                    Odometry, name, self._make_ekf_cb(user), RELIABLE_QOS
                )
                self.get_logger().info(f'Discovered A4.2 topic: {name}')

    # --- callback factories -------------------------------------------------
    def _make_dr_cb(self, user: str):
        state = self._dr[user]

        def _cb(msg: Odometry) -> None:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            state.samples.append((t, float(msg.pose.pose.position.x),
                                  float(msg.pose.pose.position.y)))
        return _cb

    def _make_ekf_cb(self, user: str):
        state = self._ekf[user]

        def _cb(msg: Odometry) -> None:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            state.samples.append((t, float(msg.pose.pose.position.x),
                                  float(msg.pose.pose.position.y)))
        return _cb

    # --- grading ------------------------------------------------------------
    def _truth_arrays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        return (np.asarray(self._truth_t),
                np.asarray(self._truth_x),
                np.asarray(self._truth_y))

    def _match_truth(self, stu_t: np.ndarray, tr_t: np.ndarray,
                     tr_x: np.ndarray, tr_y: np.ndarray
                     ) -> tuple[np.ndarray, np.ndarray]:
        """Nearest-header-stamp match. Truth is dense (50 Hz), so np.searchsorted is fine."""
        idx = np.searchsorted(tr_t, stu_t)
        idx = np.clip(idx, 1, len(tr_t) - 1)
        left = tr_t[idx - 1]
        right = tr_t[idx]
        pick_left = np.abs(stu_t - left) <= np.abs(stu_t - right)
        chosen = np.where(pick_left, idx - 1, idx)
        return tr_x[chosen], tr_y[chosen]

    def _grade_all(self) -> None:
        if len(self._truth_t) < 20:
            return
        tr_t, tr_x, tr_y = self._truth_arrays()
        t0 = tr_t[0]

        # A4.1 dead-reckoning grading.
        for user, state in self._dr.items():
            if state.verdict_latched:
                continue
            if not state.samples:
                continue
            arr = np.asarray(state.samples)
            stu_t, stu_x, stu_y = arr[:, 0], arr[:, 1], arr[:, 2]
            mask = (stu_t - t0) <= self._dr_window_s
            if mask.sum() < DR_MIN_SAMPLES:
                continue
            stu_t_w = stu_t[mask]
            stu_x_w = stu_x[mask]
            stu_y_w = stu_y[mask]

            m_x, m_y = self._match_truth(stu_t_w, tr_t, tr_x, tr_y)
            rmse = float(np.sqrt(np.mean((stu_x_w - m_x) ** 2 + (stu_y_w - m_y) ** 2)))
            verdict = 'correct' if rmse < self._dr_threshold else 'incorrect'
            # Latch once the window is complete enough that verdict is stable.
            window_complete = (stu_t_w[-1] - t0) >= self._dr_window_s
            if verdict != state.last_verdict:
                state.last_verdict = verdict
                self._publish_feedback(user, verdict, extra=f'(RMSE_DR={rmse:.3f} m)')
            if window_complete:
                state.verdict_latched = True

        # A4.2 EKF grading.
        for user, state in self._ekf.items():
            if len(state.samples) < EKF_MIN_SAMPLES:
                continue
            arr = np.asarray(state.samples)
            stu_t, stu_x, stu_y = arr[:, 0], arr[:, 1], arr[:, 2]
            m_x, m_y = self._match_truth(stu_t, tr_t, tr_x, tr_y)
            rmse = float(np.sqrt(np.mean((stu_x - m_x) ** 2 + (stu_y - m_y) ** 2)))
            verdict = 'correct' if rmse < self._ekf_threshold else 'incorrect'
            if verdict != state.last_verdict:
                state.last_verdict = verdict
                self._publish_feedback(user, verdict, extra=f'(RMSE_EKF={rmse:.3f} m)')

    # --- feedback -----------------------------------------------------------
    def _publish_feedback(self, user: str, verdict: str, extra: str = '') -> None:
        if verdict == 'correct':
            text = f'Congrats {user}, the answer is correct'
        else:
            text = f'Sorry {user}, the answer is incorrect'
        if extra:
            text = f'{text} {extra}'
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
