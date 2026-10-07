# MFE Driverless — Assignment 4: Odometry & State Estimation

This assignment introduces odometry integration and sensor fusion via Extended Kalman Filter (EKF). You'll implement two complementary state estimators:

- **A4.1** — **Dead Reckoning**: integrate wheel speed measurements to track vehicle position and orientation.
- **A4.2** — **2D EKF**: fuse dead reckoning with noisy GPS measurements to estimate vehicle state (x, y, θ, v).

Both estimators run alongside a grader that auto-discovers your topics and publishes feedback on `/grader/feedback`.

> **Grading Setup** — The grader and both student nodes run in one local Docker container on ROS domain ID 42. No remote grader or host networking is required.

---

## 1. Getting started

### 1.1 GitHub
1. Go to this repo on GitHub.
2. Create a new branch named after you, e.g. `NeilJoeGeorge`.
3. Clone and check out your branch:

```bash
git clone <repo-url>
cd Driverless-AA4
git checkout <FirstNameLastName>
```

### 1.2 Docker & Local Grading
The grader, dead-reckoning node, and EKF run in one Docker container on the same local ROS graph. The container uses ROS domain ID 42 and restricts discovery to its local network namespace.

```bash
cd docker
docker compose -f docker-compose-local.yml build
docker compose -f docker-compose-local.yml up -d
```

This starts one service, `a4`, which builds the workspace and launches the local grader, dead-reckoning node, and EKF with the `student` namespace.

Inside the container, the workspace is mounted at `/workspace` (your `ros2_ws`). Build and source:

```bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
```

View logs from either service:
```bash
docker compose -f docker-compose-local.yml logs -f a4       # grader and student logs
```

Stop everything:
```bash
docker compose -f docker-compose-local.yml down
```

---

## 2. A4.1 — Dead Reckoning Odometry

**Goal:** integrate wheel speed measurements to track vehicle position and heading.

The grader publishes IMU measurements (yaw rate and acceleration) on `/grader/imu` and wheel speeds on `/grader/wheel_speeds`. Your job is to:
1. Subscribe to both topics.
2. Integrate to estimate position **(x, y)** and heading **θ** over time.
3. Publish your odometry estimate on `/${GITHUB_USER}/dr_odom` (`nav_msgs/Odometry`).

### Implementation details
Open `ros2_ws/src/a4_new_member/a4_new_member/dead_reckoning_node.py`. There's a `TODO` block where you implement:
- Kinematic model integration using wheel speeds and yaw rate.
- State propagation with basic Euler integration.
- Publishing to the `/dr_odom` topic in your namespace.

### Run it
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch a4_new_member dr.launch.py github_user:=$GITHUB_USER
```

Watch the grader feedback:
```bash
ros2 topic echo /grader/feedback
```

Grader feedback will confirm your DR odometry is correct.

---

## 3. A4.2 — Extended Kalman Filter (EKF) for Pose Estimation

Build a 2D Extended Kalman Filter to fuse dead reckoning with noisy GPS measurements. Your EKF will estimate vehicle state **[x, y, θ, v]** with uncertainty.

### The mathematical model

**State vector:** 
```
x = [px, py, theta, vx, vy]ᵀ
```
where `px, py` = position, `theta` = heading, `vx, vy` = velocity components.

**Prediction step (IMU-driven kinematic model):**

Given yaw rate `ω` (from gyro) and forward acceleration `a` (from accelerometer):
```
px[k+1] = px[k] + Δt · vx[k]
py[k+1] = py[k] + Δt · vy[k]
theta[k+1] = theta[k] + Δt · ω[k]
vx[k+1] = vx[k] + Δt · a[k] · cos(theta[k])
vy[k+1] = vy[k] + Δt · a[k] · sin(theta[k])

P[k+1|k] = F[k] · P[k|k] · F[k]ᵀ + Q
```
where `F` is the Jacobian of the motion model, `P` is state covariance, and `Q` is process noise.

**Update step (GPS correction):**

GPS provides noisy measurements `z = [px_gps, py_gps]ᵀ`:
```
z = H · x + v,  where v ~ N(0, R)

y = z - H · x[k|k-1]           # innovation
S = H · P[k|k-1] · Hᵀ + R      # innovation covariance
K = P[k|k-1] · Hᵀ · S⁻¹        # Kalman gain
x[k|k] = x[k|k-1] + K · y      # updated state
P[k|k] = (I - K·H) · P[k|k-1]  # updated covariance
```
where `H = [1 0 0 0 0; 0 1 0 0 0]` (measure only position), and `R` is measurement noise covariance.

### Implementation
Open `ros2_ws/src/a4_new_member/a4_new_member/ekf_node.py`. There's a `TODO` block where you implement:
1. **`predict()`** — propagate state and covariance using IMU measurements. Compute Jacobian `F` and update `P`.
2. **`update_gps()`** — Kalman filter update step with GPS measurements. Compute gain `K` and fused state.
3. Publish fused odometry to `/${GITHUB_USER}/odom` (`nav_msgs/Odometry`) with covariance.

**Key parameters** (in `config/params.yaml`):
- `Q` — process noise covariance (IMU uncertainty)
- `R` — measurement noise covariance (GPS uncertainty)
- `P0` — initial state covariance

### Run it
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch a4_new_member ekf.launch.py github_user:=$GITHUB_USER
```

Watch the grader feedback:
```bash
ros2 topic echo /grader/feedback
```

The grader compares your EKF estimate against ground truth and publishes position, heading, and velocity feedback.

Grader feedback will show position, heading, and velocity estimates.

---

## 4. Topic contract (summary)

| Topic              | Type              | Owner   | Purpose                            |
| ------------------ | ----------------- | ------- | ---------------------------------- |
| `/grader/imu`       | `sensor_msgs/Imu` | Neil    | Gyro (yaw rate) + accel for A4.1   |
| `/grader/wheel_speeds` | Custom msg     | Neil    | Left/right wheel speeds for A4.1   |
| `/grader/gps`       | `sensor_msgs/NavSatFix` | Neil | Noisy GPS position for A4.2        |
| `/grader/feedback`  | `std_msgs/String` | Neil    | Per-student grading verdict        |
| `/<user>/dr_odom`   | `nav_msgs/Odometry` | Student | A4.1 dead-reckoning output        |
| `/<user>/odom`      | `nav_msgs/Odometry` | Student | A4.2 EKF-fused output              |

---

## 5. Layout

```
Driverless-A4/
├── docker/
│   ├── docker-compose-local.yml  # Single container: grader + student
│   ├── Dockerfile                # ROS2 Humble + dependencies
│   └── (entrypoint script)
├── ros2_ws/
│   └── src/
│       ├── a4_new_member/        # your template — write code here
│       │   ├── a4_new_member/
│       │   │   ├── dead_reckoning_node.py  # A4.1
│       │   │   └── ekf_node.py             # A4.2
│       │   ├── launch/
│       │   │   ├── dr.launch.py
│       │   │   └── ekf.launch.py
│       │   ├── config/params.yaml
│       │   ├── setup.py
│       │   └── package.xml
│       └── a4_grader/            # grader (runs in same container)
│           ├── a4_grader/grader.py
│           ├── config/params.yaml
│           ├── launch/grader.launch.py
│           ├── setup.py
│           └── package.xml
└── README.md
```

## 6. Troubleshooting

- **`ros2 topic list` doesn't show `/grader/imu` or `/grader/gps`.** Ensure both grader and student are running in the container. Check `docker compose logs`.
- **Topics visible but grader shows no feedback.** Verify your odometry is publishing to the right topic (`/<user>/dr_odom` for A4.1, `/<user>/odom` for A4.2).
- **EKF diverges or blows up.** Check that `P0` (initial covariance) and process noise `Q` are reasonable. Start with moderate values if unsure.

---

## 7. Submission Checklist
- [ ] `dead_reckoning_node.py` implements kinematic integration.
- [ ] `ekf_node.py` implements predict and update steps.
- [ ] Both nodes publish to correct topics with correct types.
- [ ] Grader feedback is positive for both A4.1 and A4.2.
- [ ] Code is committed to your branch.

## 8. Parameters

All constants are exposed as ROS parameters in YAML files:

- **Grader** [`ros2_ws/src/a4_grader/config/params.yaml`](ros2_ws/src/a4_grader/config/params.yaml):
  - IMU/GPS noise levels
  - Ground truth trajectory parameters
  - Grading tolerances
  - Discovery and feedback periods

- **Student** [`ros2_ws/src/a4_new_member/config/params.yaml`](ros2_ws/src/a4_new_member/config/params.yaml):
  - Initial covariance `P0`
  - Process noise matrix `Q` (IMU uncertainty)
  - Measurement noise matrix `R` (GPS uncertainty)
  - Wheel base / sensor calibration constants

You should not edit grader parameters. For student parameters, start with suggested values and tune if needed for your implementation. The launch files load these automatically.

---

## 🔴 NEIL REFERENCE

**To view complete solutions for this assignment:**

```bash
# View the reference implementation on the solution branch
git checkout solution/a4-dr-ekf

# Or clone directly from the solution branch for testing
git clone -b solution/a4-dr-ekf <repo-url>
```

**Solution Branch Reference:** [`solution/a4-dr-ekf`](https://github.com/McGillFormulaElectric/Driverless-A4/tree/solution/a4-dr-ekf)

**Pull Request:** [PR #1 - A4 Solution](https://github.com/McGillFormulaElectric/Driverless-A4/pull/1)
