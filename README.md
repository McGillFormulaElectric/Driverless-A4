# MFE Driverless — Assignment 4: 2D pose estimation with an EKF

![CI](https://github.com/McGillFormulaElectric/Driverless-A4/actions/workflows/ci.yml/badge.svg)

This assignment teaches **sensor fusion for vehicle pose estimation**: you take a noisy IMU and a noisy GPS, and you produce a clean estimate of where the car is and where it is pointing. It is split into two parts, [Advent-of-Code style](https://adventofcode.com/): Part 1 is a warm-up, Part 2 is the real challenge.

- **A4.1** — dead-reckoning from IMU only. Watch it drift. This is the "why we need fusion" demo.
- **A4.2** — implement a 2D Extended Kalman Filter that fuses IMU (predict) with GPS (update) and stays within 0.5 m RMSE of the truth.

Neil's node drives a virtual vehicle around a figure-8-shaped trajectory (two lobes of radius ~8 m) and publishes noisy IMU + noisy GPS + noise-free ground truth. Your job is to publish an odometry estimate that Neil's grader can compare against truth. The grader auto-discovers your topics on the shared class network and posts pass/fail verdicts on `/neil/feedback`.

Everything runs inside a Docker container so your local OS and Python version don't matter.

---

## 1. Getting started

### 1.1 GitHub
1. Go to this repo on GitHub.
2. Create a new branch named after you, e.g. `NeilJoeGeorge`.
3. Clone and check out your branch:

```bash
git clone <repo-url>
cd Driverless-A4
git checkout <FirstNameLastName>
```

### 1.2 Tailscale (class VPN)
Neil runs a ROS 2 node on the class Tailscale network. Every student joins the same tailnet so DDS discovery works between machines.

1. Install Tailscale: <https://tailscale.com/download>.
2. `sudo tailscale up` and sign in with the invite Neil sent.
3. Verify you can reach Neil's node: `tailscale ping neil`.
4. Note your own Tailscale hostname/IP — you'll set it via env var below if auto-detection fails.

### 1.3 Docker
Linux host with Docker + Docker Compose is the supported path (host networking + Tailscale interface work cleanly).

```bash
cd docker
export GITHUB_USER=<your-github-handle>          # required
export A4_NEIL_HOST=<neil tailnet host>          # e.g. neil.tail1234.ts.net
docker compose build
docker compose run --rm new_member
```

Inside the container you'll have `/workspace` mounted to `ros2_ws/`. Build and source:

```bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
```

> **macOS/Windows caveat:** Docker Desktop's `network_mode: host` is limited. If you're not on Linux, run the container with `--network host` on a Linux VM, or use Tailscale's [userspace networking mode](https://tailscale.com/kb/1112/userspace-networking) inside the container. Ask Neil for the current recommendation.

---

## 2. A4.1 — Dead-reckoning warm-up

**Goal:** integrate the IMU to publish an estimated pose on `/${GITHUB_USER}/odom_dr` (`nav_msgs/Odometry`, frame `map`). Then watch it drift.

### 2.1 The physics
The vehicle starts **at rest at the origin with heading = 0**. The IMU publishes body-frame forward acceleration and yaw rate:

```
a_body_x = msg.linear_acceleration.x   # forward accel, m/s^2
omega_z  = msg.angular_velocity.z      # yaw rate, rad/s
```

Semi-implicit Euler integration:

```
theta_new = theta + omega_z * dt
v_new     = v + a_body_x * dt
x_new     = x + v_new * cos(theta_new) * dt
y_new     = y + v_new * sin(theta_new) * dt
```

`dt` should come from the difference between consecutive IMU header timestamps — do **not** use wall-clock time, since Neil stamps everything with simulated time (this is what makes grading reproducible).

### 2.2 Where to put your code
Open `ros2_ws/src/a4_new_member/a4_new_member/dead_reckoning_node.py`. There's a `TODO(student)` block inside `_on_imu`. Replace the stub with the integration above.

### 2.3 Run it
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch a4_new_member dr.launch.py github_user:=$GITHUB_USER
```

### 2.4 How A4.1 grading works
Pure dead-reckoning drifts fast because you are double-integrating IMU noise, so the grader only scores your stream over the **first 10 s of sim time** and passes at RMSE < 5.0 m. That is generous by design — if your integration is correct you will comfortably pass. If your integration is wrong (e.g. you forgot to project velocity onto heading, or you're using wall-clock instead of header stamps), you will fail even the 10 s window.

**Deliverable for A4.1:** screenshot of `/neil/feedback` congratulating your handle, committed as `submissions/a4_1_feedback.png`, plus your finished `dead_reckoning_node.py`.

---

## 3. A4.2 — 2D Extended Kalman Filter

**Goal:** publish a fused pose estimate on `/${GITHUB_USER}/odom` (`nav_msgs/Odometry`, frame `map`) that stays within **0.5 m RMSE** of the ground truth over any recent 200-sample window (~4 s at 50 Hz).

### 3.1 State and models
State vector:

```
x = [px, py, theta, v]^T
```

Non-linear motion model (used in **predict**, driven by IMU at 50 Hz):

```
px'    = px + v * cos(theta) * dt
py'    = py + v * sin(theta) * dt
theta' = theta + omega_z * dt
v'     = v + a_body_x * dt
```

Motion Jacobian evaluated at the current estimate:

```
F = [[1, 0, -v*sin(theta)*dt, cos(theta)*dt],
     [0, 1,  v*cos(theta)*dt, sin(theta)*dt],
     [0, 0,  1,               0            ],
     [0, 0,  0,               1            ]]
```

Measurement model (used in **update**, driven by GPS at 5 Hz):

```
z = [px_meas, py_meas]^T
H = [[1, 0, 0, 0],
     [0, 1, 0, 0]]
```

### 3.2 Sensor noise (matches the sim; use these to seed R)
| Sensor            | Field                    | Sigma      |
| ----------------- | ------------------------ | ---------- |
| IMU forward accel | `linear_acceleration.x`  | 0.2 m/s²   |
| IMU yaw rate      | `angular_velocity.z`     | 0.02 rad/s |
| GPS position      | `pose.position.{x,y}`    | 0.5 m each |

A reasonable starting point for the EKF tuning constants (these are the defaults in `a4_new_member/config/params.yaml`; feel free to tune):

```
Q = diag([0.01, 0.01, 0.001, 0.1])     # process noise
R = diag([0.5**2, 0.5**2])             # measurement noise, matches GPS sigma
P0 = diag([0.01, 0.01, 0.001, 0.01])   # small: we know we start at rest at origin
```

### 3.3 Where to put your code
Open `ros2_ws/src/a4_new_member/a4_new_member/ekf_node.py`. There is a plain-numpy `EKF` class with `predict()` and `update_gps()`. The ROS node calls them for you; you just have to fill in the math.

### 3.4 Run it
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch a4_new_member ekf.launch.py github_user:=$GITHUB_USER
```

### 3.5 How A4.2 grading works
The grader:
1. Buffers `/neil/truth`.
2. Discovers any `/<user>/odom` topic on the network and buffers the last 200 samples per student.
3. Matches each student sample to truth by **nearest header stamp** (both are on simulated time so this is exact).
4. Computes 2D position RMSE. If `RMSE < 0.5 m` it publishes on `/neil/feedback`:
   ```
   Congrats <your-github-user>, the answer is correct (RMSE_EKF=0.312 m)
   ```
   Otherwise:
   ```
   Sorry <your-github-user>, the answer is incorrect (RMSE_EKF=1.847 m)
   ```

Feedback is only republished when your verdict flips, so if your filter is wrong you'll see one "incorrect" message per attempt.

### 3.6 Theory refs
- Wikipedia — [Extended Kalman filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter).
- Thrun, Burgard, Fox — *Probabilistic Robotics*, Chapter 3 (Gaussian filters). The EKF derivation in §3.3 is exactly what you'll implement.
- Roger Labbe — [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python), free notebooks. Chapter 11 is the EKF.

**Deliverable for A4.2:** screenshot of `/neil/feedback` congratulating your handle, committed as `submissions/a4_2_feedback.png`, plus your finished `ekf_node.py`.

---

## 4. Visualizing with Foxglove Studio (optional but strongly recommended)

Seeing your estimate, the noisy GPS, and the ground truth on the same 2D plot makes it obvious when your filter is off.

### 4.1 Install the Foxglove bridge inside the container
Already available in the image:
```bash
ros2 run foxglove_bridge foxglove_bridge port:=8765
```
Leave that terminal running.

### 4.2 Install Foxglove Studio on your host
Download from <https://foxglove.dev/download> (free, works on Linux/macOS/Windows).

### 4.3 Connect
1. Open Foxglove Studio → **Open connection…** → **Foxglove WebSocket**.
2. URL: `ws://localhost:8765` (or `ws://<your-tailscale-host>:8765` from another machine on the tailnet).
3. Add a **3D** panel and enable these topics:
   - `/neil/truth` (green — ground truth path).
   - `/neil/gps` (red dots — noisy GPS).
   - `/${GITHUB_USER}/odom_dr` (yellow — drifting DR estimate).
   - `/${GITHUB_USER}/odom` (blue — your EKF estimate).
4. Add a **Raw Messages** panel on `/neil/feedback` to see verdicts as they arrive.

You should see the yellow (DR) trace wander off within a few seconds while the blue (EKF) trace stays glued to the green (truth) trace. That is your filter working.

> Tip: save your Foxglove layout to `submissions/a4_layout.json` (**Layout → Export**) so future assignments can reuse it.

---

## 5. Topic contract (summary)

| Topic                    | Type                        | Owner   | Purpose                                         |
| ------------------------ | --------------------------- | ------- | ----------------------------------------------- |
| `/neil/imu`              | `sensor_msgs/Imu`           | Neil    | Noisy IMU (50 Hz)                               |
| `/neil/gps`              | `geometry_msgs/PoseStamped` | Neil    | Noisy GPS (5 Hz)                                |
| `/neil/truth`            | `nav_msgs/Odometry`         | Neil    | Noise-free ground truth (50 Hz)                 |
| `/neil/feedback`         | `std_msgs/String`           | Neil    | Per-student grading verdict                     |
| `/<user>/odom_dr`        | `nav_msgs/Odometry`         | Student | A4.1 dead-reckoning output                      |
| `/<user>/odom`           | `nav_msgs/Odometry`         | Student | A4.2 EKF output                                 |

All topics use `QoSProfile(reliability=RELIABLE, history=KEEP_LAST, depth=10)`. All frames are `map`. Namespaces are set at launch with `github_user:=<handle>`.

---

## 6. Layout

```
Driverless-A4/
├── .github/workflows/       # CI (colcon build + test on Humble)
├── docker/                  # Dockerfile, compose, CycloneDDS config, entrypoint
├── ros2_ws/
│   └── src/
│       ├── a4_new_member/     # your template — this is where you write code
│       └── a4_neil/         # for reference; not run by students
└── README.md
```

## 7. Troubleshooting

- **`ros2 topic list` doesn't show `/neil/imu`.** DDS discovery isn't reaching Neil. Confirm `tailscale ping <neil-host>` works, `A4_NEIL_HOST` is set, and `ROS_DOMAIN_ID` matches (`42`).
- **You see your own topics but no one else's.** Check `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` inside the container (`env | grep RMW`).
- **DR fails even though the code looks right.** Two common bugs: (1) computing `dt` from `time.time()` instead of `msg.header.stamp` — Neil uses simulated time; (2) updating position with `v_old * cos(theta_new)` when semi-implicit Euler wants `v_new * cos(theta_new)`.
- **EKF passes briefly then diverges.** Your `Q` is probably too small — the filter over-trusts its motion model and ignores GPS corrections. Try increasing the diagonal of `Q`, especially on `v` and `theta`.
- **EKF never converges.** Your covariance update or Jacobian sign is likely wrong. Double-check `F[0, 2] = -v*sin(theta)*dt` (note the minus).

---

## 8. Parameters

The scenario constants (sensor rates, noise sigmas, trajectory shape, grader thresholds, EKF tuning) are exposed as ROS parameters and loaded from YAML at launch time. You should not need to edit the Neil-side file for the graded assignment, but tweaking the new_member-side EKF knobs is exactly how you tune your filter (or, for the graded run, exactly how you keep the defaults sane).

- **Neil side** — [`ros2_ws/src/a4_neil/config/params.yaml`](ros2_ws/src/a4_neil/config/params.yaml)
  - `sensor_sim_node`: `imu_hz`, `gps_hz`, `imu_accel_sigma`, `imu_yaw_rate_sigma`, `gps_pos_sigma`, `seed`, trajectory (`v_ss`, `tau_ramp`, `omega_max`, `traj_period`).
  - `grader`: `dr_rmse_threshold`, `dr_window_s`, `ekf_rmse_threshold`, `match_window`, `discovery_period_s`, `grade_period_s`.
- **Solution side** — [`ros2_ws/src/a4_new_member/config/params.yaml`](ros2_ws/src/a4_new_member/config/params.yaml)
  - EKF tuning: `initial_pos_cov`, `initial_yaw_cov`, `initial_v_cov`, `q_accel`, `q_yaw_rate`, `r_gps`. These are HINTS you can tune — the defaults are a reasonable starting point.

The launch files (`neil.launch.py`, `dr.launch.py`, `ekf.launch.py`, `bringup.launch.py`) pass the YAML file into each node via the `parameters=[...]` argument, so `ros2 launch` picks them up automatically.

---

## 9. Composed launch

If you want to run the whole stack (Neil sensor sim + grader + your DR + your EKF) in one command — useful when hacking offline without Tailscale — use the composed launch:

```bash
ros2 launch a4_new_member bringup.launch.py github_user:=<your-handle>
```

This includes Neil's launch file (unnamespaced, so `/neil/imu`, `/neil/gps`, `/neil/truth`, `/neil/feedback` stay where the grader expects them) and both `dead_reckoning_node` + `ekf_node` under `PushRosNamespace(<your-handle>)`. For the real graded run over Tailscale you should still use `ros2 launch a4_new_member dr.launch.py` / `ekf.launch.py` and let Neil's process own the Neil side.

---

## 10. Submitting via Pull Request

Follow this flow to submit your work:

1. **Commit** your changes to your `FirstNameLastName` branch. Include both feedback screenshots in `submissions/`:
   - `submissions/a4_1_feedback.png` — a screenshot of `/neil/feedback` congratulating your handle for A4.1.
   - `submissions/a4_2_feedback.png` — a screenshot of `/neil/feedback` congratulating your handle for A4.2.
2. **Push** the branch to GitHub:
   ```bash
   git push -u origin <FirstNameLastName>
   ```
3. **Open a pull request** against `main`. Title it exactly:
   ```
   A4 submission — <Your Name>
   ```
4. **Embed both screenshots inline in the PR description** so the reviewer can see the verdicts without cloning:
   ```markdown
   ### A4.1 dead-reckoning
   ![A4.1 feedback](submissions/a4_1_feedback.png)

   ### A4.2 EKF
   ![A4.2 feedback](submissions/a4_2_feedback.png)
   ```
5. **CI must be green.** GitHub Actions builds `a4_neil` and `a4_new_member` on ROS 2 Humble on every push. If the badge at the top of this README is red for your branch, fix the build before requesting review.
6. **Wait for review and merge.** Neil will review your PR, may request changes, and will merge it into `main` once it passes.
