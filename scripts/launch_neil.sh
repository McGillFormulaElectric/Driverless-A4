#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# A4 — Neil's side launcher
# Starts the figure-8 sensor sim (IMU + GPS + ground truth) + grader.
# New members subscribe to /neil/imu, /neil/gps, /neil/truth and publish
# their EKF estimates. The grader scores them against ground truth.
#
# Usage:  bash scripts/launch_neil.sh [--build]
# ---------------------------------------------------------------------------
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE="docker compose -f $REPO_ROOT/docker/docker-compose.yml"

if [[ "${1:-}" == "--build" ]]; then
  echo "[neil] Building image..."
  $COMPOSE build neil
fi

echo "[neil] Starting A4 sensor sim + grader..."
echo "[neil] Publishing: /neil/imu (50 Hz), /neil/gps (5 Hz), /neil/truth (50 Hz)"
echo "[neil] Feedback → /neil/feedback"
echo ""

$COMPOSE run --rm neil bash -c "
  source /opt/ros/humble/setup.bash
  cd /workspace
  if [ -f install/setup.bash ]; then source install/setup.bash; fi
  colcon build --packages-select a4_neil --symlink-install --quiet
  source install/setup.bash
  echo '[neil] Launching sensor_sim_node + grader...'
  ros2 launch a4_neil neil.launch.py
"
