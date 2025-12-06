#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="${REPO_ROOT}/ros2_bluerov"
SETUP_FILE="${ROS_WS}/install/setup.bash"

if [[ ! -d "${ROS_WS}" ]]; then
  echo "ROS2 workspace not found at ${ROS_WS}" >&2
  exit 1
fi

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "${SETUP_FILE} is missing. Build the workspace first (colcon build)." >&2
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source your ROS2 distro (e.g., /opt/ros/humble/setup.bash)." >&2
  exit 1
fi

launch_teleop() {
  local cmd="cd \"${ROS_WS}\" && source \"${SETUP_FILE}\" && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/bluerov/cmd_vel"
  if command -v terminator >/dev/null 2>&1; then
    terminator -e "bash -lc '${cmd}; exec bash'" &
    echo "Teleop started in terminator with remap /cmd_vel -> /bluerov/cmd_vel."
  else
    echo "Terminator not found. Open a new terminal and run:"
    echo "  bash -lc '${cmd}'"
  fi
}

echo "[mock_pipeline] Sourcing ${SETUP_FILE}"
source "${SETUP_FILE}"

echo "[mock_pipeline] Starting teleop keyboard..."
launch_teleop

echo "[mock_pipeline] Launching mock pipeline (user_pipeline.launch.py)..."
cd "${ROS_WS}"
exec ros2 launch bringup user_pipeline.launch.py
