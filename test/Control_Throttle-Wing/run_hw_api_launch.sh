#!/usr/bin/env bash
set -e

# Source the user environment, including load_MRS and Python setup
# shellcheck disable=SC1090
source ~/.bashrc >/dev/null 2>&1 || true

if command -v load_MRS >/dev/null 2>&1; then
  load_MRS
else
  echo "load_MRS function not found in ~/.bashrc" >&2
  echo "Sourcing /opt/ros/jazzy/setup.bash and workspace install/setup.bash as fallback" >&2
  source /opt/ros/jazzy/setup.bash
  source /home/jakub-mastnik/ros2_ws/install/setup.bash
fi

export UAV_NAME=uav1

ros2 launch mrs_uav_px4_api api.launch.py \
  simulation:=false \
  fcu_url:=/dev/ttyACM0:2000000 \
  gcs_url:=udp://@127.0.0.1:14550