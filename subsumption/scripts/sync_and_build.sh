#!/usr/bin/env bash
set -euo pipefail

SRC_DIR="/home/kresna/Documents/mobile_robotics_ws/subsumption/src/turtlebot3_subsumption"
WS_DEST="/home/kresna/mobile_robotics_ws/src/turtlebot3_subsumption"

echo "[1/3] Syncing turtlebot3_subsumption -> ${WS_DEST}"
mkdir -p "${WS_DEST}"
rsync -a --delete --exclude '__pycache__' --exclude '*.pyc' "${SRC_DIR}/" "${WS_DEST}/"

echo "[2/3] Ensuring Docker container ros2_humble is running"
docker start ros2_humble >/dev/null

echo "[3/3] Building ONLY turtlebot3_subsumption inside ros2_humble"
docker exec ros2_humble bash -c \
  "source /opt/ros/humble/setup.bash && cd /mobile_robotics_ws && colcon build --packages-select turtlebot3_subsumption"

echo "Done! Source /mobile_robotics_ws/install/setup.bash inside ros2_humble to run."
