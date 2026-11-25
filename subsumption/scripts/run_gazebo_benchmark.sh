#!/usr/bin/env bash
# Headless Gazebo verification & trajectory benchmark runner (run inside ros2_humble).
set -eo pipefail

unset DISPLAY
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source /mobile_robotics_ws/install/setup.bash
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="/opt/ros/humble/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH}"

OUT_DIR="/mobile_robotics_ws/subsumption_results"
mkdir -p "${OUT_DIR}"

cleanup_pids() {
  for p in "$@"; do
    if [ -n "${p}" ] && kill -0 "${p}" 2>/dev/null; then
      kill -INT "${p}" 2>/dev/null || true
    fi
  done
  sleep 2
  pkill -9 -x gzserver 2>/dev/null || true
  pkill -9 -x robot_state_publisher 2>/dev/null || true
  pkill -9 -f "turtlebot3_subsumption" 2>/dev/null || true
  sleep 1
}

run_case() {
  local NAME=$1
  local LAUNCH_FILE=$2
  local X_POSE=$3
  local Y_POSE=$4
  local DUR=$5
  local CSV_PATH="${OUT_DIR}/${NAME}.csv"
  local LOG_PATH="${OUT_DIR}/${NAME}.log"

  echo "=========================================="
  echo "Running ${NAME}: ${LAUNCH_FILE} @ (${X_POSE}, ${Y_POSE}) for ${DUR}s"
  pkill -9 -x gzserver 2>/dev/null || true
  pkill -9 -x robot_state_publisher 2>/dev/null || true
  sleep 1

  ros2 launch gazebo_ros gzserver.launch.py \
    world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world \
    >"/tmp/gz_${NAME}.log" 2>&1 &
  local GZ_PID=$!

  for i in $(seq 1 20); do
    if ros2 service list 2>/dev/null | grep -q "/spawn_entity"; then
      break
    fi
    sleep 1
  done

  ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py \
    use_sim_time:=true \
    >"/tmp/rsp_${NAME}.log" 2>&1 &
  local RSP_PID=$!
  sleep 1

  ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py \
    x_pose:="${X_POSE}" y_pose:="${Y_POSE}" \
    >"/tmp/spawn_${NAME}.log" 2>&1
  sleep 2

  ros2 launch turtlebot3_subsumption "${LAUNCH_FILE}" \
    use_sim_time:=true \
    record_trajectory:=true \
    output_csv:="${CSV_PATH}" \
    >"${LOG_PATH}" 2>&1 &
  local LPID=$!

  sleep "${DUR}"
  cleanup_pids "${LPID}" "${RSP_PID}" "${GZ_PID}"
  echo "Finished ${NAME}: $(wc -l < "${CSV_PATH}" 2>/dev/null || echo 0) rows logged."
}

run_case "level0_open" "level0_avoid.launch.py" "-2.0" "-0.5" 12
run_case "level0_near_obstacle" "level0_avoid.launch.py" "-1.44" "-1.1" 15
run_case "level1_wander" "level1_wander.launch.py" "-2.0" "-0.5" 55
run_case "level2_explore" "level2_explore.launch.py" "-2.0" "-0.5" 55

echo "ALL GAZEBO TESTS COMPLETED"
