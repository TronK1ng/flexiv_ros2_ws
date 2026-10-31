#!/usr/bin/env bash
# Headless M0 smoke test: run the whole stack with fake hardware and a mock
# detection, send one pick-and-place goal and report the result.
#
# Usage: test/smoke_fake_hardware.sh [robot_sn]
# Exit code 0 means the action server reported success.

set +u   # ROS setup scripts use unbound variables

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROBOT_SN="${1:-Rizon4s-064230}"
LOG="${WS}/.ros_home/m0.launch.log"
GOAL_LOG="${WS}/.ros_home/m0.goal.log"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"

export ROS_LOG_DIR="${WS}/.ros_home/log"
export ROS_HOME="${WS}/.ros_home"
# Cyclone DDS avoids the FastDDS shared-memory transport, which does not survive
# every sandbox / container setup.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# Keep the test away from any demo stack the user already has running.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

mkdir -p "${WS}/.ros_home/log"

echo "== launching the demo stack (fake hardware, mock detection)"
ros2 launch flexiv_grasp_demo grasp_demo.launch.py \
    robot_sn:="${ROBOT_SN}" \
    use_fake_hardware:=true \
    detect_mode:=mock \
    start_rviz:=false \
    start_camera:=false \
    publish_camera_tf:=false >"${LOG}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
    kill "${LAUNCH_PID}" 2>/dev/null
    wait "${LAUNCH_PID}" 2>/dev/null
}
trap cleanup EXIT

echo "== waiting for /grasp_demo/pick_place"
for _ in $(seq 1 90); do
    if ros2 action list 2>/dev/null | grep -q "grasp_demo/pick_place"; then
        break
    fi
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo "launch process died, see ${LOG}"
        tail -40 "${LOG}"
        exit 1
    fi
    sleep 2
done

if ! ros2 action list 2>/dev/null | grep -q "grasp_demo/pick_place"; then
    echo "action server never appeared, see ${LOG}"
    tail -40 "${LOG}"
    exit 1
fi

echo "== sending the goal"
timeout 180 ros2 action send_goal /grasp_demo/pick_place \
    flexiv_grasp_demo/action/PickPlace \
    "{object_id: '7', grasp_force: 0.0, approach_height: 0.0}" \
    --feedback 2>&1 | tee "${GOAL_LOG}"

if grep -q "success: true\|success=True" "${GOAL_LOG}"; then
    echo "== M0 smoke test PASSED"
    exit 0
fi

echo "== M0 smoke test FAILED"
exit 1
