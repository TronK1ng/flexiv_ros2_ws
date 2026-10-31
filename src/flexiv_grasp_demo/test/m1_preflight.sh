#!/usr/bin/env bash
# Read-only pre-flight checks before the first real-robot motion (M1).
#
# Nothing here commands the robot: it only inspects the running stack, so it is
# safe to run with the arm powered and in Remote mode.
#
# Usage: test/m1_preflight.sh [robot_sn] [ros_domain_id]
# Exit code 0 = every required piece is up.
#
# IMPORTANT: run this from a shell with the same ROS environment as the terminal
# that started the stack (same RMW_IMPLEMENTATION, same ROS_DOMAIN_ID). This
# script deliberately does not pick a middleware for you.

set +u   # ROS setup scripts use unbound variables

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SN="${1:-Rizon4s-064230}"
# ROS 2 topic names cannot contain hyphens; the driver replaces them with underscores.
SN_TOPIC="${SN//-/_}"

if [ -n "${2:-}" ]; then
    export ROS_DOMAIN_ID="$2"
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"

export ROS_LOG_DIR="${WS}/.ros_home/log"
export ROS_HOME="${WS}/.ros_home"
mkdir -p "${ROS_LOG_DIR}"

FAIL=0
WARN=0
ok()      { echo "  [ok]   $*"; }
missing() { echo "  [FAIL] $*"; FAIL=1; }
warn()    { echo "  [warn] $*"; WARN=1; }
note()    { echo "  [info] $*"; }

echo "== robot: ${SN}"
echo "   RMW:    ${RMW_IMPLEMENTATION:-<default: rmw_fastrtps_cpp>}"
echo "   domain: ${ROS_DOMAIN_ID:-<default: 0>}"

# Node discovery can lag a few seconds after a launch; probe twice.
NODES="$(ros2 node list 2>/dev/null)"
has_node() { printf '%s\n' "${NODES}" | grep -qx "$1"; }

if ! has_node /controller_manager && ! has_node /move_group; then
    sleep 5
    NODES="$(ros2 node list 2>/dev/null)"
fi

echo "-- 1. core nodes"
for node in /controller_manager /move_group; do
    if has_node "${node}"; then
        ok "${node}"
    else
        missing "${node} is not running"
    fi
done
if has_node /grasp_task; then
    ok "/grasp_task"
else
    warn "/grasp_task not running yet (start it, then re-run to check the action server)"
fi

if [ "${FAIL}" -ne 0 ] && [ -z "${NODES}" ]; then
    echo
    echo "   No nodes discovered at all. Most likely causes:"
    echo "   - this shell uses a different RMW_IMPLEMENTATION than the stack"
    echo "     (run: echo \$RMW_IMPLEMENTATION in both terminals)"
    echo "   - different ROS_DOMAIN_ID"
    echo "   - the stack never finished starting (check its terminal output)"
fi

echo "-- 2. controllers"
if ros2 control list_controllers >/dev/null 2>&1; then
    ros2 control list_controllers 2>/dev/null | sed 's/^/         /'
    if ros2 control list_controllers 2>/dev/null | grep -q "rizon_arm_controller.*active"; then
        ok "rizon_arm_controller is active"
    else
        missing "rizon_arm_controller is not active"
    fi
else
    missing "controller_manager services are unreachable"
fi

echo "-- 3. action servers"
ACTIONS="$(ros2 action list 2>/dev/null)"
for action in /grasp_demo/pick_place /flexiv_gripper_node/move /flexiv_gripper_node/grasp; do
    if printf '%s\n' "${ACTIONS}" | grep -qx "${action}"; then
        ok "${action}"
    elif [ "${action}" = "/grasp_demo/pick_place" ]; then
        warn "${action} is missing (expected until the task node starts)"
    else
        missing "${action} is missing"
    fi
done

echo "-- 4. hand-eye TF"
if has_node /handeye_tf_publisher; then
    ok "handeye_tf_publisher is running"
else
    missing "handeye_tf_publisher is not running (frame camera_link will be missing)"
fi

echo "-- 5. /${SN_TOPIC}/tcp_pose (RDK view, same as the pendant)"
TCP_POSE="$(timeout 6 ros2 topic echo "/${SN_TOPIC}/tcp_pose" --once 2>/dev/null)"
if [ -n "${TCP_POSE}" ]; then
    echo "${TCP_POSE}" | sed -n '/position:/,/orientation:/p' | sed 's/^/         /'
else
    note "no data on /${SN_TOPIC}/tcp_pose (expected with fake hardware or a stopped driver)"
fi

echo "-- 6. TF ${SN}_base_link -> ${SN}_grav_tcp (MoveIt view)"
TF_OUT="$(timeout 8 ros2 run tf2_ros tf2_echo "${SN}_base_link" "${SN}_grav_tcp" 2>&1 \
    | grep -A1 "Translation" | head -2)"
if [ -n "${TF_OUT}" ]; then
    echo "${TF_OUT}" | sed 's/^/         /'
else
    missing "TF ${SN}_base_link -> ${SN}_grav_tcp is not resolvable"
fi

echo
echo "== compare the two positions above"
echo "   Same rigid body, so x/y/z should agree to a few mm. A constant offset"
echo "   means a tool/TCP definition mismatch, not a TF problem."

if [ "${FAIL}" -ne 0 ]; then
    echo
    echo "== PRE-FLIGHT FAILED"
    exit 1
fi

if [ "${WARN}" -ne 0 ]; then
    echo
    echo "== PRE-FLIGHT PASSED (with warnings)"
    exit 0
fi

echo
echo "== PRE-FLIGHT PASSED"
