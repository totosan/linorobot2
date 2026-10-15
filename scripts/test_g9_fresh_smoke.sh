#!/usr/bin/env bash
# Motionless headless runtime smoke for an isolated, already-built release.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/sim_paths.sh
source "${SCRIPT_DIR}/lib/sim_paths.sh"
REPO_ROOT="$(linorobot2_repo_root)"
RELEASE_ROOT="$(linorobot2_release_root)"
SOURCE_ROOT="${RELEASE_ROOT}/source"

[[ "${ROS_DOMAIN_ID:-}" == "42" ]] || {
    if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
        echo "FAIL: G9 smoke requires ROS_DOMAIN_ID=42; refusing $ROS_DOMAIN_ID." >&2
        exit 2
    fi
}
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export IGN_PARTITION="linorobot2_g9_${UID}_$(date +%Y%m%d_%H%M%S)_$$"

[[ -f "${RELEASE_ROOT}/evidence/release-manifest.json" ]] || {
    echo "FAIL: pending release manifest is missing." >&2
    exit 2
}
[[ -d "$SOURCE_ROOT/.git" ]] || {
    echo "FAIL: release source checkout is missing: $SOURCE_ROOT" >&2
    exit 2
}

set +u
source /opt/ros/humble/setup.bash
linorobot2_source_overlay
set -u

LOG_DIR="${RELEASE_ROOT}/log/g9-smoke-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$LOG_DIR"
SIM_PID=""
cleanup() {
    if [[ -n "$SIM_PID" ]]; then
        kill -INT "$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

python3 "${REPO_ROOT}/scripts/supervised_process.py" -- \
    ros2 launch linorobot2_gazebo fortress.launch.py gui:=false rviz:=false \
    >"${LOG_DIR}/launch.log" 2>&1 &
SIM_PID=$!
python3 "${REPO_ROOT}/scripts/wait_for_fortress.py" --pid "$SIM_PID" \
    | tee "${LOG_DIR}/readiness.log"

timeout 85 python3 -u "${REPO_ROOT}/scripts/verify_sim_sensors.py" --duration 5 \
    | tee "${LOG_DIR}/sensors.log"
ros2 topic info /clock >"${LOG_DIR}/clock-info.txt"
grep -Eq 'Publisher count: 1$' "${LOG_DIR}/clock-info.txt" || {
    echo "FAIL: expected exactly one /clock publisher." >&2
    exit 1
}
ros2 node list --no-daemon >"${LOG_DIR}/nodes.txt"
if grep -Eq '/(bt_navigator|controller_server|planner_server|behavior_server|waypoint_follower|nav2)' "${LOG_DIR}/nodes.txt"; then
    echo "FAIL: Nav2 node discovered during motionless G9 smoke." >&2
    exit 1
fi
ros2 topic info /cmd_vel >"${LOG_DIR}/cmd-vel-info.txt" 2>&1 || true
if ! grep -Eq 'Publisher count: 1$' "${LOG_DIR}/cmd-vel-info.txt"; then
    echo "FAIL: expected only the command-timeout watchdog on /cmd_vel." >&2
    exit 1
fi
ros2 topic info --verbose /cmd_vel >"${LOG_DIR}/cmd-vel-verbose.txt"
if ! grep -Eq 'Node name: command_timeout$' "${LOG_DIR}/cmd-vel-verbose.txt"; then
    echo "FAIL: /cmd_vel publisher is not the expected watchdog." >&2
    exit 1
fi

kill -INT "$SIM_PID"
set +e
wait "$SIM_PID"
SHUTDOWN_STATUS=$?
set -e
SIM_PID=""
if [[ "$SHUTDOWN_STATUS" != 0 && "$SHUTDOWN_STATUS" != 130 ]]; then
    echo "FAIL: supervisor exited unexpectedly during shutdown: $SHUTDOWN_STATUS." >&2
    exit 1
fi
sleep 2
if ps -eo args= | grep -F "$IGN_PARTITION" | grep -v grep >"${LOG_DIR}/process-residue.txt"; then
    echo "FAIL: simulator process residue remains; see ${LOG_DIR}/process-residue.txt." >&2
    exit 1
fi

python3 "${SOURCE_ROOT}/scripts/sim_release.py" manifest \
    --release "$RELEASE_ROOT" \
    --source "$SOURCE_ROOT" \
    --validation-status passed \
    --validation-command "bash scripts/build_workspace.sh" \
    --validation-command "python3 -m unittest discover -s scripts -p test_*.py" \
    --validation-command "bash scripts/test_g9_fresh_smoke.sh"
printf '%s\n' "validated" >"${RELEASE_ROOT}/BUILD_STATE"
echo "PASS: isolated motionless G9 smoke and cleanup. Evidence: $LOG_DIR"
