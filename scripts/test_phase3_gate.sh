#!/usr/bin/env bash
# Isolated, bounded Phase 3 SIL test; no physical devices or hardware topics.
set -euo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_DIR}/scripts/lib/sim_paths.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
if [[ "$ROS_DOMAIN_ID" != "42" ]]; then
    echo "FAIL: Phase 3 safety test is restricted to ROS_DOMAIN_ID=42."
    exit 1
fi
LOG_DIR="${HOME}/linobot2_sim/log/g3-safety-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$LOG_DIR"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export LINOROBOT2_BASE=2wd
set +u
source /opt/ros/humble/setup.bash
linorobot2_source_overlay
set -u

SIM_PID=""
stop_sim() {
    if [[ -n "$SIM_PID" ]]; then
        kill -TERM "$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
        SIM_PID=""
    fi
}
cleanup() { stop_sim; }
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_sim() {
    local camera_fault="$1"
    local generation="${camera_fault//[^a-zA-Z0-9]/}"
    export IGN_PARTITION="linobot2_g3_${UID}_$(date +%Y%m%d_%H%M%S)_${RANDOM}_$$"
    printf '%s\n' "[G3] Isolated headless Fortress launch (Domain ${ROS_DOMAIN_ID}, ${IGN_PARTITION}, camera_fault_test=${camera_fault})"
    python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
        ros2 launch linorobot2_gazebo fortress.launch.py gui:=false rviz:=false \
        camera_fault_test:="$camera_fault" \
        >"$LOG_DIR/launch-${generation}.log" 2>&1 &
    SIM_PID=$!
    python3 "$WORKSPACE_DIR/scripts/wait_for_fortress.py" --pid "$SIM_PID" \
        | tee "$LOG_DIR/readiness-${generation}.log" || {
        echo "FAIL: Fortress world not ready; log: $LOG_DIR/launch-${generation}.log"
        tail -n 80 "$LOG_DIR/launch-${generation}.log"
        exit 1
    }
}

start_sim false
FIRST_SIM_PID="$SIM_PID"
FIRST_PARTITION="$IGN_PARTITION"
printf '%s\n' "[G3] Bounded square, odometry/TF ownership, watchdog stop and fresh-command recovery"
timeout 300 python3 -u "$WORKSPACE_DIR/scripts/verify_phase3.py" \
    --supervisor-pid "$SIM_PID" --report "$LOG_DIR/phase3.json" \
    | tee "$LOG_DIR/phase3.log" || {
    echo "FAIL: G3 SIL validation; report: $LOG_DIR/phase3.json"
    exit 1
}
printf '%s\n' "[G3] Stop the owned simulator tree, then restart Fortress in a fresh partition"
stop_sim
if kill -0 "$FIRST_SIM_PID" 2>/dev/null; then
    echo "FAIL: prior Fortress supervisor still alive after bounded shutdown: $FIRST_SIM_PID"
    exit 1
fi
start_sim true
timeout 45 python3 -u "$WORKSPACE_DIR/scripts/verify_phase3_faults.py" \
    --supervisor-pid "$SIM_PID" --previous-supervisor-pid "$FIRST_SIM_PID" \
    --previous-partition "$FIRST_PARTITION" \
    --report "$LOG_DIR/restart-sensor.json" --core-report "$LOG_DIR/phase3.json" \
    | tee "$LOG_DIR/restart-sensor.log" || {
    echo "FAIL: G3 sensor-loss/restart validation; report: $LOG_DIR/restart-sensor.json"
    exit 1
}
echo "PASS: simulation-only Phase 3 checks, including camera-topic loss/recovery and clean Fortress restart. Evidence: $LOG_DIR"
