#!/usr/bin/env bash
# Bounded Phase-5 simulated-camera consumer check; no Nav2 goals or motor commands.
set -euo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_DIR}/scripts/lib/sim_paths.sh"
SIM_INSTALL_BASE="$(linorobot2_install_base)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
if [[ "$ROS_DOMAIN_ID" != "42" ]]; then
    echo "FAIL: Phase-5 camera probe is restricted to ROS_DOMAIN_ID=42."
    exit 1
fi
if [[ ! "${DISPLAY:-}" =~ ^:[0-9]+(\.[0-9]+)?$ ]]; then
    echo "FAIL: visual camera probe needs a local Linux DISPLAY (e.g. :10.0)."
    exit 1
fi

LOG_PARENT="${HOME}/linobot2_sim/log"
mkdir -p "$LOG_PARENT"
LOG_DIR="$(mktemp -d "${LOG_PARENT}/p5-sim-camera-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
export IGN_PARTITION="linobot2_p5cam_${UID}_$(date +%Y%m%d_%H%M%S)_${RANDOM}_$$"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export LINOROBOT2_BASE=2wd
set +u
source /opt/ros/humble/setup.bash
linorobot2_source_overlay
set -u

SIM_PID=""
cleanup() {
    if [[ -n "$SIM_PID" ]]; then
        kill -TERM "$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

RVIZ_CONFIG="${SIM_INSTALL_BASE}/linorobot2_description/share/linorobot2_description/rviz/fortress_camera.rviz"
python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
    ros2 launch linorobot2_gazebo fortress.launch.py gui:=true rviz:=true \
        rviz_config:="$RVIZ_CONFIG" \
    >"$LOG_DIR/fortress.log" 2>&1 &
SIM_PID=$!
python3 "$WORKSPACE_DIR/scripts/wait_for_fortress.py" --pid "$SIM_PID" --timeout 40 \
    | tee "$LOG_DIR/readiness.log" || {
    echo "FAIL: Fortress did not become ready; see $LOG_DIR/fortress.log"
    exit 1
}
timeout --kill-after=5s 40s python3 -u \
    "$WORKSPACE_DIR/scripts/verify_vision_sim_camera.py" \
    --timeout 30 --minimum-frames 5 --report "$LOG_DIR/camera-consumer.json" \
    | tee "$LOG_DIR/camera-consumer.log" || {
    echo "FAIL: simulated camera consumer; see $LOG_DIR/camera-consumer.json"
    exit 1
}
echo "PASS: simulated camera frames consumed through Phase-5 mock. Evidence: $LOG_DIR"
echo "The RDP desktop now shows Gazebo and RViz's live /camera/image_raw display."
if [[ -t 0 && "${P5_KEEP_OPEN:-1}" == "1" ]]; then
    echo "Windows remain open for visual monitoring; press Enter here (or Ctrl+C) to stop them."
    read -r _
fi
