#!/usr/bin/env bash
# Isolated simulation-only G2 motion smoke test; never connects to hardware.
set -euo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_DIR}/scripts/lib/sim_paths.sh"
SEGMENT_SECONDS="${G2_SEGMENT_SECONDS:-2}"
if [[ ! "$SEGMENT_SECONDS" =~ ^([0-4]?[0-9]+)(\.[0-9]+)?$ ]]; then
    echo "FAIL: G2_SEGMENT_SECONDS must be numeric in [0.5, 5]."
    exit 1
fi
if ! python3 -c 'import math,sys; x=float(sys.argv[1]); sys.exit(0 if math.isfinite(x) and 0.5 <= x <= 5 else 1)' "$SEGMENT_SECONDS"; then
    echo "FAIL: G2_SEGMENT_SECONDS must be finite and in [0.5, 5]."
    exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
if [[ "$ROS_DOMAIN_ID" != "42" ]]; then
    echo "FAIL: simulation motion test only allowed in ROS_DOMAIN_ID=42."
    exit 1
fi
RELEASE_ROOT="$(linorobot2_release_root)"
LOG_DIR="${RELEASE_ROOT}/log/g2-motion-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$LOG_DIR"
export IGN_PARTITION="linobot2_g2_${UID}_$(date +%Y%m%d_%H%M%S)_$$"
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

G2_LAUNCH_ARGS=()
if [[ -n "${G2_WHEEL_MU:-}" ]]; then
    if ! python3 -c 'import math,sys; x=float(sys.argv[1]); sys.exit(0 if math.isfinite(x) and 0.0 < x <= 5.0 else 1)' "$G2_WHEEL_MU"; then
        echo "FAIL: G2_WHEEL_MU must be finite and in (0, 5]."
        exit 1
    fi
    G2_LAUNCH_ARGS+=("wheel_mu:=${G2_WHEEL_MU}")
    printf '%s\n' "$G2_WHEEL_MU" >"$LOG_DIR/wheel-mu.txt"
    echo "[G2] Discrimination probe: wheel_mu=$G2_WHEEL_MU (hypothesis test only; product default stays 1.0)"
fi
if [[ -n "${G2_PHYSICS_STEP:-}" ]]; then
    if [[ "$G2_PHYSICS_STEP" != "0.001" && "$G2_PHYSICS_STEP" != "0.0005" ]]; then
        echo "FAIL: G2_PHYSICS_STEP must be exactly 0.001 or 0.0005."
        exit 1
    fi
    BASE_WORLD="$(linorobot2_install_base)/linorobot2_gazebo/share/linorobot2_gazebo/worlds/fortress_test.sdf"
    VARIANT_WORLD="${LOG_DIR}/fortress_g3_step.sdf"
    python3 "${WORKSPACE_DIR}/scripts/make_g3_world_variant.py" \
        --source "$BASE_WORLD" \
        --output "$VARIANT_WORLD" \
        --step "$G2_PHYSICS_STEP" \
        --manifest "${LOG_DIR}/world-variant.json"
    G2_LAUNCH_ARGS+=("world:=${VARIANT_WORLD}")
    echo "[G2] Discrimination probe: max_step_size=$G2_PHYSICS_STEP s (test only)"
fi
if [[ -n "${G2_INCLUDE_CASTER:-}" ]]; then
    if [[ "$G2_INCLUDE_CASTER" != "true" && "$G2_INCLUDE_CASTER" != "false" ]]; then
        echo "FAIL: G2_INCLUDE_CASTER must be true or false."
        exit 1
    fi
    G2_LAUNCH_ARGS+=("include_caster:=${G2_INCLUDE_CASTER}")
    printf '%s\n' "$G2_INCLUDE_CASTER" >"${LOG_DIR}/include-caster.txt"
    echo "[G2] Discrimination probe: include_caster=$G2_INCLUDE_CASTER (test only)"
fi
if [[ -n "${G2_CASTER_MU:-}" ]]; then
    if [[ "$G2_CASTER_MU" != "0.01" && "$G2_CASTER_MU" != "0.0" ]]; then
        echo "FAIL: G2_CASTER_MU must be exactly 0.01 or 0.0."
        exit 1
    fi
    G2_LAUNCH_ARGS+=("caster_mu:=${G2_CASTER_MU}")
    printf '%s\n' "$G2_CASTER_MU" >"${LOG_DIR}/caster-mu.txt"
    echo "[G2] Discrimination probe: caster_mu=$G2_CASTER_MU (test only)"
fi

printf '%s\n' "[G2] Isolated headless Fortress launch (Domain ${ROS_DOMAIN_ID}, ${IGN_PARTITION})"
python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
    ros2 launch linorobot2_gazebo fortress.launch.py gui:=false rviz:=false \
    ${G2_LAUNCH_ARGS[@]+"${G2_LAUNCH_ARGS[@]}"} \
    >"$LOG_DIR/launch.log" 2>&1 &
SIM_PID=$!
python3 "$WORKSPACE_DIR/scripts/wait_for_fortress.py" --pid "$SIM_PID" \
    | tee "$LOG_DIR/readiness.log" || {
    echo "FAIL: Fortress world not ready; log: $LOG_DIR/launch.log"
    tail -n 80 "$LOG_DIR/launch.log"
    exit 1
}
printf '%s\n' "[G2] Validate joints, bounded forward/reverse/turns, odometry, TF and watchdog"
timeout 150 python3 -u "$WORKSPACE_DIR/scripts/verify_sim_motion.py" \
    --supervisor-pid "$SIM_PID" --segment-seconds "$SEGMENT_SECONDS" \
    --report "$LOG_DIR/motion.json" | tee "$LOG_DIR/motion.log" || {
    echo "FAIL: G2 motion validation; report: $LOG_DIR/motion.json"
    exit 1
}
echo "PASS: simulation-only G2 motion smoke test. Evidence: $LOG_DIR"
