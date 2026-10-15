#!/usr/bin/env bash
# One-frame, simulation-only G6 probe against the authorized Vision service.
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_DIR}/scripts/lib/sim_paths.sh"
PROBE="${WORKSPACE_DIR}/scripts/probe_g6_vision_service.py"
MODE="standalone"
TEST_SCENE="fortress_test"
WORLD_ARGUMENTS=()
EXPECT_LABEL_ARGUMENTS=()
POSITIVE_PERSON=false
if [[ "${1:-}" == "--attach" && $# == 1 ]]; then
    MODE="attach"
elif [[ "${1:-}" == "--positive-person" && $# == 1 ]]; then
    TEST_SCENE="fortress_g6_vision_test"
    WORLD_ARGUMENTS=(world:="${WORKSPACE_DIR}/linorobot2_gazebo/worlds/${TEST_SCENE}.sdf")
    EXPECT_LABEL_ARGUMENTS=(--expect-label person)
    POSITIVE_PERSON=true
elif [[ $# -ne 0 ]]; then
    echo "Usage: $0 [--attach | --positive-person]"
    exit 2
fi
if [[ "$POSITIVE_PERSON" == "true" ]]; then
    if [[ -z "${G6_VISION_MODEL_VERSION:-}" \
        || -z "${G6_VISION_LABELS_VERSION:-}" \
        || ! "${G6_VISION_PERSON_CLASS_ID:-}" =~ ^[0-9]+$ ]]; then
        echo "FAIL: --positive-person requires verified G6_VISION_MODEL_VERSION, G6_VISION_LABELS_VERSION, and numeric G6_VISION_PERSON_CLASS_ID."
        exit 2
    fi
    EXPECT_LABEL_ARGUMENTS+=(
        --through-contract-v1
        --model-version "$G6_VISION_MODEL_VERSION"
        --labels-version "$G6_VISION_LABELS_VERSION"
        --class-id "$G6_VISION_PERSON_CLASS_ID"
    )
fi
if [[ -n "${ROS_DOMAIN_ID:-}" && "${ROS_DOMAIN_ID}" != "42" ]]; then
    echo "FAIL: G6 camera probe is restricted to ROS_DOMAIN_ID=42."
    exit 1
fi
if [[ ! -f "$PROBE" ]]; then
    echo "FAIL: missing G6 probe: $PROBE"
    exit 1
fi

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export LIBGL_ALWAYS_SOFTWARE=1
if [[ "$MODE" == "standalone" ]]; then
    export IGN_PARTITION="linorobot2_g6probe_${UID}_$(date +%Y%m%d_%H%M%S)_$$"
fi

set +u
source /opt/ros/humble/setup.bash
linorobot2_source_overlay
set -u

LOG_PARENT="${HOME}/linorobot2_sim/log"
mkdir -p "$LOG_PARENT"
LOG_DIR="$(mktemp -d "${LOG_PARENT}/g6-vision-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
SIM_PID=""
cleanup() {
    if [[ -n "$SIM_PID" ]]; then
        kill -TERM "$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
    fi
    if [[ -d "$LOG_DIR" ]] \
        && [[ -z "$(find "$LOG_DIR" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
        rmdir "$LOG_DIR"
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

topic_has_publisher() {
    local topic_info status
    topic_info="$(ros2 topic info "$1" --verbose 2>&1)" && status=0 || status=$?
    if [[ "$status" -ne 0 ]]; then
        if grep -Fq "Unknown topic '$1'" <<<"$topic_info"; then
            return 1
        fi
        echo "FAIL: unable to inspect ROS topic $1: $topic_info" >&2
        return 2
    fi
    grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$topic_info"
}

if [[ "$MODE" == "attach" ]]; then
    for topic in /camera/image_raw /camera/camera_info /clock; do
        if topic_has_publisher "$topic"; then
            continue
        else
            status=$?
            if [[ "$status" -eq 2 ]]; then
                exit 1
            fi
            echo "FAIL: --attach requires active camera, CameraInfo, and /clock publishers on Domain 42."
            exit 1
        fi
    done
    ATTACH_ARGS=(--attach-to-existing-sim)
    echo "Probe mode: attach to existing ROS_DOMAIN_ID=${ROS_DOMAIN_ID} camera stream"
else
    existing_graph=false
    PREEXISTING_TOPICS=(/camera/image_raw /camera/camera_info /clock)
    if [[ "$POSITIVE_PERSON" == "true" ]]; then
        PREEXISTING_TOPICS+=(/vision/result /vision/status)
    fi
    for topic in "${PREEXISTING_TOPICS[@]}"; do
        if topic_has_publisher "$topic"; then
            existing_graph=true
            break
        else
            status=$?
            if [[ "$status" -eq 2 ]]; then
                exit 1
            fi
        fi
    done
    if [[ "$existing_graph" == "true" ]]; then
        echo "FAIL: Domain 42 already has relevant camera/clock/vision publishers; use --attach or stop that graph first."
        exit 1
    fi
    python3 "${WORKSPACE_DIR}/scripts/supervised_process.py" -- \
        ros2 launch linorobot2_gazebo fortress.launch.py \
            ros_profile:=sim ros_domain_id:=42 gui:=false rviz:=false \
            "${WORLD_ARGUMENTS[@]}" \
        >"${LOG_DIR}/fortress.log" 2>&1 &
    SIM_PID=$!

    python3 "${WORKSPACE_DIR}/scripts/wait_for_fortress.py" \
        --pid "$SIM_PID" --world "$TEST_SCENE" --timeout 40 \
        >"${LOG_DIR}/readiness.log" 2>&1 || {
        cat "${LOG_DIR}/readiness.log"
        echo "FAIL: Fortress did not become ready; see ${LOG_DIR}/fortress.log"
        exit 1
    }
    ATTACH_ARGS=()
    echo "Probe isolation: ROS_DOMAIN_ID=${ROS_DOMAIN_ID} IGN_PARTITION=${IGN_PARTITION}"
fi

python3 "$PROBE" "${ATTACH_ARGS[@]}" \
    "${EXPECT_LABEL_ARGUMENTS[@]}" \
    --test-scene "$TEST_SCENE" \
    --endpoint "${G6_VISION_ENDPOINT:-http://192.168.0.150:8082/v1/vision/detection}" \
    --min-confidence "${G6_VISION_MIN_CONFIDENCE:-0.3}" \
    --timeout "${G6_VISION_TIMEOUT_SEC:-20}" \
    --report "${LOG_DIR}/vision-probe.json"

echo "Evidence: ${LOG_DIR}"
