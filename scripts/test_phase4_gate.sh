#!/usr/bin/env bash
# Bounded, simulation-only SLAM map capture and fresh-run Nav2 readiness gate.
set -euo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_DIR}/scripts/lib/sim_paths.sh"
SIM_INSTALL_BASE="$(linorobot2_install_base)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
if [[ "$ROS_DOMAIN_ID" != "42" ]]; then
    echo "FAIL: Phase 4 simulation gate is restricted to ROS_DOMAIN_ID=42."
    exit 1
fi

VISUAL_MODE="${G4_VISUAL:-false}"
ENABLE_GUI="${G4_GUI:-false}"
ENABLE_RVIZ="${G4_RVIZ:-false}"
ENABLE_TF2_FIX="false"
MAP_MODE="sensor-seeded"
UNREACHABLE_MAP=""
NOISE_SLIP_PROFILE="false"

for arg in "${@:-}"; do
    case "$arg" in
        --visual)
            VISUAL_MODE="true"
            ;;
        --gui)
            ENABLE_GUI="true"
            ;;
        --rviz)
            ENABLE_RVIZ="true"
            ;;
        --tf2-fix)
            ENABLE_TF2_FIX="true"
            ;;
        --fixture-map)
            MAP_MODE="fixture"
            ;;
        --sensor-seeded-map)
            MAP_MODE="sensor-seeded"
            ;;
        --noise-slip-profile)
            NOISE_SLIP_PROFILE="true"
            ;;
        --unreachable-map=*)
            UNREACHABLE_MAP="${arg#*=}"
            if [[ "$UNREACHABLE_MAP" != /* || ! -s "$UNREACHABLE_MAP" ]]; then
                echo "FAIL: --unreachable-map requires an existing absolute map YAML path."
                exit 1
            fi
            ;;
        --headless)
            VISUAL_MODE="false"
            ENABLE_GUI="false"
            ENABLE_RVIZ="false"
            ;;
        -h|--help)
            echo "Verwendung: $0 [--visual] [--gui] [--rviz] [--headless] [--tf2-fix] [--sensor-seeded-map|--fixture-map] [--noise-slip-profile] [--unreachable-map=PATH]"
            echo "  --visual    Aktiviert Gazebo-3D-GUI und RViz (z. B. auf RDP-Desktop :10.0)"
            echo "  --gui       Aktiviert nur die Gazebo-3D-GUI"
            echo "  --rviz      Aktiviert nur RViz2 (Nav2-Karten- und Pfadanzeige)"
            echo "  --headless  Kopflose Ausführung (Standard)"
            echo "  --tf2-fix   Aktiviert und prüft den separat gebauten Humble-TF2-Fix"
            echo "  --sensor-seeded-map  Roh-SLAM-Map plus begrenzter Start-Footprint-Seed (Standard; keine GT-Trajektorie)"
            echo "  --fixture-map  GT-assistierte Fixture explizit für Baseline-Diagnose"
            echo "  --noise-slip-profile  Aktiviert das gepinnte G4-SIL-Rausch-/Schlupfprofil"
            echo "  --unreachable-map=PATH  Nur Abbruchtest mit gespeicherter Karte; kein Mapping/Baseline"
            exit 0
            ;;
    esac
done

if [[ -n "$UNREACHABLE_MAP" && "$ENABLE_TF2_FIX" != "true" ]]; then
    echo "FAIL: unreachable scenario requires the tested --tf2-fix overlay."
    exit 1
fi

if [[ "$VISUAL_MODE" == "true" ]]; then
    ENABLE_GUI="true"
    ENABLE_RVIZ="true"
fi

if [[ "$ENABLE_GUI" == "true" || "$ENABLE_RVIZ" == "true" ]]; then
    if [[ ! "${DISPLAY:-}" =~ ^:[0-9]+(\.[0-9]+)?$ ]]; then
        if [[ -S "/tmp/.X11-unix/X10" ]]; then
            export DISPLAY=":10.0"
        else
            echo "FAIL: Visueller Modus angefordert, aber kein lokales X11-Display (z. B. :10.0) aktiv."
            echo "Hinweis: Bitte starte die RDP-Sitzung (scripts/connect_rdp_mac.sh / Windows App) oder setze DISPLAY=:10.0."
            exit 1
        fi
    fi
    export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
    echo "INFO: Visueller Modus aktiv auf DISPLAY=${DISPLAY} (Gazebo-GUI=${ENABLE_GUI}, RViz=${ENABLE_RVIZ})"
fi

LOG_PARENT="${G4_LOG_PARENT:-${WORKSPACE_DIR}/log/g4_runtime}"
mkdir -p "$LOG_PARENT"
LOG_DIR="$(mktemp -d "${LOG_PARENT}/g4-nav-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
G4_BAG_TOPICS=(/clock /tf /tf_static /scan /map /map_updates /odom /odom/unfiltered
    /joint_states /cmd_vel /amcl_pose /plan /global_costmap/costmap
    /local_costmap/costmap)
printf '%s\n' "${G4_BAG_TOPICS[@]}" >"$LOG_DIR/rosbag-topics.txt"
export IGN_PARTITION="linobot2_g4_${UID}_$(date +%Y%m%d_%H%M%S)_${RANDOM}_$$"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export LINOROBOT2_BASE=2wd
set +u
source /opt/ros/humble/setup.bash
linorobot2_source_overlay
set -u

G4_WORLD_DIR="${SIM_INSTALL_BASE}/linorobot2_gazebo/share/linorobot2_gazebo/worlds"
G4_WORLD="${G4_WORLD_DIR}/fortress_g4_test.sdf"
G4_ROBUSTNESS_LAUNCH_ARGS=()
if [[ "$NOISE_SLIP_PROFILE" == "true" ]]; then
    G4_WORLD="${G4_WORLD_DIR}/fortress_g4_noise_slip.sdf"
    PROFILE_FILE="$WORKSPACE_DIR/scripts/g4_robustness_profile.json"
    if [[ ! -s "$PROFILE_FILE" || ! -s "$G4_WORLD" ]]; then
        echo "FAIL: pinned G4 robustness profile or Fortress world is missing."
        exit 1
    fi
    cp "$PROFILE_FILE" "$LOG_DIR/g4_robustness_profile.json"
    sha256sum "$PROFILE_FILE" >"$LOG_DIR/g4_robustness_profile.sha256"
    mapfile -t PROFILE_VALUES < <(python3 -c 'import json,sys; p=json.load(open(sys.argv[1])); print("\n".join(str(p[k]) for k in ("lidar_range_stddev_m", "imu_angular_velocity_stddev_rad_s", "imu_linear_acceleration_stddev_m_s2", "wheel_mu1")))' "$PROFILE_FILE")
    if [[ "${#PROFILE_VALUES[@]}" -ne 4 ]]; then
        echo "FAIL: malformed pinned G4 robustness profile."
        exit 1
    fi
    G4_ROBUSTNESS_LAUNCH_ARGS=(
        "lidar_noise_stddev:=${PROFILE_VALUES[0]}"
        "imu_gyro_noise_stddev:=${PROFILE_VALUES[1]}"
        "imu_accel_noise_stddev:=${PROFILE_VALUES[2]}"
        "wheel_mu:=${PROFILE_VALUES[3]}"
    )
    echo "INFO: pinned G4 noise/slip profile enabled; manifest and hash are in $LOG_DIR"
fi

if [[ "$ENABLE_TF2_FIX" == "true" ]]; then
    TF2_FIX_SETUP="${HOME}/linobot2_tf2_fix/install/local_setup.bash"
    if [[ ! -r "$TF2_FIX_SETUP" ]]; then
        echo "FAIL: requested TF2 fix overlay is missing: $TF2_FIX_SETUP"
        exit 1
    fi
    set +u
    source "$TF2_FIX_SETUP"
    set -u
    python3 "$WORKSPACE_DIR/scripts/verify_tf2_overlay.py" \
        --report "$LOG_DIR/tf2-overlay.json"
fi

SIM_PID=""
SLAM_PID=""
NAV_PID=""
BAG_PID=""
stop_owned() {
    local pid="$1"
    if [[ -n "$pid" ]]; then
        kill -TERM "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    fi
}
cleanup() {
    stop_owned "$NAV_PID"
    stop_owned "$SLAM_PID"
    stop_owned "$SIM_PID"
    stop_owned "$BAG_PID"
}
start_rosbag() {
    python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
        ros2 bag record --storage sqlite3 --output "$LOG_DIR/rosbag" \
        "${G4_BAG_TOPICS[@]}" \
        >"$LOG_DIR/rosbag-record.log" 2>&1 &
    BAG_PID=$!
}
stop_rosbag() {
    stop_owned "$BAG_PID"
    BAG_PID=""
    local bag_info="$LOG_DIR/rosbag-info.txt"
    if [[ ! -s "$LOG_DIR/rosbag/metadata.yaml" ]]; then
        echo "FAIL: G4 rosbag metadata missing; inspect $LOG_DIR/rosbag-record.log"
        return 1
    fi
    ros2 bag info "$LOG_DIR/rosbag" >"$bag_info" 2>&1 || {
        echo "FAIL: G4 rosbag validation failed; inspect $bag_info"
        return 1
    }
    if ! grep -Eq '^[[:space:]]*Messages:[[:space:]]*[1-9][0-9]*' "$bag_info"; then
        echo "FAIL: G4 rosbag contains no recorded messages; inspect $bag_info"
        return 1
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start_rosbag

new_partition() {
    printf 'linobot2_g4_%s_%s_%s_%s_%s' "$UID" "$(date +%Y%m%d_%H%M%S)" "$RANDOM" "$1" "$$"
}

start_fortress() {
    local stage="$1"
    local rviz_cfg="${2:-}"
    local world_path="${3:-}"
    local expected_world="${4:-fortress_test}"
    local rviz_flag="false"
    if [[ -n "$rviz_cfg" && "$ENABLE_RVIZ" == "true" ]]; then
        rviz_flag="true"
    fi
    local gui_flag="${ENABLE_GUI}"
    export IGN_PARTITION="$(new_partition "$stage")"
    python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
        ros2 launch linorobot2_gazebo fortress.launch.py \
            gui:="$gui_flag" rviz:="$rviz_flag" \
            ${rviz_cfg:+rviz_config:="$rviz_cfg"} \
            ${world_path:+world:="$world_path"} \
            "${G4_ROBUSTNESS_LAUNCH_ARGS[@]}" \
        >"$LOG_DIR/fortress-${stage}.log" 2>&1 &
    SIM_PID=$!
    python3 "$WORKSPACE_DIR/scripts/wait_for_fortress.py" \
        --pid "$SIM_PID" --world "$expected_world" \
        | tee "$LOG_DIR/fortress-${stage}-readiness.log" || {
        echo "FAIL: Fortress world not ready in ${stage}; inspect $LOG_DIR/fortress-${stage}.log"
        exit 1
    }
}

stop_fortress() {
    stop_owned "$SIM_PID"
    SIM_PID=""
}

start_nav2() {
    local map_yaml="$1"
    local params="${SIM_INSTALL_BASE}/linorobot2_navigation/share/linorobot2_navigation/config/navigation_fortress.yaml"
    local common_params="${HOME}/linorobot2_sim/install/linorobot2_navigation/share/linorobot2_navigation/config/navigation_common.yaml"
    python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
        ros2 launch linorobot2_navigation navigation.launch.py sim:=true rviz:=false \
            map:="$map_yaml" sim_params_file:="$params" common_params_file:="$common_params" \
        >"$LOG_DIR/nav2.log" 2>&1 &
    NAV_PID=$!
    timeout --kill-after=5s 75s python3 -u "$WORKSPACE_DIR/scripts/verify_phase4_nav.py" \
        --timeout 60 --report "$LOG_DIR/nav2-readiness.json" \
        | tee "$LOG_DIR/nav2-readiness.log" || {
        echo "FAIL: Nav2 readiness; inspect $LOG_DIR/nav2.log and $LOG_DIR/nav2-readiness.json"
        tail -n 120 "$LOG_DIR/nav2.log"
        exit 1
    }
}

if [[ -n "$UNREACHABLE_MAP" ]]; then
    echo "[G4] Single unreachable-goal scenario, saved map reuse only (Domain 42)"
    UNREACHABLE_BT="${HOME}/linorobot2_sim/install/linorobot2_navigation/share/linorobot2_navigation/behavior_trees/navigate_to_pose_fail_closed.xml"
    if [[ ! -s "$UNREACHABLE_BT" ]]; then
        echo "FAIL: goal-specific fail-closed behavior tree is missing: $UNREACHABLE_BT"
        exit 1
    fi
    printf '%s\n' "$UNREACHABLE_MAP" >"$LOG_DIR/reused-map.txt"
    sha256sum "$UNREACHABLE_MAP" >"$LOG_DIR/reused-map.sha256"
    NAV_RVIZ_CFG="${SIM_INSTALL_BASE}/linorobot2_navigation/share/linorobot2_navigation/rviz/linorobot2_navigation.rviz"
    UNREACHABLE_WORLD=""
    if [[ "$NOISE_SLIP_PROFILE" == "true" ]]; then
        UNREACHABLE_WORLD="$G4_WORLD"
    fi
    start_fortress "unreachable" "$NAV_RVIZ_CFG" "$UNREACHABLE_WORLD" fortress_test
    start_nav2 "$UNREACHABLE_MAP"
    timeout --kill-after=10s 160s python3 -u "$WORKSPACE_DIR/scripts/verify_phase4_goal.py" \
        --scenario unreachable --supervisor-pid "$SIM_PID" --readiness-timeout 45 \
        --goal-timeout 90 --nav2-log "$LOG_DIR/nav2.log" \
        --unreachable-behavior-tree "$UNREACHABLE_BT" \
        --report "$LOG_DIR/unreachable-goal.json" \
        | tee "$LOG_DIR/unreachable-goal.log" || {
        echo "FAIL: unreachable-goal scenario; inspect $LOG_DIR/unreachable-goal.json and $LOG_DIR/nav2.log"
        cat "$LOG_DIR/unreachable-goal.json" 2>/dev/null || true
        tail -n 120 "$LOG_DIR/nav2.log"
        exit 1
    }
    stop_owned "$NAV_PID"
    NAV_PID=""
    stop_fortress
    stop_rosbag
    echo "PASS: accepted unreachable goal aborted and stopped safely. Evidence: $LOG_DIR"
    exit 0
fi

printf '%s\n' "[G4] Fortress SLAM capture + isolated map reuse readiness (Domain 42)"
MAPPING_RVIZ_CFG="${SIM_INSTALL_BASE}/linorobot2_description/share/linorobot2_description/rviz/fortress.rviz"
if [[ ! -s "$G4_WORLD" ]]; then
    echo "FAIL: G4 sensor-mapping world is missing: $G4_WORLD"
    exit 1
fi
start_fortress "mapping" "$MAPPING_RVIZ_CFG" "$G4_WORLD" fortress_test
SLAM_PARAMS_FILE="${SIM_INSTALL_BASE}/linorobot2_navigation/share/linorobot2_navigation/config/slam.yaml"
python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
    slam_params_file:="$SLAM_PARAMS_FILE" \
    >"$LOG_DIR/slam.log" 2>&1 &
SLAM_PID=$!
timeout --kill-after=10s 480s python3 -u "$WORKSPACE_DIR/scripts/verify_phase4_mapping.py" \
    --supervisor-pid "$SIM_PID" --report "$LOG_DIR/slam-mapping.json" \
    | tee "$LOG_DIR/slam-mapping.log" || {
    echo "FAIL: Fortress SLAM mapping; inspect $LOG_DIR/slam.log and $LOG_DIR/slam-mapping.json"
    tail -n 120 "$LOG_DIR/slam.log"
    exit 1
}
MAP_PREFIX="$LOG_DIR/fortress_map"
timeout --kill-after=5s 45s ros2 run nav2_map_server map_saver_cli -t /map -f "$MAP_PREFIX" \
    >"$LOG_DIR/map-save.log" 2>&1 || {
    echo "FAIL: saving the measured SLAM map; inspect $LOG_DIR/map-save.log"
    cat "$LOG_DIR/map-save.log"
    exit 1
}
if [[ ! -s "$MAP_PREFIX.yaml" || ! -s "$MAP_PREFIX.pgm" ]]; then
    echo "FAIL: map saver did not produce nonempty YAML and PGM artifacts."
    exit 1
fi
NAV_MAP_PREFIX="$LOG_DIR/fortress_map_nav"
if [[ "$MAP_MODE" == "sensor-seeded" ]]; then
    python3 "$WORKSPACE_DIR/scripts/prepare_phase4_map.py" \
        --mode sensor-seeded --yaml "$MAP_PREFIX.yaml" \
        --output-prefix "$NAV_MAP_PREFIX" \
        >"$LOG_DIR/map-prepare.log" 2>&1
else
    python3 "$WORKSPACE_DIR/scripts/prepare_phase4_map.py" \
        --mode fixture --yaml "$MAP_PREFIX.yaml" \
        --mapping-report "$LOG_DIR/slam-mapping.json" \
        --output-prefix "$NAV_MAP_PREFIX" \
        >"$LOG_DIR/map-prepare.log" 2>&1
fi || {
    echo "FAIL: preparing the selected Nav2 map; inspect $LOG_DIR/map-prepare.log"
    cat "$LOG_DIR/map-prepare.log"
    exit 1
}
stop_owned "$SLAM_PID"
SLAM_PID=""
stop_fortress

# Restart both Fortress and Nav2 under a distinct Ignition partition. This
# verifies disk persistence and map loading rather than reusing the live SLAM map.
NAV_RVIZ_CFG="${SIM_INSTALL_BASE}/linorobot2_navigation/share/linorobot2_navigation/rviz/linorobot2_navigation.rviz"
start_fortress "reuse" "$NAV_RVIZ_CFG" "$G4_WORLD" fortress_test
start_nav2 "$NAV_MAP_PREFIX.yaml"
timeout --kill-after=10s 510s python3 -u "$WORKSPACE_DIR/scripts/verify_phase4_goal.py" \
    --supervisor-pid "$SIM_PID" --readiness-timeout 45 --goal-timeout 180 \
    --report "$LOG_DIR/reachable-goal.json" \
    | tee "$LOG_DIR/reachable-goal.log" || {
    echo "FAIL: bounded Nav2 reachable-goal test; inspect $LOG_DIR/nav2.log and $LOG_DIR/reachable-goal.json"
    if [[ -s "$LOG_DIR/reachable-goal.json" ]]; then
        echo "G4 goal diagnostic report:"
        cat "$LOG_DIR/reachable-goal.json"
    fi
    tail -n 120 "$LOG_DIR/nav2.log"
    exit 1
}
stop_owned "$NAV_PID"
NAV_PID=""
stop_fortress
stop_rosbag
echo "PASS: measured Fortress SLAM map saved and reloaded after a fresh Fortress restart; bounded free-space, obstacle-avoidance, and narrow-passage Nav2 goals passed. Evidence: $LOG_DIR"
