#!/usr/bin/env bash
# Reversible Humble geometry2 #990 overlay; no simulator or hardware starts.
set -euo pipefail
[[ "$(uname -s)" == "Linux" && -f /opt/ros/humble/setup.bash ]] || {
    echo "FAIL: requires Linux with ROS Humble."; exit 1;
}
ROOT="${HOME}/linobot2_tf2_fix"
SOURCE="${ROOT}/src/geometry2"
COMMIT="9997e969556f5fe5d32ad498d02b3b3473da70b6"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
set +u
source /opt/ros/humble/setup.bash
set -u
mkdir -p "$ROOT"
if [[ ! -e "$SOURCE" ]]; then
    git clone --no-checkout https://github.com/ros2/geometry2.git "$SOURCE"
    git -C "$SOURCE" checkout --detach "$COMMIT"
fi
[[ "$(git -C "$SOURCE" rev-parse HEAD)" == "$COMMIT" ]] || {
    echo "FAIL: source checkout differs from pinned upstream commit."; exit 1;
}
[[ -z "$(git -C "$SOURCE" status --porcelain)" ]] || {
    echo "FAIL: refusing to build modified upstream sources."; exit 1;
}
LOG_DIR="$(mktemp -d "${ROOT}/evidence-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
printf 'upstream=ros2/geometry2\ncommit=%s\nbackport=990\n' "$COMMIT" \
    >"$LOG_DIR/source.txt"
# --allow-overriding belongs to an optional colcon extension, not colcon-core.
OVERRIDE_ARGS=()
COLCON_BUILD_HELP="$(colcon build --help)"
if [[ "$COLCON_BUILD_HELP" == *"--allow-overriding"* ]]; then
    OVERRIDE_ARGS=(--allow-overriding tf2 tf2_ros)
fi
colcon --log-base "$ROOT/log" build --base-paths "$SOURCE" --merge-install \
    --build-base "$ROOT/build" --install-base "$ROOT/install" \
    --packages-select tf2 tf2_ros ${OVERRIDE_ARGS[@]+"${OVERRIDE_ARGS[@]}"} \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    2>&1 | tee "$LOG_DIR/build.log"

# Build upstream's regression against the original headers/ABI. Runtime library
# paths below make the A/B selection explicit, rather than trusting an overlay.
cmake -S "$SCRIPT_DIR/tf2_regression" -B "$ROOT/regression" \
    -DGEOMETRY2_SOURCE="$SOURCE" -DBUILD_TESTING=ON \
    2>&1 | tee "$LOG_DIR/regression-build.log"
cmake --build "$ROOT/regression" --parallel 2 \
    2>&1 | tee -a "$LOG_DIR/regression-build.log"
TEST="$ROOT/regression/tf2_buffer_regression"
FILTER="test_buffer.wait_for_transform_does_not_deadlock_with_set_transform"
# Preserve ROS's architecture/vendor directories (e.g. CycloneDDS libddsc).
# The explicit leading directory still selects the original TF2 for baseline.
OLD_LIBS="/opt/ros/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
NEW_LIBS="$ROOT/install/lib:$OLD_LIBS"
env LD_LIBRARY_PATH="$OLD_LIBS" ldd "$TEST" >"$LOG_DIR/baseline-libraries.txt"
grep -E 'libtf2\.so.*=> /opt/ros/humble/lib/' "$LOG_DIR/baseline-libraries.txt"
set +e
env LD_LIBRARY_PATH="$OLD_LIBS" timeout --kill-after=2s 15s \
    "$TEST" --gtest_filter="$FILTER" >"$LOG_DIR/baseline.log" 2>&1
BASELINE_STATUS=$?
set -e
cat "$LOG_DIR/baseline.log"
if [[ "$BASELINE_STATUS" != 1 ]] || ! grep -F 'Deadlock between waitForTransform' "$LOG_DIR/baseline.log"; then
    echo "FAIL: baseline did not reproduce the specific upstream deadlock (exit=$BASELINE_STATUS)."
    exit 1
fi
env LD_LIBRARY_PATH="$NEW_LIBS" ldd "$TEST" >"$LOG_DIR/fixed-libraries.txt"
grep -F "libtf2.so => $ROOT/install/lib/" "$LOG_DIR/fixed-libraries.txt"
env LD_LIBRARY_PATH="$NEW_LIBS" timeout --kill-after=2s 30s \
    "$TEST" >"$LOG_DIR/fixed.log" 2>&1 || {
    cat "$LOG_DIR/fixed.log"; echo "FAIL: patched upstream buffer suite."; exit 1;
}
cat "$LOG_DIR/fixed.log"
echo "PASS: upstream deadlock reproduced on original libraries; patched buffer suite passed. Evidence: $LOG_DIR"
echo "Activate explicitly after the robot overlay: source $ROOT/install/local_setup.bash"