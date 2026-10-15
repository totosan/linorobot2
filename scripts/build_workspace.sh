#!/usr/bin/env bash
# build_workspace.sh - Baut die Linorobot2-Simulationspakete in ein explizites Release.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/sim_paths.sh
source "${SCRIPT_DIR}/lib/sim_paths.sh"
REPO_ROOT="$(linorobot2_repo_root)"
RELEASE_ROOT="$(linorobot2_release_root)"

echo "=========================================================="
echo " Baue Linorobot2 Workspace (Simulation Humble / Fortress)"
echo " Datum: $(date)"
echo " Quelle: $REPO_ROOT"
echo " Release: $RELEASE_ROOT"
echo "=========================================================="

if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
else
    echo "[FEHLER] /opt/ros/humble/setup.bash nicht gefunden!"
    exit 1
fi

cd "$REPO_ROOT"
mkdir -p "${RELEASE_ROOT}/build" "${RELEASE_ROOT}/install" "${RELEASE_ROOT}/log"

# Bauen mit Symlink-Install
# Hinweis: Hardware-Bringup (micro-ROS etc.) wird für Simulation ignoriert, falls Abhängigkeiten fehlen
colcon --log-base "${RELEASE_ROOT}/log" build --symlink-install \
    --base-paths "$REPO_ROOT" \
    --build-base "${RELEASE_ROOT}/build" \
    --install-base "${RELEASE_ROOT}/install" \
    --packages-select \
        linorobot2_description \
        linorobot2_base \
        linorobot2_gazebo \
        linorobot2_navigation \
        linorobot2_vision \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo "=========================================================="
echo " Build erfolgreich abgeschlossen!"
echo " Aktiviere das Workspace-Overlay mit: source ${RELEASE_ROOT}/install/setup.bash"
echo "=========================================================="
