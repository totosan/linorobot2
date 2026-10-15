#!/usr/bin/env bash
# enter_sim_env.sh - Aktiviert die Simulationsumgebung gemäß docs/DEVELOPMENT_PLAN.md (sim: Domain 42)

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/sim_paths.sh
source "${SCRIPT_DIR}/lib/sim_paths.sh"

# Sicherstellen, dass das Skript gesourct wird
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "Bitte dieses Skript sourcen: source scripts/enter_sim_env.sh"
    exit 1
fi

export ROS_DOMAIN_ID=42
export LINOROBOT2_BASE=2wd
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[OK] ROS Humble gesourct."
else
    echo "[FEHLER] ROS Humble fehlt in dieser Linux-Umgebung. Erst setup_sim_env.sh ausführen."
    return 1
fi

SIM_INSTALL_BASE="$(linorobot2_install_base)"
if [ -f "${SIM_INSTALL_BASE}/setup.bash" ]; then
    source "${SIM_INSTALL_BASE}/setup.bash"
    echo "[OK] Linux-Simulations-Overlay gesourct: ${SIM_INSTALL_BASE}"
else
    echo "[HINWEIS] Noch kein Linux-Overlay unter ${SIM_INSTALL_BASE}; ROS-Basis ist aktiv."
fi

echo "=========================================================="
echo " Simulationsumgebung aktiv:"
echo "   ROS_DOMAIN_ID:        $ROS_DOMAIN_ID (Simulation)"
echo "   LINOROBOT2_BASE:      $LINOROBOT2_BASE"
echo "   RMW_IMPLEMENTATION:   $RMW_IMPLEMENTATION"
echo "=========================================================="
