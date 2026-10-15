#!/usr/bin/env bash
# Configure per-user Xfce autostart for the stationary Fortress RDP view.
set -euo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
LAUNCH_SCRIPT="${WORKSPACE_DIR}/scripts/start_gazebo_rdp.sh"
AUTOSTART_DIR="${XDG_CONFIG_HOME:-${HOME}/.config}/autostart"
DESKTOP_FILE="${AUTOSTART_DIR}/linorobot2-gazebo.desktop"

usage() {
    echo "Usage: $0 --enable | --disable" >&2
    exit 2
}

[[ $# -eq 1 ]] || usage

case "$1" in
    --enable)
        [[ -f "$LAUNCH_SCRIPT" ]] || {
            echo "FAIL: Gazebo launch script is missing: $LAUNCH_SCRIPT" >&2
            exit 1
        }
        [[ ! -L "$DESKTOP_FILE" ]] || {
            echo "FAIL: refusing to replace symlink: $DESKTOP_FILE" >&2
            exit 1
        }
        if [[ -e "$DESKTOP_FILE" ]] && \
            ! grep -q '^X-Linorobot2-Managed=true$' "$DESKTOP_FILE"; then
            echo "FAIL: refusing to replace unmanaged autostart entry: $DESKTOP_FILE" >&2
            exit 1
        fi

        mkdir -p "$AUTOSTART_DIR"
        temporary_file="$(mktemp "${AUTOSTART_DIR}/.linorobot2-gazebo.XXXXXX")"
        trap 'rm -f "$temporary_file"' EXIT
        printf '%s\n' \
            '[Desktop Entry]' \
            'Type=Application' \
            'Name=Linorobot2 Gazebo (Simulation only)' \
            'Comment=Start the stationary Fortress test world on RDP login' \
            "Exec=/usr/bin/env LIBGL_ALWAYS_SOFTWARE=1 ROS_DOMAIN_ID=42 bash ${LAUNCH_SCRIPT}" \
            "TryExec=${LAUNCH_SCRIPT}" \
            'Terminal=false' \
            'OnlyShowIn=XFCE;' \
            'X-GNOME-Autostart-enabled=true' \
            'X-Linorobot2-Managed=true' \
            > "$temporary_file"
        chmod 0644 "$temporary_file"
        mv -f "$temporary_file" "$DESKTOP_FILE"
        trap - EXIT
        echo "Enabled: Gazebo will start on the next Xfce/RDP login."
        echo "Current desktop was not restarted. Disable with: $0 --disable"
        ;;
    --disable)
        if [[ ! -e "$DESKTOP_FILE" ]]; then
            echo "Already disabled: $DESKTOP_FILE"
            exit 0
        fi
        [[ ! -L "$DESKTOP_FILE" ]] || {
            echo "FAIL: refusing to remove symlink: $DESKTOP_FILE" >&2
            exit 1
        }
        grep -q '^X-Linorobot2-Managed=true$' "$DESKTOP_FILE" || {
            echo "FAIL: refusing to remove unmanaged autostart entry: $DESKTOP_FILE" >&2
            exit 1
        }
        rm -- "$DESKTOP_FILE"
        echo "Disabled: future Xfce/RDP logins will not auto-start Gazebo."
        ;;
    *) usage ;;
esac