#!/usr/bin/env bash
# VM-seitige RDP-Brücke: systemd-Socket auf dem OrbStack-Interface, der an das
# unveraendert loopback-gebundene xrdp weiterleitet. Startet mit der VM, damit
# auf dem Mac kein SSH-Tunnel und kein LaunchAgent mehr noetig ist.
#
# Zugriffsgrenze: erreichbar vom Mac und von anderen OrbStack-Maschinen auf dem
# Host-Bridge-Netz. Nicht aus dem Heim-LAN, da der Mac kein IP-Forwarding macht.
# xrdp selbst bleibt auf 127.0.0.1:3389; configure_rdp.py wird nicht angefasst.
set -euo pipefail

UNIT_NAME="linorobot2-rdp-bridge"
MARKER="# X-Linorobot2-Managed: true"
UNIT_DIR="/etc/systemd/system"
SOCKET_UNIT="${UNIT_DIR}/${UNIT_NAME}.socket"
SERVICE_UNIT="${UNIT_DIR}/${UNIT_NAME}.service"
BRIDGE_PORT="${RDP_BRIDGE_PORT:-13389}"
XRDP_TARGET="${RDP_BRIDGE_TARGET:-127.0.0.1:3389}"
PROXY_BIN="/usr/lib/systemd/systemd-socket-proxyd"

usage() {
    echo "Usage: $0 --enable | --disable | --status | --print-units" >&2
    exit 2
}

require_linux_vm() {
    [[ "$(uname -s)" == Linux ]] || {
        echo "FAIL: Diese Bruecke gehoert in die VM, nicht auf den Mac." >&2
        exit 1
    }
}

# Interface-Scoping per BindToDevice= ist in dieser VM nicht nutzbar: OrbStack
# aktiviert keine Netzwerk-Device-Units (sys-subsystem-net-devices-eth0.device
# bleibt auch im gebooteten Zustand inactive), wodurch die automatische
# Device-Abhaengigkeit den Socket beim Boot mit "Dependency failed" abbricht.
# Die VM hat ohnehin nur lo und eth0; die Aussengrenze setzt OrbStack mit
# machines.expose_ports_to_lan=false (Mac-Weiterleitung nur auf Loopback).
render_socket_unit() {
    printf '%s\n' \
        '[Unit]' \
        'Description=Linorobot2 RDP bridge socket (OrbStack host network only)' \
        "$MARKER" \
        '' \
        '[Socket]' \
        "ListenStream=0.0.0.0:${BRIDGE_PORT}" \
        "ListenStream=[::]:${BRIDGE_PORT}" \
        'BindIPv6Only=ipv6-only' \
        'Backlog=8' \
        '' \
        '[Install]' \
        'WantedBy=sockets.target'
}

render_service_unit() {
    printf '%s\n' \
        '[Unit]' \
        "Description=Linorobot2 RDP bridge to loopback xrdp (${XRDP_TARGET})" \
        "$MARKER" \
        "Requires=${UNIT_NAME}.socket" \
        "After=${UNIT_NAME}.socket xrdp.service" \
        'Wants=xrdp.service' \
        '' \
        '[Service]' \
        "ExecStart=${PROXY_BIN} ${XRDP_TARGET}" \
        'DynamicUser=yes' \
        'NoNewPrivileges=yes' \
        'CapabilityBoundingSet=' \
        'AmbientCapabilities=' \
        'PrivateTmp=yes' \
        'PrivateDevices=yes' \
        'ProtectHome=yes' \
        'ProtectSystem=strict' \
        'ProtectKernelTunables=yes' \
        'ProtectKernelModules=yes' \
        'ProtectControlGroups=yes' \
        'RestrictAddressFamilies=AF_INET AF_INET6 AF_UNIX' \
        'RestrictNamespaces=yes' \
        'RestrictRealtime=yes' \
        'LockPersonality=yes' \
        'MemoryDenyWriteExecute=yes'
}

assert_managed() {
    local unit="$1"
    [[ ! -L "$unit" ]] || {
        echo "FAIL: refusing to touch symlink: $unit" >&2
        exit 1
    }
    if [[ -e "$unit" ]] && ! grep -qF "$MARKER" "$unit"; then
        echo "FAIL: refusing to replace unmanaged unit: $unit" >&2
        exit 1
    fi
}

install_unit() {
    local target="$1" content="$2" temporary_file=""
    temporary_file="$(mktemp)"
    printf '%s' "$content" > "$temporary_file"
    sudo install -o root -g root -m 0644 "$temporary_file" "$target"
    rm -f "$temporary_file"
}

[[ $# -eq 1 ]] || usage

case "$1" in
    --print-units)
        echo "### ${SOCKET_UNIT}"
        render_socket_unit
        echo "### ${SERVICE_UNIT}"
        render_service_unit
        ;;
    --enable)
        require_linux_vm
        [[ -x "$PROXY_BIN" ]] || {
            echo "FAIL: systemd-socket-proxyd fehlt: $PROXY_BIN" >&2
            exit 1
        }
        assert_managed "$SOCKET_UNIT"
        assert_managed "$SERVICE_UNIT"
        install_unit "$SOCKET_UNIT" "$(render_socket_unit)"
        install_unit "$SERVICE_UNIT" "$(render_service_unit)"
        sudo systemctl daemon-reload
        sudo systemctl enable --now "${UNIT_NAME}.socket"
        echo "Aktiviert: ${UNIT_NAME}.socket lauscht auf :${BRIDGE_PORT} -> ${XRDP_TARGET}"
        echo "Startet automatisch mit der VM. Auf dem Mac kein Tunnel mehr noetig."
        ;;
    --disable)
        require_linux_vm
        sudo systemctl disable --now "${UNIT_NAME}.socket" 2>/dev/null || true
        sudo systemctl stop "${UNIT_NAME}.service" 2>/dev/null || true
        for unit in "$SOCKET_UNIT" "$SERVICE_UNIT"; do
            [[ -e "$unit" ]] || continue
            assert_managed "$unit"
            sudo rm -- "$unit"
        done
        sudo systemctl daemon-reload
        echo "Deaktiviert: Bruecke entfernt; xrdp bleibt auf ${XRDP_TARGET}."
        ;;
    --status)
        require_linux_vm
        # systemctl liefert fuer "inactive" den Code 3; bei socket-aktivierten
        # Diensten ist das der Normalfall und darf --status nicht abbrechen.
        systemctl is-enabled "${UNIT_NAME}.socket" 2>&1 | sed 's/^/socket enabled: /' || true
        systemctl is-active "${UNIT_NAME}.socket" 2>&1 | sed 's/^/socket active: /' || true
        systemctl is-active "${UNIT_NAME}.service" 2>&1 | sed 's/^/service active: /' || true
        ss -lnt 2>/dev/null | grep -E ":(${BRIDGE_PORT}|3389|3350)\b" \
            || echo "Keine passenden Listener gefunden."
        ;;
    *) usage ;;
esac
