#!/usr/bin/env bash
# Auf dem Mac starten und während der RDP-Sitzung laufen lassen.
# Läuft bewusst im Vordergrund, damit launchd denselben Pfad überwachen kann:
#   scripts/configure_rdp_tunnel_autostart.sh --enable
set -euo pipefail

RDP_TUNNEL_PORT="${RDP_TUNNEL_PORT:-13389}"
RDP_TUNNEL_TARGET="${RDP_TUNNEL_TARGET:-toto@ros-humble@orb}"

log() { printf '%s %s\n' "$(date '+%Y-%m-%dT%H:%M:%S%z')" "$*"; }

[[ "$(uname -s)" == Darwin ]] || { echo "Diesen Tunnel auf dem Mac starten, nicht in der VM." >&2; exit 1; }

# Ein zweiter Tunnel auf demselben Port würde mit ExitOnForwardFailure hart
# scheitern und eine laufende RDP-Sitzung verunsichern. Darum vorher prüfen.
existing_pid="$(lsof -nP -iTCP:"${RDP_TUNNEL_PORT}" -sTCP:LISTEN -t 2>/dev/null | head -n 1 || true)"
if [[ -n "$existing_pid" ]]; then
    log "Tunnel auf 127.0.0.1:${RDP_TUNNEL_PORT} läuft bereits (PID ${existing_pid}); kein zweiter Start."
    exit 0
fi

log "RDP-Tunnel: Mac 127.0.0.1:${RDP_TUNNEL_PORT} / [::1]:${RDP_TUNNEL_PORT} -> ros-humble 127.0.0.1:3389"
log "In Windows App ein PC-Profil mit 127.0.0.1:${RDP_TUNNEL_PORT}, ohne Gateway, verwenden."
log "Im Vordergrund gestartet; Ctrl+C beendet den Tunnel."
# OrbStack konfiguriert Hostalias und Schlüssel. Keine Passwortabfrage und
# keine Lockerung der Hostkeyprüfung; beide Listener sind ausschließlich lokal.
exec ssh -N -T \
    -o BatchMode=yes \
    -o StrictHostKeyChecking=yes \
    -o ExitOnForwardFailure=yes \
    -o ConnectTimeout=5 \
    -o ServerAliveInterval=30 \
    -o ServerAliveCountMax=3 \
    -L "127.0.0.1:${RDP_TUNNEL_PORT}:127.0.0.1:3389" \
    -L "[::1]:${RDP_TUNNEL_PORT}:127.0.0.1:3389" \
    "${RDP_TUNNEL_TARGET}"
