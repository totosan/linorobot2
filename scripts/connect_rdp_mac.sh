#!/usr/bin/env bash
# Auf dem Mac starten und während der RDP-Sitzung laufen lassen.
set -euo pipefail

[[ "$(uname -s)" == Darwin ]] || { echo "Diesen Tunnel auf dem Mac starten, nicht in der VM."; exit 1; }
echo "RDP-Tunnel: Mac 127.0.0.1:13389 / [::1]:13389 -> ros-humble 127.0.0.1:3389"
echo "In Windows App neues PC-Profil mit 127.0.0.1:13389, ohne Gateway, verwenden."
echo "Terminal offen lassen; Ctrl+C beendet den Tunnel."
# OrbStack konfiguriert Hostalias und Schlüssel. Keine Passwortabfrage und
# keine Lockerung der Hostkeyprüfung; beide Listener sind ausschließlich lokal.
exec ssh -N -T \
    -o BatchMode=yes \
    -o StrictHostKeyChecking=yes \
    -o ExitOnForwardFailure=yes \
    -o ConnectTimeout=5 \
    -o ServerAliveInterval=30 \
    -o ServerAliveCountMax=3 \
    -L '127.0.0.1:13389:127.0.0.1:3389' \
    -L '[::1]:13389:127.0.0.1:3389' \
    toto@ros-humble@orb