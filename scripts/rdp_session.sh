#!/usr/bin/env bash
# Wird vom Setup ins Linux-Home kopiert und durch xrdp-sesman gestartet.
set -euo pipefail

case "${DISPLAY:-}" in
    :*|unix:*) ;;
    *) echo "[FEHLER] Kein lokales RDP-X11-Display: ${DISPLAY:-nicht gesetzt}" >&2; exit 1 ;;
esac

# Rendering findet IN Linux statt, nicht über den XQuartz-GLX-Pfad.
export LIBGL_ALWAYS_SOFTWARE=1
unset LIBGL_ALWAYS_INDIRECT MESA_GL_VERSION_OVERRIDE MESA_GLSL_VERSION_OVERRIDE
export XDG_SESSION_TYPE=x11
export XDG_CURRENT_DESKTOP=XFCE
export DESKTOP_SESSION=xfce

if [[ "${1:-}" != --with-dbus ]]; then
    unset SESSION_MANAGER DBUS_SESSION_BUS_ADDRESS
    exec dbus-run-session -- bash "$0" --with-dbus
fi

# Ein zusätzlicher Compositor kostet bei CPU-Rendering unnötig Leistung.
xfconf-query -c xfwm4 -p /general/use_compositing --create -t bool -s false

# Deutsches QWERTZ auch dann, wenn der RDP-Client zunächst ein US-Layout meldet.
xfconf-query -c keyboard-layout -p /Default/XkbDisable --create -t bool -s false
xfconf-query -c keyboard-layout -p /Default/XkbModel --create -t string -s pc105
xfconf-query -c keyboard-layout -p /Default/XkbLayout --create -t string -s de
xfconf-query -c keyboard-layout -p /Default/XkbVariant --create -t string -s ''
setxkbmap -model pc105 -layout de -variant ''
exec startxfce4