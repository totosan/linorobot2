#!/usr/bin/env bash
# Manuell in ros-humble ausführen, NICHT mit sudo vor dem gesamten Skript.
set -euo pipefail

[[ "$(uname -s)" == Linux ]] || { echo "Nur in der Linux-VM ausführen."; exit 1; }
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 && "$(uname -m)" == aarch64 ]] || {
    echo "Erwartet: Ubuntu 22.04 ARM64."; exit 1;
}
[[ "$EUID" != 0 ]] || { echo "Als normaler VM-Benutzer ausführen, nicht als root."; exit 1; }
sudo -n true
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RDP_USER="$(id -un)"

# Dauerhafte, bewusst gesetzte Masken nicht aufheben.
for service in xrdp.service xrdp-sesman.service; do
    [[ "$(readlink "/etc/systemd/system/$service" || true)" != /dev/null ]] || {
        echo "[FEHLER] $service ist dauerhaft maskiert; erst Ursache prüfen."; exit 1;
    }
done

echo "[1/5] Installiere XFCE, xorgxrdp und Mesa (bestehende RDP-Verbindungen werden getrennt)."
sudo -n systemctl daemon-reload
if command -v xrdp >/dev/null; then
    sudo -n systemctl stop xrdp.service xrdp-sesman.service
fi
# Verhindert, dass Paket-Postinst-Skripte vor der Loopback-Konfiguration einen
# RDP-Server mit dem Standardlistener auf allen Interfaces starten.
sudo -n systemctl mask --runtime xrdp.service xrdp-sesman.service
trap 'sudo -n systemctl stop xrdp.service xrdp-sesman.service || true; sudo -n systemctl mask --runtime xrdp.service xrdp-sesman.service || true; echo "[FEHLER] Setup abgebrochen. RDP wurde gestoppt und laufzeitmaskiert; nach Behebung dasselbe Skript erneut starten." >&2' ERR
sudo -n apt-get -o APT::Update::Error-Mode=any update
sudo -n env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    xfce4 xfce4-terminal xrdp xorgxrdp xserver-xorg-core \
    dbus-x11 mesa-utils libgl1-mesa-dri ssl-cert python3 openssl

echo "[2/5] Konfiguriere Loopback, TLS und eingeschränkten Login."
sudo -n groupadd -f linobot2-rdp
sudo -n usermod -aG linobot2-rdp "$RDP_USER"
sudo -n usermod -aG ssl-cert xrdp
sudo -n python3 "$SCRIPT_DIR/configure_rdp.py"
sudo -n -u xrdp test -r /etc/xrdp/cert.pem
sudo -n -u xrdp test -r /etc/xrdp/key.pem

echo "[3/5] Installiere lokale XFCE-Sitzung für $RDP_USER."
mkdir -p "$HOME/.local/bin"
SESSION_FILE="$HOME/.local/bin/linobot2-rdp-session"
[[ ! -L "$SESSION_FILE" ]] || { echo "[FEHLER] Sitzungsdatei ist ein Symlink: $SESSION_FILE"; exit 1; }
if [[ -f "$SESSION_FILE" ]] && ! cmp -s "$SCRIPT_DIR/rdp_session.sh" "$SESSION_FILE"; then
    cp -p "$SESSION_FILE" "$SESSION_FILE.backup-$(date +%Y%m%d-%H%M%S)"
fi
install -m 0755 "$SCRIPT_DIR/rdp_session.sh" "$SESSION_FILE"

echo "[4/5] Aktiviere Dienste und prüfe Listener."
sudo -n systemctl daemon-reload
sudo -n systemctl unmask --runtime xrdp.service xrdp-sesman.service
sudo -n systemctl enable xrdp.service xrdp-sesman.service
sudo -n systemctl restart xrdp-sesman.service xrdp.service
systemctl is-active --quiet xrdp.service xrdp-sesman.service
python3 "$SCRIPT_DIR/check_rdp_ready.py"
openssl x509 -in /etc/xrdp/cert.pem -noout -fingerprint -sha256

echo "[5/5] Aktiviere die VM-seitige RDP-Brücke (Autostart ohne Mac-Tunnel)."
# Ab hier ist die loopback-gebundene RDP-Installation vollstaendig und soll bei
# einem Bruecken-Fehler nicht wieder maskiert werden.
trap - ERR
# Bricht ab, wenn OrbStack machines.expose_ports_to_lan=true hat; die Brücke
# waere sonst aus dem gesamten LAN erreichbar.
if ! bash "$SCRIPT_DIR/configure_rdp_bridge.sh" --enable; then
    echo "[WARNUNG] Brücke nicht aktiviert; xrdp läuft weiter auf 127.0.0.1:3389." >&2
    echo "Ursache oben beheben, dann: bash scripts/configure_rdp_bridge.sh --enable" >&2
    echo "Bis dahin als Fallback auf dem Mac: scripts/connect_rdp_mac.sh" >&2
fi

echo "Mac: RDP-Client mit 127.0.0.1:13389 verbinden, Benutzer $RDP_USER, Sitzung Xorg."
echo "Brücke prüfen: bash scripts/configure_rdp_bridge.sh --status (Tunnel-Fallback: scripts/connect_rdp_mac.sh)."
PASSWORD_STATE="$(passwd -S "$RDP_USER" | awk '{print $2}')"
case "$PASSWORD_STATE" in
    L|NP)
        echo "[AKTION ERFORDERLICH] $RDP_USER hat Passwortstatus $PASSWORD_STATE; RDP-Anmeldung ist noch nicht vorbereitet."
        echo "Passwort DIREKT im VM-Terminal setzen: sudo -n passwd $RDP_USER"
        echo "Danach passwd -S $RDP_USER prüfen: Status P erwartet. Kein Passwort hier oder in Dateien speichern."
        ;;
    P) echo "[OK] Linux-Passwort ist gesetzt; erfolgreiche RDP-Anmeldung separat prüfen." ;;
    *) echo "[HINWEIS] Unbekannter Passwortstatus $PASSWORD_STATE; Linux-Konto vor Anmeldung prüfen." ;;
esac
echo "In einem Terminal IM RDP-Desktop: glxinfo -B; danach source scripts/enter_sim_env.sh."
echo "OpenGL, RViz, Gazebo und Bildrate sind noch separat zu prüfen."