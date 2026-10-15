# Archiviert: Linux-Simulationsdesktop über RDP

> Historischer Setup-Stand vom 05.10.2026. Nicht als aktuelle Betriebsanleitung
> verwenden: RDP-Ziel/Port und Gate-Status sind überholt. Aktuelle Hinweise stehen
> in [`../RDP_GUIDE.md`](../RDP_GUIDE.md).

## Entscheidung

**XFCE + xorgxrdp + Mesa-Software-Rendering** statt XQuartz oder zusätzlichem VNC-Server.
RDP bietet einen vollständigen Desktop, Zwischenablage und einen macOS-Client
(Microsoft Windows App). Xorg und OpenGL laufen innerhalb von Linux, nicht über
XQuartz. Das ist ein geeigneter nächster Grafikpfad, aber noch kein Nachweis für
flüssiges Gazebo/Ogre2: Ohne Linux-GPU kostet Mesa/llvmpipe CPU-Leistung.

## Installation

1. Im bestehenden **VM-Terminal**: `cd /Users/toto/Projects/linobot2_ws`.
2. Als Benutzer `toto`, **nicht das gesamte Skript mit sudo starten**:
   `bash scripts/setup_rdp_env.sh`.
3. Falls das Linux-Konto gesperrt ist oder noch kein Passwort hat:
  `sudo -n passwd toto` direkt in diesem
   Terminal ausführen. Passwort nie in Skripte, Chat oder Konfigurationsdateien schreiben.
   Das Mac-Anmeldepasswort ist nicht automatisch das Linux-Passwort.
4. Auf dem Mac Windows App installieren/öffnen, einen PC mit
   **`127.0.0.1:3389`** hinzufügen, Benutzer `toto`, Linux-Passwort bei Verbindung
   eingeben. OrbStack macht Maschinenserver am Mac über localhost erreichbar.
5. Das selbstsignierte TLS-Zertifikat nur akzeptieren, wenn es zu deiner VM passt;
   das Setup zeigt den SHA256-Fingerabdruck. In der xrdp-Auswahl **Xorg** nutzen.

Startauflösung **1600×900**, ein Bildschirm, keine Retina-Auflösung; später erhöhen.
Audio und Laufwerksumleitung im Client nicht aktivieren, wenn nicht benötigt.
RDP wird nur an `127.0.0.1:3389` gebunden. Sesman bleibt auf Loopback-Port 3350;
je nach xrdp-Build zeigt `ss` dafür `127.0.0.1:3350` oder `[::1]:3350`.
Kein `0.0.0.0`, kein `xhost +`, keine Firewallöffnung und keine Root-Anmeldung.
Login ist auf Mitglieder der Gruppe `linobot2-rdp` beschränkt.

## Grafik und Simulation prüfen

**Ein XFCE-Terminal innerhalb des RDP-Desktops öffnen**, nicht das bisherige
OrbStack-Terminal verwenden:

1. `echo "$DISPLAY"` — muss ein Linux-Display wie `:10.0` sein, kein Mac-Socket
   und nicht `host.orb.internal:0`. Nicht selbst überschreiben.
2. `glxinfo -B` — erwartet Mesa/llvmpipe und für Ogre2 mindestens OpenGL 3.3.
   Bei Fehlern zuerst den Grafikpfad lösen, nicht ROS neu bauen.
3. `cd /Users/toto/Projects/linobot2_ws`.
4. `source scripts/enter_sim_env.sh`.
5. Zuerst `ros2 run rviz2 rviz2` testen und wieder beenden.
6. Dann `ros2 launch linorobot2_gazebo fortress.launch.py` starten.

Die Sitzung setzt `LIBGL_ALWAYS_SOFTWARE=1`, entfernt erzwungenes indirektes GLX
und deaktiviert den XFCE-Compositor. Es werden keine OpenGL-Versionen vorgetäuscht.
Keine Roboter-/Jetson-Konfiguration wird verändert. G1 bleibt bis zu Sensor-,
Neustart- und Stabilitätstests offen.

## Wiederholung, Sicherung und Fehlerdiagnose

- Setup wiederholbar; erste xrdp-Konfigurationen bleiben als
  `/etc/xrdp/*.ini.linobot2-backup` erhalten. Kommentare und wiederholte
  Xorg-Parameter werden bewahrt. Bestehende `.xsession` bleibt unangetastet.
- Der Sitzungsstarter wird nach `~/.local/bin/linobot2-rdp-session` kopiert;
  geänderte Vorversionen werden gesichert. Nach Skriptänderungen Setup wiederholen.
- Setup stoppt bestehende RDP-Sitzungen. Paketinstallation erfolgt mit
  laufzeitmaskierten RDP-Diensten, damit keine unkonfigurierte LAN-Freigabe entsteht.
  Bei Abbruch nach Fehlerbehebung dasselbe Setup erneut ausführen; permanente
  systemd-Masken werden nicht aufgehoben.
- Dienstzustand: `systemctl status xrdp xrdp-sesman --no-pager`.
- Listener: `ss -ltn '( sport = :3389 or sport = :3350 )'`.
- Das Setup wartet bis zu 15 Sekunden auf Listener und eine erfolgreiche
  TCP-Verbindung zu RDP. IPv6-Loopback für sesman ist zulässig; öffentliche oder
  Wildcard-Listener führen sofort zum Abbruch. Systemd-„active“ allein ist kein
  Bereitschaftsnachweis. Ohne erfolgreiches Setup bleiben die Dienste gestoppt
  und laufzeitmaskiert; nach Fehlerbehebung das Setup erneut starten.
- Logs: `sudo -n journalctl -u xrdp -u xrdp-sesman -n 80 --no-pager` und
  `~/.xorgxrdp.*.log`. Logs vor Weitergabe auf private Daten prüfen.
- Falls localhost am Mac nicht verbindet, zuerst Portkonflikt/OrbStack-Weiterleitung
  prüfen; **nicht** den Listener auf alle Interfaces umstellen.
- Deaktivieren: `sudo -n systemctl disable --now xrdp xrdp-sesman`.
- Lokal getestete Skriptsyntax/Konfigurationslogik ersetzt keine echte RDP-Anmeldung
  und keinen OpenGL-/Gazebo-Test nach der Installation.

## RDP erreicht die Anmeldung, aber Login scheitert

Am 05.10.2026 meldete sesman `pam_authenticate failed: Authentication failure`
für `toto`. `passwd -S toto` zeigte Status **L** (Passwortanmeldung gesperrt),
die RDP-Benutzergruppe war korrekt; es gab noch keinen Xorg-Sitzungsstart.
OrbStack-Shell/SSH mit Schlüssel benötigt kein funktionierendes Linux-Passwort.

1. Im **VM-Terminal** `sudo -n passwd toto` ausführen und ein neues Linux-Passwort
  zweimal direkt dort eingeben. Keine Passwörter an den Assistenten senden.
2. `passwd -S toto` muss anschließend **P** statt **L** oder **NP** zeigen.
3. Über das RDP-Profil mit Benutzer **toto**, ohne Domain/Microsoft-Konto und mit
  diesem Linux-Passwort anmelden. Gespeicherte alte Clientpasswörter nicht nutzen.
4. Bei erfolgreicher Anmeldung, aber anschließendem Sitzungsabbruch erst dann
  Xorg-/XFCE-Logs prüfen; ein Passwortfehler ist kein Grafikfehler.

Keine PAM-Prüfung deaktivieren, kein leeres Passwort zulassen und nicht bloß
`passwd -u` ausführen: Ein reguläres Passwort setzen, nicht die Sperre umgehen.
Der Assistent setzt oder verarbeitet das Passwort nicht. Kein Neubuild,
Paket-Reinstall oder RDP-Neustart ist für das Setzen des Passworts erforderlich.

## Windows App meldet 0x104

**Konkrete Ursache am 05.10.2026 gefunden:** Im betroffenen Profil war
`127.0.0.1:13389` nur als Anzeigename gespeichert; der tatsächliche Servername
war weiterhin `Orb-ROS2HumbleVM`. Der Client meldete „Name resolution failed“;
die fehlgeschlagenen Versuche erreichten den Server deshalb nicht. PC-Name und
Anzeigename sind unterschiedliche Felder.

Das passwortfreie Profil **[ros-humble-rdp.rdp](../../ros-humble-rdp.rdp)** setzt
`full address` ausdrücklich auf `127.0.0.1:13389` und verwendet kein Gateway.
Mit laufendem SSH-Tunnel auf dem Mac in Windows App öffnen. Nutzer bestätigt:
Zertifikatswarnung bzw. Anmeldung erscheint jetzt; damit ist 0x104 für dieses
Profil überwunden. Vollständige Desktopanmeldung/OpenGL bleiben separat zu prüfen.

Der Fehler allein unterscheidet keine Firewall-, Client- oder Profilprobleme.
Am 05.10.2026 wurden vom Mac aus TCP, RDP-Negotiation und TLS 1.2 mit dem
VM-Zertifikat als geprüftem Vertrauensanker erfolgreich getestet. OrbStack
lauschte auf dem Mac an `127.0.0.1:3389`. Eine vollständige Client-Anmeldung
ist dadurch noch nicht bestätigt.

1. In macOS **Systemeinstellungen → Datenschutz & Sicherheit → Lokales Netzwerk**
  prüfen, ob **Windows App** (gegebenenfalls **Microsoft Remote Desktop**) erlaubt ist.
  Danach den Client vollständig mit Cmd+Q beenden und erneut öffnen.
2. Ein neues **PC**-Profil erstellen (kein Workspace/Gateway): PC-Name
  **`127.0.0.1:3389`**, Gateway **Kein Gateway**, Benutzer zunächst **Bei Bedarf fragen**.
  Mit dem Linux-Benutzer und dessen Passwort anmelden, nicht dem Microsoft-Konto.
3. Nicht `localhost`, `::1` oder `ros-humble.orb.local` verwenden: In dieser
  Konfiguration ist nur der IPv4-Loopback-Pfad bestätigt; IPv6 und die VM-Domain
  waren nicht erreichbar.
4. Falls 0x104 bleibt, den genauen Zeitpunkt des Versuchs mitteilen und die neuesten
  `/var/log/xrdp.log`-Einträge prüfen. Eine Zertifikatswarnung, ein Loginfehler und
  ein schwarzer Desktop sind unterschiedliche Diagnoseschritte.

### Alternativer Zugriff über lokalen SSH-Tunnel

Beim Clientversuch um 11:00 kam kein neuer Versuch beim Server an. Ein sicherer
Diagnoseschritt ist, die automatische OrbStack-Portweiterleitung zu umgehen:

1. **Auf dem Mac** im Workspace `bash scripts/connect_rdp_mac.sh` starten.
2. Dieses Terminal offen lassen. Windows App: neues PC-Profil
  **`127.0.0.1:13389`**, Gateway **Kein Gateway**, Benutzer bei Bedarf abfragen.
3. Ctrl+C beendet den Tunnel. Wenn Port 13389 schon belegt ist, keinen zweiten
  Tunnel starten; vorhandenen Listener prüfen.

Der Tunnel verwendet den vorhandenen OrbStack-SSH-Schlüssel und bindet nur an
IPv4-/IPv6-Loopback auf dem Mac. Der RDP-Server bleibt unverändert lokal in Linux.
Das umgeht die automatische Weiterleitung, behebt aber nicht zwangsläufig eine
macOS-Clientberechtigung oder fehlerhafte Clientprofile. Eine volle Anmeldung und
die Grafikleistung müssen weiterhin getestet werden.

Keine Firewall global deaktivieren und den Server nicht auf `0.0.0.0` umstellen.
Kurze TCP-/TLS-Probes schließen vor der eigentlichen RDP-Anmeldung; dadurch können
erwartbare „header read error“-/EOF-Meldungen im xrdp-Log entstehen.

Referenzen: [OrbStack-Netzwerk](https://docs.orbstack.dev/machines/network),
[xrdp.ini für Ubuntu 22.04](https://manpages.ubuntu.com/manpages/jammy/en/man5/xrdp.ini.5.html),
[sesman.ini](https://manpages.ubuntu.com/manpages/jammy/en/man5/sesman.ini.5.html).