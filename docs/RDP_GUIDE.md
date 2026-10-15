# Aktueller RDP-Zugriff auf die Simulationsumgebung

Stand: 15.10.2026. Dies beschreibt die vorhandene OrbStack-VM `ros-humble`, nicht eine Neuinstallation.

## Verbindung vom Mac

1. Im Workspace-Terminal auf dem Mac den Tunnel starten und dieses Terminal offen lassen:
   `bash ./scripts/connect_rdp_mac.sh`
2. In Windows App das bestehende RDP-Profil mit Ziel **`127.0.0.1:13389`** öffnen; kein Gateway verwenden.
3. Nicht mit `127.0.0.1:3389` auf dem Mac verbinden. Port 3389 ist der Guest-Loopback-Dienst in der VM; der lokale SSH-Tunnel stellt ihn auf Mac-Port 13389 bereit.
4. Keinen zweiten Tunnel starten, wenn Port 13389 bereits belegt ist. Den vorhandenen Listener prüfen oder das aktive Tunnelterminal weiterverwenden.

## Standardumgebung in jeder neuen VM-Bash

Der kanonische Checkout liegt unter
`/Users/toto/Projects/linobot2_ws/src/linorobot2`. Sobald ein G9-Release aktiv
ist, `source scripts/enter_sim_env.sh` verwenden; das Skript löst
`/home/toto/linobot2_sim/active` auf und fällt nur ohne aktiven Release auf das
bisherige Overlay zurück.

Die interaktive Bash lädt jetzt automatisch ROS Humble, danach das Linux-Workspace-Overlay `/home/toto/linobot2_sim/install`, setzt das Simulationsprofil (`ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `LINOROBOT2_BASE=2wd`) und wechselt nach `/Users/toto/Projects/linorobot2_ws`. Prüfung in einer frisch geöffneten VM-Shell: `echo "$ROS_DISTRO $ROS_DOMAIN_ID $PWD"` sowie `ros2 pkg prefix linorobot2_base`.

Der Autostart gilt nur für interaktive Bash-Shells; Skripte und Prozesse mit eigenem Profil werden nicht global umkonfiguriert. `use_sim_time` bleibt launch-/node-spezifisch. Diese Shell ist fest auf die Simulation (Domain 42) ausgerichtet, nicht für Hardwaresteuerung verwenden. Der markierte Block in `/home/toto/.bashrc` kann durch Löschen zwischen `# >>> linorobot2 simulation shell setup >>>` und `# <<< linorobot2 simulation shell setup <<<` zurückgenommen werden.

## Simulationsdisplay

- Linux-RDP-Display: `:10.0`; die bestätigte Grafik nutzt Mesa/llvmpipe (Software-Rendering).
- Das Profil `ros-humble-rdp.rdp` fordert jetzt fest 1280×800 an und deaktiviert dynamische Auflösungsänderungen. Änderung greift erst nach vollständigem Trennen und erneutem Öffnen des RDP-Profils; Rückbau: Breite/Höhe im Profil wieder auf 1600×900 setzen und `dynamic resolution:i:1` setzen/entfernen.
- Für OrbStack-Aufrufe Display und Software-Rendering explizit in die VM-Umgebung geben, z. B. `orb -m ros-humble env DISPLAY=:10.0 LIBGL_ALWAYS_SOFTWARE=1 ...`.
- Bereitschaftsprüfung in der VM: `orb -m ros-humble python3 /Users/toto/Projects/linobot2_ws/src/linorobot2/scripts/check_rdp_ready.py`.
- Listener ausschließlich an Loopback belassen. Niemals `0.0.0.0`, `xhost +` oder eine Firewallöffnung als Fehlerbehebung einsetzen.
- **Renderlast:** Die aktive Windows-App-Sitzung handelte am 09.10.2026 trotz Profilwert 1600×900 ein xorgxrdp-Display `1920×1481` aus. llvmpipe rendert auf der CPU; Gazebo-GUI kann dabei mehrere CPU-Kerne belegen. xorgxrdp meldete nur den aktuellen Modus und lehnte eine kleinere RandR-Framebuffergröße ab. Bei zu hoher Last in Windows App eine kleinere/feste Desktopauflösung (z. B. 1280×800) wählen und RDP neu verbinden; keine `xhost`-/Firewall-/RandR-Erzwingung verwenden.

## Fortress in Gazebo anzeigen

Im Mac-Terminal innerhalb des Workspaces starten:

`orb -m ros-humble env DISPLAY=:10.0 LIBGL_ALWAYS_SOFTWARE=1 bash /Users/toto/Projects/linobot2_ws/src/linorobot2/scripts/start_gazebo_rdp.sh`

Alternativ direkt in einem **XFCE-Terminal innerhalb des RDP-Desktops** (die VM-`.bashrc` setzt dort bereits ROS und den Workspace):

`bash scripts/start_gazebo_rdp.sh`

Der Start öffnet die stehende Fortress-Testwelt samt 2WD-Roboter **und RViz** im vorhandenen RDP-Desktop. RViz zeigt RobotModel, TF und `/scan` aus `fortress.rviz`, mit Simulationszeit. Der Start setzt und prüft `ROS_DOMAIN_ID=42`, verwendet eine eigene Ignition-Partition, startet **kein Nav2** und sendet keine Fahrziele. Für Software-Rendering ist Mesa `LP_NUM_THREADS` standardmäßig auf 2 begrenzt; überschreiben lässt sich das beim Start z. B. mit `LP_NUM_THREADS=4 bash scripts/start_gazebo_rdp.sh`. Weniger Threads senken CPU-Spitzen, können aber die Bildrate reduzieren. **Gemessene SIL-Momentaufnahme 09.10.2026:** mit `LP_NUM_THREADS=2` lag Gazebo-GUI bei ~192 % CPU, Server ~22 %, RViz ~27 %, Load-Average (1 min) 3,30; vorher ohne Limit waren es ~533 % / ~43 % / ~33 % und Load 15,39. RDP-Auflösung blieb dabei 1920×1481; die 1280×800-Profileinstellung wird erst nach erneutem Verbinden wirksam. Dies ist ein beobachteter Lauf, kein Performance-Gate. Den Start im selben Terminal mit `Ctrl+C` beenden; der Supervisor räumt den zugehörigen Prozessbaum begrenzt auf. Vor erneutem Start die vorige Ansicht beenden, damit keine zweite Simulation läuft. Bei doppelten ROS-Knotennamen oder mehreren `/clock`-Publishern nicht weiter navigieren; zuerst alle älteren Simulator-Launches sauber mit `Ctrl+C` beenden und anschließend genau einen Launch starten.

Falls Gazebo/RViz gerade nicht laufen, den oben genannten Startbefehl **manuell in einem XFCE-Terminal innerhalb der RDP-Sitzung** ausführen. Die aktuelle 1280×800 Profilauflösung greift erst nach erneutem Verbinden; `LP_NUM_THREADS=2` wird beim nächsten Launcher-Start aktiv.

Am 08.10.2026 wurde der gemeinsame Pfad in der aktiven RDP-Sitzung geprüft: Gazebo (`1516×738`) und RViz (`700×525`) sind beide auf `:10.0` gemappt und ihre Window-Pings bestehen. RViz lädt `fortress.rviz`; der Gazebo-Screenshot-Service lieferte eine valide PNG. Aktuelle Sitzung verwendet `fortress_test`. Der Wrapper schreibt pro Start nach `/home/toto/linobot2_sim/log/rdp-gazebo-*/launch.log` und speichert den aktuellen Pfad in `/home/toto/linobot2_sim/log/rdp-gazebo-current.txt`.

### Simulierten Roboter per Tastatur steuern

In der RDP-Sitzung ein XFCE-Terminal öffnen und dort Teleop starten (Terminal muss Tastaturfokus haben):

```bash
source /opt/ros/humble/setup.bash
source scripts/enter_sim_env.sh
export ROS_DOMAIN_ID=42
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.08 -p turn:=0.30
```

Die Humble-Installation wurde mit `teleop_twist_keyboard` geprüft. Tasten: `i` vorwärts, `,` rückwärts, `j`/`l` drehen, `k` stoppen; `u`/`o` und `m`/`.` fahren diagonal. Jede andere Taste sendet ebenfalls Null. `Ctrl+C` beendet Teleop mit abschließendem Nullbefehl. Die Startwerte sind auf `0.08 m/s` und `0.30 rad/s` gesetzt; mit `q/z`, `w/x` und `e/c` lassen sich die Teleop-Maximalwerte verändern—für diesen Test bitte nicht erhöhen. Diese Kommandos gelten ausschließlich für die laufende Simulation in Domain 42; kein Hardware-ROS-Domain verwenden. Nav2 ist in der RDP-Ansicht nicht gestartet. RViz zeigt Diagnose und Visualisierung, steuert den Roboter aber nicht selbst.

### Gazebo automatisch beim nächsten RDP-Anmelden öffnen

Einmalig in der VM ausführen (ändert nur den Autostart des aktuellen Linux-Benutzers; startet/restartet keine RDP-Dienste und lässt den laufenden Desktop unangetastet):

`orb -m ros-humble bash /Users/toto/Projects/linobot2_ws/src/linorobot2/scripts/configure_rdp_gazebo_autostart.sh --enable`

Danach startet die stationäre Gazebo-Testwelt bei jeder **neuen XFCE-/RDP-Anmeldung** automatisch. Es wird kein Nav2/RViz gestartet und es gibt keine Fahrziele. Ausschalten:

`orb -m ros-humble bash /Users/toto/Projects/linobot2_ws/src/linorobot2/scripts/configure_rdp_gazebo_autostart.sh --disable`

Die Änderung gilt erst bei der nächsten Desktop-Anmeldung; sie trennt oder startet die aktive RDP-Sitzung nicht neu. Die Domain-/Displayprüfungen und der überwachte Prozess-Cleanup aus `start_gazebo_rdp.sh` bleiben aktiv.

Der alte Setup-/Fehlerbehebungsstand wurde unverändert nach [`archive/RDP_SETUP_legacy.md`](archive/RDP_SETUP_legacy.md) verschoben. Er enthält historische Anweisungen und ist nicht die aktuelle Betriebsanleitung.
