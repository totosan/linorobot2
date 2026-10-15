# Handoff: aktueller Startpunkt für einen neuen Chat

Stand: 15.10.2026 · kanonischer Git-Root: `src/linorobot2`

## Zuerst lesen

1. [`README.md`](README.md) und [`OPEN_TASKS.md`](OPEN_TASKS.md) für Dokumentstruktur und nächste Schritte.
2. [`DEVELOPMENT_PLAN.md`](DEVELOPMENT_PLAN.md) ist der verbindliche Simulation-first-Fahrplan.
3. [`IMPLEMENTATION_STATUS.md`](IMPLEMENTATION_STATUS.md) enthält vollständige Laufbelege und technische Historie.
4. Workspace-Instruction: [`robotics-development.instructions.md`](../.github/instructions/robotics-development.instructions.md). Sie gilt allgemein für Robotikentwicklung; AI-/Vision-Spezifika stehen in Abschnitt 6.
5. Bei RDP-Bedarf: [`RDP_GUIDE.md`](RDP_GUIDE.md). Keine alten Laufbefehle aus dem Archiv übernehmen.
6. Für Frischaufbau und Rollback: [`G9_LIFECYCLE.md`](G9_LIFECYCLE.md).

Die alte ausführliche Übergabe bleibt als [`archive/HANDOFF_2026-10-07.md`](archive/HANDOFF_2026-10-07.md) erhalten; sie ist nur historischer Snapshot.

## Verifizierter Stand

- **G1–G5 und lokaler G6:** im SIL bestanden. Runs 42/43 belegen alle vier G4-Szenarien nominal; Runs 44/45 bestehen dieselbe sensor-derived Szenarienmatrix mit `g4-noise-slip-v1` (festem Seed und rosbag). Der G5-Off/Mock-Nav2-Vergleich ist ebenfalls bestanden. G4/G5/G6 lokal sind **SIL-abgenommen**, nicht hardwarevalidiert. G6-Contract-v1-Probe mit genau einem Domain-42-Fotocapture: `log/person_runtime/g6_oneshot_20261013_1759/vision-probe.json`; dabei lief der Mac-gehostete CPU-Detector als `.71`-Protokollzwilling, nicht der echte Jetson. Das ist vom fixturebasierten ROS-`mock`-Backend zu unterscheiden. Ein echter `.71`-TensorRT-Vergleich ist optional: Nutzer kann den Dienst bei Bedarf starten, ein einzelner Inferenzrequest braucht separate Freigabe. `.150` ist für diesen Zweck nicht der richtige Endpunkt.
- **Nächste Arbeit:** G9 ist mit zwei Remote-Frischreleases, Headless-Smokes und
  atomarem Rollback bestanden. Die G3-Diskriminatoren sind abgeschlossen:
  halbierter Physikschritt und Caster-Nullreibung änderten den Faktor nicht
  messbar; Caster-Entfernung war physikalisch ungültig. Der RDP-Reconnect
  bestand mit 1280×800 und ausschließlich lokalen Tunnel-/Guest-Listenern.
  Damit ist keine weitere hardwareunabhängige Arbeit offen;
  Pi-/Jetson-/HIL-Arbeit bleibt nachgelagert.
- **TF2:** gepinnter Humble-Fix manuell per A/B geprüft. Original reproduziert den Deadlock, Overlay besteht 11/11 upstream Buffer-Tests. `/opt/ros` wurde nicht überschrieben.
- **Run 25:** PASS für freie Fahrt und Hindernisumfahrung mit TF2-Fix. Log: `/home/toto/linobot2_sim/log/g4-nav-20261007-134435-LJ0KXV/`. Die Navigation nutzt eine Ground-Truth-assistierte Map-Fixture; kein sensor-only SLAM-/Hardwaretransferbeweis.
- **Run 29:** Mapping und sensor-derived Map-Preparation bestanden (3.144 neue known, 3.768 known, 171 occupied, Mindest-Clearance `0,302 m`). Nav2-Readiness scheiterte wegen doppeltem Fortress-Start im Map-Reuse; TF-Zeitsprünge, sechs inaktive Lifecycle-Nodes, keine Goals.
- **Run 30:** Ein Fortress-Server; Map-Reuse-Readiness PASS. Goal-Preflight stoppte vor Nav2-Action/Fahrt bei Unknown-Zelle `(0,025; 0)` unter dem lidarblinden Start-Footprint. Kein G4-PASS. Sensor-seeded-Preprocessor begrenzt die Ausnahme auf die konfigurierte Start-Footprint-Region; 58 ROS-freie Tests bestehen. Noch kein Runtime-Nachweis für den Seed.
- **Run 31:** Mapping, sensor-seeded Map-Preparation, Map-Reuse/Nav2-Readiness, freie Fahrt und Hindernisumfahrung meldeten alle PASS; JSONs gingen nach Ende der VM-Session verloren.
- **Run 32:** archivierter Zwei-Szenarien-PASS unter `log/g4_runtime/g4-nav-20261008-105004-6Z7nA4/`. Raw map `106×80`, 3.727 known/165 occupied; 11 Unknown-Zellen unter dem konfigurierten Start-Footprint seeded, keine Occupied-Zellen entfernt, keine GT-Trajektorie. Freies Goal `0,0713 m`, Umfahrung `0,0856 m`, ausgeführte Clearance `0,1847 m`; beide `SUCCEEDED` mit Nullbefehl. G4 bleibt offen: enge Passage und unerreichbares Ziel fehlen.
- **Run 33/34:** archiviertes unreachable-Szenario PASS: Action angenommen, Ziel in Unknown, zielbezogener Plannerfehler, `ABORTED`; keine `/cmd_vel`-Samples, Ground-Truth-Maxbewegung `0 m`, sieben/acht Stillstandssamples ohne Drift, Clearance `1,280 m`. No-motion-BT erzeugte keine Recovery-Bewegung. G4 bleibt offen.
- **G4-Passage:** separate `fortress_g4_test.sdf`, Geometrie aus `scripts/phase4_geometry.py` hergeleitet: Gate `1,00 m` = 2 × (0,22 m Radius + 0,15 m Clearance + 0,13 m Seitenbudget), `y=−1,60…−0,60 m`. Nav2: physischer `robot_radius 0,22` (wie Hardware), Inflation `0,45`, SmacPlanner2D `cost_travel_multiplier 2.0`, `allow_unknown false`. Runs 37–41 (0,84 m, Planner-Radius 0,395) bleiben als FAIL archiviert.
- **Run 35:** `log/g4_runtime/g4-nav-20261008-112045-GhzX6B/`. Sensor-Mapping (11.073 neue bekannte Zellen, 5-m-Gate-Fahrt), Map-Preparation und Nav2-Readiness PASS. Erstes Goal `SUCCEEDED`, GT-Abweichung `0,1064 m`, aber alter Verifier verlangte Nullbefehl nach Action-Ende und meldete `zero_count=0`; keine Umfahrung/Passage. Reihenfolge nicht gespeichert, kein rückwirkender Stopnachweis. Verifier verlangt jetzt finales Null nach letztem empfangenen Nichtnullbefehl plus mindestens drei Stillstandssamples nach Terminalstatus (Drift ≤`0,02 m`); auch nach Action-Ende empfangene Befehle zählen in die Folge und müssen von final Null gefolgt werden.
- **Run 36:** gleiche Persistenz unter `log/g4_runtime/g4-nav-20261008-115346-Iz7h2q/`; Mapping (11.010 neue bekannte Zellen), Map-Preparation und Nav2-Readiness PASS, freie Goal-Aktion `SUCCEEDED`. Verifier erkannte danach empfangenes Nichtnull-`/cmd_vel`, konnte wegen fehlender persistierter Command-/Posefolge aber nicht zwischen queued Sample und echter Bewegung unterscheiden. Lauf **FAIL vor Goal 2/3**, keine Prozesse zurückgelassen. Keine Ziele-/Grenzänderung.
- **Run 37:** `log/g4_runtime/g4-nav-20261008-120221-smzYbc/`. Mapping (11.078 neue bekannte Zellen), Map-Preparation und Nav2-Readiness PASS. Alle drei Nav2-Actions `SUCCEEDED`; GT-Abstände `0,0821 / 0,0820 / 0,0296 m`; alle finalen Null-/Stillstandsprüfungen bestanden (max. Drift `0,00069 m`). Gesamtgate **FAIL**, weil der Passageplan bei `(3,989;−1,103)` eine Unknown-Zelle (`−1`) der sensor-derived Map quert; Plan-/Ground-Truth-Clearance-Prüfung nicht beendet. Keine Karteingabe, Ziel-/Geometrie-/Grenzänderung oder Prozesse nach Cleanup.
- **Run 38:** `log/g4_runtime/g4-nav-20261008-122822-xD45du/`. Mapping (11.080 neue Zellen), Map-Preparation/Nav2-Readiness PASS; freie Fahrt `0,0689 m`, Umfahrung `0,1148 m`; beides `SUCCEEDED` mit Stopproof. Gesamtgate **FAIL** am planmäßigen Hindernis-Clearancewert `0,14136 m` (Schwelle bleibt `0,15 m`), vor Goal 3. Rosbag2 **PASS** (75,6 MiB; 438.753 Nachrichten, einschließlich Plan-/Costmap-/Scan-/TF/Odom/Cmd-Vel); Bagzeitpaarung zeigt frische sensor-cleared Live-Costmap-Zellen. Der Nav2-Plan liegt räumlich zu nahe am Hinderniseck; keine Ziel-/Grenzänderung, alle Prozesse bereinigt.
- **Run 39:** `log/g4_runtime/g4-nav-20261008-130020-GBYDdw/`. Goals 1–2 `SUCCEEDED` (`0,0996 / 0,1305 m`); Passage nach 180 s timeout, Fortschritt `0,970 m`, drei Progress-/Costmap-Clear-Zyklen, keine TF-/Bounds-Störung. Rosbag2 109,7 MiB/614.912 Nachrichten. Ursache des Inflatordesigns: `0,45 m > 0,42 m` fester Gate-Halbbreite; planzeitlicher Gate-Costmapquerschnitt hatte keine Low-Cost-Mittellinie. Korrektur jetzt global/lokal `0,40 m`, floor-gerundet aus `0,42 m`/`0,05 m`-Raster; Radius `0,22 m`, Clearance `0,15 m`, Goals/Geometrie bleiben fest.
- Vision Contract v1, synthetische Fixtures und Empty-Mock-Konsument existieren. Ein historischer externer G6-Einbildaufruf lieferte null Detektionen; der spätere lokale G6-Foto-Capture ist positiv bestanden (siehe aktuellen Laufbeleg oben). Externe Live-Provenienz und Hardwaretransfer bleiben offen.
- **Run 42:** `log/g4_runtime/g4-nav-20261008-145048-k4lioS/` **PASS**: Mapping (Clearance `0,193 m`), Reuse, freies Ziel GT `0,1085 m`, Umfahrung GT `0,1205 m` (Plan/GT-Clearance `0,2011/0,2405 m`), Passage GT `0,0442 m`; Plan-Gatequerung einmal `0,255 m`, GT-Gatequerung einmal `0,2798 m`, Live-Costmap ohne Unknown; Stopproofs 7 Samples/Drift 0; Bag 512.753 Nachrichten. Hashes SDF `2bfec5fb5ec9f49e…`, Config `8495c1bf7144f888…`.
- **Run 43:** `log/g4_runtime/g4-nav-20261008-150754-HJjSQk/` **PASS** unreachable: Ziel Unknown, `ABORTED`, 0 Fahrbefehle, 0 Pläne, 7 GT-Samples ohne Bewegung; Bag 13.518 Nachrichten. Keine Prozessreste; 75 ROS-freie Tests.
- **Run 44:** `log/g4_runtime/g4-nav-20261008-154438-w67Hqz/` **PASS** mit Profil `g4-noise-slip-v1`: Seed `20261008`, Lidar σ=`0,01 m`, Gyro σ=`0,01 rad/s`, Beschleunigung σ=`0,1 m/s²`, Rad-μ=`0,8`. Mapping: 63 Updates, 1.026→12.013 bekannte Zellen, 8/8 Segmente, Mindest-Clearance `0,1809 m`; Start-Seed 3 Zellen, 0 belegte Zellen entfernt, keine GT-Karteneingabe. Frei/Umfahrung/Passage `SUCCEEDED` (GT `0,1100/0,1091/0,1129 m`). Umfahrung Plan/GT-Clearance `0,2177/0,2534 m`; Passage `0,2550/0,2045 m`, eine Gatequerung. 8 Nach-Result-Stillstandsproben je Ziel, Drift max. `0,00251 m`; Bag `88,6 MiB`/`505.889` Nachrichten.
- **Run 45:** `log/g4_runtime/g4-nav-20261008-163511-FBup3W/` **PASS** unreachable-only mit derselben Karte und demselben Profil: zielbezogener GridBased-Fehler, `ABORTED`, 0 Pläne/`/cmd_vel`, 7 Stillstandsproben, maximale Bewegung `0 m`, Clearance `1,28 m`; Bag `2,4 MiB`/`13.498` Nachrichten. Keine Runtime-Prozesse zurückgelassen.
- Vision Contract v1, synthetische Fixtures und Empty-Mock-Konsument existieren. Keine bestätigte Live-Jetson-API, echte Inferenz oder G5-Abnahme.

### Jetson-ROS-Container (Inventur und isolierter Build geprüft am 12.10.2026)

- Der Jetson `.71` wird bewusst manuell gestartet; fehlender OS-Autostart ist kein offener Punkt. Das Image `l4t-ros2:humble-v2.4` enthält Ubuntu 22.04/Jammy und ROS 2 Humble. Der Host-Workspace wird nach `/project/linorobot2_ws` gemountet; der Container-Checkout ist Commit `50a640f`.
- Die Image-Shell sourced `/opt/ros/humble/setup.bash` und setzt `LINOROBOT2_BASE=2wd` sowie `LINOROBOT2_LASER_SENSOR=a1`. Das Workspace-Overlay wird beim Robot-Start durch `start.sh` sourced. `ROS_DOMAIN_ID` und `RMW_IMPLEMENTATION` waren beim isolierten Login nicht gesetzt; `ROS_LOCALHOST_ONLY=0`.
- Udev bindet die identischen USB-UART-IDs per physischem Hub-Port an `/dev/rplidar` (`1-2.3`) und `/dev/ttyESP32` (`1-2.4`). Das manuelle Profil mappt ESP32→Container-`ttyUSB0`, LiDAR→`ttyUSB1`, nutzt Domain 44 und reicht nur explizite Devices durch. Am 12.10. wurde der gezielte Quellstand auf `.71` übertragen; der vorhandene `generic_laser.urdf.xacro`-Edit blieb unangetastet. Der isolierte Builder bestand für `linorobot2_base` und `linorobot2_bringup`; Domain-Modul/Installationspfad und `bringup.launch.py --show-args` bestanden. Auch der no-scan-LiDAR-Test bestand: nach 10 s Startup-Grace beendete sich der Monitor mit Status 1, `OnProcessExit` wurde beobachtet, und die Testlaunch endete innerhalb von 25 s. Dies belegt den isolierten Software-Shutdownpfad, nicht reale Sensorik. Kein physischer Hardware-Bringup. Firmwaretransport/Baudrate, Middleware und reale Alias-/Kabelrollen bleiben offen.
- Die gezielte Übertragung änderte nur die geprüften Base-/Bringup-/Docker-Dateien; der bereits lokal geänderte `generic_laser.urdf.xacro` blieb erhalten. Der Builder lief ohne Netzwerk/Device-Passthrough, mit Quellmount read-only; nur der test-only Monitor wurde gestartet, kein Robot-Service oder Motor. Die übrigen Mac-/Jetson-Worktree-Differenzen nicht blind synchronisieren. Der frühere interaktive Runner und `data/bringup.sh` behalten ihre alten Portwerte; den neuen Compose-Runner als einzige konsistente Profilvariante nutzen, bis die Legacy-Pfade separat abgestimmt sind.

## Unmittelbarer nächster Schritt

G4 ist nach Runs 44/45 und G5 nach dem Off/Mock-Vergleich im SIL abgenommen; Noise-/Slip-Profil, Seed, Szenarien und rosbag sind belegt. **Nächster Schritt:** G6 isoliert vervollständigen: ROS-Servicepfad abnehmen, aktive Gewichte/Klassen verifizieren und ein bekanntes Positivobjekt durch Contract v1 prüfen. Jetson-Paketbuild und Launch-Beschreibung sind validiert; ein sensor-/motorführender Hardwarelauf bleibt separat freigabe- und firmwareabhängig. Run 25 bleibt die getrennte GT-assistierte Fixture-Baseline.

Nicht den 0,25-m-Guard einfach erhöhen, `CANCELED` als `ABORTED` werten oder einen Sonder-BT allein zum Erreichen eines PASS einführen. Die gemeinsame Geometriequelle ist ebenfalls offen; vorhandene Geometriewerte sind nicht physisch vermessen.

## Arbeitsgrenzen und Repository

- Nutzer bevorzugt Deutsch, klare Schritte und Sichtung jeder Laufzeitausgabe. Nach einem fehlgeschlagenen manuellen Lauf stoppen; kein automatischer Retry.
- Simulation-first-Arbeit kann weitergehen. Keine Pi-/Jetson-/ESP32-/Motor-/Firmware-/Inferenzaktion ohne separate Freigabe.
- Der kanonische Git-Root ist `src/linorobot2`; dort liegen jetzt auch aktive
  `docs/`, `scripts/`, `.github/` und das RDP-Profil. Die ältere
  Workspace-Kopie bleibt bis zum bestandenen G9-Frischcheckout unangetastet.
  Kein `git clean`, Reset oder pauschales Überschreiben.
- Die aktive Dokumentation liegt in `docs/`; Referenzen und historische Dokumente liegen in `docs/reference/` und `docs/archive/`.
