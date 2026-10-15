# G9: reproduzierbarer Simulations-Lifecycle

Dieser Ablauf gilt ausschließlich für die lokale Ubuntu-22.04-ARM64-
Simulationsumgebung mit ROS 2 Humble und Gazebo Fortress. Er startet keine
Hardware, Firmware, Motoren, Nav2-Ziele oder externe Inferenz.

## Kanonische Quelle

Die vollständige Projektquelle liegt im Fork `totosan/linorobot2`. Ein
Release wird nur aus einem sauberen, gepushten Commit dieses Forks erzeugt.
Build-, Install-, Log- und Laufzeitartefakte gehören nicht in Git.

Ein Release liegt unter:

```text
~/linobot2_sim/releases/<release-id>/
  source/
  build/
  install/
  log/
  evidence/release-manifest.json
```

`~/linobot2_sim/active` zeigt auf das aktive validierte Release.
`~/linobot2_sim/previous` hält nach einem Wechsel den vorherigen Stand.
Release-Verzeichnisse werden nicht überschrieben.

## Neues Release aus einem Remote-Commit aufbauen

In der `ros-humble`-VM:

```bash
bash scripts/prepare_sim_release.sh \
  --remote=https://github.com/totosan/linorobot2.git \
  --commit=<vollständiger-commit> \
  --release-id=<eindeutige-id>
```

Der Ablauf:

1. verweigert ein vorhandenes Zielverzeichnis;
2. klont den Remote-Commit in ein neues Release;
3. verlangt einen sauberen Detached-HEAD-Checkout;
4. baut die Simulationspakete in den releaseeigenen Build-/Installpfad;
5. führt die ROS-freie Testsuite aus;
6. schreibt ein Manifest mit Commit, Remote, Plattform- und Paketversionen.

Nach diesem Schritt bleibt die Validierung `pending`. Das Release ist noch
nicht aktivierbar.

## Bewegungslosen Runtime-Smoke ausführen

Vorher sicherstellen, dass in Domain 42 keine andere Simulation läuft. Dann:

```bash
LINOROBOT2_SIM_RELEASE="$HOME/linobot2_sim/releases/<release-id>" \
  ROS_DOMAIN_ID=42 \
  bash "$HOME/linobot2_sim/releases/<release-id>/source/scripts/test_g9_fresh_smoke.sh"
```

Der Smoke startet Fortress headless mit eigener `IGN_PARTITION`, prüft
`/clock`, TF, Lidar und Kamera, verbietet Nav2 sowie unerwartete
`/cmd_vel`-Publisher und prüft den Prozess-Cleanup. Der vorhandene
Command-Timeout-Watchdog darf als einziger Publisher entdeckt werden; ohne
eingehenden Fahrbefehl publiziert er selbst nichts. Nur ein erfolgreicher Lauf
setzt den Manifeststatus auf `passed`.

Nach einem Fehler keinen automatischen Retry ausführen. Zuerst Launch-,
Readiness- und Sensorlog im releaseeigenen `log/g9-smoke-*` auswerten und
Prozess-/Graphreste prüfen.

## Aktivieren und zurückrollen

Aktivieren:

```bash
bash scripts/activate_sim_release.sh <release-id>
```

Rollback:

```bash
bash scripts/rollback_sim_release.sh
```

Beide Befehle brechen ab, wenn ein Linorobot2-Launch, ein Fortress-Prozess oder
ein Supervisor aktiv ist. Aktivierung verlangt ein vollständiges Overlay,
einen sauberen Quellstand im Manifest und `validation.status=passed`. Der
Symlinkwechsel ist atomar. Rollback verweigert einen fehlenden oder ungültigen
`previous`-Stand.

Der erste aktivierte Release hat naturgemäß noch keinen Rückfallstand. Für den
G9-Rollback-Nachweis werden deshalb zwei getrennt validierte Release-
Verzeichnisse aktiviert: erst Referenz, dann Kandidat. Danach wird bei
gestoppter Simulation auf die Referenz zurückgeschaltet und mit
`ros2 pkg prefix linorobot2_gazebo` geprüft, dass das Paket aus deren
Installpfad stammt.

## Umgebung, Start und Stop

Eine neue Shell aktiviert das aktuell gewählte Overlay mit:

```bash
source scripts/enter_sim_env.sh
```

Die stationäre RDP-Ansicht läuft im Vordergrund:

```bash
bash scripts/start_gazebo_rdp.sh
```

Regulärer Stop ist `Ctrl+C` im Startterminal. Bei einem abgelösten Lauf vor
jedem Signal PID, vollständige Kommandozeile und Eltern-/Kindbeziehung erneut
prüfen und nur der eindeutig zugehörigen PID `SIGINT` senden. Keine
prozessnamenbasierten `pkill`-/`killall`-Befehle verwenden.

Nach jedem Stop prüfen:

- kein zugehöriger Supervisor, `ros2 launch` oder `ign gazebo` mehr aktiv;
- keine Publisher mehr auf `/clock`, Kamera- oder Vision-Topics;
- keine zweite Simulation vor dem nächsten Start.

Autostart bleibt standardmäßig deaktiviert und gehört nicht zum G9-Gate.

## Recovery

- **Buildfehler:** Releaseverzeichnis und Logs erhalten, Ursache beheben und
  für den nächsten Versuch eine neue Release-ID verwenden.
- **Runtimefehler:** Logs und Cleanup auswerten; Status nicht auf `passed`
  setzen.
- **Aktivierungsfehler:** `active` bleibt unverändert. Manifest und
  Prozessblocker prüfen.
- **Fehler nach Aktivierung:** alle verwalteten Prozesse stoppen und
  `rollback_sim_release.sh` ausführen. Danach Paketpräfix und Graphzustand
  prüfen.
- **Ungültiges `previous`:** nicht manuell auf einen ungeprüften Pfad
  umbiegen; einen bekannten Release anhand seines Manifests prüfen und
  ausdrücklich aktivieren.

## G9-Abschlusskriterien

G9 ist erst bestanden, wenn:

1. ein gepushter Fork-Commit frisch geklont wurde;
2. Build und vollständige ROS-freie Suite im neuen Release bestanden;
3. der bewegungslose Headless-Smoke samt Cleanup bestand;
4. Aktivierung und Rollback zwischen zwei validierten Releases praktisch
   geprüft wurden;
5. Commit, Versionen, Testbefehle und Ergebnis im Manifest stehen.

Der separate Humble-/Fortress-EOL-Review vor Mai 2027 bleibt Folgearbeit.
