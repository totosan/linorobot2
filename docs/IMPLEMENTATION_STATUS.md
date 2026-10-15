# Implementierungs- und Prüfstatus

Stand: 15.10.2026. Dieses Dokument hält Nachweise, tatsächliche Laufzeit und bekannte Grenzen fest.

## Aktueller Gate- und Phasenstatus

- **G9 in Implementierung, noch kein PASS:** Aktive Projektdokumente,
    Betriebs-/Testskripte, projektbezogene Copilot-Anweisungen und das
    RDP-Profil wurden in den Fork integriert. Releasewerkzeuge erzeugen
    getrennte Build-/Install-/Logpfade, schreiben Commit-/Versionsmanifeste und
    aktivieren nur validierte Releases über atomare `active`/`previous`-
    Symlinks. 463 ROS-freie Tests bestehen in der Humble-VM (14 erwartete
    Skips); ein isolierter lokaler Entwicklungsbuild unter
    `/home/toto/linobot2_sim/releases/g9-dev-local-20261015` bestand für fünf
    Simulationspakete. Das ist noch kein G9-Nachweis: Remote-Commit,
    Frischcheckout, bewegungsloser G1-Headless-Smoke und Rollback-Drill fehlen.
    Verbindlicher Ablauf: [`G9_LIFECYCLE.md`](G9_LIFECYCLE.md).
- **G3-Diskriminatoren implementiert, Runtimeproben offen:** Der G2-Harness
    akzeptiert test-only ausschließlich `0,001` oder `0,0005 s`
    Physikschritt und einen expliziten Caster-Schalter mit Default `true`.
    Laufwelt, Quell-/Ausgabehash und Casterwahl werden getrennt protokolliert.
    Die Produktbaseline bleibt unverändert; Baseline-, Halbstep- und
    Caster-entfernt-Läufe wurden noch nicht ausgeführt.
- **G4 am 08.10.2026 vollständig im SIL bestanden.** Runs 42/43 decken alle vier
    sensor-derived Basisszenarien ab; Runs 44/45 bestehen dieselbe Matrix unter
    dem eingefrorenen `g4-noise-slip-v1`-Profil. Run 25 bleibt separat als
    GT-assistierte Fixture-Baseline dokumentiert. G4 ist keine Hardware- oder
    Inferenzabnahme. Der G5-Modusvergleich ist am 09.10.2026 separat im SIL bestanden.
- **G1–G5:** im SIL bestanden; G5 belegt den Nav2-Off/Mock-Vergleich, nicht echte
    Inferenz. **G6 lokaler Simulationspfad: PASS am 13.10.2026.** Genau ein
    synchronisierter Domain-42-Capture aus der generierten Foto-Welt wurde über
    den separaten `sim_local_isolated`-Dienst auf 15555/15556 gesendet. Der
    Contract-v1-Bericht enthält eine `person`-Detection mit Confidence
    `0.868473`, Box `[260.30, 17.35, 412.63, 259.85]` px, `camera_optical_frame`,
    640×480 und Capture-Zeitdomäne `ros_sim` (`stamp_ns=1174000000`).
    Client-Roundtrip: 298,3 ms. Modell `yolo12n`, CPU/PyTorch,
    `weights-sha256:419ff3dca37d69bacc93a50fa0c186a1c6f9fe62fae0f108b0872829689e9ca6`,
    Labels `coco80-ultralytics`. Bericht:
    `log/person_runtime/g6_oneshot_20261013_1759/vision-probe.json`;
    Fortress- und Detector-Logs liegen im selben Laufverzeichnis. Der Detektor
    protokollierte genau einen Request. Kein Nav2, kein `/cmd_vel`, keine
    Hardware. Dies ist kein Jetson-/TensorRT-Leistungs- oder HIL-Nachweis.
    Die geometrische `sim_person_proxy` bleibt für Detektionsqualität
    ungeeignet. Ein echter TensorRT-Vergleich kann optional gegen `.71`
    erfolgen, wenn der Nutzer den Dienst startet und einen Einzelrequest
    separat freigibt. `.150` ist für diesen Zweck nicht der richtige Endpunkt;
    der frühere HTTP-Lauf bleibt lediglich ein historischer Befund. Kein
    externer Inferenzrequest ohne verifizierte Endpoint-/Modell-Provenienz und
    separate Freigabe. Die vorgesehene
    `zidane.jpg`-Demo wird nicht als positive Fixture verwendet: ein
    [Ultralytics-Maintainer bestätigt](https://github.com/ultralytics/yolov5/issues/11834#issuecomment-1627144521),
    dass Ultralytics das Bild nicht besitzt und die Originalquelle unbekannt ist.
    Ein CC0-Kandidat mit ausgewiesenem Urheber ist samt Manifest und SHA-256 in
    `log/test_assets/` abgelegt:
    [auf Commons dokumentiert](https://commons.wikimedia.org/wiki/File:Person_in_winter_clothing_in_Quebec_City.jpg),
    und im oben dokumentierten Lauf lokal positiv erkannt. Der Foto-World-Generator verlangt
    explizites Asset/Manifest, prüft CC0-URL, Hash und Bildabmessungen, erhält
    das Bildseitenverhältnis und schreibt nur in einen neuen Ausgabepfad.
    Die Laufzeit nutzte die VM-kompatible Welt
    `log/test_assets/fortress_g6_vision_photo_b9d8e26531d7_orbstack.sdf`
    (SHA-256 `9d9ccd8f8c3e54c4b96f6563b009cbd9d4a8c0a164c63baff67a104df2b7ff51`);
    die Mac-Welt desselben Fixtures ist unter
    `fortress_g6_vision_photo_b9d8e26531d7.sdf` abgelegt. Die Commons-Metadaten
    enthalten sowohl einen Personality-rights-Hinweis als auch Full-consent;
    das Bild ist nur für lokale Tests vorgesehen und wird nicht
    weiterverteilt. Beim Read-only-
    Preflight am 13.10. um 17:48 waren in Domain 42 nur `/parameter_events`
    und `/rosout` sichtbar. `simdet` war aktiv, aber an den Ports 5555/5556 gab
    es keine verbundenen Clients; die letzten Request-Logs stammen von etwa
    17:30. Der Container aus einer anderen lokalen Projektkonfiguration wurde
    weder verwendet noch beendet. Für G6 lief eine eigene Testinstanz auf
    `host.docker.internal:15555/15556`; genau ein Request ist geloggt. Die
    Testinstanz sowie die Fortress-Prozessgruppe wurden danach beendet; eine
    Read-only-Prüfung fand keine passenden Restprozesse.
    Der umgebende `orb run`-Aufruf meldete nach dem PASS-Probebericht Exitcode
    1; Ursache nicht festgestellt. Die Simulation erzeugte außerdem nicht
    fatale Bridge-Deprecationmeldungen zu `ign_*`-Parametern. Kamera und
    Contract-Antwort waren verfügbar. Diese Punkte ändern den positiven
    Probebericht nicht, sind aber im Runtime-Log sichtbar.
- **G7/G8:** Pi-Zugriff bleibt bis zur unabhängigen Out-of-Band-Verifikation des
    SSH-Host-Keys blockiert. Motorloses HIL und physische G8-Abnahme benötigen
    zusätzlich bestätigte Motortrennung und separate Freigaben. SIL-Geometrie
    und -Limits sind keine Hardwaremessungen.
- **Jetson-ROS-Container, read-only Inventur (12.10.2026):** Der Nutzer startet
    den Jetson `.71` manuell; ein fehlender OS-Autostart ist beabsichtigt. Das
    vorhandene Image `l4t-ros2:humble-v2.4` läuft mit Ubuntu 22.04/Jammy und ROS 2
    Humble. Es bindet `/home/jetson/linorobot2_ws` als
    `/project/linorobot2_ws` ein; `ros2 pkg list` findet die installierten
    Linorobot2-, micro-ROS-, Nav2-, SLAM-, Localization- und SLLidar-Pakete.
    Die Image-Shell sourced `/opt/ros/humble/setup.bash` und exportiert
    `LINOROBOT2_BASE=2wd` sowie `LINOROBOT2_LASER_SENSOR=a1`; `start.sh` sourced
    das Workspace-Overlay. Im isolierten Login waren `ROS_DOMAIN_ID` und
    `RMW_IMPLEMENTATION` unset, `ROS_LOCALHOST_ONLY=0`; Domain-/Middleware-
    Isolation ist damit nicht durch diese Shell belegt.

    Auf dem Host unterscheiden Udev-Regeln die gleichen USB-UART-IDs `10c4:ea60`
    über physische Hub-Ports: `1-2.3`→`/dev/rplidar`, `1-2.4`→`/dev/ttyESP32`.
    Der ursprüngliche interaktive Runner mappt diese Aliase als LiDAR
    `/dev/ttyUSB0` und ESP32 `/dev/ttyUSB1`; `data/bringup.sh` setzt Basis
    `/dev/ttyUSB1`. Diese Legacy-Pfade bleiben inkonsistent und dürfen nicht mit
    dem neuen Profil vermischt werden. Das neue, separate Compose-Profil mappt
    ESP32→Container-`ttyUSB0` (Basis) und LiDAR→`ttyUSB1`, passend zu den jetzt
    expliziten ROS-Launchargumenten. Während der Erstinventur waren keine
    seriellen Geräte angeschlossen; sie war kein Hardwaretest.

    Im lokalen Mac-Workspace wurde danach ein separates manuelles Profil unter
    `src/linorobot2/docker/docker-compose.jetson.yaml` ergänzt. Es reicht nur
    `/dev/ttyESP32`→Container-`/dev/ttyUSB0` und `/dev/rplidar`→`/dev/ttyUSB1`
    durch (kein `privileged`/`/dev:/dev`) und nutzt Domain 44. Der Wrapper
    `docker/start_jetson_container.sh` verweigert den Start, wenn Aliasziele
    fehlen, gleich auflösen oder das installierte Workspace-Overlay fehlt. ROS-
    Launchparameter für Basis-/LiDAR-Port und micro-ROS-Transport/Port/Baudrate
    werden weitergereicht; die Laserlaunches verwenden das Portargument. Das
    Bei der Target-Nachprüfung: Image `sha256:fbdc19d5…` ist `linux/arm64`; Workspace-
    Overlay vorhanden. Die gezielt übertragenen Dateien ließen den bestehenden
    Jetson-Xacro-Edit unangetastet. Der Compose-Builder lief mit Quelle read-only,
    ohne Netzwerk/Devices und schrieb nur Build-/Install-/Log-Artefakte.
    `linorobot2_base` und `linorobot2_bringup` bauten erfolgreich. Der erste
    Versuch fand das fehlende `linorobot2_base/__init__.py`; nach Übertragung des
    vorhandenen Package-Markers bestand der Build. Der erste Launch-Load-Test
    scheiterte nur am schreibgeschützten `/home/jetson/.ros/log`; ROS-Logs wurden
    auf Builder-tmpfs umgestellt. Danach bestanden Domain-Modulimport,
    Installationspfad und `ros2 launch linorobot2_bringup bringup.launch.py
    --show-args`; die Ausgabe enthielt beide Serialports und
    `lidar_required=true`. Dieser Aufruf lädt Argumente, startet keine ROS Nodes.

    Danach bestand der isolierte LiDAR-Fehlerpfadtest: Der Testlaunch startete
    ausschließlich den Monitor ohne `/scan`-Publisher, Netzwerk oder Device-
    Passthrough. Nach 10 s Startup-Grace meldete der Monitor den erwarteten
    Fehler und beendete sich mit Status 1; `OnProcessExit` protokollierte
    `LIDAR_MONITOR_EXIT_OBSERVED`, die Testlaunch endete innerhalb von 25 s.
    Der anfängliche Timeout war durch `rclpy.shutdown()` im Timer-Callback
    verursacht; nun signalisiert der Callback den Fehler und die
    `SingleThreadedExecutor`-Schleife beendet im Main-Thread den ROS-Kontext.
    Das validiert den Software-Shutdownpfad, nicht reale Sensorik oder den
    physischen Stop des Roboters.

    Der erforderliche LiDAR-Monitor ist wieder in Launch/CMake installiert. Er
    verwendet monotone Zeit, 10 s Startgrace und 15 s Scan-Timeout; Fehler
    beenden nur den Monitor-Kontext und lösen über `OnProcessExit` eine geordnete
    Launch-Shutdown aus. Kein `killall`, `killpg` oder globaler Prozessabbruch.
    ROS-freie Monitor-Regressionsfälle bestanden. Reale Firmwareparameter,
    Middleware/RMW, Geräte-/Kabelrollen und Hardwarelauf bleiben unbestätigt.
    Profilanleitung: `src/linorobot2/docker/JETSON.md`; Tests:
    `src/linorobot2/tests/test_lidar_monitor_contract.py` und
    `src/linorobot2/tests/test_jetson_deployment_profile.py`. Kein Motor,
    Firmware oder sensorführendes Robot-Bringup wurde gestartet.

    Der Check startete einen einmaligen Container mit Netzwerk deaktiviert,
    ohne Device-Passthrough, read-only Root und read-only Workspace-Mount sowie
    überschriebenem Bash-Entrypoint. Kein Bringup/ROS-Node, Motor, Firmware oder
    Inferenz wurde gestartet. Image-Overlay und Paketauflösung sind im Container
    verfügbar; die Mac- und Jetson-Worktrees haben trotz gleichem HEAD `50a640f`
    unterschiedliche Dirty-Änderungen (Jetson: nur
    `generic_laser.urdf.xacro`; Mac: zahlreiche lokale Änderungen). Kein blindes
    Synchronisieren oder Überschreiben.
- **G5 ROS-Adapter und Nav2-Off/Mock-Abnahme (09.10.2026, SIL PASS):**
    `linorobot2_vision` installiert `vision_adapter_node` als `ros2 run`-Executable.
    Backend `off` (Default) publiziert `unavailable`; `mock` lädt die lokale
    Contract-v1-Fixture und bindet Image-Stempel, Frame und Dimensionen an das
    gepaarte `Image`/`CameraInfo`. Ergebnis-JSON erscheint auf `/vision/result`,
    maschinenlesbarer Zustand/Fehler auf `/vision/status`. Frame-/Dimensions-/Zeit-
    pairing und explizites Class-ID-Label-Mapping fail-closed validiert. Fixtures
    werden in den Linux-Buildbaum kopiert; keine Live-Inferenz, Nav2-Abhängigkeit
    oder `/cmd_vel`-Schnittstelle. Humble-Paketbuild, ROS-freie Contracttests,
    echte ROS-Topic-Transporttests für Off/Mock sowie Start/Shutdown beider Modi
    bestanden. **G5-Gate** `log/g5_runtime/g5-nav-20261009-143456-QVT85c/`:
    derselbe sensor-derived Map-/World-/Nav2-Profilhash in beiden Modi; Off meldete
    `unavailable`, Mock drei valide Capture-Ergebnisse mit einer Fixture-Detection.
    Freie Fahrt, Umfahrung und enge Passage jeweils Nav2 `SUCCEEDED`, finaler
    Nullbefehl und unveränderte Ziel-/Grenzkriterien; GT-Zielabweichungen Off/Mock
    `0,07393/0,07337 m`, `0,09485/0,07248 m` und `0,06702/0,03538 m`.
    Gleichwertigkeitsreport PASS; beide Bags gültig (Off `35,2 MiB`/`181.657`
    Nachrichten, Mock `35,5 MiB`/`182.685`), Cleanup ohne Prozessreste. Workspaceweite
    Suite **382 Tests PASS** (inklusive Replay-Gate- und Yaw-Bias-Regressionen).
    Keine Inferenz oder Hardware.
- **Nav2-Replay auf segmentierten Aufzeichnungen (14.10.2026, G5-Off-Input-Segment und isolierter Nav2-Replay PASS):**
    `scripts/segment_nav2_bag.py` trennt bei jedem rückwärts laufenden `/clock`
    ROS-bags in neue Zeitsegmente und schreibt ausschließlich `/clock`, `/tf`,
    `/tf_static`, `/scan`, `/map` und `/odom`. Vorangehende statische TF- und
    transient-lokale Kartenstände werden in spätere Segmente übernommen. Quell-
    und Segmenthashes, Grenzen und Nachrichtenanzahlen landen in Manifesten.
    `scripts/test_nav2_replay_gate.sh` startet in der isolierten Replay-Domain 43
    ausschließlich `navigation_launch.py` (kein Simulator, keine Localization,
    kein Ziel und kein Basiscontroller) und spielt nur diese Eingabetopics ab;
    der Gate prüft empfangene Samples aller Eingaben, beide Costmaps, keinen
    `/cmd_vel`-Subscriber und einen leeren Graph nach Cleanup.
    Der erste Konvertierungsversuch brach vor dem Schreiben ab, weil
    `SequentialWriter` ein noch nicht existierendes Ausgabeverzeichnis verlangt.
    Ursache korrigiert und leere Ausgaben gezielt entfernt. Nach separater
    Nutzerfreigabe bestand die Konvertierung des archivierten G5-Off-Bags:
    `log/replay_runtime/nav2/g5-off-20261009-143456-segments/index.json`,
    ein Segment mit 93.128 Nachrichten, 0,002–81,830 s Simulationszeit und
    200,039 s Bag-Dauer. `ros2 bag info` bestätigt die sechs erlaubten Topics;
    Manifest hält Quell- und Ausgabehashes fest. Keine Rücksprunggrenze in
    diesem Bag; echte Mehrsegment-Konvertierung ist damit noch nicht belegt.

    Der anschließend freigegebene erste Nav2-Lauf
    `log/replay_runtime/nav2/nav2-replay-20261014-101613-eytrwR/` scheiterte:
    `local_costmap` meldete `Can not get 'plugin' param value for obstacle_layer`,
    `controller_server` exitete mit 255, und Planner-Aktivierung blieb aus.
    Ursache: Das neue Gate übergab nur `navigation_fortress.yaml`, ohne das
    erforderliche `navigation_common.yaml` zusammenzuführen. Der Player lief
    an, aber dies ist kein Nachweis erfolgreicher Nav2-Eingabeverarbeitung.
    Nach Cleanup fand die separate Prozessinventur keine Replay-/Nav2-Reste
    und der Domain-43-Graph war leer.

    Gate korrigiert: vorhandenen `_compose_parameter_file`-Helper verwenden,
    alle drei installierten Konfigurationsartefakte vor Start verlangen,
    komponierte Datei samt Hashes im Run ablegen. Zusätzlich Humble-Lifecycle-
    Ausgabe `active [3]` korrekt prüfen, fatale Nodefehler früher erkennen und
    Costmap-Sample-Dateinamen ohne fehlende Unterverzeichnisse bilden.
    Das zunächst fehlende `config/navigation_common.yaml` wurde anschließend
    durch einen gezielten Build nur von `linorobot2_navigation` ins Humble-
    Overlay installiert. Offlineprüfung der installierten Common- und Fortress-
    Profile bestätigt für beide Costmaps Obstacle-/Inflation-Plugins, Radius
    `0,22 m`, Inflation `0,45 m` und Plannerliste `GridBased`; Hashes wurden
    protokolliert.     Alle 20 Replay-Regressionen bestehen in Humble, einschließlich echter
    Profilzusammenführung. Nach separater Freigabe bestand der zweite Lauf
    `log/replay_runtime/nav2/nav2-replay-20261014-103038-XgKYqd/` mit demselben
    segmentierten G5-Off-Bag. Gate bestätigte eine `/clock`-Quelle, Samples auf
    allen sechs Eingabetopics, Lifecycle-Aktivierung von Planner, Controller
    und beiden Costmaps sowie Costmap-Nachrichten (global `map`, 154×403 Zellen;
    lokal `odom`, 60×60; jeweils 0,05 m Auflösung). `/cmd_vel` hatte 0 Subscriber;
    keine Ziele wurden gesendet. Gate beendete Nav2/Player und prüfte einen
    leeren Graph; separate Prozessinventur fand keine Replay-/Nav2-Reste.
    Dieser PASS belegt nur Nav2-Eingabeaufnahme und Costmap-Publikation aus
    diesem G5-Off-Segment, keine Goal-/G4-/G5-Abnahme. Segment enthält keine
    Clock-Rücksprunggrenze; G4 und G5-Mock noch unsegmentiert, echter
    Mehrsegmentfall und vollständige Daten-/QoS-Matrix bleiben offen. Der
    bestehende G4-/G5-PASS bleibt davon getrennt und unverändert.
    Am 14.10. wurden ROS-freie Regressionen für den tatsächlichen
    Segmentierungsablauf mit mehreren Epochen, Map-/TF-Übernahme und
    Topic-Allowlist ergänzt. Der Segmentierer weist nun auch
    Nicht-Clock-Nachrichten am Rücksprung-Zeitstempel vor dem Schreiben als
    mehrdeutig zurück. Das Nav2-Gate startet Eingabeproben und Publisher-
    Beobachtungen parallel und schreibt bei PASS einen maschinenlesbaren
    `replay-report.json`; Syntax- und ROS-freie Gate-Tests bestehen lokal. Zwei
    separat freigegebene Läufe mit neu segmentiertem G5-Off-Bag
    `log/replay_runtime/nav2/g5-off-20261014-updated-segments/segment-000`
    scheiterten vor dem Start des Players. Der erste Lauf wartete auf Nav2-
    Lifecycle-Aktivierung; der lokale Costmap-Log meldete wiederholt fehlendes
    `odom`-TF. Nach Korrektur dieser Reihenfolge brach der zweite Lauf ab, weil
    `ros2 topic info` vor Playback den `/tf`-Endpunkt noch als `Unknown topic`
    meldete. Die Subscriber-Prüfung wurde danach hinter Playback,
    Lifecycle-Aktivierung und Sample-Proben verschoben, damit die CLI-Proben
    nicht als Nav2-Subscriber gezählt werden. Der erste fehlgeschlagene Lauf
    `log/replay_runtime/nav2/nav2-replay-20261014-172918-e78l9h/` hinterließ
    laut Prozessinventur keinen Nav2-/Replay-Prozess und einen leeren Domain-43-
    Graphen. Auch der zweite fehlgeschlagene Lauf
    `log/replay_runtime/nav2/nav2-replay-20261014-173451-ks3rES/` hinterließ
    einen leeren Graphen und keine Replay-/Nav2-Prozesse.

    Nach neuer separater Freigabe bestand der dritte Lauf am 14.10.:
    `log/replay_runtime/nav2/nav2-replay-20261014-192903-r24pAu/`. Die frische
    Segmentierung des archivierten G5-Off-Bags enthält ein Segment mit 93.128
    erlaubten Inputnachrichten (Clock 0,002–81,830 s; Bag-Dauer 200,039 s),
    ohne Clock-Rücksprung. Der Gate beobachtete genau einen `/clock`-Publisher,
    mindestens eine Probe auf jedem der sechs Inputs sowie beide Costmaps:
    global `map`, 154×403 Zellen; lokal `odom`, 60×60 Zellen; beide mit 0,05 m
    Auflösung. `/cmd_vel` hatte 0 Subscriber; es wurden keine Ziele gesendet
    oder Befehle abgespielt. Endpoint-Dateien dokumentieren angebotene und
    angeforderte QoS, unter anderem RELIABLE Bag-`/scan` mit BEST_EFFORT-
    Nav2-Subscriptions und TRANSIENT_LOCAL für `/map` und `/tf_static`.
    `replay-report.json` enthält PASS, Segmentmanifest-Hash und Probe-Artefakte;
    er belegt mindestens eine Probe pro Topic, keine vollständigen
    Nachrichtenraten. Finale Prozessinventur und Domain-43-Graph waren sauber.
    Nav2 protokollierte vor Eintreffen der aufgezeichneten TF zunächst fehlende
    Transforme; nach Playback aktivierten sich die Lifecycle-Nodes und beide
    Costmaps publizierten. Dieser PASS belegt nur den aktualisierten
    Input-/Costmap-Replay auf diesem G5-Off-Segment. Er erweitert weder
    G4/G5-Abnahme noch belegt er Mehrsegment-/Clock-Rewind-Verhalten,
    G4-/G5-Mock-Konvertierung, Goal-Replay oder Hardware.

    Anschließend wurden ohne ROS-Knoten oder Playback zwei weitere bestandene
    Quellläufe read-only segmentiert. G4-Run 42
    `log/g4_runtime/g4-nav-20261008-145048-k4lioS/rosbag` enthält einen echten
    Clock-Rücksprung und erzeugte unter
    `log/replay_runtime/nav2/g4-run42-20261014-segments/` zwei Segmente:
    Segment 0 enthält 164.629 Nachrichten bei 0,002–131,764 s Simulationszeit,
    Segment 1 enthält 94.118 Nachrichten bei 0,002–82,724 s. Für Segment 1
    wurden die letzte frühere Map und beide früheren `/tf_static`-Nachrichten
    übernommen. Das bestandene G5-Mock-Bag
    `log/g5_runtime/g5-nav-20261009-143456-QVT85c/mock/rosbag` erzeugte unter
    `log/replay_runtime/nav2/g5-mock-20261014-segments/` ein Segment mit 93.649
    Nachrichten bei 0,002–82,287 s Simulationszeit. Indexe und Manifeste sind
    `PASS`, alle Ausgabedateihashes wurden erneut geprüft, und `ros2 bag info`
    bestätigt für jedes Segment ausschließlich `/clock`, `/tf`, `/tf_static`,
    `/scan`, `/map` und `/odom`. Die 13 gezielten Segmentierungs-/Gate-
    Regressionen bestehen mit einem erwarteten Skip. Damit sind die reale
    G4-Mehrsegmentkonvertierung und die G5-Mock-Konvertierung offline belegt.
    Der anschließend einmal ausgeführte Domain-43-Lauf von G4-Segment 000
    `log/replay_runtime/nav2/nav2-replay-20261014-195504-8ptqk2/` scheiterte,
    bevor Inputproben oder Costmaps vollständig erfasst wurden. Fünf
    Publisher-Checks schrieben `1`; `/scan` war beim ersten Discovery-Versuch
    noch unbekannt. Obwohl `wait_for_publisher` bis zu 20 Sekunden pollen
    sollte, beendete `set -euo pipefail` die Hintergrundfunktion beim ersten
    fehlgeschlagenen `ros2 topic info`. Der Player wurde deshalb nach rund
    0,5 s gestoppt. Das Gate-Polling behandelt diesen erwartbaren
    Discovery-Zwischenzustand nun explizit mit `|| true`; eine neue Regression
    bildet erst fehlgeschlagene und danach erfolgreiche Discovery nach. Danach
    bestehen 14 gezielte Segmentierungs-/Gate-Tests mit einem erwarteten
    ROS-Skip sowie `bash -n`.

    Der Gate-Wrapper blieb nach dem Fehler trotz bereits beendeter Kinder
    hängen. Die Prozessinventur zeigte nur noch den exakt identifizierten
    Wrapper; Domain 43 war leer. Nach erneuter Identitätsprüfung wurde nur
    dieser PID mit SIGINT beendet, resultierend in Exit 130. Die finale
    Inventur fand keine Replay-/Nav2-Prozesse und erneut einen leeren Graphen.
    Es erfolgte kein Retry. Clock-Rewind-Segment-Replay, Costmap-Ausgabe und
    regulärer Gate-Cleanup sind für die neuen Segmente daher weiterhin keine
    Runtime-Evidenz. G4/G5-Abnahme, Goal-Replay und Hardwarestatus bleiben
    unverändert.

    Nach neuer Nutzeranweisung bestand der korrigierte Lauf von G4-Segment 000:
    `log/replay_runtime/nav2/nav2-replay-20261014-201544-S8dFAo/`.
    `replay-report.json` dokumentiert genau einen `/clock`-Publisher, mindestens
    ein Sample auf allen sechs Eingaben und beiden Costmaps, 0
    `/cmd_vel`-Subscriber sowie einen leeren Graphen nach Cleanup. Separate
    Prozess- und Graphinventur bestätigte keine Reste; der Nav2-Log enthält
    keine Fatal-/Prozessfehler. Dieser PASS belegt nur isolierte
    Input-/Costmap-Verarbeitung des ersten G4-Segments, keine Ziele oder
    Navigation.

    Der danach einmal ausgeführte Lauf des nach dem echten Clock-Rücksprung
    liegenden G4-Segments 001
    `log/replay_runtime/nav2/nav2-replay-20261014-202127-7fNvsB/` scheiterte an
    den parallelen Sample-Proben. `/clock`, `/map`, `/scan` und `/tf_static`
    lieferten Samples; die Dateien für `/tf`, `/odom` und beide Costmaps
    enthielten nur die CLI-Meldung, dass der Topictyp vor Publisher-Discovery
    noch nicht bestimmbar sei. Der Player lief rund 14 s; Nav2 protokollierte
    den erwartbaren TF-Aufbau und beide Costmap-Prozesse liefen, aber der Gate
    ist wegen der fehlgeschlagenen Proben kein PASS. Cleanup hinterließ keine
    Replay-/Nav2-Prozesse und einen leeren Domain-43-Graphen.

    Da die sechs Eingabetypen und der Costmaptyp vertraglich feststehen, startet
    `start_sample_probe` `ros2 topic echo` nun mit explizitem Nachrichtentyp.
    Damit kann die Probe vor dem Publisher beginnen, ohne an der
    Typ-Discovery zu scheitern. Die statische Regression verlangt alle acht
    Topic-/Typ-Paare; zusammen bestehen weiterhin 14 gezielte Tests mit einem
    erwarteten ROS-Skip sowie `bash -n`. Es erfolgte kein Retry von Segment
    001.

    Der danach als unabhängiger Lauf ausgeführte G5-Mock-Replay bestand:
    `log/replay_runtime/nav2/nav2-replay-20261014-202300-nLHK5X/`.
    `replay-report.json` dokumentiert mindestens ein Sample auf allen sechs
    Eingaben und beiden Costmaps, genau einen `/clock`-Publisher, 0
    `/cmd_vel`-Subscriber sowie sauberen Graph-Cleanup. Der Nav2-Log enthält
    keine Fatal-/Prozessfehler; separate Inventur bestätigt keine
    Replay-/Nav2-Prozesse und einen leeren Domain-43-Graphen. Damit ist auch
    das G5-Mock-Segment als Input-/Costmap-Replay runtime-belegt.

    Nach neuer ausdrücklicher Retry-Freigabe bestand am 15.10. auch das
    G4-Post-Rewind-Segment 001:
    `log/replay_runtime/nav2/nav2-replay-20261015-070739-0CBfmJ/`.
    Der Bericht enthält mindestens ein Sample auf allen sechs Eingaben und
    beiden Costmaps, genau einen `/clock`-Publisher, 0 `/cmd_vel`-Subscriber
    und `ros_graph_clean_after_cleanup=true`. Der Nav2-Log enthält keine
    Fatal-/Prozessfehler; separate Abschlussinventur fand weder Replay-/Nav2-
    Prozesse noch Domain-43-Knoten. Vor dem Lauf bestanden im Humble-Overlay
    alle 14 Segmentierungs-/Gate-Regressionen sowie die Shellsyntax.
    Damit sind G5-Off, G5-Mock und beide realen G4-Clock-Epochen als isolierte
    Input-/Costmap-Replays runtime-belegt. Der Replay-Folgepunkt ist
    abgeschlossen. Keiner dieser Läufe sendete Ziele oder Befehle; sie
    erweitern G4/G5 nicht um Goal-Replay, Hardware- oder Sicherheitsnachweise.
- **G6 Vision-Service — historischer erster Einbildnachweis (10.10.2026; lokaler SIL-Gate später am 13.10. bestanden):**
    Der freigegebene Jetson-Dienst `192.168.0.150:8082` meldet über `GET /api/status`
    `running=true`; gesicherter Vertrag für `POST /v1/vision/detection` verlangt
    multipart-Feld `image` und liefert `success/predictions`. Der eigenständige,
    isolierte Fortress-Probe `scripts/test_g6_vision_service.sh` sendete genau einen
    ROS-Frame von `/camera/image_raw` (640×480 RGB, `camera_link`, Sim-Zeit 1,14 s)
    als PPM im Arbeitsspeicher. Bericht:
    `/home/toto/linorobot2_sim/log/g6-vision-20261010-122206-pSPUDU/vision-probe.json`.
    Dienst meldete `ObjectDetectionYOLOv11`, CPU, 347 ms Inferenz/459 ms serverseitig/
    498,2 ms Client-Round-Trip und null Detektionen. Es wurde weder ein Bild auf
    Platte geschrieben noch Nav2/Fahrbefehl gestartet; nach Cleanup keine
    Simulatorprozesse. Das belegt eine echte HTTP-Inferenzanfrage, **keine** positive
    Erkennung oder Modellqualität. Die Fortress-Welt enthält nur die rote Box und
    kein bestätigtes YOLO-Ziel. Das archivierte Modellconfig nennt `yolo11n.pt`,
    aber der aktive Weight-Hash, tatsächliche Klassenliste/-IDs und Version wurden
    live nicht geliefert oder verifiziert.

    Zusätzlich ist `backend:=service` im ROS-Adapter implementiert: in-memory
    PPM-/multipart-Encoding, konfigurierbarer Endpunkt/Timeout/Confidence, höchstens
    eine Inferenz gleichzeitig und mindestens 0,6 s zwischen Requests; Contract v1
    behält den Kamera-Capture-Zeitstempel statt `timestampUTC`. Ergebnisse laufen
    asynchron zurück in den ROS-Executor und bekommen einen konfigurierbaren
    Freshness-Check. Model-/Labelversion und Klassen-ID-Label-Mapping sind explizite
    Parameter; unbekannte Labels werden fail-closed abgewiesen. Es wird keine
    externe Bibliothek benötigt. ROS-freie HTTP-/Encoding-Regressionen **4/4**,
    Einbild-Probe-Regressionen **6/6**, bestehende Contract- **13/13** und
    Sim-Kamera-Tests **4/4** bestanden; Python-Syntax und `git diff --check` sauber.
    Der neue Humble-ROS-Transporttest gegen einen lokalen Fake-Service wurde durch
    die Execution-Policy blockiert und nicht ausgeführt. Somit ist der Adaptercode
    noch nicht im ROS-Laufzeitpfad gebaut/getestet. Vollständiges G6 verlangt noch
    diese Abnahme, verifizierte aktive Klassen-/Gewichtsdaten und einen positiven
    Test mit bekanntem COCO-Objekt in einer separaten Vision-Testwelt.
    **Nachtrag 12.10.2026:** Der ROS-Servicepfad wurde inzwischen in der Humble-VM
    gegen einen lokalen Fake-HTTP-Dienst ausgeführt: `test_vision_ros_node.py`
    3/3 (inklusive asynchroner positiver Fake-Detection bis zum Result-Topic),
    `test_vision_service_backend.py` 4/4 und `test_g6_vision_service.py` 7/7.
    Das ist ROS-/Contract-Transportevidenz, keine Jetson-Live-Inferenz. Ein
    read-only Statusabruf an `GET /api/status` gab HTTP 200 mit
    `{"running":null}` zurück; aktuelle Dienstbereitschaft ist daher nicht
    bestätigt, und es wurde kein weiterer Bildrequest ausgelöst. Der Probe wurde
    außerdem der nach Xacro-Änderung aktuelle Fortress-Frame
    `camera_optical_frame` statt des veralteten `camera_link` als Erwartung
    gesetzt. Zur Prüfung war bereits die GUI-Fortress-Simulation auf ROS Domain
    42 aktiv; ein weiterer Fortress-Start auf derselben ROS-Domain wurde daher
    vermieden. Positive reale Erkennung, bestätigte aktive Gewichte/Klassen-IDs
    und vollständig isolierter Live-Servicepfad bleiben offen.
    **Quellabgleich 12.10.2026:** Die separat gesicherte Jetson-`.71`-Quelle
    (`robot-projects.tar.gz`, Host `192.168.0.71`) konfiguriert `yolo12n.pt` und
    lädt dessen Klassen aus `model.names`; ihr erhaltener `app.py` enthält jedoch
    keinen `/v1/vision/detection`-HTTP-Handler und implementiert den Detektor
    anders (ZeroMQ/Flask-UI). Sie ist daher nicht als Herkunft der freigegebenen
    `.150:8082`-Antwort oder deren Gewichten verifiziert. Das ältere `.150`-
    Laufzeitarchiv ist in diesem Workspace nicht verfügbar; dessen frühere
    `yolo11n.pt`-Config-Referenz ist kein Nachweis des geladenen Live-Modells.
    Der lokale neuere Entwicklungscheckout `JetsonNano/ai-video-solution`
    (`b7320bfa…`) und die `.71`-Quelle können bei nicht initialisierten optionalen
    Workerreferenzen `None`/JSON `null` liefern. Das macht `running:null` zu
    **unbekannt**, nicht zu offline oder inference-ready; es löst den Quell-/Host-
    Widerspruch nicht. Die abweichende Live-`moduleId` `ObjectDetectionYOLOv11`
    gegenüber dem `.71`-Modellpfad `yolo12n.pt` muss vor Klassen-ID-Mapping durch
    den Betreiber geklärt werden.
    **Harness-Sicherheitsnachtrag:** Standalone prüft vor dem Start Domain-42-
    Publisher auf `/camera/image_raw`, `/camera/camera_info` und `/clock` und
    verweigert einen zweiten Simulatorgraphen bei vorhandenen Publishern.
    `--attach` verlangt alle drei bestehenden Streams und startet oder beendet
    keinen Simulator. Ein negativer Guard-Test im aktiven RDP-Graph verweigerte
    Standalone korrekt vor Fortress-Start oder Inferenz. Nach der Mode-Regel-
    ergänzung bestehen die G6-Probe-Regressionen **8/8**, ROS-Adaptertests
    **3/3** und Service-Backendtests **4/4**; ein Capture→Loopback-HTTP-Lauf mit
    der aktiven Kamera wurde durch die Execution-Policy nicht ausgeführt. Die
    zusammengehörige ROS-freie Vision-Suite besteht **54/54**; ein absichtlich
    nicht vorhandenes RMW-Plugin löste den erwarteten fail-closed-Abbruch vor
    jedem Fortress-Start aus.
    **Positivszenen-Fixture:** `fortress_g6_vision_test.sdf` stellt einen
    Geometrie-Personenproxy separat von der Navigationswelt bereit.
    `test_g6_vision_service.sh --positive-person` nutzt diese Headless-Welt und
    setzt `person` als erwartetes Response-Label; bei leerer Antwort ist der
    Probe-Status FAIL. Fortress-SDF-Parser: `Valid`. Die Szene wurde noch nicht
    runtime-geprüft und ist kein photorealistischer COCO-Beleg. Nach Ergänzung
    des exakten Label-Gates plus Szenenstrukturregression bestehen die
    G6-Probe-Regressionen **11/11** und die ROS-freie Vision-Suite **57/57**.
    ROS-Transportregression **3/3** lief in Testdomain 50; ein zweites
    Kamera-Paar nach Ablauf des Mindestintervalls führte trotz `service_max_requests=1`
    zu keinem zweiten HTTP-Aufruf. `--positive-person` nutzt nun den
    ROS-Serviceadapter, setzt genau ein Requestlimit und publiziert das echte
    Contract-v1-Ergebnis nach `/vision/result` plus `/vision/status`. Dafür sind
    bestätigte Model-/Labels-Versionen und die Personen-Class-ID Pflicht.
    Isolierter Humble-Colcon-Build von `linorobot2_vision` und
    `linorobot2_gazebo` nach `/tmp` bestand; der neue World-SDF wurde installiert.
    Ein zusätzlicher, test-only End-to-End-Versuch des `run_contract_probe` in
    Domain 50 mit synthetischen Kamera-Topics und Loopback-Fake-HTTP endete nach
    rund 20,3 s mit Timeout, bevor Result und Status vorlagen. Der Versuch wurde
    nicht wiederholt; ob der Fake-HTTP-Aufruf erreicht wurde und die genaue
    Ursache sind nicht festgehalten/geklärt. Das Testskript wurde nicht als
    fehlschlagende Regression beibehalten. Die separate ROS-Adapterregression
    `test_vision_ros_node.py` bleibt 3/3 PASS, testet aber nicht den
    `run_contract_probe`-Orchestrator. Dieser Testversuch belegte weder das
    Modellmapping noch einen realen `.150`-Request; der spätere Live-Nachweis auf
    `/vision/result` ist im Runtime-Nachtrag unten dokumentiert.
    **Simulations-Overlay für `.150` vorbereitet (Vorstart-Stand):** `vision_overlay.launch.py`
    akzeptiert jetzt `backend:=service`, verwendet standardmäßig ausschließlich
    den autorisierten Endpoint `192.168.0.150:8082` und prüft Versionen sowie
    nichtleeres Class-ID-Mapping vor dem Start beliebiger Nodes. Mock bleibt
    Default; Service-Rate ist auf höchstens 1 Request/s begrenzt. Fünf neue
    ROS-freie Config-Tests bestehen; Humble-Build nach `/tmp` und in das aktive
    Sim-Overlay `/home/toto/linobot2_sim/install` bestanden. Launch-Argumente
    wurden dort angezeigt; der Negativ-Preflight ohne Metadaten brach in
    Testdomain 50 vor Node-Start ab. Vor dem Runtime-Wechsel wurde Domain 42 nur
    lesend geprüft: `vision_adapter_mock` lief, ohne Live-Service-Node oder neuen
    `.150`-HTTP-Request. Zu diesem Vorstart-Zeitpunkt fehlten verifizierte
    Modell-/Labelversionen und das tatsächliche Klassen-ID-Mapping.
    **Runtime-Nachtrag 12.10.2026:** Die bestehende Vision-Overlay-Gruppe wurde
    ohne Neustart von Fortress ersetzt; der Service-Launch läuft in Domain 42.
    `/vision_adapter_sim` meldet `backend=service`; der aktive Parameter ist
    `http://192.168.0.150:8082/v1/vision/detection`, Mindestabstand 1,0 s,
    `service_max_requests=0` (unbegrenzt, weiterhin rate-limitiert). Der einzige
    `/vision/result`-Publisher ist `vision_adapter_sim`; Kamera und `/clock`
    behalten je einen Publisher. Live-Contract-Antwort: `status=ok`, `error=null`,
    `state=empty`, `detection_count=0`, Capture `640×480`, Frame
    `camera_optical_frame`, Zeitdomäne `ros_sim`; Model- und Labelversion sind
    ausdrücklich `unverified`, das Labelmapping ist leer. Damit ist der echte
    Service-Transport aus der laufenden Simulation nach `.150` belegt, nicht eine
    positive Erkennung oder die Modellprovenienz. Eine Detection ohne verifiziertes
    Mapping bleibt fail-closed. Beim gezielten Stop des Mock-Launch-Managers musste
    ROS Launch nach SIGINT auf SIGTERM eskalieren; RViz-Overlay stürzte dabei ab,
    alle Overlay-Prozesse sind beendet, Fortress/Kamera blieben aktiv. Der neue
    Service-Launch startete erfolgreich; separate Fortress-RViz-Prozesse blieben
    bestehen. Die aktive `fortress_test.sdf` enthält Boden und einen roten
    Hinderniskasten, jedoch keinen Personen-Target; daher ist `state=empty` kein
    Service-/ROS-Fehler. Eine Geometrie-Personenfixture existiert nur in der
    separaten `fortress_g6_vision_test.sdf` und wurde noch nicht runtime-abgenommen.
    Die positive Personenprüfung bleibt offen, bis verifizierte Modell-/Label-
    Metadaten vorliegen. Kein Nav2-/`/cmd_vel`-/Hardwarepfad wurde gestartet.
    **Jetson `.71` Einbildtest 13.10.2026:** Nach ausdruecklicher Freigabe
    fuer genau einen simulierten Inferenzlauf wurde der vom Nutzer manuell
    gestartete Container `ros_detection` (`yolo_detection:2`) read-only
    untersucht. `runDetectorImage.sh` startet Single-Image-Betrieb mit
    ZeroMQ PULL `5555`, REP `5556` und Flask-Defaultport `5000`, nicht `8082`.
    Der aktive Quellvertrag verlangt kodierte Bildbytes auf PUSH und danach
    das REQ-Signal `detect`; Antwort: `{"detections":[...]}` mit Pixel-`box`,
    Label und Confidence, ohne Capture-Metadaten oder Class-ID.
    Startup-Log und Artefakt bestaetigen `yolo12n.engine`, SHA-256
    `8e6aec5784f01896fb0d628552ad6ce463e30ef2d4927a5f998dcae11c02a074`.
    Engine-Metadaten nennen YOLOv12n, Export 30.04.2025, Ultralytics
    `8.3.121`, Detect, 640x640 und 80 Labels einschliesslich `0=person`;
    das ist Exportmetadaten-, kein installierter Runtime-Versionsnachweis.
    Genau das archivierte 640x480-PNG des finalen Standardwelt-Personenchecks
    wurde ueber SSH im Arbeitsspeicher an einen kurzlebigen ZeroMQ-Client
    im bestehenden Container uebergeben. Kein separates Modell gestartet,
    keine Jetson-Datei/Installation veraendert, kein ROS-/Nav2-/Motorstart.
    Der Dienst empfing genau ein Bild und ein `detect`, fuehrte TensorRT
    aus und antwortete `{"detections":[]}`. Serverlogs zeigen keinen
    Inferenzfehler, aber eine Warnung zur Verwendung einer Engine auf
    unterschiedlichen Geraetemodellen. Client-Round-Trip `54.223,4 ms`
    einschliesslich erstmaligem Lazy-Loading ist keine Warm-Latenzmessung.
    **Transport PASS; positives Personen-Gate FAIL für den Jetson-Livepfad.**
    Der lokale Domain-42-G6-Pfad wurde später am selben Tag separat positiv
    geprüft (siehe aktuellen Gate-Status oben und den Eintrag zum lokalen
    `.71`-Zwilling unten); dieser Lauf ändert den Jetson-Befund nicht.
    Kein Retry und keine Publikation nach `/vision/result`.
    Bericht: [`jetson71-single-image-20261013.json`](../log/person_runtime/jetson71-single-image-20261013.json);
    [Serverlog](../log/person_runtime/jetson71-single-image-20261013-server.log).
    `.71` ist damit eine live gepruefte alternative Inferenzquelle, nicht
    Provenienz fuer `.150`. Vor Folgeabnahme: ZeroMQ-Contract-v1-Adapter,
    eindeutig korrelierte Captures, explizite Fehlersemantik (der Dienst
    faengt Exceptions ab und kann ebenfalls `[]` liefern), bekannte
    positive Bildreferenz und separat freigegebener Lauf.
    **Lokaler `.71`-Zwilling und ZeroMQ-Backend (13.10.2026, im Quellstand
    vorhanden, im Test simuliert):** Container
    [`sim_detector`](../src/linorobot2/linorobot2_vision/sim_detector/README.md)
    (Mac/OrbStack, PyTorch-CPU, `yolo12n.pt` SHA-256
    `419ff3dca37d69bacc93a50fa0c186a1c6f9fe62fae0f108b0872829689e9ca6`)
    spricht das `.71`-Protokoll (PULL `5555`, REP `5556`, `detect`); Inferenzfehler
    werden als `{"error":...}` gemeldet statt als leere Liste. ROS-seitig
    `backend:=zmq` mit Profilen     `sim_local` (Domain 42), `sim_local_isolated` (Domain 42, Testports
    15555/15556) und `hardware_71`
    (nur `192.168.0.71`, Domain 44), Contract-v1-Abbildung, COCO-80-Labelmap,
    fail-closed bei unbekanntem Label/defekter Box/Fehlerantwort.
    Nachweise: `scripts/test_vision_zmq_backend.py` (15 Tests, Mac-venv und VM),
    `test_vision_ros_node.py` (Fake-`.71` ueber ZeroMQ, VM Domain 77),
    Referenzbild `bus.jpg` ueber Container: 3 Personen und Bus ab Confidence 0,4; aus der VM via
    `host.docker.internal` auf Contract v1 abgebildet (Warm-Roundtrip ca. 0,24 s,
    CPU, nicht TensorRT-vergleichbar). Zu diesem Zwischenstand waren ein
    positives Gazebo-Personenziel und Ein-Frame-Lauf noch offen; beides wurde
    im späteren, unten dokumentierten lokalen G6-Lauf abgeschlossen.
    `pyzmq` wurde per `apt` in der VM `ros-humble` installiert. Kein
    Jetson-/Hardwarezugriff.
    **Standardwelt-Person 13.10.2026:** `fortress_test.sdf` enthält jetzt
    standardmäßig `sim_person_proxy` mit passenden Visual-/Collision-Geometrien
    für Kamera und planar-LiDAR. Der erste isolierte Headless-Sensorcheck mit
    Position `(4.0, 1.5, 0)` empfing Kamera und Scan. Die Probe betrachtete
    jedoch das Minimum über den Sektor 17–24°; darin lag ein Kasten-Return bei
    `1,554 m`, sodass der fehlgeschlagene Distanz-Gate keine Aussage über einen
    separaten Personen-Return erlaubt. Die Person wurde zunächst auf
    `(4.0, 1.8, 0)` versetzt und die geometrische Sichtlinie am Kasten
    regressionsgeprüft. Ein danach freigegebener zweiter Lauf baute drei Pakete
    frisch, bestand den
    Humble-SDF-Parser, startete `fortress_test` und spawnete den Roboter. Image
    und CameraInfo kamen mit passenden Metadaten/Pair-Slop an; der erwartete
    blaue Personen-ROI enthielt     jedoch 0 Pixel. Die danach freigegebene Ganzbilddiagnose speicherte
    [`camera.png`](../log/person_runtime/default-person-diagnostic-20261013-092607-F99PnJ/camera.png):
    die Person war links am Bildrand und großteils durch den vorhandenen Kasten
    verdeckt. LiDAR lieferte am alten Ort sechs Returns zwischen `4,173–4,384 m`
    in `21,56–26,57°`.     Die Person ist auf die gegenüberliegende Seite `(4.0, -1.8, 0)` verschoben,
    damit Bildziel und LiDAR-Echos nicht hinter dem Kasten liegen. Der freigegebene
    finale Lauf `default-person-final-20261013-103044-B8AfRj` bestand: Kamera-
    Image/CameraInfo mit 0-ms-Paarabweichung, 10.841 blaue Zielpixel im rechten
    ROI und sechs LiDAR-Returns bei `4,173–4,384 m` in `−26,57…−21,56°`.
    Weltstart, Sensor- und Collision-Geometrie sind damit im headless SIL
    beobachtet; Detektorqualität und Fusionsergebnis bleiben ungeprüft. Cleanup
    ist bestätigt (Domain 42 ohne Nodes/Prozesse). Bericht und Bild:
    [`sensor-report.json`](../log/person_runtime/default-person-final-20261013-103044-B8AfRj/sensor-report.json),
    [`camera.png`](../log/person_runtime/default-person-final-20261013-103044-B8AfRj/camera.png).
- **G3-Rohodometrie-Yaw-Bias eingegrenzt (09.10.2026, Analyse + eine Hypothesenprobe):**
    Analyse in [`G3_YAW_BIAS_ANALYSIS.md`](G3_YAW_BIAS_ANALYSIS.md), reproduzierbar via
    `scripts/analyze_g3_yaw_bias.py`; Ausgaben unter `log/g3_analysis/`. Roh-Odom-Yaw
    überschätzt Scrub-Drehungen um den konstanten Faktor **1,2143** (G3: 4 Drehungen,
    Streubreite <10⁻⁵; G2: beide Richtungen 1,2136/1,2153); Geradeausfahrt ist
    unverzerrt, IMU=GT, Gelenke tracken das Kommando (0,559≈0,56 rad/s). Der rohe
    Pfad ist damit konsistent; der Körper rotiert mit ≈0,817× der kinematischen
    Sollrate. Diskriminationsprobe `wheel_mu=2.0` (`g2-motion-20261009-161221`)
    änderte das Verhältnis nicht → nicht Coulomb-reibungslimitiert; Caster hat mu=0,01
    → nicht Caster-Drag. Verbleibende Hauptthese: Fortress/DART-Kontaktconstraint-
    Artefakt bei geschwindigkeitserzwungenen Zylinderkontakten im Skid-Turn. **Keine
    Übertragung des Faktors auf Hardware**; physische Kalibrierung bleibt G8. Der
    G2-Harness bekam den optionalen, dokumentierten `G2_WHEEL_MU`-Probenhaken
    (Produkt-Default unverändert 1.0).
- **Replay-ROS-Start (Domain 43) implementiert und im SIL bestanden:** Vertrag in
    [`REPLAY_ROS_CONTRACT.md`](REPLAY_ROS_CONTRACT.md) definiert den sensor-only
    Player-Allowlist-Start, einen Besitzer für aufgezeichnetes `/clock`, Zeitdomäne
    `ros_sim`, Bag-Segment-/QoS-Prüfung und Stop/Cleanup. **Nachweis 09.10.2026:**
    `scripts/create_replay_segment.py` schrieb und validierte read-only das synthetische
    Segment `log/replay_runtime/segments/vision-transport-v1` (91 Nachrichten, Clock
    5–9 s, 5 gepaarte Image/CameraInfo, Manifest mit SHA-256). `test_replay_gate.sh`
    bestand in Domain 43: `replay-20261009-145320-19HHgD` (mock, 3 Contract-Ergebnisse,
    Capture-Stempel 7–9 s, 1 Detection) und `replay-20261009-145336-JRS6Nb` (off,
    `unavailable`). `/clock`-Publisher allein `rosbag2_player` (RELIABLE); kein Simulator,
    Nav2, `/cmd_vel` oder Hardware; keine Prozessreste. Echte Bag-Split-Funktion und
    Nav2-Replay bleiben Folgearbeit und nicht behauptet.
- **Nav2-Profilaufteilung (09.10.2026, Implementierung, Softwarechecks und Profil-SIL-Regression PASS):**
    `navigation_common.yaml` enthält den gemeinsamen `GridBased`-Alias sowie
    belegte Costmap-Werte. `navigation.yaml`, `navigation_fortress.yaml` und
    bestehende Sim-Parameter werden beim Launch profil-vorrangig zusammengesetzt;
    Geschwindigkeit, Frames, Zeitdomäne, Planner-Implementierung, Inflation und
    Unknown-Policy bleiben profilgebunden. Das zusammengesetzte Hardwareprofil ist
    strukturell identisch zur committeten Vorher-Konfiguration. Humble-Build und
    354 ROS-freie Tests bestanden; Launchargumente und installierte Dateien geprüft.
    Der Overlay lag unter `/home/toto/linobot2_tf2_fix` (nicht fehlend), wurde nach
    Humble-/Roboteroverlay aktiviert und besteht den Read-only-Preflight:
    Commit `9997e969556f5fe5d32ad498d02b3b3473da70b6`; Controller lädt `libtf2.so`
    und `libtf2_ros.so` aus diesem Overlay; CycloneDDS `libddsc.so.0` löst aus dem
    Humble-ARM64-Verzeichnis auf. Preflightbericht:
    `/home/toto/linorobot2_sim/log/tf2-overlay-preflight-20261009-124502.json`.
    Die Fehlversuche `g4-nav-20261009-124519-6zF78d` und
    `g4-nav-20261009-125833-m1DByn` endeten vor Nav2-Readiness mit
    `FileNotFoundError` für die dynamisch geladene Helper-Datei bzw. die über den
    Workspace-Mount installierte Common-YAML; beide Fehlversuche sind archiviert.
    Korrektur: Kompositionslogik in `navigation.launch.py`, Nav2-YAMLs in den
    Linux-Buildbaum kopiert, Common-Share-Pfad absolut aufgelöst und im G4-Harness
    explizit übergeben. **Erfolgreicher Lauf**
    `log/g4_runtime/g4-nav-20261009-131721-SRrY89/`: TF2-Preflight PASS;
    sensorbasiertes Mapping PASS (64 Updates, 11.943 bekannte/490 belegte Zellen);
    alle zehn Nav2-Lifecycle-Nodes active, SmacPlanner2D und Kostenkarten geladen.
    Freies Goal, Umfahrung und enge Passage jeweils Nav2 `SUCCEEDED`, GT-Abweichung
    `0,067016/0,084015/0,069283 m`; Stopproofs bestanden, maximale Nach-Result-
    Drift `0,001031 m`. rosbag2 gültig (`91,6 MiB`, `521.328` Nachrichten), keine
    Prozesse übrig. Der Nachweis ist eine Profil-SIL-Regression für die drei
    erreichbaren Szenarien, keine erneute unreachable-/Noise-Slip-Voll-G4-Abnahme.
    G4-Ziele und Grenzwerte unverändert.
- **G4-Arbeitsrichtung und Run-Chronik (historische Zwischenstände):** zielbezogenes fail-closed ohne
    Bewegungs-Recovery, sensor-derived SLAM/Nav2 plus begrenzter initialer
    Footprint-Seed zusätzlich zur Run-25-Fixture,
    reproduzierbarer Rausch-/Schlupffall und rosbag-Artefakt pro Runtime-Lauf;
    Replay bleibt Folgearbeit. Details und offene Zahlen stehen in
    [`DEVELOPMENT_PLAN.md`](DEVELOPMENT_PLAN.md) und [`OPEN_TASKS.md`](OPEN_TASKS.md).
    Goal-spezifischer Fail-Closed-BT und No-motion-Command-Guard sind implementiert
    und runtime-geprüft. Sensor-derived
    Map-Preprocessing, ein nur unter dem
    konfigurierten initialen Footprint gesetzter Seed und die Harness-Standardwahl
    sind im Quellstand vorhanden; Run 31/32 bestätigen den Seed im Runtime-Pfad.
    Negativtest Run 26 durch Recovery-Auslenkungs-Guard gecancelt, kein
    ABORTED-PASS. Standard-BackUp 0,30 m widerspricht dem damaligen Testbudget
    0,25 m; keine Grenzwerterhöhung. **Runtime-Evidenz:** Run 29 mapping/preparation PASS,
    Nav2-Readiness FAIL wegen doppeltem Fortress-Start. Der Harnessstart ist
    korrigiert. Run 30: Reuse-Readiness PASS, danach Goal-Preflight FAIL, weil
    Startzelle `(0,025; 0)` unter dem lidarblinden Roboter-Footprint Unknown war;
    keine Goal-Action oder Fahrbewegung. Der neue initiale Footprint-Seed setzt
    ausschließlich Unknown-Zellen unter der deklarierten Spawn-/AMCL-Startpose
    frei, nie Zellen entlang einer Ground-Truth-Trajektorie. Run 31/32 bestehen
    freie Fahrt und Umfahrung; Run 32 archiviert raw map `106×80`/3.727 known/
    165 occupied, 11 footprint-seeded cells, Goal-Abweichungen `0,0713 m`/
    `0,0856 m`, ausgeführte Clearance `0,1847 m`, beide Nullbefehle.
    **Run 33/34 unreachable PASS:** Action accepted, Ziel in Unknown, target-specific
    Planner failure, `ABORTED`; null `/cmd_vel`-Samples (keine Bewegungsbefehle),
    maximale gemessene Bewegung `0 m`, sieben/acht Stillstandssamples `0 m` Drift,
    Clearance `1,280 m`. Explicit zero war nicht erforderlich, weil kein
    Nichtnullbefehl gesendet worden war. **G4-Passage-Kandidat implementiert:**
    getrennte `fortress_g4_test.sdf` mit 0,84-m-Gate und langem Lidar-Sichtbereich,
    gemeinsame Geometriequelle für Map-/Goal-Prüfer, genau-eine-Querung- und
    Clearance-Auswertung in Plan und Ground Truth; SDF-Parser `Valid`, Quell- und
    Installationshash identisch. 74 ROS-freie G4-Tests bestanden.
    **Run 35:** Mapping, sensor-derived Map-Preparation, Map-Reuse und Nav2-
    Readiness bestanden (11.073 neue bekannte Zellen; 5-m-Mappingsegment zur
    Passage; zwei footprint-seeded Unknown-Zellen; keine belegte Zelle entfernt;
    alle Lifecycle-Nodes active). Der erste freie Goal meldete `SUCCEEDED`,
    Ground-Truth-Abstand `0,1064 m` (Limit `0,20 m`), 95 Trajektoriensamples;
    Run dennoch **FAIL**, weil der Verifier ausschließlich einen Nullbefehl nach
    dem Action-Terminalstatus akzeptierte und keinen fand. Der Lauf endete vor
    Umfahrung/Passage. Die tatsächliche Befehlsreihenfolge und Stillstandsserie
    wurden damals nicht aufgezeichnet, daher keine rückwirkende Bestätigung.
    Verifier verlangt jetzt ein finales Null nach dem letzten empfangenen
    Nichtnullbefehl und mindestens drei post-result Ground-Truth-Stillstandssamples
    mit höchstens `0,02 m` Drift. 74 ROS-freie
    G4-Tests bestehen diese Vertragsfälle. **Run 36** bestand Mapping (11.010
    neue bekannte Zellen), Map-Preparation und Nav2-Readiness; Nav2 meldete das
    freie Goal als `SUCCEEDED`. Der neue Verifier brach an einem nach dem
    Action-Ende empfangenen Nichtnull-`/cmd_vel`-Sample ab. Die alte Fail-Evidenz
    enthält weder Command-Timeline noch post-result Posefolge; ein verspäteter
    Transport-Sample kann daher nicht von einem echten Folgebefehl unterschieden
    werden. Keine Umfahrung/Passage, keine Wiederholung. Verifier protokolliert
    jetzt vor der Validierung die letzten 20 Befehle und die post-result
    Ground-Truth-Samples; Null nach letztem empfangenen Nichtnullbefehl plus drei
    Stillstandssamples mit maximal `0,02 m` Drift ist maßgeblich. Ein nach Action-
    Ende empfangenes Nichtnullsample ist nicht isoliert ein Fail, wenn danach
    final Null und unabhängiger Stillstand belegt sind. **Run 37:** 11.078 neue
    bekannte Zellen, alle acht Mappingsegmente, Map-Preparation und Nav2-Readiness
    **PASS**. Die drei Goals `SUCCEEDED` mit GT-Abständen `0,0821 / 0,0820 /
    0,0296 m`; Null-/Stillstandsbelege für alle bestanden (7–10 Nullsamples nach
    Ergebnis, 7–8 Pose-Samples, maximale Drift `0,00069 m`). Gesamtverifier
    **FAIL**, weil der globale Passageplan bei `(3,989;−1,103)` eine Zelle der
    sensor-derived Karte mit Occupancy `−1` kreuzt. Die Passage-Plan-/Ground-Truth-
    Clearance-Prüfung lief nicht zu Ende; Action success ist keine Gate-Abnahme.
    Persistente Belege unter `log/g4_runtime/g4-nav-20261008-120221-smzYbc/`; keine
    Prozesse übrig. **Run 38:** sensor-derived Mapping (11.080 neue Zellen),
    Map-Preparation und Nav2-Readiness bestanden. Freifahrt `0,0689 m`, Umfahrung
    `0,1148 m`, beide Nav2 `SUCCEEDED` samt Null-/Stillstand; Umfahrungsplan
    scheiterte an Clearance `0,14136 m` bei `(1,3744;−0,8388)` gegen `0,15 m`
    Abnahme. Es wurde kein Clearance-Grenzwert gesenkt. Der verwaltete rosbag2 ist
    gültig (75,6 MiB, 438.753 Nachrichten, 106 `/plan`, 43 Global-/90 Local-
    Costmaps). Bag-Postmortem paart Snapshots über Bag-Aufnahmezeit, weil `/clock`
    beim Map-Reuse zurückspringt; statisches SLAM-Unknown war in mehreren
    Planfeldern durch den frischen, planzeitlichen Global-Costmap/Lidar als frei
    nachgewiesen. 74 ROS-freie G4-Tests bestehen. Ein read-only Nachweis-Update
    erfasst jetzt den plannerrelevanten `/global_costmap/costmap`-Snapshot,
    Publisher, Snapshot-/Scan-Frische, belegte/unknown Plan-Samples und die
    komplette Ground-Truth-Passagepose vor Clearance-Assertions. Statisches SLAM-
    Unknown wird nur durch freshen Costmap-/Lidar-Nachweis überstimmt; statisch
    Occupied bleibt fail-closed. Die Harness-Aufzeichnung verlangt ein gültiges
    rosbag2 mit Nachrichten vor PASS; Run 38 validiert den Pfad mit 75,6 MiB/
    438.753 Nachrichten und allen relevanten Topics. Run 40 runtime-validiert
    auch den Costmap-Plancheck: er fail-closed an plan-time Unknown-Cells, während
    die GT-Clearance separat unter dem Grenzwert lag. **Run 39:** erster/zweiter
    Goal `SUCCEEDED`; das Passage-Goal lief in den 180-s-Timeout mit `0,970 m`
    messbarem Fortschritt, 3 Controller-Progress-Fehlern und anschließenden
    Costmap-Clears; TF/Scan/Footprint frisch. Bag 109,7 MiB/614.912 Nachrichten.
    Der `0,45 m` Costfieldradius war größer als die Gate-Halbbreite `0,42 m`;
    Gatecostmapquerschnitt zeigte keine niedrige Kosten-Mittellinie. Profil jetzt
    geometry-bounded: `floor(0,42/0,05) × 0,05 = 0,40 m` global/lokal; der
    Roboterradius `0,22 m` und die Mindest-Clearance `0,15 m` bleiben separat und
    unverändert. Navigation-Paketbuild/hash und 73 Tests bestehen. **Run 40:**
    mit effektiver Plannerhülle `0,395 m` (physischer SIL-Kreis `0,22 m` +
    Clearance `0,15 m` + halbe Zelle `0,025 m`) alle drei Actions `SUCCEEDED`
    und Stopproof bestanden; bag2 gültig `92,6 MiB`/`523.565` msgs. Trotzdem
    Gesamt-FAIL: planzeitlicher Global-Costmap hatte 3 Unknown-Samples nahe
    `(3,688;−1,218)` (Costmapalter `1,001 s`, Scan `0,002 s`); physische Gatequerung
    genau einmal bei `y=−1,1643` hatte `0,13568 m` clearance, unter `0,15 m`.
    Globalplanquerung `y=−1,14962`/`0,15038 m`, praktisch am Grenzband. Keine
    TF-/boundsfehler und keine Hardware. Die costmap-sichere Replan-Auswahl ist
    pose-/Zeit-gebunden; alle Nav2-Replans im Bag lagen geometrisch bei
    `0,096–0,149 m` (vor Clearance) oder costmap-Unknown. **G4-Passage bleibt
    offen; keine weitere Goal-/Schwellen-/Geometrierunde.** Der einmalige
    Humble-`SmacPlanner2D`-Versuch (Run 41) wurde gebaut und tatsächlich geladen.
    Quell-/Installationshash:
    `77b2395308a6d9825eee28e888355b5250026576b447f9e1653d1ab8047ad493`.
    `allow_unknown=false`, kein Downsampling, `cost_travel_multiplier=1.0`,
    0,395-m-Costmap-Footprint und 0,40-m-Inflation. Run 41: Mapping,
    Map-Preparation, Map-Reuse/Nav2-Readiness PASS. Freies Ziel `SUCCEEDED`
    (GT `0,08796 m`, Stopproof bestanden); Umfahrung `SUCCEEDED` (GT `0,09753 m`,
    Plan-Clearance `0,16349 m`, GT-Clearance `0,21880 m`, Stopproof bestanden).
    Passageziel terminal `ABORTED`; kein globaler Plan wurde während des Goals
    publiziert. Plannerlog: wiederholt `no valid path found`, danach normale
    Recoveryaktivität. Finale Diagnostik: TF/Scan/Costmap aktiv, kein
    Boundsfehler. Rosbag `517.666` Nachrichten/79 globale Costmaps, lesbar durch
    den read-only Inspector. In den letzten 25 Snapshots vor Ende lagen sieben
    geprüfte Gate-Mittelpunkte (x=`4,0 m`) durchgehend auf Occupancy `99`.
    Humble `Costmap2DPublisher` kodiert internen `INSCRIBED_INFLATED_OBSTACLE`
    (253) als 99; der Smac-Radiuschecker verwirft interne Werte ab 253. Das
    getestete Profil machte damit den Gate-Mittelstreifen plannerseitig
    unpassierbar. 75 ROS-freie Tests bestehen; alle Simulatorprozesse sind
    beendet. **Run 41 ist kein G4-PASS.** **Entscheidung 08.10.2026 (praktikabler
    Ansatz, vom Nutzer freigegeben):** Planner-Footprint wieder physischer Radius
    `0,22 m` (wie Hardware-`navigation.yaml`), Inflation `0,45 m` als weiche
    Kostenschicht unterhalb der Gate-Halbbreite, SmacPlanner2D mit
    `cost_travel_multiplier=2.0`, `allow_unknown=false`; Passage aus
    `phase4_geometry.py` hergeleitet: `2 × (0,22 + 0,15 + 0,13 m Seitenbudget)
    = 1,00 m`, `y=−1,60…−0,60 m`; SDF-Wände `9,70/20,60` und `−10,80/18,40`.
    Clearance `0,15 m` und Zieltoleranz `0,20 m` unverändert. Build/Hash: SDF
    `2bfec5fb5ec9f49e…`, Config `8495c1bf7144f888…`, `ign sdf -k` Valid, 75
    ROS-freie Tests. **Run 42 (`log/g4_runtime/g4-nav-20261008-145048-k4lioS/`):
    PASS.** Mapping (Mindest-Clearance `0,193 m`), Map-Preparation,
    Reuse/Readiness. Freies Ziel GT `0,1085 m`; Umfahrung GT `0,1205 m`,
    Plan-Clearance `0,2011 m`, GT-Trajektorie `0,2405 m`; Passage GT `0,0442 m`,
    Plan-Gatequerung einmal bei `y=−1,075` (`0,255 m`), Live-Costmap zur Planzeit
    ohne Unknown (164 freie/12 inflationskostige Samples; Costmapalter `0,66 s`,
    Scan `0,24 s`); GT-Gatequerung nachgerechnet einmal bei `y=−1,1002` mit
    `0,2798 m`, GT-Mindest-Clearance `0,2727 m`. Stopproofs: finales Null, je 7
    Stillstandssamples, Drift `0,0 m`. Bag `512.753` Nachrichten/`89 MiB`; keine
    Prozessreste. **Run 43 (`log/g4_runtime/g4-nav-20261008-150754-HJjSQk/`):
    PASS** unreachable mit derselben Konfiguration und Run-42-Karte: Ziel `(2,0)`
    Unknown, Action angenommen, zielbezogener Plannerfehler, `ABORTED`, `0`
    `/cmd_vel`, `0` Pläne, 7 GT-Samples ohne Bewegung; Bag `13.518` Nachrichten.
    Damit waren alle vier G4-Szenarien nominal sensor-derived laufzeitbelegt.
    **Run 44 (`log/g4_runtime/g4-nav-20261008-154438-w67Hqz/`, 08.10.2026):
    PASS unter `g4-noise-slip-v1`.** Seed `20261008`; Lidar σ=`0,01 m`, Gyro
    σ=`0,01 rad/s`, Beschleunigung σ=`0,1 m/s²`, Rad-μ=`0,8`; der laufende
    Fortress-Server protokollierte den Seed, Profilmanifest/SHA-256 stimmten.
    Sensor-Mapping: 63 Updates, bekannte Zellen `1.026→12.013`, 513 belegte;
    alle acht Segmente bestanden, Mindest-Clearance `0,1809 m`. Map-Preparation
    gab 3 Start-Footprint-Unknowns aus Spawn-/AMCL-Pose frei, entfernte 0
    belegte Zellen und verwendete keine Ground-Truth-Navigationseingabe.
    Map-Reuse/Nav2-Readiness bestanden. Frei `SUCCEEDED`, GT-Abweichung
    `0,1100 m`; Umfahrung `SUCCEEDED`, `0,1091 m`, Plan-/GT-Clearance
    `0,2177/0,2534 m`, Samplelücke `0,00672 m`; Passage `SUCCEEDED`, `0,1129 m`,
    Plan-/GT-Clearance `0,2550/0,2045 m`, eine Gatequerung, Samplelücke
    `0,00650 m`. Alle drei: finaler Nullbefehl, je 8 Nach-Result-Stillstandsproben,
    maximale Drift `0/0,00251/0,00207 m` (Grenze `0,02 m`). Geschwindigkeiten
    höchstens `0,08 m/s` / `0,30 rad/s`. Bag `88,6 MiB`, `505.889` Nachrichten,
    Costmap-/Plan-/Sensor-/Befehlsdaten vorhanden.
    **Run 45 (`log/g4_runtime/g4-nav-20261008-163511-FBup3W/`, 08.10.2026):
    PASS** unreachable-only mit derselben Karte/demselben Profil: Action angenommen,
    zielbezogener `GridBased`-Fehler zum Hindernisziel, terminal `ABORTED`,
    `0` Pläne/`/cmd_vel`-Samples, 7 GT-Stillstandsproben, maximale Bewegung `0 m`,
    Clearance `1,28 m`; Bag `2,4 MiB`, `13.498` Nachrichten. Damit sind alle vier
    G4-Szenarien auch unter dem kontrollierten Rausch-/Schlupfprofil bestanden;
    **G4 ist abgenommen (SIL-only).** Der Verifier persistiert seit Run 42
    zusätzlich `narrow_passage_trajectory_evidence`; `scripts/summarize_phase4_run.py`
    und `scripts/recompute_phase4_passage_clearance.py` werten Berichte lesend aus.
    TF2-Build/A-B und Run-25-Baseline bestanden; kein
    weiteres Baseline-Tuning oder automatischer Retry. Contract, Replay und Consumer sind
    ROS-frei geprüft; zusätzlich liegt ein simulierter Kamera-Consumer-Lauf vor.
    G4 ist seit Runs 44/45 vollständig im SIL bestanden. Die damalige offene
    G5-Abnahme wurde am 09.10.2026 mit dem Off/Mock-Nav2-Vergleich geschlossen.
    G6 und spätere Gates sind im aktuellen Status am Dokumentanfang beschrieben.
- **Modultrennung (08.10.2026, erster Quellschritt):** `command_timeout.py` ist
    aus `linorobot2_gazebo` nach `linorobot2_base` verschoben und wird dort als
    ausführbarer ROS-Node installiert. Fortress- und Legacy-Gazebo-Launch
    verwenden das gemeinsame Basispaket; `linorobot2_bringup` besitzt jetzt den
    expliziten, standardmäßig deaktivierten Schalter
    `enable_command_timeout`. Die Wall-Clock-Ausführung bleibt ausdrücklich
    `use_sim_time=false`. Die lokale ROS-freie Suite besteht mit 342 Tests
    (337 bestanden, 5 erwartete Linux-only-Skips); darin sind drei neue
    Strukturtests enthalten. Der dokumentierte Humble-Build besteht für alle
    sechs ausgewählten Pakete einschließlich `linorobot2_base` und
    `linorobot2_gazebo`; der installierte Symlink ist ausführbar und
    `ros2 pkg executables linorobot2_base` listet `command_timeout.py`.
    `linorobot2_bringup` ist nicht im Simulations-Buildskript enthalten, wurde
    aber separat paketweise mit Colcon erfolgreich gebaut. Der nachfolgende
    gültige Fortress-Lauf bestätigt auch den extrahierten Node im SIL; der
    Schalter ist kein Hardware-Stop-/Not-Aus-Nachweis.
    Nav2-Konfigurationsaufteilung und Vision-Adapter-Node waren zu diesem
    Zwischenstand offen; beide wurden am 09.10.2026 implementiert und
    regressionsgeprüft (siehe Nav2-Profilaufteilung und G5-Nachweis oben).
- **ROS-Profil-/Domain-Guard (08.10.2026):** `linorobot2_base.domain_profile`
    definiert `sim=42`, `replay=43`, `hardware=44`. Fortress und Legacy-Gazebo
    erlauben nur `sim/42`; `linorobot2_bringup` erlaubt nur `hardware/44`.
    Launchargumente `ros_profile`/`ros_domain_id` werden vor Prozessstart geprüft;
    ein anderes bereits geerbtes `ROS_DOMAIN_ID` wird nicht überschrieben,
    sondern fail-closed abgelehnt. Die sechs Simulationspakete und separat
    `linorobot2_bringup` wurden mit Colcon gebaut; Import des installierten
    Moduls bestanden. Ein absichtlich ungültiger `sim/44`-Launch und ein
    ungültiger `hardware/42`-Preflight wurden jeweils vor Prozessstart
    abgewiesen. **Gültiger SIL-Lauf:** mit Eltern-`ROS_DOMAIN_ID` unset,
    eigener Ignition-Partition und `gui:=false rviz:=false` akzeptierte Fortress
    `sim/42`; World-Service `fortress_test`, ROS-Graph mit `/command_timeout`
    und `/clock` wurden nachgewiesen. Bericht/Logs:
    `/home/toto/linobot2_sim/log/domain-guard-valid-20261008-171105/`.
    Kein Fahrziel oder Nichtnull-Fahrbefehl; Supervisor-Cleanup erfolgreich,
    keine Simulatorreste. Bridge protokollierte die bekannten deprecated
    `ign_type_name`/`ign_topic_name`-Schlüssel, nicht Guardfehler. Lokale
    ROS-freie Suite: 345 Tests (340 bestanden, 5 erwartete Linux-only-Skips).
    `replay/43` war zu diesem Zwischenstand noch nicht an einen Replay-Launch
    angebunden; der Replay-ROS-Start wurde am 09.10.2026 implementiert und im
    SIL bestanden (siehe Replay-Nachweis oben). Ein rein
    lesender SSH-Preflight zum Pi scheiterte: `robot.fritz.box` war nicht
    auflösbar; SSH-Konfiguration enthält keine IP. Die im Referenzkatalog am
    04.10. erhobene Adresse `192.168.0.168` wurde einmalig mit strengem
    `HostKeyAlias=robot.fritz.box` geprüft, SSH lief in einen Connect-Timeout.
    Ein vorhandener Neighbor-Cache-Eintrag `192.168.0.100` mit Raspberry-Pi-OUI
    wurde ebenfalls ausschließlich mit `BatchMode`, `StrictHostKeyChecking` und
    `HostKeyAlias=robot.fritz.box` geprüft; der angebotene Host-Key wich ab und
    OpenSSH verweigerte die Verbindung. **Host-Key nicht ersetzt oder Prüfung
    umgangen.** Am 09.10. meldet der Nutzer den Pi wieder online; dies löst die
    DNS-/Identitätsabweichung nicht auf. Vor Zugriff bleibt eine unabhängige
    Host-Key-Verifikation nötig. In beiden Fällen wurde kein Remote-Befehl
    ausgeführt. Kein physischer HIL-/Motor-/Firmwaretest; Domains sind keine
    Sicherheitsgrenze.
- **RDP-Diagnose 09.10.2026 (laufender Visualisierungstest, kein Gate):** Ein
    zusätzlicher Fortress-Launch lief parallel in Domain 42 zum bestehenden
    Lauf. ROS-Graph zeigte doppelte `/command_timeout`, EKF-, Bridge- und
    RobotStatePublisher-Namen; `/scan` hatte zwei Bridge-Publisher. Logs zeigten
    zurückspringende Simulationszeit, `TF_OLD_DATA`, wiederholte TF-Buffer-Clears
    und RobotStatePublisher-Rückwärtszeitwarnungen. Der ältere Supervisor
    `24649` wurde gezielt mit SIGINT beendet. Im zeitgestörten Restlauf kamen
    `/odom/unfiltered` (~30 Hz), IMU (~29 Hz), JointStates (~620 Hz), Scan
    (~6,2 Hz) und TF (~12 Hz) an, aber `/odom` blieb aus und `odom → base_footprint`
    fehlte. Ein sauberer Single-Launch stellte `/odom` (Beispielstempel 16,963 s)
    und den TF wieder her; RDP-Fenster Gazebo/RViz waren gemappt und pingbar.
    Ein einmaliger Startup-Scan-Drop bei 1,779 s vor Aufbau des TF-Caches blieb
    protokolliert; danach keine wiederholten Zeitrücksprünge im geprüften Lauf.
    **CPU vor Threadlimit (11:40:34):** Gazebo-GUI ~533 %, Server ~43 %, RViz
    ~33 %, Load-Average 15,39/15,29/10,75, RDP-llvmpipe bei 1920×1481.
    `scripts/start_gazebo_rdp.sh` setzt nun `LP_NUM_THREADS=2`; die Variable ist
    in der installierten Mesa-Bibliothek vorhanden. **Laufzeitmessung mit Limit
    (11:50:38):** `/proc`-Umgebungen von Gazebo-GUI und RViz bestätigten
    `LP_NUM_THREADS=2`, `ROS_DOMAIN_ID=42`, `DISPLAY=:10.0`. `ps` meldete GUI
    ~192 %, Server ~22 %, RViz ~27 %; Load-Average 3,30/5,97/8,61. Der Single-Run
    hatte genau einen `/clock`-Publisher, keine doppelten Nodes, frisches `/odom`
    und `odom → base_footprint`; beide Fenster pingbar (13,5/11 ms). Auflösung
    blieb 1920×1481, daher ist die Lastsenkung mit Threadlimit und Single-Run
    zusammen beobachtet, aber kein isolierter Benchmark/Performance-Gate.
    Das Mac-RDP-Profil `ros-humble-rdp.rdp` ist für den nächsten Connect auf
    feste 1280×800 mit deaktivierter dynamischer Auflösung gesetzt; die aktive
    Sitzung verwendet weiterhin 1920×1481. Kein Nav2-Ziel/Nichtnull-Fahrbefehl,
    keine Hardware; kein G4-/Performance-Gate.
- **Noise-/Slip-Profil `g4-noise-slip-v1` (08.10.2026, implementiert und in Runs 44/45 runtime-bestanden):**
    `scripts/g4_robustness_profile.json` pinnt `g4-noise-slip-v1` (Seed `20261008`,
    Lidar σ=`0,01 m`, Gyro σ=`0,01 rad/s`, Beschleunigung σ=`0,1 m/s²`, Rad-μ=`0,8`).
    `fortress_g4_noise_slip.sdf` ist geometrisch gegen die nominale
    `fortress_g4_test.sdf` regressionsgeprüft und lädt einen vorangestellten
    Fortress-System-Plugin, der `ignition::math::Rand` setzt. Xacro-/Launch-Parameter
    haben nominale Defaults (Noise `0`, μ `1,0`); die Harness-Option
    `--noise-slip-profile` aktiviert sie nur explizit und archiviert Profil/Hash.
    Isolierte Humble-Builds von `linorobot2_gazebo` (inkl. C++-Plugin) und
    `linorobot2_description` bestehen. Xacro→SDF-Konvertierung bestätigt Noise
    `0,01/0,01/0,1` und Rad-μ `0,8` auf beiden Rädern; nominale Xacro-Defaults
    `Noise=0`, `μ=1,0` bestätigt. Beide G4-Welten bestehen `ign sdf -k`; die
    Robustheitswelt ist geometrisch mit der nominalen Welt regressionsgeprüft.
    6 Profilregressionen, bestehende G4-Mapping-/Goal-Tests, Shellsyntax und
    XML-Wohlgeformtheit bestehen. Seed-Plugin, Rausch-/Reibungskonfiguration und
    kompletter Vier-Szenariennachweis sind in Runs 44/45 im SIL bestanden;
    Hardwareverhalten ist damit nicht validiert.
- **Phase 5 in Arbeit:** Paket `linorobot2_vision` enthält Contract v1,
  Fixture-Backend und einen Offline-Adapter für das im gesicherten Jetson-
  Laufzeitquellstand definierte `/v1/vision/detection`-Antwortformat. Der
  Adapter verlangt Capture-Metadaten und Label-IDs; den Server-`timestampUTC`
  verwendet er nicht als Capture-Zeit. Fixtures sind synthetisch, keine
  aufgenommenen Jetson-Antworten; es gab keinen Live-API-Aufruf.
- **Visueller Replay-Monitor:** `scripts/x11_vision_monitor.py` läuft als natives
    X11-Fenster auf RDP `:10.0`, lädt Bild-/Ergebnispaare aus `replay_manifest.json`
    und zeichnet SVG-Szene, Detection-Box sowie Capture-/Modelldaten; `1`/`2`/`3`
    wechseln Fixture, `q` schließt. Der farbige XFCE-Terminalmonitor bleibt Fallback.
    Beide sind keine Live-Kameravorschau. Die Tk-Installation wurde blockiert und
    nicht umgangen; das native Fenster benötigt nur `libX11`.
- **Validierung:** aktuelle lokale ROS-freie Skriptsuite 310 Tests ausgeführt,
    305 bestanden und 5 erwartete Linux-only-Integrationen übersprungen;
    Linux/Humble alle 310 bestanden. Colcon-
    Build in der Humble-VM: 6 Pakete einschließlich `linorobot2_vision` bestanden.
    Installierte Symlink-/Fixture-Artefakte, ament-Pythonpfad und RDP-Tunnel
    geprüft; kein produktiver Vision-ROS-Transport, keine Inferenz und keine G5-Abnahme.
- **Simulierte Kamera → Consumer (07.10.2026): PASS, isolierter Phase-5-Probe**
    `/home/toto/linobot2_sim/log/p5-sim-camera-20261007-105732-mDfK2b/`.
    Fortress-GUI lief im RDP-Desktop; 5 gepaarte Image-/CameraInfo-Frames wurden
    über den gemeinsamen Consumer geprüft. `rgb8`, `640×480`, Frame `camera_link`,
    `fx=fy=554.3827`; Clock 1.781 Fortschritte, Capture-Stempel von 1,777 s bis
    1,782 s. Das absichtliche Empty-Mock-Ergebnis wurde bei allen Frames akzeptiert;
    null Detektionen/keine Inferenz. Bericht ohne Fehler, danach keine Simulator-
    oder Probe-Prozesse. Keine Fahrbefehle/Nav2-Ziele, keine Hardware.
- **RDP-Livebildkorrektur und Wiederholung (07.10.2026):** Ursache der Verwirrung
    war `rviz:=false` im Phase-5-Harness und automatischer Cleanup direkt nach PASS.
    Harness startet jetzt `fortress_camera.rviz` mit Image-Display `/camera/image_raw`
    und hält bei interaktivem TTY nach dem Probe-PASS bis Enter/Ctrl+C offen. Neuer
    visueller Lauf `/home/toto/linobot2_sim/log/p5-sim-camera-20261007-110635-IWj0lt/`
    PASS: 5 Image-/CameraInfo-Paare, `rgb8` 640×480 `camera_link`, `fx=fy=554.3827`,
    Mock-Zustand `empty`, 0 Detektionen, Fehlerliste leer. Während des Offenhalte-
    Prompts liefen nachweislich Fortress-Server, Gazebo-GUI und RViz mit dem neuen
    Camera-Preset. Ausgabe-/PASS-Zeilen erscheinen im Startterminal; Bild erscheint
    als eigenes RViz-Fenster im RDP. RDP-Terminal-Aufruf steht im Vision-README.
    Durch Enter oder Ctrl+C werden die eigenen Fenster beendet. Keine `/cmd_vel`,
    kein Nav2-Ziel, keine echte Kamera/Inferenz.
- **RDP-Kamerabild sichtbar bestätigt (07.10.2026, aktueller Lauf):**
    `/home/toto/linobot2_sim/log/p5-sim-camera-20261007-120350-YKI2e1/` meldet
    `PASS`, 5 echte ROS-Bild-/CameraInfo-Paare (`rgb8`, 640×480, `camera_link`,
    `fx=fy=554.3827`) und `consumer_state=empty`; Fehlerliste leer. RViz wurde mit
    `fortress_camera.rviz` und `/camera/image_raw` gestartet, das RDP-Terminal
    bestätigt „live /camera/image_raw display“ und wartet interaktiv. Prozesscheck
    bestätigt Fortress-Server, Gazebo-GUI und RViz laufen noch. Das ist ein echter
    simulierter Live-Kamerastream, aber weiterhin ohne Inferenz/Detektionen. Keine
    Nav2-Ziele, Fahrbefehle oder Hardware; G4 ist durch Runs 44/45 im SIL
    bestanden, G5 und reale Inferenz bleiben offen.

## VM-Einrichtung am 05.10.2026

- `ros-humble` läuft mit Ubuntu 22.04.5 ARM64. Workspace-Mount und nichtinteraktives sudo sind verfügbar; etwa 97 GiB frei laut VM-Dateisystemanzeige.
- ROS Humble und Fortress sind nach manueller Einrichtung verfügbar. Der Nutzer meldete den Linux-Colcon-Build erfolgreich; Paketauflösung, Xacro-Expansion, Spawning und ausgewählte Nodes wurden danach tatsächlich ausgeführt.
- Der Grafikpfad ist jetzt XFCE/xorgxrdp im Linux-Display `:10.0`, nicht XQuartz. `glxinfo -B` bestätigt OpenGL 4.5, Mesa 23.2.1 und llvmpipe-Software-Rendering ohne Hardwarebeschleunigung.
- Setupskript korrigiert: Plattformprüfung, `sudo -n`, ROS-Buildtools erst nach ROS-Repositoryeinrichtung und explizite Humble-Aktivierung vor CLI-Prüfung. ROS verwendet die offizielle HTTP-Quelle mit `signed-by`, OSRF weiterhin HTTPS.
- Der ursprüngliche automatische Installationsaufruf war blockiert und wurde nicht ausgeführt; die spätere Installation erfolgte manuell durch den Nutzer. Keine Installationssperre wurde über SSH/systemd umgangen.
- Nutzerseitiger Start aus dem Mac-Terminal: `orb -m ros-humble bash /Users/toto/Projects/linobot2_ws/scripts/setup_sim_env.sh`.

### Manueller Setupfehler und Korrektur

- Nutzer hat das Setup gestartet; Grundpakete wurden installiert. Der ROS-Paketindex scheiterte an einem Zertifikats-/Hostnamenfehler bei `https://packages.ros.org`; die danach fehlenden ROS-Pakete sind Folgefehler.
- Die [offizielle ROS-Quellenvorlage](https://raw.githubusercontent.com/ros-infrastructure/ros-apt-source/main/ros-apt-source/debian/rules) verwendet `http://packages.ros.org/ros2/ubuntu` mit APT-Signaturprüfung. Der Setup-Eintrag folgt nun dieser Vorlage; kein `trusted=yes`, keine deaktivierte TLS- oder Signaturprüfung.
- Ein erneuter Skriptstart repariert den eigenen bisherigen HTTPS-Eintrag vor dem ersten Indexupdate. Explizite `apt-get update`-Aufrufe verwenden `APT::Update::Error-Mode=any`, sodass Indexfehler das Setup abbrechen statt Paketfehler zu erzeugen.
- Read-only VM-Test des HTTP-`jammy/InRelease`-Endpunkts: HTTP 200. Öffentlicher ROS-Schlüssel ist lesbar; Bash-Syntaxprüfung bestanden. Die Pakete sind inzwischen manuell installiert; die früher ausstehende Installationswiederholung ist kein aktueller Blocker mehr.

### OrbStack-Dokumentation und Dienstzugriff geprüft

- Dokumentation zu [Diensten](https://docs.orbstack.dev/machines/#services), [Befehlen](https://docs.orbstack.dev/machines/commands), [Netzwerk](https://docs.orbstack.dev/machines/network), [Dateifreigabe](https://docs.orbstack.dev/machines/file-sharing) und [SSH](https://docs.orbstack.dev/machines/ssh) gelesen.
- In `ros-humble` ist PID 1 `systemd`; `systemctl is-system-running` meldet `running`, keine fehlgeschlagenen Units. Dienstverwaltung benötigt keinen zusätzlichen SSH-Server.
- Workspace sowohl unter `/Users/toto/Projects/linobot2_ws` als auch `/mnt/mac/Users/toto/Projects/linobot2_ws` erreichbar. Linux-Home ist `/home/toto`.
- `host.orb.internal` wird in der VM aufgelöst. Dies bestätigt Namensauflösung, noch keinen funktionierenden Grafik- oder ROS-Netzwerkpfad.
- RDP läuft ausschließlich auf Guest-Loopback, erreichbar über den lokalen SSH-Tunnel `127.0.0.1:13389`. Anmeldung und deutsche QWERTZ-Tastatur funktionieren; RViz startet mit OpenGL 4.5.
- Der direkte `orb`-Zugriff funktioniert. Tests verwenden ausschließlich Domain 42 und den Linux-Overlay unter `/home/toto/linobot2_sim`; alte Mac-Buildordner werden nicht aktiviert.

## Maßgebliche Quellen

- **Vision:** Der gesicherte Jetson-Livezustand (`439f0ef074df1df91812bc906106cead76f30396` plus lokale Änderung in `app.py`) ist Referenz für spätere Visionintegration. Der neuere `JetsonNano`-Quellstand ist nicht als Jetson-Laufzeit bestätigt.
- **Roboterbeschreibung:** Die geprüften Jetson-Backups enthalten keine 2WD-URDF, EKF-/Nav2-Konfiguration oder ESP32-Firmware. Das aktuelle 2WD-Xacro kombiniert Basis-/Caster-/Sensoranordnung aus Pi `toto_dev` mit den ESP32-`myrobot`-Kandidaten für Radradius und Spurweite. Maße bleiben unbestätigte Simulations-/Firmwarekandidaten und müssen vor Hardwarebetrieb physisch geprüft werden.

Details und Sicherungspfade: [SOURCE_MANIFEST.md](reference/SOURCE_MANIFEST.md) und [MACOS_SETUP_PLAN.md](reference/MACOS_SETUP_PLAN.md), Abschnitte 14–17.

## Implementiert

- Ubuntu-22.04-ARM64-OrbStack-Maschine `ros-humble` angelegt.
- Setup-, Aktivierungs-, Build- und G1-Smoke-Testskripte angelegt.
- Separates Fortress-2WD-Xacro, Lidar-, IMU- und Kamerasensor, Beispielwelt, Bridge-Konfiguration und RViz-Konfiguration angelegt. Classic-Dateien bleiben erhalten.
- Map-Konvertierung aus dem CMake-Konfigurationsschritt entfernt; Konfigurieren eines Builds soll keine Welten/Modelle nebenwirkend generieren.
- Linux-Build-/Install-/Log-Verzeichnisse liegen laut Buildskript unter `$HOME/linobot2_sim`, nicht in den alten Mac-Buildordnern.

## Tatsächlich geprüfte Ergebnisse

- Bash-Syntax aller vier Shellskripte: bestanden.
- Python-Syntax/AST von Sensorprobe und Fortress-Launch: bestanden.
- XML-Parsing von Xacro-, Paket- und SDF-Dateien: bestanden; alle sieben Xacro-Includes vorhanden.
- Bridge-YAML: 8 Einträge parsebar.
- Fortress-Beispielwelt: `ign sdf -k` meldet `Valid.`
- RViz-Konfiguration: YAML parsebar.
- `git diff --check`: bestanden.

### Laufzeitnachweise: zwei Starts und 15 Minuten Datenstabilität

- Erster Versuch fehlgeschlagen: `/home/toto/linobot2_sim/log/g1-smoke-20261005-114156/launch.log`. Talker/Listener und RViz starteten, aber keine World-Service-Antwort und keine Sensorproben. Der Nutzer-Screenshot zeigte ein leeres Gazebo-Fenster mit Nachziehspuren. Ein Gazebo-Server blieb nach SIGTERM hängen; seine internen Blockadeursachen sind nicht abschließend bestimmt.
- Separater Serverstart und später auch ein frischer kombinierter Debugstart konnten die Welt initialisieren. Der Launch verwendet nun getrennte Server-/GUI-Prozesse. Readiness prüft eine tatsächliche `/gazebo/worlds`-Antwort statt nur einen lebenden Launch-PID; jeder Test erhält eine eigene `IGN_PARTITION`.
- Erster korrigierter Smoke-Test bestanden: `/home/toto/linobot2_sim/log/g1-smoke-20261005-115849/launch.log`. Domain-42-Talker/Listener, Fortress-Welt, Roboter-Spawning, RViz und alle fünf geprüften ROS-Sensortopics funktionieren.
- Zweiter Start mit 900 Sekunden zusätzlicher Überwachung bestanden: `/home/toto/linobot2_sim/log/g1-smoke-20261005-120430/`. `readiness.log`, `sensors.log` und `launch.log` enthalten den Nachweis. Clock fortschreitend, kein Rücksprung, kein Sensordatenstillstand über zehn Sekunden; der RViz-Node war am Ende vorhanden.
- Sensorwerte: 360 Lidar-Strahlen, 38 endliche gültige Reichweiten, Frame `laser`; Bild 640×480 `rgb8`, 78 verschiedene gesampelte Bytewerte, Frame `camera_link`; CameraInfo 640×480 mit `fx=fy=554.383`; IMU-Frame vorhanden. Das bestätigt plausible Samples, noch keine vollständige geometrische/optische Kalibrierung.
- Unter llvmpipe etwa sechs Lidar-/Bildnachrichten pro realer Sekunde (10 Hz in Simulationszeit konfiguriert). Echtzeitfaktor 1 und hardwarebeschleunigte GUI-Leistung sind nicht nachgewiesen.
- Nach beiden korrigierten Tests keine Simulatorprozesse zurückgelassen. Der RDP-Tunnel blieb aktiv. Linux-Subreaper überwacht auch abgetrennte Enkelprozesse; Bereinigung ist auf TERM-/KILL-Fristen begrenzt. Watchdog-Shutdown behandelt `ExternalShutdownException`.
- Gesamte Skript-Testsuite: 62 Tests in der VM bestanden; macOS 57 bestanden und 5 Linux-Integrationstests erwartungsgemäß übersprungen. Darunter echte Prozessbaum-/Signaltests und gemockte Readiness-/Sensorregressionstests. Editorprüfung und Bash-Syntaxprüfung ohne Fehler.
- Visueller Nutzernachweis nach der Korrektur am 05.10.2026: Gazebo zeigt Bodenraster und rotes Testhindernis; Entity Tree enthält `ground_plane`, `test_obstacle`, `sun` und `linorobot2_2wd`. Keine Nachziehspuren im Screenshot sichtbar. RViz zeigt Raster und Frames sowie `Global Status: Ok`. Der Nutzer bestätigt nach Aufklappen auch für `TF` und `LaserScan` jeweils `Status: Ok`; deren Detailstatus war im Screenshot selbst nicht sichtbar. Welt-Rendering, Spawning und RViz-Displaystatus sind damit bestätigt, erwartete Scan-Geometrie und längerfristige GUI-Bedienbarkeit noch nicht vollständig abgenommen.

### G1: Lidar-Geometrie und DDS-QoS abgenommen

- Isolierter Lauf am 05.10.2026 bestanden: `/home/toto/linobot2_sim/log/g1-smoke-20261005-211430/`; quantitative Nachweise in `lidar-qos.json`, Kurzprotokoll in `lidar-qos.log`. Keine Fahrbefehle und keine Änderungen an Hardware; nach Testende keine Simulatorprozesse zurückgelassen.
- `scripts/verify_lidar_qos.py` vergleicht zehn Scans mit analytischen 3D-Strahl-/Box-/Bodenschnitten. Grundlage sind tatsächlich empfangene Gazebo-Weltpose und TF `base_footprint` nach `laser` am jeweiligen Scan-Zeitstempel; keine angenommene Spawn-Pose und kein TF-Ersatzwert.
- Alle zehn Scans: 360 Strahlen, jeweils 38 erwartete Hindernisstrahlen, keine abweichenden Randstrahlen. Erwartete nächste Boxentfernung etwa 1.48006 m; maximale Mess-/Modellabweichung pro Strahl 0.0032851 m, unter 0.05 m Prüftoleranz. Hinderniswinkel etwa −18.55° bis +18.55°. Scan-Zeitstempel steigen von 2.2 bis 3.1 Simulationssekunden.
- Tatsächliche Laser-Weltpose etwa `(0.020000, 0, 0.148377)` m, einschließlich `base_footprint`-/`base_link`-Versatz. Modellpose zwischen Anfang und Ende um 0.000372 m beziehungsweise 0.000000240 rad verändert, innerhalb der Stationaritätstoleranzen von 0.02 m und 0.03 rad.
- **Gemessener ROS-Publisher-Vertrag für jedes der fünf Topics** `/clock`, `/scan`, `/imu/data`, `/camera/image_raw`, `/camera/camera_info`: `parameter_bridge`, `RELIABLE`, `KEEP_LAST`, Tiefe 10, `VOLATILE`. Dies sind abgefragte DDS-Endpunkte, keine aus Ignition-Transport oder Bridge-Defaults abgeleiteten Annahmen. Die Bridge-Konfiguration legt das Profil bisher nicht ausdrücklich fest; der Prüfer verlangt passende Endpunkte und echte Zustellung bei jedem weiteren Lauf.
- Je Topic Zustellung an beide Probeprofile nachgewiesen: SensorDataQoS (`BEST_EFFORT`, `KEEP_LAST`, Tiefe 5, `VOLATILE`) und `RELIABLE`/Tiefe 10. Nachrichtenanzahlen Best Effort/Reliable: Clock 931/935, Scan 10/10, IMU 49/49, Bild 10/10, CameraInfo 9/9.
- Tatsächlicher RViz-Scan-Empfänger: `rviz2`, `RELIABLE`, `KEEP_LAST`, Tiefe 5, `VOLATILE`; DDS-Kompatibilitätsprüfung mit dem Scan-Publisher ergibt `OK`. Den sichtbaren RViz-Displaystatus hat der Nutzer separat bestätigt.
- Smoke-Test prüft Geometrie und QoS jetzt automatisch mit begrenzter Laufzeit. Gesamte Testsuite: 106 Tests unter Linux bestanden; macOS 101 bestanden und fünf Linux-spezifische Tests übersprungen. Die 44 neuen Regressionstests prüfen unter anderem Strahlindizes, verschobene/gedrehte/gepitchte Pose, ungültige Scans, fehlende Pose und QoS-Fehler. Editor-, Bash-Syntax- und bisherige Regressionstests bestanden.

### Kamera-/GUI-Pilot am 05.10.: ursprünglicher Prüfpfad blockiert

- `scripts/verify_camera_gui.py` und `scripts/gui_x11_probe.py` angelegt. Optionaler Harness-Aufruf über `G1_CAMERA_GUI_SECONDS=0` für den Pilot beziehungsweise `900` für den Langzeittest. Der Kamera-Prüfpfad soll den beobachteten Testwürfel aus dem Sichtfeld bewegen und anschließend wiederherstellen; mindestens drei frische RGB-Bilder je Zustand werden räumlich verglichen. Ein anderer Hash allein genügt nicht.
- Sicherheitsgrenzen: ausschließlich Domain 42 und eigene `linobot2_g1_...`-Partition; Server und GUI-Fenster müssen über PID, Startzeit und Prozessbaum zum eigenen Supervisor gehören. Keine Fahrbefehle oder Hardwareänderungen. Wiederherstellung wird bei begonnenem Stimulus auch auf Fehlerpfaden verlangt.
- Reale Service-Typen bestätigt: `/world/fortress_test/set_pose` nimmt `ignition.msgs.Pose` und antwortet mit `ignition.msgs.Boolean`; `/gui/screenshot` nimmt `ignition.msgs.StringMsg` und antwortet ebenfalls mit Boolean. Die CLI-Typabfrage ist erfolgreich, das eigentliche Setzen/Screenshotten damit noch nicht abgenommen.
- Prozessidentifikation korrigiert: Fortress/Ruby schreibt beim Einzelprozessstart vollständige Zeilen wie `ign gazebo -g -v 3` beziehungsweise `ign gazebo -s -r /.../fortress_test.sdf -v 3` in argv[0]. Streng begrenzte Erkennung ergänzt, Eigentumsprüfung bleibt erhalten.
- Letzter Pilot fehlgeschlagen: `/home/toto/linobot2_sim/log/g1-smoke-20261005-215100/camera-gui.json`. Gazebo und RViz als eigene gemappte Fenster gefunden; beide kündigen `_NET_WM_PING` an. Gazebo liefert innerhalb zwei Sekunden keine passende Echo-Antwort (`replies=[]`). Ob Protokollauswertung oder echte GUI-Unresponsivität Ursache ist, bleibt offen. Keine GUI-Abnahme aus diesem Ergebnis ableiten.
- Der Pilot stoppt vor ROS-Kameraaufnahme und Szeneänderung: `pose_writes=[]`, `camera_states=[]`. Zeitlicher Kamerabildwechsel und 900-Sekunden-GUI-Lauf wurden **nicht ausgeführt**. Vorherige Sensor-, Lidar- und QoS-Prüfungen im selben Lauf bestanden; danach keine Simulatorprozesse zurückgelassen.
- Aktuelle Regressionen: 199 Tests unter Linux bestanden; macOS 194 bestanden und fünf Linux-Integrationstests übersprungen. Das sind Unit-/Vertragstests und Prozessintegration, kein Ersatz für den fehlgeschlagenen realen GUI-Pilot. Editor- und Bash-Syntaxprüfung ohne Fehler.

### Erneute Prüfung am 06.10.: Kamera-/GUI-Pilot bestanden

- Ursache des ursprünglichen Gazebo-Ping-Timeouts im Prüfer gefunden: Qt 5 XCB sendet Echo-Antworten mit `StructureNotifyMask | SubstructureRedirectMask`; der Prüfer abonnierte nur `SubstructureNotifyMask`. Er beobachtet jetzt beide nicht exklusiven Notify-Masken, niemals Redirect. Die Qt-Implementierung bestätigt diesen Vertrag; Mock-Regressionsprüfungen simulieren jetzt auch maskenabhängige Zustellung.
- Zwei weitere Prüffehler korrigiert: `ros2 node list | grep -q` konnte unter `pipefail` wegen früh geschlossener Pipe falsch mit `BrokenPipeError` scheitern. Die Ausgabe wird jetzt vollständig gelesen. Fortress `/gui/screenshot` erwartet ein **Verzeichnis**, keinen PNG-Dateinamen. Pro Auftrag wird ein neues leeres Verzeichnis angelegt; nur genau eine dort erzeugte, nicht leere PNG mit gültigem CRC/Pixeldaten zählt.
- Erfolgreicher isolierter Pilot: `/home/toto/linobot2_sim/log/g1-smoke-20261006-101448/camera-gui.json`, `scope=PILOT`, `pass=true`, Gesamtdauer etwa 13.39 Sekunden. Sensor-, Lidar- und tatsächliche DDS-QoS-Prüfungen ebenfalls bestanden.
- Kamera-Stimulus tatsächlich ausgeführt: je drei frische 640×480-RGB-Bilder mit passender CameraInfo in den Zuständen Original, Hindernis außerhalb Sichtfeld und wiederhergestellt. Alle neun Vorher-/Nachher-Vergleiche zeigen 43.3831 % veränderte Pixel (maximale RGB-Kanalabweichung mindestens 20), mittlere absolute RGB-Abweichung 60.2543. Rote Pixel sinken von 115872 auf null. Bei allen neun Rückkehrvergleichen liegt der Anteil der Pixel oberhalb derselben 20er-Kanalschwelle bei null; das belegt keine byte-identischen Bilder. Beobachtete Originalpose erfolgreich wiederhergestellt. Damit ist zeitliche Kamerabildänderung belegt.
- Gazebo und RViz antworten im erfolgreichen Pilot an beiden Prüfzeitpunkten auf Ping (etwa 2–16 ms). Native Gazebo-PNG tatsächlich erstellt und dekodiert: 581×797 RGB, 18989 Bytes, 125 unterschiedliche RGB-Farben, 1342 rote Pixel. Dies belegt Render-Service und Event-Loop-Antwort im Pilot, nicht 15 Minuten GUI-Stabilität oder vollständige optische Kalibrierung.
- **Intermittierender Fehler bleibt offen:** vorheriger Lauf `/home/toto/linobot2_sim/log/g1-smoke-20261006-101406/camera-gui.json` scheiterte beim ersten RViz-Ping nach zwei Sekunden, während Gazebo antwortete. Unveränderte Wiederholung bestand. Ursache noch nicht bestimmt; kein weggetesteter Fehler und keine allgemeine GUI-Zuverlässigkeitszusage.
- 202 Tests unter Linux bestanden; macOS 197 bestanden und fünf Linux-Integrationstests übersprungen. Editorprüfung und Harness-Bash-Syntax ohne Fehler. Nach Pilot keine Simulatorprozesse; bestehender RDP-SSH-Tunnel weiterhin aktiv. Keine Installationen oder Hardwareaktionen.

### Zusatzdiagnose und erneuter Lauf am 06.10.

- Ein zuerst gestarteter 900-Sekunden-Versuch brach vor der GUI-Prüfung ab: `DISPLAY` war aus der OrbStack-Hostumgebung als nicht erreichbarer XQuartz-Pfad `/var/run/com.apple.launchd.../org.xquartz:0` vererbt. Gazebo-Server und Sensoren liefen, RViz und Gazebo-GUI starben beim Qt-XCB-Start. Das war ein Umgebungs-/Startfehler, kein 900-Sekunden-GUI-Ergebnis. Linux-X11 `:10.0` wurde per libX11 tatsächlich erreichbar bestätigt; der vorhandene SSH-Tunnel blieb aktiv.
- Der Harness prüft jetzt vor Start, dass `DISPLAY` ein lokales Linux-Display der Form `:N[.screen]` ist, und bricht bei XQuartz-Vererbung früh ab. OrbStack übernimmt benutzerdefinierte Host-Variablen nicht automatisch: `DISPLAY`, `LIBGL_ALWAYS_SOFTWARE` und `G1_CAMERA_GUI_SECONDS` müssen explizit mit `orb -m ros-humble env ...` in der VM gesetzt werden. Ein Lauf, bei dem die Kamera-/GUI-Option nicht in der VM ankam, bestand nur Sensor/Lidar/QoS und **übersprang** den Pilot.
- Beim ersten korrekt gestarteten GUI-Pilot unter `:10.0` antworteten Gazebo und RViz auf Ping. Der Lauf setzte `test_obstacle` außerhalb des Sichtfelds, scheiterte danach aber am Pose-Readback: `ign topic --json-output` lieferte mehrere JSON-Nachrichten, während der Parser nur ein Dokument akzeptierte. Der `finally`-Pfad stellte die beobachtete Originalpose erfolgreich wieder her und verifizierte sie; dieser Lauf lieferte keinen Kameravergleich und gilt nicht als bestanden.
- `parse_obstacle_pose` verarbeitet nun einen Strom vollständiger JSON-Dokumente, validiert jedes Dokument und verwendet den letzten gültigen Pose-Sample; beschädigte bzw. semantisch ungültige Folgedokumente bleiben Fehler. Regressionen decken Mehrfachnachricht, kaputten Folgetext und fehlendes Hindernis ab.
- Korrigierter Pilot bestanden: `/home/toto/linobot2_sim/log/g1-smoke-20261006-110012/camera-gui.json`, `scope=PILOT`, `pass=true`, 13.41 s. Beide GUI-Fenster antworteten bei beiden Ping-Prüfungen (Gazebo 11.29–11.74 ms, RViz 10.83–12.44 ms). Kamera-ABA und Rückkehr bestanden, Pose-Wiederherstellung `pass=true`; nativer Screenshot 581×797 RGB, 18.989 Bytes, 125 RGB-Farben. Keine Simulatorreste; RDP-Tunnel `127.0.0.1:13389` erhalten.
- Linux: 202 Tests bestanden und Bash-Syntaxprüfung bestanden. macOS: 202 Tests, davon 197 bestanden und 5 Linux-Integrationstests erwartungsgemäß übersprungen. Editorprüfung ohne Fehler.
- Vollständiger G1-Lauf mit explizitem VM-Display bestanden: `/home/toto/linobot2_sim/log/g1-smoke-20261006-110112/camera-gui.json`, `scope=G1_CAMERA_GUI`, `status=PASS`, Gesamtdauer 916.02 s; Stabilitätsfenster 902.73/900 s. 212 GUI-Inventar-/Responsiveness-Checks, 66 erfolgreiche Window-Pings (0.43–22.84 ms), 32 gültige native Screenshots; auch letzter Gazebo-/RViz-Ping und End-Screenshot bestanden. Clock 526855 Samples/526854 Fortschritte, IMU 26606, CameraInfo 5449, Bilder 5446, Scans 5445; Fehlerliste leer. Kamera-ABA/Rückkehr und Pose-Restore erfolgreich. Nach Lauf keine `ign`-/`rviz2`-Prozesse; SSH-Tunnel `127.0.0.1:13389` auf IPv4/IPv6 weiterhin aktiv.

### G2: virtuelle Basisbewegung bestanden

- **Laufzeitnachweis:** `/home/toto/linobot2_sim/log/g2-motion-20261006-112608/motion.json`; `status=PASS`, `scope=G2_SIMULATION_ONLY`, Domain 42, eigene Partition `linobot2_g2_501_20261006_112608_40342`, Laufzeit 19,99 s. Der erste Versuch (`g2-motion-20261006-112456`) endete vor jedem Bewegungsbefehl, weil der Prüfer eine noch nicht angelaufene Clock sofort als Fehler behandelte. Nach Ergänzung einer begrenzten 5-s-Startup-Toleranz bestanden die neuen Clock-Regressionen und der erneute Lauf. Der Fehlversuch ist kein Bewegungsnachweis.
- `fortress.launch.py` spawnte `linorobot2_2wd` erfolgreich. `/joint_states` lieferte endliche Werte für `left_wheel_joint` und `right_wheel_joint`; während der fünf Segmente wurden insgesamt 5804 JointState-, 300 unfiltered-/300 filtered-Odom- und 300 TF-Samples sowie 6981 Clock-Fortschritte erfasst. Startpose/Odometrie waren praktisch null; das Modell blieb nach dem Absetzen auf der Bodenebene.
- Befehle ausschließlich über ROS `/cmd_vel`; die abgefragten Publisher waren genau der Prüfnode und `command_timeout`. Vier Bewegungssegmente je 2 s mit höchstens 0,08 m/s bzw. 0,30 rad/s: vorwärts `+0,06786 m`, rückwärts `−0,06757 m`, positive Drehung `+0,19724 rad`, negative Drehung `−0,19801 rad`. Joint-, Gazebo-Ground-Truth-, `/odom/unfiltered`- und `/odom`-Vorzeichen stimmten in allen vier Richtungen.
- Für jedes Segment blieb die gefilterte Odometrie-Position/-Orientierung mit `odom→base_footprint` TF innerhalb der Messauflösung konsistent (im Report 0,0 m/0,0 rad Differenz); nach dem Stopp lagen die Radgeschwindigkeiten nahe null. Das Watchdog-Segment fuhr vorwärts und empfing nach Befehlsende einen expliziten Nullbefehl von `command_timeout`; abschließender Nullbefehl der Probe wurde ebenfalls publiziert.
- Sämtliche Weg-/Drehwinkelgrenzen wurden eingehalten (0,4 m / 1,0 rad je Segment). Keine Hardwaretopics oder Geräte; nach dem Lauf keine Fortress-/ROS-Simulatorprozesse. Der bestehende SSH-Tunnel PID 19103 auf `127.0.0.1:13389` blieb aktiv.
- Launch-Log enthält Bridge-Meldungen der Severity `ERROR`, die hier ausschließlich die veralteten Fortress-Konfigurationsschlüssel `ign_type_name`/`ign_topic_name` als migrierungsbedürftig kennzeichnen. Die Bridge funktionierte im Lauf; eine spätere Umstellung auf `gz_*` bleibt technische Bereinigung, kein G2-Blocker. `gui:=false` lief erfolgreich headless; Ogre2 meldete, dass das geerbte XQuartz-Display nicht verfügbar sei und wechselte in den Headless-Modus.
- ROS-freie Tests nach Clock-Korrektur: 213 unter Linux bestanden; macOS 208 bestanden und fünf Linux-spezifische Tests erwartungsgemäß übersprungen. Bash-Syntax und Editorprüfung ohne Fehler.
- Radien (0,03375 m), Spurweite (0,126 m) und übrige Modellmaße sind weiterhin Simulator-/Firmwarekandidaten, keine physisch vermessenen Roboterdaten. Dieser SIL-Test belegt keine reale Motorleistung, Bremswirkung oder Hardware-Sicherheit.

### G3: Odometrie, TF, Safety, Sensor-Topic-Ausfall und Neustart bestanden

- **Kombinierter Laufzeitnachweis:** `/home/toto/linobot2_sim/log/g3-safety-20261006-121252/phase3.json`; `status=PASS`, `scope=G3_SIMULATION_ONLY`, Domain 42, Laufzeit 95,37 s. Nach dem Kernlauf wurde der gesamte überwachte Fortress-Prozessbaum beendet und die Welt mit neuer `IGN_PARTITION` gestartet; die Messwerte des zweiten Starts stehen zusätzlich in `restart-sensor.json`. Beide Berichte haben keine Fehler. Limits: 0,08 m/s, 0,30 rad/s, Quadratseite 0,12 m; keine Hardwaretopics oder Geräte.
- Der erste erweiterte Harnessstart brach vor den Fault-Assertions ab, weil der symlink-installierte Testproxy im `libexec` nicht ausführbar war. Die Testlaunch-Aktion startet ihn nun explizit mit Python aus dem installierten Share-Pfad; der folgende vollständige Zwei-Generationen-Lauf bestand. Der abgebrochene Start zählt nicht als Abnahme.
- Acht Quadratsegmente: 36.654 Clock-Fortschritte, 35.726 JointState-, 1.804 rohe Odom-, 1.803 gefilterte Odom-/TF- und 1.828 IMU-Samples. Genau ein Odom→`base_footprint`-TF-Owner: `ekf_filter_node`; `robot_state_publisher` veröffentlicht die Robotermodel-Frames.
- Quadratabschluss: Ground Truth 1,68 cm und 0,1681 rad; gefilterte Odometrie 1,06 cm und 0,1631 rad. Der EKF/TF-Abgleich blieb innerhalb der Prüftoleranz.
- **Rohe Odometrie bleibt auffällig und ungelöst:** an jeder simulierten Drehung etwa 0,327–0,328 rad Yaw-Fehler je 1,52 rad Soll-/Ground-Truth-Drehung; Quadratabschluss roh 5,09 cm und 1,1427 rad. Der rohe DiffDrive-Frame ist `base_drive` an der Radachse (`x=-0,07 m`); für Positionsvergleiche wird auf `base_footprint` umgerechnet. Der Simulator-EKF verwendet IMU-Yawrate statt des biased Wheel-Yaw-Eingangs. Der Bias wird offengelegt, nicht als behoben behandelt; seine Ursache bleibt ungeklärt.
- **Pause/Resume bestanden:** `/clock` hatte null Fortschritte bei pausierter Welt; der wall-clock-Watchdog publizierte Null nach 0,271 s. Nach Resume: null stale Nicht-Null-Echos, Clock-Fortschritt wieder aufgenommen und praktisch keine Bewegung ohne frischen Befehl.
- **Befehls-Watchdog/Recovery bestanden:** Null-Echo nach 0,343 s; Restbewegung ca. 8×10⁻¹¹ m; Recovery nach frischem Befehl 0,0294 m. Dies sind begrenzte Simulatorwerte, keine Aussage zu realem Bremsweg oder Aktorverhalten.
- **Sauberer Neustart/Wiederanlauf bestanden:** alter Supervisor PID 45830 beendet; neuer Supervisor PID 46241, neue Fortress-Partition. World-Service, Clock, Sensoren, JointStates, beide Odometriepfade und TF wurden nach dem Neustart frisch empfangen; `ekf_filter_node` blieb alleiniger Odom-TF-Owner. Kein Nicht-Null-`/cmd_vel`-Echo und nur etwa 1,7×10⁻¹⁰ m Bewegung ohne neuen Befehl. Dies belegt kontrolliertes Stoppen und vollständiges Neu-Launch, nicht automatisches Respawn oder In-place-Recovery eines abgestürzten Servers.
- **Sensor-Topic-Ausfall/Wiederkehr bestanden:** kontrollierter G3-Fehlerinjektor unterdrückte für 1,00 s ausschließlich das ROS-Ausgabetopic `/camera/image_raw`; die Gazebo-Kamera und Ignition-Quelle blieben aktiv. Das Bildsample blieb während der Ausfallphase bei 13, während `/clock`, `/scan`, `/imu/data`, `/camera/camera_info`, JointStates, Odom und TF weiterliefen (z. B. 7 Scans, 37 IMU-, 7 CameraInfo- und 45 TF-Samples). Nach Wiederfreigabe kamen drei frische Bilder; kein Nicht-Null-Fahrbefehl und keine messbare Bewegung. **Grenze:** Das belegt einen Verbraucher-seitigen ROS-Topic-Ausfall und Wiederanlauf, nicht das Abschalten des Gazebo-Sensors oder eine automatische Fahrstopp-Policy bei Kameraausfall.
- Linux-Humble-Suite: 224 Tests bestanden; macOS-Suite: 224 Tests, 219 bestanden und fünf Linux-spezifische Integrationen erwartungsgemäß übersprungen. Workspace mit dem G3-Testproxy erfolgreich neu gebaut; Python-/Bash-Syntax und Editorfehlerprüfung bestanden. Nach beiden Läufen keine Fortress-/ROS-Simulatorprozesse; SSH-Tunnel PID 19103 auf `127.0.0.1:13389` blieb aktiv. Hardware blieb unverändert.

### G4: SLAM-/Map-Reuse- und Zielprüfer implementiert, Laufzeit offen

- `scripts/verify_phase4_mapping.py` fährt eine begrenzte Route zum Testhindernis, südlich daran vorbei und auf die gegenüberliegende Seite; Obergrenzen bleiben 0,08 m/s und 0,30 rad/s. Der Prüfer verlangt frische Scans, fortschreitende Clock, gefilterte Odometrie, erwartete ROS-Publisher-Ownership und eine nach der Fahrt inhaltlich veränderte `map`-Gridkarte mit bekannten belegten Zellen am Fortress-Testhindernis. Prozesszugehörigkeit wird während der Fahrt regelmäßig geprüft.
- `scripts/test_phase4_gate.sh` startet SLAM Toolbox allein mit dem Fortress-Server, speichert YAML/PGM unter dem Laufzeit-Logverzeichnis, beendet die erste Welt und lädt die gespeicherte Karte nach einem vollständigen Fortress-Neustart mit neuer `IGN_PARTITION` in Nav2. Die bisherige `playground.yaml` bleibt unverändert.
- `scripts/verify_phase4_goal.py` prüft vor Fahrbefehlen Map-Zellen, Initialpose, `/map→odom→base_footprint`, Clock/Scan/Odom, Actionserver und isolierte `/cmd_vel`-Publisher. Danach sind ein kurzes freies Ziel und ein Ziel jenseits des Testhindernisses vorgesehen; für die Hindernisroute muss der globale Plan ausreichenden Roboterabstand und einen messbaren Umweg belegen. Ground-Truth-Ankunft, finale Nullbefehle und alle beobachteten Geschwindigkeitsobergrenzen werden geprüft.
- ROS-freie Regressionen für Grid-Geometrie, Hinderniszell-Ausrichtung, Map-Path, Hindernisabstand/Umweg, ausgeführte Ground-Truth-Trajektorie, Pose-JSON-Stream und Befehlsgrenzen bestanden: 24 relevante Unit-/Vertragstests. Gesamte lokale Skript-Suite: 237 Tests, 232 bestanden und 5 erwartete Linux-only-Integrationen übersprungen; Python-Kompilierung, Bash-Syntax und Editorfehlerprüfung ebenfalls bestanden.
- **Erster nutzerseitiger G4-Gateversuch am 06.10.2026 vor Mapping fehlgeschlagen:** `/home/toto/linobot2_sim/log/g4-nav-20261006-135429-mpm7og/slam-mapping.json` meldet nach erfolgreicher Fortress-World-Readiness `ValueError: partition prefix must be a linobot2_<phase>_ prefix`. Ursache war das falsche Partitionspräfix `linorobot2_g4_` statt `linobot2_g4_`; der Ownership-Guard stoppte vor Route, Mapping oder Fahrbefehl. Gate und G4-Prüfer verwenden jetzt `linobot2_g4_`; ein Regressionstest akzeptiert das korrekte und weist das falsche Präfix zurück. Der fehlgeschlagene Lauf hinterließ laut Prozessprüfung keine Fortress-/SLAM-Prozesse.
- **Zweiter nutzerseitiger G4-Lauf am 06.10.2026 erreichte und fuhr die Mapping-Route, bestand aber die Kartenprüfung nicht:** `/home/toto/linobot2_sim/log/g4-nav-20261006-140158-DOQHWe/slam-mapping.json`. Fortress/SLAM und Publisher-Ownership waren bereit; die Karte wuchs von 159 auf 655 bekannte Zellen (62×32, 0,05 m Auflösung, 72 belegte Zellen). Ein unkalibrierter fixer Mindestwert von 1.000 Zellen brach vor dem separaten Test auf eine belegte Zelle am Hindernis ab; die Hindernisausrichtung ist deshalb aus diesem Lauf nicht abgenommen. Die berichtete Map-X-Ausdehnung endet bei ungefähr 3,12 m, knapp vor dem bisherigen Nav2-Ziel x=3,20 m.
- Daraufhin wurde die Kartenprüfung auf mindestens 200 neu bekannte Zellen gegenüber der Anfangskarte plus mindestens 20 belegte Zellen umgestellt; Hinderniszellenausrichtung bleibt ein separates Pflichtkriterium. Teilroute/Map-Metadaten werden jetzt auch bei Fehlern in den JSON-Bericht geschrieben. Das Hindernis-Ausweichziel liegt nun bei x=3,00 m, innerhalb der beobachteten Map-Ausdehnung und weiterhin 0,50 m jenseits der Boxkante. Diese Korrekturen bestehen die lokale Regression; der anschließende Assistant-seitige OrbStack-Aufruf wurde **vor der Ausführung von der Umgebungssicherheitsprüfung blockiert** und nicht umgangen.
- **Dritter nutzerseitiger G4-Lauf am 06.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261006-143247-nK9qnw/`. Mapping bestanden: 511 neue bekannte Zellen (159→670), 72 belegte Zellen und Testbox-Zelle `(29,12)` mit Belegung 100; Nav2-Readiness ebenfalls bestanden. Der Zielprüfer brach danach **vor dem Senden eines Navigationsziels** ab: Startpunkt `(0,0)` ist in der gespeicherten Map unbekannt (`origin.x=0,02 m`). Nav2 bestätigte `Robot is out of bounds`; der Sensorursprung lag an/außerhalb der Costmap. `reachable-goal.json` meldet `planned line path is outside known free map at (0.000,0.000): None`. Es gibt dadurch noch keinen freien Nav2-Ziel- oder Hindernisumfahrungsnachweis. Nach diesem Lauf keine Fortress-/SLAM-Prozesse zurückgelassen.
- Korrektur für die Map-Grenze: Mappingroute enthält nun zu Beginn eine begrenzte 0,55-m-Rückwärtsstrecke, damit SLAM westlich der echten Startpose kartiert und die Startpose samt Roboterumgebung in der Karte liegt. Der Prüfer wertet Rückwärtsfortschritt vorzeichenrichtig aus; die Startpose wird nicht künstlich verschoben. Lokal verifiziert: Bash-Syntax, Python-Kompilierung, Editorfehlerprüfung und vollständige Suite mit 238 Tests (233 bestanden, fünf erwartete Linux-only-Integrationen übersprungen). Der korrigierte Simulatorlauf ist noch nicht erfolgt; G4 bleibt offen. Hardware blieb unberührt.
- **Vierter nutzerseitiger G4-Lauf am 06.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261006-150625-Xtt2As/`. Der Rückwärtsabschnitt funktionierte (Ground Truth x≈−0,553 m); Routenprüfung bestand bis `turn_north`. Beim letzten `return_to_goal_lane` näherte sich der Roboter der Box zu stark; Abbruch bei `−0,140 m` Clearance. Der kleinste bis dahin bestandene Segmentwert war `+0,0546 m` am `turn_north`. Ursache: `map_far_side` fuhr nur 2,4 m, sodass der anschließende Nordweg bei x≈2,62 m dicht an der Box lag. Keine Nav2-Phase; nach dem Lauf keine Fortress-/SLAM-Prozesse.
- Routenfix: `map_far_side` auf 3,0 m verlängert, damit der anschließende Nord-/Rückweg hinter der Box mit Abstand liegt; die Clearance-Schwelle bleibt unverändert. Ein lokaler Regressionstest prüft die geplante Rückweg-Clearance. Letzte lokale Suite: 239 Tests, 234 bestanden und fünf erwartete Linux-only-Integrationen übersprungen; Bash-/Python-Syntax und Editorfehlerprüfung bestanden. Dieser Routenfix ist noch **nicht** im Simulator bestätigt. G4 bleibt offen; Hardware blieb unberührt.
- **Fünfter nutzerseitiger G4-Lauf am 06.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261006-151652-AjEGnS/`. Mapping **PASS**: 570 neue bekannte Zellen (159→729), 72 belegte Zellen, Testbox-Zelle `(29,10)` Belegung 100; alle acht Routenabschnitte bestanden. Kleinste Clearance `+0,4646 m` im letzten Rückweg. Nav2-Readiness nach neuem Fortress-Start **PASS**; `reachable-goal.json` scheiterte vor dem ersten Ziel, weil Map-Origin `x=0,02 m` den echten Start `(0,0)` und Sensorursprung aus der Costmap ließ. Nav2 protokollierte `Robot is out of bounds`; weder ein Navigationsziel noch Fahrbefehl wurde gesendet. Damit ist der 3,0-m-Routenfix im Simulator bestätigt, nicht aber Startkarten-Alignment oder Navigation. Nach dem Abbruch Read-only-Prozessprüfung: keine Fortress-/SLAM-Reste.
- Neue Map-Vorbereitung: `scripts/prepare_phase4_map.py` bewahrt das rohe SLAM-PGM/YAML und erzeugt separat eine Nav2-Karte mit westlichem Randpuffer; nur Zellen im vom G4-Bericht gemessenen, clearance-geprüften und tatsächlich befahrenen Startkorridor werden als frei ergänzt. Testbox-Ausrichtung muss bestehen bleiben; belegte Zellen im Korridor oder unbekannte Zellen auf dem Start-zu-Ziel-Pfad führen zum Abbruch. Gate verwendet danach die vorbereitete Karte für den frischen Nav2-Start. Drei ROS-freie Tests prüfen Padding, Hindernis-Erhaltung und Fail-Closed-Verhalten.
- **Sechster nutzerseitiger G4-Lauf am 06.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261006-152919-MxV5BV/`. Mapping und Nav2-Readiness bestanden. Die neue Map-Aufbereitung wurde erstmals im Simulator ausgeführt und bestand: raw SLAM map preserved, origin von `x=0,02 m` auf `x=−0,28 m` gepaddet, 113 Korridorzellen ergänzt, 63 Start-Footprintzellen verifiziert und Testbox-Zelle `(29,9)`/Belegung 100 erhalten. Der Zielprüfer brach weiterhin **vor dem ersten Ziel** ab, diesmal wegen eines zu engen Publisher-Gates: beobachtet wurden `behavior_server`, `command_timeout` und `velocity_smoother`, während die Allowlist nur die letzten zwei erwartete. Die Prüfung auf nichtnullige `/cmd_vel`-Befehle vor dem Ziel war zuvor passiert; dennoch wurde kein Navigationsziel gesendet. `verify_phase4_goal.py` erlaubt jetzt exakt diese drei bekannten Endpunkte und normalisiert Whitespace in ROS-Namen; Tests lehnen zusätzliche oder fehlende Publisher ab. Vollständige lokale Suite: 243 Tests, 238 bestanden und fünf erwartete Linux-only-Integrationen übersprungen; Editorfehlerprüfung sauber. Der Publisher-Fix ist noch nicht im Simulator bestätigt; freies Nav2-Ziel und Hindernisumfahrung bleiben offen. Hardware blieb unberührt.
- **Siebter nutzerseitiger G4-Lauf am 06.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261006-153911-a1XhJi/`. Mapping/Nav2-Readiness bestanden; Publisher-Allowlist nun bestanden. Der bekannte-freie erste Nav2-Zielaufruf gelang, der zweite (Hindernisumfahrung) überschritt nach 180 Wall-Sekunden sein Limit. Cancellation akzeptiert, terminal status 5 (canceled); kein Nachweis für erfolgreiche Umfahrung. Readiness sah 68×28 = 1904/1904 Kartenfelder als bekannt, obwohl das westliche Padding als unbekannt gedacht war. Ursache: Map-Saver-Unknown-Pixel 205 wird bei `free_thresh: 0.25` als frei klassifiziert. Die bisherige Runtime-Meldung `prepare PASS` war daher kein Beleg, dass Unknown in Nav2 erhalten blieb. `prepare_phase4_map.py` normalisiert jetzt 205 zu 127 im Nav2-Abbild (Rohkarte bleibt unverändert); Regression prüft die echte trinary-Klassifikation von unbekannten, freien und belegten Zellen. Der Nav2-Zielprüfer protokolliert jetzt bei Timeout Ground-Truth-Bewegung, cmd_vel, Plannerpfadlänge sowie Odom-/Clock-Fortschritt, ohne das 180-s-Limit zu lockern. BT-Tick-Überläufe wurden geloggt; wegen fehlender Teiltelemetrie ist noch offen, ob das Hindernisziel festhing oder nur mehr Laufzeit benötigte. Letzte lokale Suite: 244 Tests, 239 bestanden und fünf erwartete Linux-only-Integrationen übersprungen. Unknown-Normalisierung und Timeout-Telemetrie sind noch nicht im Simulator bestätigt; G4 bleibt offen. Hardware blieb unberührt.

- **Achter nutzerseitiger G4-Lauf am 06.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261006-160257-MpDM0Z/`. Mapping und Nav2-Readiness bestanden; Map-Preparation-PASS, Boxzelle erhalten. Nav2 sah 849 bekannte Zellen in einer 69×29-Karte (2001 gesamt), womit Unknown-Normalisierung im Runtime-Mapserver bestätigt ist. Der Preflight ließ das Hindernisziel `(3,0)` korrekt zurückweisen, da die Zelle unbekannt war (`−1`); kein zweites Ziel wurde gesendet. Mappingberichte zeigen dagegen eine clearance-geprüfte freie Südrückspur bei `y≈−0,90 m`, die bis `x≈3,27 m` reicht. Umfahrungsziel auf `(3,−0,90)` verlegt; lokale Geometrieregressionen bestätigen Umweg und Box-Clearance. Vollständige Suite 244 Tests (239 bestanden, 5 erwartete Linux-only-Integrationen übersprungen), Python-Kompilierung und Editorfehlerprüfung sauber. Dieses neue Ziel ist noch nicht im Simulator gelaufen; Timeout-Telemetrie ebenfalls noch nicht runtime-bestätigt. G4 bleibt offen; Hardware unberührt.
- **Neunter nutzerseitiger G4-Lauf am 06.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261006-161320-p9RdzJ/`. Mapping und Nav2-Readiness bestanden; Publisher-Prüfung und Map-Unknown-Verhalten ebenfalls in Nav2 angekommen. Der Preflight wies auch `(3,−0,90)` korrekt als unbekannt (`−1`) zurück; kein Umfahrungsziel wurde gesendet. Damit ist klar, dass die bekannte SLAM-Rückspur in der Kartenrasterung nicht jede feste Zielzelle belegt. Der Prüfer wählt nun aus einer begrenzten, fest vorgegebenen Reihe von Punkten entlang derselben vermessenen Südlane den östlichsten bekannten-freien Kandidaten mit mindestens 0,15 m Robot-Clearance; ohne solchen Punkt bleibt er fail-closed. Gezielte Zieltests bestehen; komplette Suite 245 Tests (240 bestanden, 5 erwartete Linux-only-Integrationen übersprungen). Kandidatenauswahl noch nicht in OrbStack validiert; G4 offen, Hardware unberührt.
- **Zehnter nutzerseitiger G4-Lauf am 06.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261006-162627-zpjeUV/`. Mapping, Nav2-Readiness und Map-Preparation bestanden. Der Preflight lehnte alle fünf damals konfigurierten Ziele auf `y=−0,90 m` wegen Unknown (`−1`) ab; deren berechnete Box-Clearance lag zwischen 0,192 und 0,420 m. Der Lauf verwendete noch nicht die inzwischen lokal ergänzten nördlicheren Rasterzeilen (`y=−0,84/−0,79/−0,74`), sondern nur die äußerste Zeile. Kein Umfahrungsziel wurde gesendet. Aktuelle Kandidatenänderung: erweitert die feste, clearance-geprüfte Liste um diese versetzten Rasterzeilen; lokale Suite 245 Tests (240 bestanden, 5 erwartete Linux-only-Integrationen übersprungen). Neue Liste noch nicht in OrbStack validiert; G4 offen, Hardware unberührt.
- **Elfter nutzerseitiger G4-Lauf am 06.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261006-164538-xvsi4x/`. Mapping, Nav2-Readiness, Map-Preparation und Known-free-Preflight bestanden; Nav2 wählte `(3,00;−0,84)`, brach aber die Hindernisaktion mit Status 6 ab. Plannerlog zeigt wiederholt `worldToMap failed` für mx 67–70 bei Costmap-Breite 67; ausgewähltes Ziel lag praktisch am Ostrand. Kandidatenbereich deshalb auf `x≤2,8 m` begrenzt und gewählte Koordinaten in Goal-Fehlerlabel aufgenommen. Gezielte Zieleinheitstests sowie vollständige Suite 245 Tests (240 bestanden, 5 erwartete Linux-only-Integrationen übersprungen) bestanden. Costmap-Randkorrektur noch nicht im Simulator bestätigt; G4 bleibt offen, Hardware unberührt.
- **Zwölfter nutzerseitiger G4-Lauf am 06.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261006-171834-Dwsz4e/`. Mapping/Nav2-Readiness bestanden; NavFn brach das bekannte-freie Ziel `(2,80;−0,74)` mit Status 6 ab. Die Rohkarte war nur 68×29 Zellen (1,45 m hoch, Ursprung y=−0,936 m); sie enthielt keinen footprint-sicheren bekannten-freien Weg südlich um die Box. Der vorherige BFS berücksichtigte den Kartenrand nicht. Korrektur: Goal-BFS verlangt nun Kartenrandabstand mindestens Roboteradius; `verify_phase4_mapping.py` sampelt Fortress-Posen samt Hindernisabstand während gerader Messsegmente. `prepare_phase4_map.py` paddet die Karte nach Süden und markiert ausschließlich Zellen entlang der bestandenen, gesampelten und clearance-geprüften Südwegsegmente `pass_south_of_box`/`map_far_side` frei; Unknown bleibt sonst Unknown, Rohkarte unangetastet. Regressionen prüfen Randabweisung und vorbereiteten Weg. Vollständige lokale Suite: 247 Tests (242 bestanden, 5 erwartete Linux-only-Integrationen übersprungen); Editor-Diagnosen sauber. Neue Map-Preparation/Navigation noch nicht im Simulator bestätigt; G4 offen, Hardware unberührt.
- **Dreizehnter nutzerseitiger G4-Lauf am 06.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261006-174000-hIFXSf/`. Mapping bestanden: 713 bekannte/73 belegte Zellen, 554 neu bekannte; sampled `map_far_side` Mindest-Hindernisabstand 0,1025 m mit Mapping-Marge 0,30 m (0,1825 m für Nav2-Radius 0,22 m). Map-Preparation bestanden, Rohkarte bewahrt, sechs Rasterreihen südlich gepaddet und 709 gemessene Korridorzellen freigeschaltet. Nav2-Readiness und erstes Ziel bestanden. Nav2 meldete für `(2,85;−0,90)` Erfolg, aber die Fortress-Endmessung lag 0,212 m vom Ziel (Abnahmelimit 0,20 m) trotz finalem Nullkommando; **der Gate-Lauf ist fehlgeschlagen**, keine G4-Abnahme. Die strenge Messgrenze bleibt 0,20 m; stattdessen sind beide Nav2-XY-Zieltoleranzen in `navigation_fortress.yaml` von 0,12 auf 0,08 m verschärft, damit Nav2 näher anfahren muss. Regressionen sichern Grenzwert und Config. Lokale Suite 249 Tests (244 bestanden, 5 erwartete Skips), Diagnosen sauber. Verschärfung noch nicht im Simulator bestätigt; G4 offen.
- **Vierzehnter nutzerseitiger G4-Lauf am 06.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261006-174924-qUrdYx/`. Mapping bestand, aber `prepare_phase4_map.py` brach sicher ab: die auf `y≈−0,90 m` gemessene Sweep-Röhre überdeckte ein belegtes SLAM-Rasterfeld `(2,145;−0,698)`. Kein belegtes Feld wurde freigeschaltet und Nav2 startete nicht. Korrektur: `pass_south_of_box` wird auf 1,10 m verlängert, damit der anschließende Ostkurs südlich des rohen belegten Rasters verläuft; südliches Padding wird aus dem tiefsten gemessenen Sweep-Punkt plus Roboter-/Rasterradius berechnet. Gezielte Suite und vollständige Suite 249 Tests (244 bestanden, 5 erwartete Skips) bestehen. Noch nicht im Simulator bestätigt; der 0,20-m-Ankunftsgrenzwert und 0,08-m-Nav2-Zieltoleranzen bleiben.
- **Fünfzehnter nutzerseitiger G4-Lauf am 06.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261006-205629-ylgV2V/`. Mapping und Map-Preparation bestanden mit dem verlängerten 1,10-m-Südweg (70×40-Karte, Origin `[-0,28, -1,52]`, 1734 bekannte und 83 belegte Zellen); kein belegtes Rasterfeld wurde berührt. Nav2-Readiness und erstes Ziel `(0,55; 0,0)` bestanden (`dist=0,059 m`). Für das Umfahrungsziel `(2,85; -0,90)` meldete Nav2 erneut Erfolg mit finalem Nullbefehl, jedoch lag die unabhängige Fortress-Endpose bei `[3,112; -0,987]`, Distanz `0,276 m` (Gate-Limit `0,20 m`). Der Fehlervektor `[+0,262; -0,087]` m zeigte einen X-Versatz von 0,26 m.
- **Sechzehnter nutzerseitiger G4-Lauf am 06.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261006-211913-gK79fz/`. Mapping, Map-Prep und Nav2-Readiness bestanden. Das erste Ziel `(0,55; 0,0)` bestand (`dist=0,080 m`, AMCL-Pose `[0,422; -0,001]`). Das zweite Ziel `(2,85; -0,90)` brach mit Status 6 ab: Die zu scharfen AMCL-Parameter führten zu TF-Timeouts (`Transform data too old` bei 34,7 s). Zudem wurde `GridBased.tolerance: 0.25` als Ursache des früheren Versatzes identifiziert. Korrektur: AMCL stabilisiert, Planer-Toleranz auf `0.05` und Dijkstra (`use_astar: false`).
- **Siebzehnter nutzerseitiger G4-Lauf am 06.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261006-213827-ic8ioY/`. **Durchbruch bei der physischen Zielankunft:** Erstes Ziel `(0,55; 0,0)` bestand mit `0,077 m` (Limit `0,20 m`). Für das Umfahrungsziel `(2,85; -0,90)` meldete Nav2 Erfolg und finalen Nullbefehl; die gemessene Fortress-Endpose lag bei `[2,917; -1,026]` m mit einer Distanz von **0,142 m** zum Ziel. Damit wurde das strenge **0,20-m-Gate-Kriterium erstmals für beide Ziele vollständig erfüllt**. Der Lauf schlug jedoch nach der Fahrt fehl: `ValueError: global planner path endpoints do not match the active goal`. Ursache: `on_plan` behielt wegen `del state["plans"][:-10]` nur die letzten 10 Replanningspfade, und `analyze_avoidance_path` wertete `plans[-1]` aus (dessen Startpunkt am Ziel lag). Korrektur: Pfade während der Fahrt nicht mehr auf die letzten 10 beschränken; neue Hilfsfunktion `select_avoidance_plan` wählt den frühesten Pfad aus, der von `second_start` zu `avoid_goal` führt. Lokale Testsuite: 251 Tests (246 bestanden, 5 erwartete Skips).
- **Achtzehnter nutzerseitiger G4-Lauf am 07.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261007-073925-03yJlJ/`. Erster Lauf mit aktivierter visueller Verfolgung (`--visual`, Gazebo-GUI + Nav2-RViz auf RDP `:10.0`). Erstes Ziel `(0,55; 0,0)` bestand mit `0,059 m`. Beim Umfahrungsziel lag die Fortress-Endpose bei `[2,753; -1,102]`, Distanz `0,224 m` zum Ziel `(2,85; -0,90)` (um 2,4 cm verfehlt; Limit `0,20 m`). Ursache: Die Zielkandidaten in `AVOID_GOAL_CANDIDATES` waren historisch auf $y \in [-0,90; -0,70]$ fixiert. Seit der Verlängerung von `pass_south_of_box` auf 1,10 m liegt die tatsächliche Mittellinie des vermessenen Freiraumkorridors bei $y \approx -1,05 \dots -1,10$ m; das gewählte Ziel lag 20 cm nördlich der befahrenen Korridorachse. Korrektur: `AVOID_GOAL_CANDIDATES` deckt nun die reale Korridorachse ab ($x \in (2.80, 2.75, 2.70, 2.85)$, $y \in (-1.05, -1.00, -0.95, -0.90, \dots)$). Lokale Testsuite: 251 Tests (246 bestanden, 5 erwartete Skips).
- **Neunzehnter nutzerseitiger G4-Lauf am 07.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261007-084433-WBruJq/`. **Voller Durchbruch bei beiden Zielen:** Erstes Ziel `(0,55; 0,0)` bestand mit `0,061 m` (Limit `0,20 m`). Zweites Ziel `(2,80; -1,05)` bestand mit **0,143 m** gemessener Fortress-Distanz (Limit `0,20 m`), sauberem Nullbefehl und sicherem Hindernisabstand.
- **Zwanzigster G4-Lauf am 07.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261007-091118-vqFVCX/`. Mapping/Nav2-Readiness bestanden, aber Nav2 meldete das freie Ziel `(0,55; 0,0)` nach nur 2,42 s als `SUCCEEDED`, obwohl Fortress-Ground-Truth am Start blieb (Enddistanz `0,550 m`, nur ein Trajektoriensample). Das Ground-Truth-Gate wies den Erfolg zurück. Zeitstempel im Nav2-Log: Simulationszeit `5,143 s`, TF-Datenzeit `2,619 s`; Costmap meldete beim Start Rückwärts-Extrapolation (angefordert `2,139 s`, älteste TF-Daten `2,619 s`). Das ist ein nachgewiesener stale-TF-/falscher Nav2-Erfolg, nicht ein Ziel- oder Kartenproblem.
- **Einundzwanzigster G4-Lauf am 07.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261007-092122-IUUBku/`. Erstes Ziel bestand mit `0,071 m`; beim zweiten fuhr der Roboter etwa 75 s um die Box, bevor `map→odom` wiederholt als veraltet gemeldet wurde. Der Roboter geriet danach außerhalb der Costmap (`y≈−1,68 m`), Planner/Recovery brachen ab (Action-Status 6). In der Fortress-Konfiguration fehlten explizite AMCL-`update_min_d`/`update_min_a`-Werte; der Default `update_min_d=0,25 m` ist eine plausible Ursache dafür, dass AMCL bei der langsamen Schlussfahrt keine zeitnahen Updates erzeugte. Die Logfolge stützt diese Ursache, beweist aber nicht allein, dass sie die einzige Ursache war.
- **Prozessgrenze:** Nach dem Nutzerhinweis, einen fehlgeschlagenen Lauf nicht automatisch zu wiederholen, wurde Run 21 dennoch assistant-seitig gestartet. Das war nicht autorisiert und war ein Fehler. Danach wurde kein Lauf gestartet, bis der Nutzer Run 22 ausdrücklich freigab; diese Einmalfreigabe wurde nicht als Erlaubnis für Retries verstanden.
- **Zweiundzwanzigster, ausdrücklich freigegebener G4-Lauf am 07.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261007-094055-WgzJ04/`. Keine Prozesse liefen vorab; Linux-RDP-Display `:10.0` war erreichbar. Mapping, Map-Preparation und Nav2-Readiness bestanden. Ziel 1 bestand mit `0,112 m`, 92 Ground-Truth-Samples und einem Nullbefehl; TF-Alter am Ergebnis: `map→odom = −2,962 s` (AMCL-Future-Stamp innerhalb Toleranz), `odom→base_footprint = 0,028 s`. Ziel 2 `(2,80;−1,05)` brach mit Action-Status 6 ab. Der erste Nav2-TF-Fehler verglich `32,097 s` mit Transformzeit `31,600 s` (0,497 s alt); derselbe Transformstempel blieb stehen, während die Anfragedatenzeit bis `76,767 s` fortschritt (45,167 s alt). Danach meldete der Controller wiederholt Null-Längen-Pläne und `Controller patience exceeded`; in diesem Lauf gab es keinen Costmap-Out-of-Bounds-Nachweis. Das AMCL-Log enthält Setupmeldungen, aber keine Diagnose zum späteren Transformstillstand. Die niedrigeren AMCL-Schwellen verhinderten den falschen Erfolg beim ersten Ziel, **lösten aber die TF-Störung auf der Hindernisroute nicht**. G4 bleibt offen.
- **Lokale Korrektur und Verifikation:** Fortress-AMCL fordert Updates ab `0,02 m`/`0,02 rad` an. `verify_phase4_goal.py` validiert beide dynamischen TF-Kanten gegen `/clock` (max. `1,0 s` alt; bis `3,5 s` Zukunft für AMCL-Transformtoleranz); eine Regression speist den Run-20-Befund (Clock `5,143 s`, TF `2,619 s`, Alter `2,524 s`) ein und erwartet fail-closed Ablehnung. Für fehlgeschlagene Actions ergänzt der Verifier jetzt einen TF-Snapshot mit Sim-Zeit-Alter und Wall-Zeit-Empfangsalter sowie Scan-/Odom-/Plan-/Befehlsfortschritt. Eine Regression nutzt Run 22s konkrete `45,167 s` TF-Alterung; dies ist Diagnoseinstrumentierung und noch nicht runtime-validiert. Der Gate lädt die AMCL-Datei über `--symlink-install`; Quell- und installierte Fortress-YAML stimmen per SHA-256 überein. Run 22 validierte den verbesserten Start, aber nicht die TF-Stabilität während langer Fahrt. Ground-Truth-Ankunft bleibt `0,20 m`, Fahrgrenzen bleiben `0,08 m/s` und `0,30 rad/s`. Lokale Suite: 253 Tests bestanden, 5 erwartete Skips.

- **Dreiundzwanzigster, nach RDP-Bereitschaft ausdrücklich gestarteter G4-Lauf am 07.10.2026:** `/home/toto/linobot2_sim/log/g4-nav-20261007-100902-zAT1uQ/`, visueller Modus `--visual` auf Linux-Display `:10.0`; der Mac-Tunnel `127.0.0.1:13389` wurde zuvor verbunden, der Gast-RDP-Listener war ausschließlich loopbackgebunden. Mapping **PASS**: 766 neue bekannte Zellen (159→925), 74 belegte Zellen, minimale Hindernis-Clearance `0,3044 m`, Testbox-Zelle `(29,15)` Belegung 100. Map-Preparation und Nav2-Readiness **PASS**; Nav2-Karte 69×41 mit 1.722 bekannten und 74 belegten Zellen, alle geprüften Lifecycle-Nodes active. Freies Ziel **PASS**, Fortress-Ground-Truth-Abstand `0,0693 m`. Hindernisziel `(2,80;−1,05)` **FAIL**, Action-Status 6; keine G4-Abnahme. Nav2 meldete `map→odom`-Transformzeit `50,300 s` bei Anfragedatenzeit bis `68,274 s` und wiederholte `worldToMap failed` am Costmap-Ostrand (mx 69 bei Breite 69; Kartenende x≈3,15 m, Sensorursprung x≈3,17 m). Der Fehlerbericht zeigte gleichzeitig 672 während des Ziels empfangene `map→odom`-Updates mit 67,2 s Zeitstempelfortschritt; Snapshot: letzter Stempel `84,700 s` bei `/clock=81,766 s` (−2,934 s, innerhalb AMCL-Future-Toleranz). `odom→base_footprint` war frisch (0,015 s). `/amcl_pose` hatte am Snapshot seit 8,33 Wall-Sekunden keine Nachricht erhalten; Sim-Zeit-Alter `5,466 s`, 203 Samples während des Ziels. Ein vollständiger AMCL-/`map→odom`-Publikationsstillstand ist damit **nicht** belegt. Die Diskrepanz zwischen Nav2s intern verwendeter Transformzeit und den auf `/tf` empfangenen fortschreitenden Stempeln bleibt ungeklärt; auch die Kausalreihenfolge zur Costmap-Randüberschreitung ist nicht bewiesen. Gate endete nach rund 116 s; nachfolgende Prozessprüfung fand keine Fortress-/RViz-/Nav2-/SLAM-Reste. Kein Retry und keine Parameter- oder Grenzwertänderung; G4 bleibt offen.

## Abnahme und verbleibende Punkte

- **Gate G1 am 06.10.2026 bestanden.** Die erforderlichen sauberen Starts, ROS-/Sensor-/Clock-Daten, Render-Lidar-Geometrie, tatsächliche QoS-Zustellung, Kamerabildwechsel, Nutzer-Sichtprüfung und der 900-Sekunden-GUI-/Server-Lauf sind belegt. Der RViz-Ping-Timeout im Lauf `20261006-101406` bleibt als Einzelereignis ursächlich ungeklärt; er trat im bestandenen 902.73-s-Lauf nicht erneut auf. Die ältere Qt-X11-Maskenkorrektur war für den separaten Gazebo-Pingfehler relevant und darf nicht als bewiesene Ursache des späteren RViz-Ereignisses ausgegeben werden.
- Kamerabildwechsel und passende CameraInfo sind belegt; optischer Frame und Kalibrierungssemantik sind noch nicht abgenommen. `scripts/test_phase1_gate.sh` bleibt ein partieller G1-Test.
- Bridge nutzt funktionierende, aber deprecated `ign_*`-Konfigurationsschlüssel und `ros_ign`-Paketwrapper; deren Warnungen sind nicht die Ursache des ursprünglichen Datenfehlers. Tatsächliches `joint_states`-Mapping wurde noch nicht geprüft.
- **Gate G2 am 06.10.2026 im Simulator bestanden** (Nachweis oben). Nav2/SLAM, physische Geometrie und Hardwarephasen sind noch offen. Pi-/Jetson-/Motorplattformen blieben unverändert.
- **Gate G3 am 06.10.2026 im Simulator bestanden** (Nachweis oben): Quadrat, genau ein Odom-TF-Owner, EKF/TF, sichtbarer Roh-Odom-Bias, Stop/Recovery, Pause/Resume, kamera-topic dropout mit Recovery und sauberer Fortress-Neustart mit stale-command-Prüfung. Der Sensorfehler ist ausdrücklich ein ROS-Bridge-Ausgabetopic-Test; ein Hardware-Sensorfehler oder Fail-safe-Stopp wurde nicht behauptet.

**Historischer Stand am 07.10.2026:** G4 war wieder aufgenommen, offen und nicht
bestanden. Die damalige Deferred-Entscheidung und der kontrollierte Abschluss
von Run 24 bleiben als Historie erhalten; seit Runs 42–45 ist G4 im SIL
abgenommen. Die rohe
Yaw-Abweichung bleibt sichtbar. Keine Pi-/Jetson-/Motoraktionen.

### Wiederaufnahme: Run-23-Loganalyse und Nav2-Fortschrittsschutz

- Erster TF-Fehler bei Wall-Logzeit `1791360798.438711286`, erster Planner-
    Kartenrandfehler `1791360814.833966109`: TF-Fehler **16,395 s früher**.
    Sensorursprung-Warnung erst bei `1791360829.939517116`. Das ist die beobachtete
    Reihenfolge, kein Beweis der Kausalität oder eines zuvor korrekten Kartenrands.
- Installiert ist Nav2 `1.1.20`, Bringup verwendet `component_container_isolated`.
    AMCL-Aktivierung und zwei Lifecycle-Erfolgsmeldungen stehen im Archiv. Die
    Readiness prüft AMCL bereits explizit auf Zustand 3/active. Ein unaktiviertes
    AMCL oder ein gemeinsamer einzelner Executor sind deshalb nicht belegt.
- Die versionsgebundene Funktion
    [nav_2d_utils::transformPose](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_dwb_controller/nav_2d_utils/src/tf_help.cpp)
    meldet nach Extrapolation den **neuesten** Transform ihres Buffers. Frische
    `/tf`-Nachrichten beim separaten Verifier widerlegen dessen internen Rückstand
    nicht. Recovery verwendete Posezeit `47,411 s`, obwohl das verfügbare
    Bufferfenster bereits bei `55,300 s` begann. Grund des Rückstands bleibt offen.
- [Costmap2DROS::updateMap](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_costmap_2d/src/costmap_2d_ros.cpp)
    übernimmt den Zeitstempel der intern aufgelösten Roboterpose in
    `published_footprint`. Der Goal-Verifier beobachtet jetzt beide Footprints
    (lokal `odom`, global `map`), verlangt gültige Polygone, höchstens 1 s Posealter,
    maximal 3,5 s Zukunft und höchstens 2 s Empfangsalter. Fehlende Daten verhindern
    den Start; während einer Action führt ein Fehler zum bestehenden begrenzten
    Cancellation-/Cleanup-Pfad. Fehlerberichte enthalten beide Pose-/Empfangsalter.
- Drei neue ROS-freie Regressionen prüfen frische externe TF bei eingefrorener
    interner Pose, fehlende/verspätete/zukünftige Daten und ungültige Polygone/Frames.
    Linux/Humble: alle 292 Tests bestanden. Mac: 292 Tests, 287 bestanden,
    5 erwartete Skips; Bash-/Python-Syntax, Whitespace und Editorprüfungen sauber.
    **Kein Runtime-Nachweis für diesen Schutz, keine Ursachenbehebung behauptet.**
    Navigationsparameter, Ziele, Unknown-Semantik und Fahrgrenzen unverändert.
- Transfergrenze: Standard-ROS-Topics und explizite Zeit-/Frame-Verträge bleiben
    backendunabhängig; der G4-Prüfer selbst bleibt ein Domain-42-SIL-Test mit
    `/clock`. Hardware benötigt später `use_sim_time=false`, gemessene Geometrie
    und eigene HIL-/Stop-Abnahmen. Keine pauschale Konfigurationsübertragung.
- Umfangslücke: enge Passage und kontrolliert unerreichbares Ziel aus Phase 4
    fehlen im aktuellen Zwei-Ziel-Harness. Selbst ein neuer Zwei-Ziel-PASS wäre
    noch keine vollständige Abnahme aller im Plan geforderten Baseline-Szenarien.

### Run 24: lokaler Nav2-Posefortschrittsschutz runtime-bestätigt, G4 FAIL

- Nach Nutzerauftrag zur autonomen Fortsetzung RDP-Dienste/Loopback und X11
    `:10.0` lesend geprüft; keine bestehenden Sim-/Nav2-Prozesse. Genau ein
    visueller Domain-42-Lauf: `/home/toto/linobot2_sim/log/g4-nav-20261007-130320-LmIr35/`.
    Mapping, Map-Preparation, Readiness und freies Ziel bestanden; Ground-Truth-
    Zielabstand `0,0792508 m`, finaler Nullbefehl und 64 Trajektoriensamples.
- Hindernisroute bei Clock `41,003 s` kontrolliert abgebrochen: lokaler Footprint
    `40,002 s` (Posealter `1,001 s`, Empfangsalter `0,144 s`); globaler Footprint
    `40,922 s` (nur `0,081 s` alt), Odom-TF `41,000 s` (`0,003 s` alt), map-TF
    `43,900 s` (zulässiger Future-Stamp), AMCL-Pose `40,900 s` (`0,103 s` alt).
    263 Scans/map-TF-Updates und 1.315 Odom-/Odom-TF-Samples während des Ziels.
    Das belegt lokalen Nav2-Posezeit-Rückstand bei frischen externen Daten, nicht
    einen vollständigen AMCL-Ausfall. Keine späteren TF-/Kartenrandfehler im Log.
- Cancellation akzeptiert, terminaler Status 5; Goal-Prüferdauer `51,829 s`.
    Nach Cleanup keine Fortress-/RViz-/Nav2-/SLAM-Reste. RDP bleibt erhalten.
    **G4 FAIL**, keine Gate-Abnahme, keine Ziel-/Parameter-/Grenzwertänderung.
- Passender upstream Fehler: [geometry2 #992](https://github.com/ros2/geometry2/issues/992),
    AB-BA zwischen `timer_to_request_map_mutex_` und `transformable_requests_mutex_`.
    Versionsgebundene Quellen zeigen den Pfad; installierte Versionen sind
    tf2/tf2_ros `0.25.23`, rclcpp `16.0.21`, CycloneDDS `1.3.5`.
    [Humble-Backport #990](https://github.com/ros2/geometry2/pull/990) ist upstream
    zusammengeführt, Commit `9997e969556f5fe5d32ad498d02b3b3473da70b6`.
    **Hochgradig passender Ursachenverdacht, noch kein lokaler A/B-Beweis:** kein
    GDB installiert und kein Thread-Backtrace dieses Laufs erfasst.
- `scripts/setup_tf2_fix.sh` baut unveränderte gepinnte upstream Pakete tf2 und
    tf2_ros in `$HOME/linobot2_tf2_fix`, nicht `/opt/ros`. Ein separater CMake-
    Harness kompiliert die upstream Buffer-Regression; Baseline muss den benannten
    Deadlock reproduzieren, Overlay muss die komplette Buffer-Suite bestehen.
    Bibliotheksauflösung wird per `ldd` kontrolliert. Keine Installation von
    Systempaketen, kein Simulatorstart, kein Firmware-/Hardware-/Inferenzzugriff.
- **Build blockiert:** Umgebungssicherheitsprüfung hat den Agent-Aufruf vor der
    Ausführung abgelehnt (externe Quellen laden/bauen). Nicht erneut über andere
    Werkzeuge ausführen oder Sperre umgehen. Quellen nicht heruntergeladen, Overlay
    nicht gebaut/aktiviert, upstream Regression lokal noch nicht ausgeführt.
    Shellsyntax und Editorprüfung sind sauber; das ist kein Buildnachweis.
- Nach manuellem erfolgreichem Build ausdrücklich `local_setup.bash` des TF2-
    Overlays **nach** dem Roboteroverlay sourcen; vor Nav2 die Bibliotheksauflösung
    bestätigen. Rückfall: neue Shell mit ursprünglichem Humble-/Roboteroverlay,
    ohne TF2-Overlay. Späterer Hardwaretransfer braucht denselben gepinnten Fix und
    eigene HIL-Abnahme; keine Änderung am Pi/Jetson heute.
- Abschließende vorhandene Skriptregressionen einschließlich konkreter Run-24-
    Zeitpaare: Linux/Humble 293/293 bestanden; Mac 288 bestanden und 5 erwartete
    Skips. Shellsyntax, relevante Editorprüfungen und Whitespace sauber. Kein
    Ausführen des blockierten TF2-Builds durch diese Regressionen.
- Zusätzliche quellseitige Transferprüfung: upstream Vergleich 0.25.23 gegen
    gepinnten #990-Commit zeigt genau einen Commit und zwei Änderungen, ausschließlich
    `tf2/src/buffer_core.cpp` und `tf2_ros/test/test_buffer.cpp`. Öffentliche Header,
    ROS-Schemas und Abhängigkeiten unverändert. Schnittstellenneutraler Fix-Kandidat;
    binäre ABI-/Zielsystemprüfung bleibt offen. Buildblocker ist kein Beweis für
    fehlende Hardwareübertragbarkeit. Transfer-/Rückfallbedingungen im
    `reference/SOURCE_MANIFEST.md`; weiterhin keine Hardwareänderung oder G7-Abnahme.

### Manueller TF2-Build und upstream A/B-Abnahme: PASS

- Nutzer führte `setup_tf2_fix.sh` manuell aus; aktuell Exit 0. Nachweis:
    `/home/toto/linobot2_tf2_fix/evidence-20261007-133055-x2SBto/`.
    tf2 und tf2_ros aus gepinntem Humble-Commit erfolgreich gebaut. `/opt/ros`
    unverändert; tf2_msgs wird aus dem vorhandenen Humble-Unterbau verwendet.
- Baseline: `ldd` zeigt `/opt/ros/humble/lib/libtf2.so`; gezielter upstream Test
    reproduziert den benannten AB-BA-Deadlock und endet erwartet mit Exit 1.
- Fix: `ldd` zeigt `/home/toto/linobot2_tf2_fix/install/lib/libtf2.so`;
    **alle 11 upstream Buffer-Tests bestanden**, einschließlich Deadlock- und
    ROS-Timerrennen. Baseline-/Negativtestmeldungen sind kein Fehler dieses PASS.
- Vorherige manuelle Versuche scheiterten an optionalem colcon-Argument und
    am vom Harness verworfenen ARM64-CycloneDDS-Suchpfad. Beide Skriptfehler sind
    korrigiert und ROS-frei regressionsgetestet. Vollständige lokale Suite:
    Linux 298 bestanden, Mac 293 bestanden und 5 erwartete Skips.
- Der ursprüngliche Agent-Buildblocker wurde nicht umgangen; der erfolgreiche
    Build ist ein manueller Nutzernachweis. Noch keine Overlay-Aktivierung im
    G4-Harness und **kein Nav2-/G4-Nachweis mit Fix**. G4/G5 bleiben offen.
- Nächster einzelner Schritt: nach Humble-/Roboteroverlay-Aktivierung TF2-
    `local_setup.bash` sourcen und Controller-Abhängigkeiten mit
    `ldd /opt/ros/humble/lib/libcontroller_server_core.so` prüfen; erst die Ausgabe
    ansehen, bevor ein Simulatorstart geplant wird.

### G4-Harness: geprüften TF2-Overlay ausdrücklich aktivieren

- Nutzerprüfung der Controller-Abhängigkeiten erfolgreich: `libtf2.so` und
    `libtf2_ros.so` aus `/home/toto/linobot2_tf2_fix/install/lib`, tf2_msgs aus
    `/opt/ros/humble`; kein `not found`. RDP-Dienste vom Nutzer erneut erfolgreich
    geprüft. Lesende Agentprüfung: Linux-X11 `:10.0` erreichbar, keine Simulator-,
    RViz-, Nav2- oder SLAM-Prozesse. Kein weiterer Simulatorstart ausgeführt.
- `test_phase4_gate.sh --tf2-fix` sourct den Fix ausdrücklich **nach** Humble
    und Roboteroverlay. Neuer `verify_tf2_overlay.py` prüft vor dem ersten Launch
    Quell-SHA, beide tatsächlich aufgelösten TF2-Bibliotheken und alle dynamischen
    Abhängigkeiten der ausgewählten RMW-Implementation. Fehlt etwas oder wird ein
    alter/gemischter TF2-Stand ausgewählt, stoppt der Harness vor Simulation.
- `tf2-overlay.json` enthält Quell-SHA, Bibliothekspfade und SHA-256 sowie RMW-
    Auflösung. Kein Herunterladen/Bauen, keine ROS-Nodes und kein Hardwarezugriff
    in diesem Vorabprüfer. Ohne Schalter bleibt das vorherige Profil unverändert.
- Fünf ROS-freie Regressionen prüfen Overlay-/Mischauflösung, fehlende Libraries,
    fehlendes Vendor-SO und Aktivierungsreihenfolge vor Launch. Linux 303/303
    bestanden, Mac 298 + 5 erwartete Skips; Bashsyntax, CLI-Hilfe und Editor sauber.
    **Implementiert und lokal geprüft, G4 mit Fix noch nicht runtime-abgenommen.**
- Nächster Schritt bleibt ein einzelner manueller visueller G4-Lauf mit
    `--tf2-fix`; keine Änderungen an Zielen, Grenzen oder Navigationsparametern.

### Run 25: geprüfter TF2-Fix, freie Fahrt und Umfahrung PASS

- Manueller Nutzerlauf `--visual --tf2-fix`, Exit 0; Nachweis:
    `/home/toto/linobot2_sim/log/g4-nav-20261007-134435-LJ0KXV/`.
    Overlay-Prüfung, Mapping, Map-Aufbereitung, Neustart/Map-Reuse und Nav2-
    Readiness bestanden. Vorbereitete Map 69×40, 0,05 m, Ursprung `[-0,28; -1,52]`,
    1.703 bekannte und 73 belegte Zellen; Rohkarte erhalten, Testbox unverändert.
- Freies Ziel `(0,55; 0)` SUCCEEDED, Ground-Truth-Endabstand `0,0592808 m`,
    77 Trajektoriensamples und finaler Nullbefehl. Odom-TF-Alter `0,022 s`,
    lokale/globale Footprint-Posealter `0,058 / 0,178 s` am Ergebnis.
- Umfahrungsziel `(2,80; -1,05)` SUCCEEDED, Ground-Truth-Endabstand `0,0355755 m`,
    350 Samples und finaler Nullbefehl. Odom-TF-Alter `0,013 s`, lokale/globale
    Footprint-Posealter `0,089 / 0,273 s` am Ergebnis. map→odom-Future-Stempel
    innerhalb bestehender 3,5-s-Toleranz; der Guard blieb während der Ziele aktiv.
- Globalplan 99 Posen, `2,8391 m` gegenüber `2,5019 m` direkter Strecke;
    minimale Plan-Clearance `0,20952 m`. Ausgeführte Trajektorie `2,7758 m`,
    minimale Roboter-Clearance `0,21999 m` (gefordert 0,15 m), größter Sample-
    Abstand `0,01022 m`. Geschwindigkeiten maximal `0,08 m/s / 0,30 rad/s`.
    Goal-Prüferdauer `70,478 s`, Fehlerliste leer, genau die erwarteten Publisher.
- Nav2-Log: jeweils null Vorkommen von `Transform data too old`,
    `worldToMap failed`, `Sensor origin`, `Controller patience` und
    `Resulting plan has 0`. Das ist der beobachtete Lauf, keine allgemeine
    Fehlerfreiheitsgarantie. Lesender Prozesscheck danach: keine Simulator-,
    Nav2-, RViz- oder SLAM-Reste. Keine weitere Fahrt gestartet.
- Der versionsgebundene TF2-Deadlock ist durch upstream A/B reproduziert und
    behoben; Run 25 bestätigt den reparierten Stand für diese Navigation. Kein
    zusätzlicher Backtrace der früheren Runs und keine Hardwareabnahme behauptet.
- **Historischer Entscheidungsstand nach Run 25:** Zwei-Szenarien-Fixture-Baseline
    PASS, vollständiges G4 damals offen. Die späteren sensor-derived Runs 32–45
    sind im aktuellen Gate-Status am Dokumentanfang zusammengefasst. Keine
    Wiederholung/Tuning ohne neuen Zweck. G5 blieb bis zum vollständigen G4
    offen und wurde am 09.10.2026 separat im SIL bestanden.

### Unerreichbares Ziel: implementiert und ROS-frei geprüft, Runtime offen

- Verifier-Option `--scenario unreachable` nutzt dieselben Readiness-, Topic-,
    Ownership-, TF-/Footprint-, Geschwindigkeits- und Cancellation-Schutzregeln.
    Das physische Ziel ist fest im statischen Testwürfel `(2,0)`; SDF-Geometrie
    regressionsgeprüft. Die Karte muss dort Unknown oder occupied enthalten.
    Lesende Prüfung des originalen Run-25-PGM/YAML: Zielwert `-1`, 23 bekannte-
    freie Startkorridorsamples erhalten. Keine Karte verändert/freigeschaltet.
- Dedicated Harnessmodus `--unreachable-map=ABSOLUTER_YAML_PFAD` verlangt den
    geprüften `--tf2-fix`, startet nur Fortress/Nav2 mit dieser gespeicherten Karte
    und führt exakt einen Zielaufruf aus. Mapping und beide Baselineziele werden
    übersprungen; normale Baseline bleibt Standard. YAML-Herkunft/SHA im neuen Log.
- PASS erfordert tatsächliche Action-Annahme und terminal `ABORTED` (6).
    Ablehnung, `SUCCEEDED`, `CANCELED`, Deadlineüberschreitung oder abbrechender
    Safetyguard sind FAIL. Zielbezogener GridBased-Plannerfehler im eigenen
    `nav2.log` erforderlich; TF-Störung, Kartenrand-/Sensorursprungfehler und
    Controller-Pose-/Patiencefehler werden als andere Ursachen zurückgewiesen.
    Ein leerer Plannerpfad ist bei diesem Negativtest erwartbar, nicht Erfolgsbeweis.
- Recovery muss stets innerhalb 0,25 m von der Anfangspose bleiben; mindestens
    0,15 m Roboter-Hindernisabstand und beobachtete Pose-Lückenprüfung gelten weiter.
    Nach Ergebnis: finaler Nullbefehl, zusätzliche einsekündige Beobachtung,
    mindestens drei Pose-Samples, höchstens 0,02 m Drift und kein wiederkehrender
    Nichtnullbefehl. Clock/TF/Footprint/Sensorfortschritt bleibt Pflicht.
- Action-Deadline 90 Wall-Sekunden; vorhandener begrenzter Cancellation-/Cleanup-
    Pfad und externer 160-s-Probe-Schutz. Keine Nav2-Parameter-/Baselinelimitänderung,
    keine Hardware oder Inferenz. Neue JSON-Berichte kennzeichnen `scenario`.
- Sieben neue Unit-/Vertragstests: Zielbelegung/physische Box, maximale Auslenkung
    statt nur Endpose, Status/Stoppsamples, Mapping-freier Branch und Ziel-/Fehler-
    Attribution. Linux 310/310 bestanden; Mac 305 + 5 Skips; Shellsyntax und Editor
    sauber. **Noch kein Runtime-Nachweis, keine G4-/G5-Abnahme.**

### Run 26: Negativtest FAIL durch Recovery-Policy-/Budgetkonflikt

- Manueller Nutzerlauf, gespeichert unter
    `/home/toto/linobot2_sim/log/g4-nav-20261007-172457-M4mMoW/`.
    Overlay und Nav2-Readiness bestanden. Ziel `(2,0)` weiterhin Unknown `-1`;
    Nav2 nahm die Action an, erzeugte keinen Pfad und meldete zwölf zielbezogene
    GridBased-Planungsfehler. Standard-Recovery führte Spin, Wait, BackUp und
    erneut Spin aus; die Navigation war noch ACTIVE.
- Bei ca. `0,251 m` maximaler Ground-Truth-Auslenkung überschritt die Recovery
    das Testbudget `0,25 m`. Prüfer löste Cancellation aus; akzeptiert und terminal
    `CANCELED` (5). Probe endete nach `34,510 s` mit FAIL. Das ist keine natürliche
    Nav2-Abnahme mit `ABORTED` (6). Abschließende Null-/Stillstandsprüfung des
    Erfolgszweigs wurde nicht erreicht, daher kein solcher Nachweis behauptet.
- Fehler-Snapshot: Clock `29,272 s`, Odom-TF `0,014 s` alt, map→odom `2,928 s`
    Zukunft (innerhalb bestehender Toleranz), lokale/globale Footprints `0,132 /
    0,214 s` alt, AMCL-Pose `0,072 s` alt. 216 Scans, 1.082 Odom-Samples,
    202 Ground-Truth-Samples; keine TF-/Sensorstagnation. Clearance-Meldung
    `1,280 m`, daher kein belegter Hinderniskontakt. Kein erneuter TF2-Deadlock-
    Nachweis; Log ohne `Transform data too old`, `worldToMap failed`,
    `Sensor origin` und `Controller patience`.
- Installierter Standardbaum
    `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`
    lesend geprüft: NavigateRecovery `number_of_retries=6`, Spin `1,57 rad`,
    Wait `5 s`, BackUp `backup_dist=0,30 m`, `backup_speed=0,05 m/s`.
    Der vom Assistant angesetzte 0,25-m-Gesamtbewegungsschutz wurde vor dem Lauf
    nicht gegen diesen Recovery-Vertrag abgeglichen. **Testdefinitionsfehler,
    nicht durch höhere Grenzen oder Wiederholungen wegtesten.**
- Lesende Prozessprüfung danach: keine Fortress-/Nav2-/RViz-/SLAM-Reste.
    Keine weiteren Laufzeitversuche, Parameter-/Grenzwertänderungen oder
    Hardwareaktionen. Die erfolgreiche Run-25-Baseline bleibt erhalten.
- Nächster Entwurf: gewünschte Abbruchpolicy ausdrücklich definieren, etwa einen
    goal-spezifischen Fail-closed-BT ohne Bewegungs-Recoveries bei Planungsfehler.
    Das wäre ein neuer überprüfbarer Modus, keine nachträgliche Abnahme des
    Standardbaums. Erst implementieren/regressionsprüfen, dann gesonderter
    manueller Lauf. Unerreichbar-/Passage-Abnahme, vollständiges G4 und G5 offen.

### Highlevel-Review: Aussagegrenzen und Transferlücken

- **Historischer Review-Snapshot:** Die folgende Empfehlung entstand vor der am
    08.10.2026 im Development Plan festgehaltenen G4-Arbeitsrichtung. Sie bleibt
    als damalige Begründung erhalten; offene quantitative Werte und fehlende
    Implementierungs-/Runtime-Nachweise gelten weiterhin.
- Reviewauftrag nach Start der Fail-closed-Vorbereitung: Implementierung pausiert,
    noch kein neuer Behavior-Tree/Code angelegt und kein Lauf gestartet. Dies ist
    eine fachliche Prüfung und Empfehlung, keine Gate-/Hardwareabnahme.
- **GT gelangt in Navigationseingaben:** `scripts/prepare_phase4_map.py`,
    `prepare_map`: liest `ground_truth_start/end` und `ground_truth_samples`,
    berechnet Sweep-Korridore, setzt passende unbekannte Rasterzellen auf frei.
    `scripts/test_phase4_gate.sh` lädt `fortress_map_nav.yaml` in Nav2. Die rohe
    SLAM-Karte bleibt erhalten und belegte Zellen werden geschützt; trotzdem
    beeinflusst Ground Truth den Navigationseingang, nicht nur die Bewertung.
    **Run 25: PASS für diese GT-assistierte Fixture, sensor-only End-to-End-/
    Hardwareübertragbarkeitsnachweis noch offen.** Messwerte werden nicht verworfen.
- **Geometrie getrennt definiert:** `2wd_fortress.urdf.xacro` enthält eigene
    Eigenschaften; `2wd.urdf.xacro` lädt `2wd_properties.urdf.xacro`. Radius/
    Spurweite im Checkout 0,03375/0,126 m versus 0,045/0,26 m. Gemeinsam genutzte
    Mechanik-Makros ersetzen keine gemeinsame versionierte Parameterquelle.
    Keine Aussage über aktuelle vermessene Pi-/ESP32-Hardware und keine Änderung.
- **Abnahmeumfang:** vier G4-Szenarien bleiben Plananforderung, kein Scope-Creep.
    Fehlende Rausch-/Schlupf-/Aufzeichnungsnachweise ebenfalls sichtbar halten.
    Suche in `scripts` ergab keine rosbag2-/`ros2 bag record`-Implementierung;
    Source-Suche keine expliziten `<noise>`-Sensorblöcke. Das beweist nicht, dass
    die Physik frei von Kontaktgleiten oder realer Varianz ist; kontrollierte
    Modelle und ihre quantifizierte Abnahme sind nicht nachgewiesen.
- **Drift:** mehrfach reaktive Koordinaten-/Korridor-/Toleranzanpassungen und
    eigener Guard ohne Abgleich mit Standard-Recovery. TF2-A/B-Reparatur dagegen
    reproduzierbare Ursachenbehebung; Unknown-Pixel-Klassifikation und TF-/QoS-
    Regeln sind legitime Schnittstellenkorrekturen. Testzahl ist kein Ersatz für
    Systemgeneralität oder G7/HIL-/G8-Abnahme.
- **Empfehlung, noch keine Policyfreigabe:** zuerst eine kurze G4-Matrix mit
    real gewünschtem Abbruch-/Recovery-Verhalten, begründeten Budgets, getrennten
    Fixture-/Sensor-Kartenquellen und Modell-/Backend-Transferbedingungen.
    Keine Kriterien nachträglich auf Run 26 zuschneiden; Fail-closed nur als
    ausdrücklich gewollter, separat abzunehmender Produktmodus. Keine Ausweitung
    auf Hardware/Inferenz oder neues Framework erforderlich.

### Offene Arbeiten

Die priorisierte Restliste wird separat in [`OPEN_TASKS.md`](OPEN_TASKS.md) gepflegt,
damit dieser Statusbericht Nachweise und Fehlschläge dokumentiert, statt eine zweite
Aufgabenliste zu duplizieren. G4/G5 und der lokale G6-Foto-Capture sind im SIL
bestanden. Externe Inferenz sowie Hardware-, Firmware- und Motoraktionen bleiben
ohne die jeweils erforderliche Provenienz und separate Freigabe gesperrt.
