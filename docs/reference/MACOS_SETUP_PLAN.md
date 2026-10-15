# Referenzkatalog: Linorobot2, Mac, Pi, ESP32 und Jetson

**Finale Entwicklungsreihenfolge:** [DEVELOPMENT_PLAN.md](../DEVELOPMENT_PLAN.md).
Dieses Dokument bewahrt Bestandsaufnahme, Quellenwissen, Sicherungen und technische
Alternativen. Es ist **nicht mehr der führende Ablaufplan**. Bei abweichenden
Reihenfolgeempfehlungen gilt der neue Simulation-first-Fahrplan: zuerst lokale
Humble-/Fortress-Entwicklungsfähigkeit, dann virtuelles Modell und Navigation,
entkoppelte Vision, anschließend Hardwareintegration und physisches Feintuning.
Jetson-Speicher-/Offlinearbeiten sind kein Blocker für lokale Simulation.

Stand: 04.10.2026. **Bestandsaufnahme und priorisierte Sicherung ausgeführt;
keine Installation, Migration oder Dateisystemvergrößerung.**

**Aktualisierung nach SSH-Bestandsaufnahme:** Der Pi 4 läuft bereits mit Ubuntu
22.04 ARM64, 8 GB RAM und ROS Humble. Deshalb ist **Humble beibehalten** jetzt
die primäre Entscheidung. Die ursprünglichen Jazzy/Harmonic-Wege in Abschnitten
6–8 sind nur noch spätere Alternativen, keine aktuelle Installationsanweisung.
Der konkrete verteilte Ablauf und die verifizierten Pi-Befunde stehen in Abschnitt 13.

**Neue Priorität: Jetson erhalten und sichern.** Laut Nutzer liegt dort möglicherweise
der einzige vollständige letzte Entwicklungsstand. Bestandsaufnahme bestätigt ein
Vision-Projekt mit lokalen Änderungen und eigenem Containerimage, außerdem ein
volles 29-GiB-Dateisystem auf einer etwa 238-GiB-Partition. ESP32/Lidar und ROS-Stack
sind aktuell nicht sichtbar. Vor Umverteilung, Bereinigung oder Neuinstallation
gilt Abschnitt 14; insbesondere Quelle und Image zuerst sichern.

**Sicherungsstatus:** Vision-Quellen, Modelle, Git-Historie, Container-Dateisystem
und geschützte Laufzeitmetadaten sind geprüft auf Mac und persönlichem NAS gesichert
(Abschnitt 15). Original-Docker-Images und vollständiges Jetson-Systemabbild sind
noch nicht vollständig gesichert. Die externe USB-SSD wurde vom Nutzer wieder
entfernt und ist nicht mehr Bestandteil des Plans. Das persönliche NAS ist das
verfügbare externe Sicherungsziel; ein sicherer Offline-Abbildweg bleibt zu klären.

**Zusätzliche Entwicklungsquellen:** `/Users/toto/Projects/ROS_experiments` enthält
ESP32-Humble-Firmware, YOLO-ROS, Kamera- und Kalibrierungsmodule mit lokalen
Änderungen. Abschnitt 16 dokumentiert Herkunft, Konfigurationsabweichungen und
die zusätzliche geprüfte Sicherung. Insbesondere ist `myrobot` auf WiFi-micro-ROS
eingestellt; serielle Pi-Defaults nicht als bestätigten Firmwarezustand behandeln.

`/Users/toto/Projects/JetsonNano` ergänzt eine neuere Kopie der Vision-Anwendung,
Python-Pakete, PPE-Trainingsstände und einen Focal-Rootfs-/Imagebuilder. Abschnitt 17
ordnet diese Quellen dem gesicherten Jetson-Stand zu. Die lokalen Buildrezepte
verwenden andere NVIDIA-Versionen als der aktive Jetson; nicht ungeprüft deployen.

## 1. Ziel und Entscheidung

Ziel: die sieben Workspace-Pakete bauen, den Roboter in Gazebo simulieren,
mit Teleop bewegen und anschließend SLAM und Nav2 mit RViz benutzen.

Zwei Anforderungen sind zu unterscheiden:

- **Nativ auf macOS kompilieren und simulieren:** Community-/Portierungsweg;
  machbar zu untersuchen, aber für diesen Gesamtstack nicht garantiert.
- **Auf diesem Mac entwickeln und simulieren:** Linux ARM64 in einer VM oder
  einem Container ist deutlich besser durch ROS-Pakete abgedeckt. Dabei entstehen
  Linux-Binaries, keine nativen macOS-Binaries.

Aktuelle Empfehlung: Ubuntu 22.04 / ROS Humble auf dem Pi erhalten, ESP32 für
Motorregelung, Jetson mit unverändertem OS für Vision/Inferenz, Mac für Entwicklung
und Simulation. Der Pi 4 mit 8 GB kann jetzt auch als Ziel für SLAM/Nav2 an Bord
eingeplant werden; die Laufzeitabnahme steht noch aus.

Der finale Fahrplan wählt Ubuntu 22.04 ARM64 / Humble / Fortress auf dem Mac
als Entwicklungsziel und migriert die Classic-Simulationsanbindung separat.
Classic bleibt eine isolierte Legacy-Vergleichsoption (Abschnitt 9), kein
primärer dauerhafter Entwicklungsstandard.
Jazzy/Harmonic bleibt eine bewusst getrennte spätere Modernisierung, nicht die
Voraussetzung für den Pi-/Jetson-Betrieb. Jazzy und Harmonic sind bis Mai 2029
unterstützt; Humble und Fortress bis Mai 2027, Classic ist bereits EOL.

## 2. Verifizierter Bestand

| Bereich | Befund |
| --- | --- |
| Rechner | MacBook Pro, Apple M4 Max, ARM64, 16 CPU-Kerne, 64 GB RAM |
| Betriebssystem | `sw_vers`: macOS 27.0.1, Build 26A434; Homebrew bezeichnet die Plattform als `golden_gate` |
| Freier Speicher | ungefähr 109 GiB zum Analysezeitpunkt |
| Entwicklungswerkzeuge | Homebrew, Xcode-Pfad, CMake 4.1.0, Ninja und mehrere Homebrew-Python-Versionen vorhanden |
| ROS im aktuellen Terminal | `ros2` und `colcon` nicht im PATH |
| Gazebo | Homebrew `gz-harmonic`, `gz-sim8` 8.6.0 sowie Fortress installiert; `gz sim --versions` bestätigt 8.6.0 |
| Gazebo Classic | ausführbares `gazebo` nicht im PATH |
| Container | Docker-CLI und Podman vorhanden; ausgewählter Docker-Kontext `orbstack`; Engine/Simulation noch nicht getestet |
| Alte ROS-Umgebung | CMake-Caches verweisen auf `/Users/toto/miniconda3/envs/ros_env`; unter den geprüften Standardpfaden keine entsprechende Conda-Installation gefunden |

**Gazebo fehlt also nicht vollständig. Es fehlt die zum aktuellen Projekt passende
Classic-Anbindung beziehungsweise eine Migration auf das vorhandene Harmonic.**
Die Versionsauskunft bestätigt keine funktionierende GUI oder Sensorsimulation.
Für Golden Gate/M4 liegt aus den konsultierten Quellen keine ausdrückliche
Zertifizierung des gesamten nativen ROS/Gazebo-Stacks vor.

## 3. Projektstruktur und konkrete Hindernisse

Der Workspace heißt lokal `linobot2_ws`; die ROS-Pakete heißen `linorobot2_*`.
Es handelt sich um einen ROS-2-Colcon-Workspace, überwiegend aus Launch-,
Konfigurations- und Beschreibungsdateien, nicht um sieben große C++-Programme.

| Paket | Funktion / wesentliche Abhängigkeiten |
| --- | --- |
| `linorobot2` | Metapaket |
| `linorobot2_base` | EKF-Konfiguration für `robot_localization` |
| `linorobot2_bringup` | Hardwarestart, `joy_linux`, Teleop, micro-ROS, Bild-/Laserverarbeitung |
| `linorobot2_description` | Xacro/URDF, `robot_state_publisher`, `joint_state_publisher`, RViz |
| `linorobot2_gazebo` | Gazebo Classic, EKF, Python-Timeout-Node und Map-Konvertierung |
| `linorobot2_navigation` | `nav2_bringup`, `slam_toolbox` |
| `linorobot2_viz` | separate RViz-Launches für Darstellung/Navigation |

### Versions- und Plattformprobleme

1. `src/linorobot2` steht auf `humble`, Commit `6b7ae2e` vom 09.02.2025.
   `linorobot2_viz` steht auf `master`. Git zeigt in beiden Repositories nur
   unversionierte macOS-/Editor-Dateien, keine Änderungen an versionierten Dateien.
   Bereits eingecheckte projektspezifische Modelle trotzdem gegen Upstream vergleichen.
2. `linorobot2_gazebo/launch/gazebo.launch.py` startet `gazebo` mit
   `libgazebo_ros_factory.so` und `libgazebo_ros_init.so` und verwendet
   `gazebo_ros/spawn_entity.py`. Das ist **Gazebo Classic**, nicht Harmonic.
3. Die Controller-/Sensor-Xacros laden `libgazebo_ros_*.so`. Diese Plugins sind
   nicht mit `gz sim` kompatibel. Dateiendungen umzubenennen löst das nicht.
4. Gazebo Classic 11 ist seit Januar 2025 End-of-Life. Humble wird bis Mai 2027
   unterstützt; das verlängert den Classic-Support nicht.
5. Der lokale Installer benutzt `apt`, `/opt/ros`, Linux-udev und Hardwaretreiber.
   Er erzeugt außerdem `COLCON_IGNORE` im Gazebo-Paket. **Nicht für das Mac-
   Simulationssetup ausführen**, auch nicht als vermeintliche Komplettlösung.
6. `build/`, `install/`, `log/` stammen aus früheren macOS/Conda-Builds.
   Die Caches und Setup-Dateien enthalten alte absolute Pfade und dürfen nicht
   als aktuelle Installation oder als Linux-Build weiterverwendet werden.
7. Das lokale Gazebo-CMake ruft beim Konfigurieren `python3 .../map2gazebo.py` auf.
   Ein Fehler führt nur zu einer WARNUNG. Somit kann ein Build erfolgreich enden,
   obwohl Meshes/Modelle fehlen. Das Skript schreibt Welten ins Installationsziel;
   der anschließende Installationsschritt kopiert ebenfalls `worlds` dorthin.
   Generierte und mitgelieferte Welten sowie Modellpfade getrennt prüfen.
8. Viele Laufzeitabhängigkeiten sind nur als `exec_depend` eingetragen. Ein
   erfolgreicher Colcon-Build beweist deshalb nicht ihre Verfügbarkeit.

### Simulationsumfang versus Hardware

Für die Simulation sind keine USB-Sensoren, udev-Regeln, CUDA, ZED-SDKs oder
micro-ROS-Firmware erforderlich. `joy_linux` ist Linux-spezifisch und für
Keyboard-Teleop entbehrlich. Alle sieben Projektpakete zu bauen ist etwas anderes
als alle Hardware-Launches auf macOS lauffähig zu machen.

Für native Builds Hardwareabhängigkeiten sauber optionalisieren beziehungsweise
als bewusst nicht unterstützten Laufzeitumfang dokumentieren. Nicht die gesamte
Bringup-Struktur löschen, nur um eine Abhängigkeitsprüfung zu bestehen.

## 4. Mögliche Wege

| Weg | Stack | Bewertung |
| --- | --- | --- |
| A: nativ, spätere Option | macOS ARM64, RoboStack/Pixi Jazzy, Harmonic, `ros_gz` | Native Kompilierung erst nach Machbarkeitstest; derzeit nicht versionsgleich zum Pi |
| B: spätere Modernisierung | Ubuntu 24.04 ARM64, Jazzy, Harmonic | Nicht für die erste Inbetriebnahme; erfordert eine separate Kompatibilitäts-/Migrationsentscheidung |
| C: optionale Legacy-Referenz | Ubuntu 22.04 ARM64, Humble, Classic 11, `gazebo_ros_pkgs` | Isolierter Vergleich/Fallback, kein primärer Entwicklungspfad wegen EOL |
| D: finaler Entwicklungsstandard | Ubuntu 22.04 ARM64, Humble, Fortress, `ros_gz` | ROS-Version bleibt gleich; Simulator-/Plugin-Migration in DEVELOPMENT_PLAN.md eingeordnet |

Für B/C: vollständige Desktop-VM ist für interaktive GUI-Arbeit eine gute Option;
ein ARM64-Container mit VNC/noVNC/KasmVNC ist für reproduzierbaren Betrieb ebenfalls
geeignet. Vor einer zusätzlichen VM erst die vorhandene OrbStack-Lösung prüfen.
Nicht ungeprüft x86_64-Images emulieren.

**Grafik ist das Hauptrisiko, nicht CPU/RAM.** Apple-GPU-Leistung steht Linux-
Containern nicht automatisch als Linux-GPU-Passthrough zur Verfügung. CUDA ist
hier kein Weg. Software-Rendering kann reichen, insbesondere bei reduzierter
Auflösung/Sensorrate, muss aber gemessen werden. VM-Grafikunterstützung vor der
Wahl des Produkts konkret prüfen; UTM/QEMU, Parallels und Container sind nicht
gleichwertig bezüglich 3D-Unterstützung.

Zunächst ROS, Gazebo, Bridge, Nav2 und RViz vollständig in derselben Umgebung
betreiben. Ein späterer geteilter Mac-/Linux-Betrieb verlangt zusätzliche DDS-,
Multicast-, Firewall- und Gazebo-Transport-Prüfungen und ist kein erster Meilenstein.

## 5. Gemeinsame Vorbereitung – später auszuführen

1. Quellen, lokale Robotermodelle, Karten und bisherige Logs sichern; Git-SHAs
   und Paketversionen protokollieren. Keine alten Build-Ordner vorzeitig löschen.
2. Separaten Humble-Arbeitsstand außerhalb des gescannten `src`-Baums anlegen;
   zuerst Pi-`toto_dev` gegen Mac-`humble` vergleichen. Spätere Jazzy-Versuche
   separat halten. Keinen der bestehenden Checkouts ungeprüft ersetzen.
3. Neues Build-/Install-/Log-Ziel je Plattform und ROS-Version verwenden.
   Linux-Builds vorzugsweise auf dem Linux-Dateisystem/benannten Volumes ablegen.
4. Ersten Roboter festlegen: standardmäßig `2wd`. Danach eigene Varianten,
   `4wd` und Mecanum einzeln abnehmen. Lokale `turtlebot`-/`vattenkar`-Modelle
   nicht stillschweigend durch den Upstream-Standard ersetzen.
5. Einmalige Umgebung statt globaler Shell-Umbauten vorsehen. Bash benutzt
   `setup.bash`, Zsh `setup.zsh`. ROS-Versionen nicht im selben Terminal mischen.
6. Die vorbereitete `.env` im Workspace ist nur eine Vorlage und wird von ROS
   nicht automatisch eingelesen. Später einen expliziten Aktivierungs-/Export-
   Schritt erstellen; keine globalen Sensorvariablen voraussetzen.
7. Rund 30–50 GiB Budget für eine Umgebung einplanen, mit Reserve für Images,
   Quellbuilds und Modelle. Mehrere große VMs/Images bei 109 GiB frei vermeiden.

## 6. Weg A – nativer macOS-Machbarkeitstest

### A1: isolierte ROS-Umgebung

- Pixi mit RoboStack-Jazzy und conda-forge für `osx-arm64` vorsehen; Lockfile
  erstellen. Kein ROS im Conda-Base installieren, keine alten Underlays sourcen.
- Vor Installation eine Solver-/Verfügbarkeitsprüfung durchführen: ROS Desktop,
  Colcon, Ament, Xacro, RViz, `robot_localization`, Nav2, SLAM Toolbox,
  `ros_gz_sim`, `ros_gz_bridge`, gegebenenfalls `ros_gz_image` und Bildverarbeitung.
- Kanalnamen und Paketnamen anhand des aktuellen RoboStack-Index übernehmen;
  nicht annehmen, dass jedes Ubuntu-ROS-Paket als macOS-Binary vorhanden ist.
- Python, Compiler und CMake innerhalb dieser Umgebung konsistent auflösen.
  Homebrew-CMake 4.1 ist wegen alter `cmake_minimum_required(VERSION 3.5)`-Dateien
  ein Kompatibilitätsrisiko. Passenden CMake-Stand pinnen oder gezielt Policies
  aktualisieren, nicht global Homebrew downgraden.
- Abnahme: ROS Talker/Listener kommunizieren; RViz startet; einfacher Ament-Build
  funktioniert. **Stopppunkt:** zentrale fehlende Pakete oder Solverkonflikte
  bewerten, bevor umfangreiche ROS-Quellbuilds begonnen werden.

### A2: Gazebo unabhängig testen

- Vorhandenes Harmonic verwenden, nicht blind erneut installieren/upgraden.
- Auf macOS Server und GUI getrennt testen: `gz sim -s -r -v 4 shapes.sdf`
  sowie in einem zweiten Terminal `gz sim -g -v 4`.
- Danach eine Welt mit IMU, GPU-Lidar und Kamera prüfen. Server ohne GUI heißt
  nicht ohne Rendering: Lidar/Kameras benötigen weiterhin einen Renderkontext.
- Die offizielle Dokumentation warnt vor instabiler macOS-GUI. Bei Problemen
  Ogre-/Qt-Logs prüfen; Ogre1-Fallback nur testen, wenn die benötigten Sensoren
  damit tatsächlich funktionieren. Keine SIP-Abschaltung als Standardmaßnahme.

### A3: ROS/Gazebo-Verknüpfung

- Bevorzugt konsistente Gazebo-/Bridge-Pakete aus derselben Umgebung verwenden.
  Das installierte Homebrew-Gazebo bleibt für den Standalone-Test erhalten.
- Falls die Bridge gegen Homebrew-Harmonic gebaut werden muss: passenden
  Jazzy-`ros_gz`-Stand pinnen; ARM64, Gazebo-Hauptversion, Protobuf, Qt und
  Suchpfade prüfen. Gleiche Architektur allein gewährleistet keine ABI-Kompatibilität.
- Abnahme: `/clock` aus Gazebo erreicht ROS; Testkommando erreicht Gazebo;
  Test-LaserScan kommt mit korrektem Frame und Zeitstempel an.
- Zeitbudget für diese Machbarkeitstests vorab begrenzen, z. B. auf einen Arbeitstag.
  Bei zentralen Blockern Weg B nutzen statt eine unkontrollierte Portierung beginnen.

## 7. Jazzy/Harmonic-Projektmigration – für A und B

Der aktuelle Upstream-`jazzy`-Zweig verwendet bereits `ros_gz_sim` und
`ros_gz_bridge`, getrennte Server-/GUI-Launches und moderne Welten. Diesen als
Basis vergleichen, nicht sämtliche Classic-Integration selbst neu entwickeln.
Upstream-SHA bei Ausführung pinnen; Webinhalte können sich bis dahin ändern.

1. **Beschreibungen übernehmen:** eigene Geometrie, Massen, Trägheiten,
   Radabstände, Sensorposen, Zusatzkameras und Karten gegen Upstream vergleichen.
2. **Launch/Spawn:** `gazebo_ros` durch `ros_gz_sim` ersetzen; Spawner `create`
   und seine tatsächlichen Optionen prüfen (`-name` statt altem `-entity`, sofern
   vom gepinnten Stand verlangt). Auch Upstream nicht ungeprüft übernehmen.
3. **Plugins:** Differential-/Skid-/Mecanum-Antrieb und JointStatePublisher auf
   passende Gazebo-Systeme umstellen; IMU-/Sensors-Systeme hinzufügen.
   Classic-Ray-Konfiguration auf unterstützten Lidar-Typ migrieren; RGB/RGBD-
   Sensoren, Frame-IDs, Topics und Bildformate kontrollieren.
4. **Welten:** Physics, UserCommands, SceneBroadcaster, Sensors und IMU explizit
   konfigurieren, falls eigene World-Plugins die Defaults ersetzen. Classic-
   Materialskripte, Model-URIs und Physikparameter nicht unverändert voraussetzen.
5. **Ressourcen:** `GAZEBO_MODEL_PATH` durch den benötigten
   `GZ_SIM_RESOURCE_PATH` ergänzen/ersetzen; alle lokalen Meshes installieren.
6. **Bridge:** `/clock`, `/cmd_vel`, `/odom/unfiltered`, `/imu/data`, `/scan`,
   `/joint_states` und anschließend Kamera-/Tiefenthemen explizit zuordnen.
   Richtungen und QoS pro Topic festlegen; Bilder bei Bedarf über `ros_gz_image`.
7. **TF:** der EKF soll weiter genau einmal `odom → base_footprint` veröffentlichen.
   Keine zweite konkurrierende Odom-TF aus dem Antriebsplugin einschalten.
   Rad-/Sensor-TFs und `map → odom` gesondert überprüfen.
8. **Navigation:** Humble-YAML nicht blind in Jazzy verwenden. Plugins,
   Parameter, Footprint, Costmap-Sensorquellen und `/cmd_vel`-Nachrichtentyp
   (`Twist` versus `TwistStamped`) entlang der ganzen Kette prüfen.
9. **Timeout:** Schutzfunktion des lokalen `command_timeout.py` erhalten;
   wiederholte Teleop-Kommandos oberhalb 5 Hz vorsehen. Aktuell kann ein einzelner
   Tastendruck nach etwa 0,2–0,4 Sekunden durch die Timerprüfung gebremst werden.
10. **Build-Metadaten:** neue Laufzeitabhängigkeiten deklarieren. Im betrachteten
    Jazzy-Upstream-Paketmanifest fehlen explizite `ros_gz_sim`-/Bridge-Abhängigkeiten
    trotz Nutzung im Launch; diese gezielt installieren/deklarieren.
    Upstream ändert das Gazebo-Paket außerdem auf `ament_python`: alten CMake-
    Installationsweg nicht mit dem neuen Python-Paket vermischen.
11. **Map-Konvertierung:** auf separates, prüfbares Tool umstellen oder den
    vorhandenen Ablauf härten. NumPy, OpenCV, PyYAML, trimesh, Matplotlib und
    weitere tatsächlich verwendete Bibliotheken konsistent pinnen. Alte
    trimesh-Methoden wie `remove_duplicate_faces()` auf aktuelle API prüfen.

## 8. Weg B – moderne Linux-Umgebung auf dem Mac

1. Ubuntu 24.04 ARM64, zunächst ungefähr 8 CPU-Kerne und 16 GB RAM; bei Bedarf
   auf 24 GB erhöhen. Snapshot beziehungsweise reproduzierbares Image vorsehen.
2. GUI-Pfad zuerst entscheiden/testen: Desktop-VM oder ARM64-Container mit
   virtuellem Display und browserbasierter VNC-Oberfläche. Kein pauschales `xhost +`.
   VNC-Port nur lokal veröffentlichen; Authentifizierung bei Netzwerkzugriff.
3. ROS Jazzy Desktop, Buildtools, rosdep, Nav2, SLAM Toolbox,
   `robot_localization`, Xacro, Teleop und Bildverarbeitung installieren.
   `ros-jazzy-ros-gz` liefert die passende Harmonic-Anbindung aus dem ROS-Repository.
4. Alle sieben Projektpakete samt externem Viz-Repository in den neuen Workspace
   übernehmen. Keine Mac-Buildartefakte und keine absoluten Host-Symlinks kopieren.
5. rosdep für den gepinnten Quellstand prüfen; für reine Simulation
   `micro_ros_agent` und `microxrcedds_agent` bewusst überspringen.
   Pip-basierte Schlüssel und Ubuntu-Python/PEP-668-Regeln gesondert behandeln;
   kein globales `sudo pip` und keine parallelen OpenCV-Installationen.
6. Linux-spezifische Compose-Annahmen des Upstream-Docker-Setups anpassen:
   ARM64-Basisimages, kein CUDA/NVIDIA-/`/dev/dri`-Zwang, Portfreigaben, Volumes,
   UID/GID, virtueller Bildschirm und gemeinsames ROS-Netzwerk.
7. Standalone-Gazebo und Sensoren testen, dann Migration aus Abschnitt 7 bauen.
   Software-Rendering bei Bedarf gezielt einschalten; Sensorauflösung und Raten
   zunächst reduzieren. Grafikfehler vor Nav2-Debugging lösen.

## 9. Weg C – optionale Humble/Classic-Legacy-Referenz

Nur isolierter Vergleich beziehungsweise zeitlich begrenzter Fallback für den
heutigen Simulationscode. Primärweg ist Humble/Fortress gemäß DEVELOPMENT_PLAN.md.
Modelle mit Pi-/Firmwarequellen abgleichen; gleicher Branchname allein reicht nicht.

- Getrennte Ubuntu-22.04-ARM64-Umgebung mit Humble, Classic 11,
  `ros-humble-gazebo-ros-pkgs`, Nav2 und SLAM Toolbox vorbereiten.
- ARM64-Paketverfügbarkeit und Legacy-Repository-Erreichbarkeit vorab prüfen.
  Fehlende EOL-Binaries können einen gepinnten Quellbuild erfordern; nicht zugesichert.
- Quellen unverändert übernehmen, rosdep mit dokumentierten micro-ROS-Ausnahmen
  verwenden und alle sieben Pakete frisch bauen. Map-Konvertierung separat prüfen.
- `LINOROBOT2_BASE=2wd` setzen. Lokaler Simulationsstart:
  `ros2 launch linorobot2_gazebo gazebo.launch.py`.
- Weder Fortress noch Harmonic ersetzen Classic durch bloße Installation.
- Nach erfolgreicher Referenzmessung Fortress mit Humble als Weg D bewerten.
   Keine automatische Jazzy-Migration; Pi-/Jetson-Kompatibilität bleibt vorrangig.
   Classic-Umgebung isolieren und nicht als dauerhaft unterstützte Lösung behandeln.

## 10. Build- und Abnahmefolge

Jede Stufe erst nach erfolgreicher vorheriger Stufe durchführen:

| Stufe | Prüfung | Erfolgskriterium |
| --- | --- | --- |
| 1 | ROS Talker/Listener | Nachrichten laufen zwischen zwei Prozessen |
| 2 | Gazebo allein | Welt lädt, Physik läuft, GUI verbindet sich |
| 3 | Render-Sensoren | Laser-/Bilddaten entstehen, nicht nur sichtbare Geometrie |
| 4 | Basis/Description/Viz | Xacro expandiert, RViz zeigt konsistentes Robotermodell |
| 5 | alle Pakete | sieben Pakete entdeckt, gebaut und installiert; kein Gazebo-`COLCON_IGNORE` |
| 6 | ROS/Gazebo-Brücke | Simulationszeit und Sensoren in ROS mit passenden QoS/Frames |
| 7 | Fahrbetrieb/EKF | Vorwärtsfahrt/Drehung, plausible `/odom`, eindeutige TF-Kette |
| 8 | Kameras | RGB-/Tiefe-/PointCloud-Daten und optische Frames stimmen |
| 9 | SLAM | Karte wächst, lässt sich speichern und wieder laden |
| 10 | Nav2 | Initialpose akzeptiert, Ziel erreichbar, Hindernisse werden umfahren |
| 11 | Stabilität | mindestens 15 Minuten, zweimal sauberer Neustart, keine verwaisten Prozesse |

Build zunächst paketweise diagnostizieren, danach vollständiger
`colcon build --symlink-install` im neuen Workspace; anschließend
`colcon test` und `colcon test-result --verbose`. Wo das Projekt keine Tests
enthält, manuelle Abnahme nicht durch einen leeren Testbericht ersetzen.

Für die heutigen Humble-Launches sind `sim:=true rviz:=true` bei SLAM/Nav2
verfügbar. Beim Jazzy-Stand erst `--show-args` prüfen: Weltargumente ändern sich
von `world` auf `world_name`/`world_path`; Defaultwelt ist dort
`turtlebot3_world`, mit empfohlenem `spawn_x:=0.5` wegen eines Hindernisses im Ursprung.
Die Navigationskarte muss immer zur geladenen Welt passen.

`LINOROBOT2_BASE` ist erforderlich, sonst entsteht ein Pfad zu
`None.urdf.xacro`. Hardware-Sensorvariablen sind nicht Voraussetzung für den
lokalen Standard-Simulationslaunch. Für Kameranavigation später zur simulierten
Topic-Konfiguration passende Werte gezielt wählen.

## 11. Spätere Liefergegenstände und Rückbau

- gepinnter Quellenstand, Pixi-Lockfile oder Docker-/VM-Setup;
- separater Aktivierungsschritt und dokumentierte Build-/Launch-Aufrufe;
- Simulation ohne Firmware-/Hardwareinstallation;
- übernommene lokale Robotermodelle und Karten;
- Abnahmeprotokoll für alle sieben Pakete, Sensoren, Teleop, SLAM und Nav2;
- Stop-/Cleanup-Ablauf und Liste tatsächlicher Änderungen.

Rückbau: neue Umgebung/Volumes separat entfernen; Humble-Quellen und alte
Artefakte bleiben bis zur erfolgreichen Abnahme erhalten. Homebrew/ROS nicht
pauschal aktualisieren, Shellprofile nicht automatisch überschreiben.

**Noch nicht ausgeführt:** Installationen, GUI-/Simulationstests, Paket-Solver,
Migration, Builds und Laufzeitabnahme. Die Planung behauptet deren Erfolg nicht.

## 12. Quellen

- ROS-Zielplattformen und Support: https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-2000.rst
- ROS/Gazebo-Paarungen: https://gazebosim.org/docs/latest/ros_installation/
- Classic-End-of-Life: https://classic.gazebosim.org/
- Harmonic macOS: https://gazebosim.org/docs/harmonic/install_osx/
- Getrennte Prozesse / GUI-Warnung: https://gazebosim.org/docs/harmonic/getstarted/
- Rendering-Fehler: https://gazebosim.org/docs/harmonic/troubleshooting/
- Classic-Migration: https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/
- RoboStack/Pixi: https://robostack.github.io/GettingStarted.html
- Native Paketverfügbarkeit: https://robostack.github.io/jazzy.html
- Projektinstallation: https://linorobot.github.io/linorobot2/installation/
- Container/VNC: https://linorobot.github.io/linorobot2/docker/
- Aktueller Jazzy-Launcher: https://raw.githubusercontent.com/linorobot/linorobot2/jazzy/linorobot2_gazebo/launch/gazebo.launch.py

ROS-Dokumentationsseiten waren beim Abruf durch einen Browser-Prüfschutz
eingeschränkt; REP-2000 wurde deshalb direkt aus dem offiziellen Quellrepository
gelesen. Paketauflösung und konkret verwendete Versionen vor Ausführung erneut prüfen.

## 13. Verfeinerter Plan nach Pi-SSH-Analyse

### 13.1 Verifizierter Pi-Bestand

Erhebung am 04.10.2026 per bestehendem SSH-Schlüssel als `pi@robot.fritz.box`.
Nur lesende Abfragen; keine Pakete installiert, kein Build und keine Motor-,
Sensor- oder ROS-Daemon-Prozesse gestartet. Kein sudo verwendet.

| Bereich | Befund |
| --- | --- |
| Board | Raspberry Pi 4 Model B Rev 1.5, 4 × Cortex-A72 |
| RAM | 7,6 GiB nutzbar, entsprechend 8-GB-Modell; ca. 7,4 GiB verfügbar bei Abfrage |
| OS | Ubuntu 22.04.5 LTS, 64 Bit, `aarch64` |
| Kernel | `5.15.98-rt62-raspi`, angepasster RT-Kernel; nicht automatisch austauschen |
| Speicher | Root auf microSD, 59 GB Dateisystem, ungefähr 48 GB frei; kein Swap |
| Werkzeuge | Python 3.10.12, CMake 3.22.1, Colcon, rosdep, Docker vorhanden |
| ROS | `/opt/ros/humble`; CLI funktioniert nach expliziter Umgebungsaktivierung |
| Workspace | `/home/pi/linorobot2_ws` |
| Linorobot-Quellen | Zweig `toto_dev`, sauberer Git-Zustand, SHA `3f5fb12a555cd4d4209a924b056b14a6636eda88` |
| micro-ROS-Setup | Zweig `humble`, Commit `7c93d68`; ARM64-Agent-Executable im Overlay vorhanden |
| Navigation | `nav2_bringup` vorhanden; Navigation2-Paket 1.1.18 |
| SLAM / EKF | SLAM Toolbox 2.6.9 und robot_localization 3.5.3 vorhanden |
| Middleware | Cyclone DDS installiert, `.bashrc` deklariert `rmw_cyclonedds_cpp` |
| Roboterwahl | `.bashrc`: `LINOROBOT2_BASE=2wd`, Laser `rplidar`, Depth leer |
| Vision | zusätzliches Projektpaket `linorobot2_vision` vorhanden und im Overlay auffindbar |
| Simulation | Gazebo-Paket durch `COLCON_IGNORE` ausgeschlossen; `gazebo_ros`/`ros_gz_bridge` nicht auffindbar, auf dem Hardware-Pi nicht nötig |
| Netzwerk | WLAN aktiv, Ethernet-Link aus; IPv4 zum Erhebungszeitpunkt `192.168.0.168` |
| Zeit | NTP synchronisiert, Zeitzone Europe/Berlin |
| Geräte | keine `/dev/ttyUSB*`, `/dev/ttyACM*` oder USB-Kamera/Lidar sichtbar |
| Rechte | Benutzer `pi` bereits in `dialout`, `video`, `gpio`, `spi`, `i2c` |
| Kühlung | ca. 59,4 °C im Leerlauf; `get_throttled=0x0`, kein gemeldetes Throttling/Unterspannung zum Abfragezeitpunkt |
| Prozesse | keine durch die Namensabfrage erkennbaren Roboternodes oder passenden Systemdienste |

Installierte Pakete/Executables sind noch kein Nachweis funktionierender Hardware,
Firmware, eines aktuellen Builds oder erfolgreicher Navigation. Das historische
Build-Ergebnis wurde durch die Abfragen nicht abschließend bestätigt.

### 13.2 Endgültige Rollen für die erste Inbetriebnahme

| Gerät | Verantwortlichkeit |
| --- | --- |
| ESP32 | Encoder, geschlossener Radregelkreis, optional IMU, Befehls-Watchdog; steuert separaten Motor-Leistungstreiber |
| Pi 4 | micro-ROS-Agent, Lidar, Roboter-TF, EKF, Befehlsvermittlung; anschließend SLAM/Nav2 an Bord |
| Jetson Nano | Kamera-Treiber, Vision und Inferenz; Host-OS/JetPack unverändert lassen |
| Mac | Editor, Quellenverwaltung, Humble-Buildumgebung, Gazebo, RViz und Analyse |

Pi/ESP32 müssen ohne Jetson autonom sicher stoppen können. Motorregelung und
Stoppsignal dürfen weder von Vision noch vom WLAN abhängen. Einen physischen
Not-Aus und eine sichere Stromversorgung zusätzlich vorsehen. Ein ESP32-Watchdog
ersetzt kein unabhängig wirksames Hardware-Sicherheitskonzept.

Mit 8 GB RAM ist der Pi nicht mehr auf bloße Basisfunktionen begrenzt. SLAM/Nav2
an Bord sind ein sinnvoller Zielumfang, aber CPU-Last, Latenz, Kühlung und
microSD-I/O unter realer Last messen. Kamera-Inferenz bleibt auf dem Jetson.

### 13.3 Quellenabgleich vor jeder Installation

Mac-Stand: `humble`, SHA `6b7ae2ebd5e3782560363dca136c3a92864f5a46`.
Pi-Stand: eigener `toto_dev` mit Webcam-/micro-ROS-Anpassungen und Vision-Paket.

Prüfsummenvergleich bestätigt Unterschiede in:

- `linorobot2_description/urdf/robots/2wd.urdf.xacro`;
- `linorobot2_description/urdf/2wd_properties.urdf.xacro`;
- `linorobot2_base/config/ekf.yaml`;
- `linorobot2_navigation/config/navigation.yaml`.

`navigation_sim.yaml` ist in beiden geprüften Ständen identisch. Das bestätigt
nicht, dass die Simulationsgeometrie bereits zur Hardware passt.

Vorgehen: zuerst Jetson nach Abschnitt 14 sichern, dann Pi-Quellen separat sichern.
Der Pi ist nur eine vorläufige Vergleichsquelle, nicht automatisch der neueste Stand.
Jetson-/Pi-/Mac-Diff und Prüfung gegen tatsächliche Rad-/Sensorabmessungen
durchführen. Erst danach gemeinsamen Humble-Stand erstellen. Keinen pauschalen
Git-Pull, kein rsync mit Löschoption und keine alten Installationsordner übertragen.
Quellen mit installierten Launch-/Config-Dateien vergleichen: sauberes Git beweist
nicht, dass der letzte Build genau diesen Quellstand installiert hat.

### 13.4 Konkrete Launch-Risiken, noch nicht behoben

1. Pi-`bringup.launch.py` setzt `webcam_enabled=true`. Für die Pi-Basis später
   ausdrücklich `webcam_enabled:=false` wählen, damit Kamera/Vision nicht doppelt
   auf Pi und Jetson starten. Der Default-Launch schaltet damit auch Vision aus.
2. `default_robot.launch.py` erwartet ESP32 auf `/dev/ttyUSB1` mit `921600` Baud.
   Die neu gefundene `myrobot`-Quelle verwendet dagegen WiFi und `BAUDRATE=115200`
   (Abschnitt 16). Tatsächlichen Transport und geflashte Firmware prüfen; stabile Gerätepfade
   wie `/dev/serial/by-id/...` oder gezielte udev-Symlinks verwenden.
3. RPLIDAR wird in `lasers.launch.py` auf `/dev/ttyUSB0` mit `115200` Baud gestartet.
   Diese Baudrate ist nicht für jedes RPLIDAR-Modell korrekt. Sensorvariante zuerst
   identifizieren. Einige Zweige verwenden feste Pfade trotz deklarierter Argumente.
4. Der äußere Bringup deklariert Transport/Port, reicht diese an den Default-Launch
   aber nicht ausdrücklich weiter. Launch-Kontextauflösung und Parameterdurchgriff
   gezielt testen/härten, bevor auf UDP umgestellt wird; seriell zuerst verwenden.
5. Paket `sllidar_ros2` liegt im Installationsbaum, aber seine Quellen wurden unter
   den geprüften Suchpfaden nicht gefunden. Reproduzierbarkeit klären; ein altes
   Overlay nicht als vollständige Buildgrundlage behandeln.
6. Nur ein Knoten darf Fahrbefehle final an die Basis geben. Später Cmd-Vel-Mux/
   Prioritäten für Nav2, Teleop und Stopp vorsehen; Jetson-Ergebnisse erhalten kein
   unkontrolliertes direktes Schreibrecht auf den Motorbefehl.

### 13.5 Schrittfolge für die Umsetzung

Die folgende ältere Aufstellung enthält Hardware-Bausteine, **keine aktuell
verbindliche Entwicklungsreihenfolge**. Lokale Simulation und virtuelle Tests
kommen gemäß DEVELOPMENT_PLAN.md vor Hardwareinbetriebnahme und Feintuning.

1. **Sichern:** SD-Kartenimage beziehungsweise geeignete vollständige Sicherung,
   Git-Stände und installierte Paketversionen sichern. RT-Kernel, Bootdateien und
   Jetson-OS nicht verändern. Keine pauschalen apt-/Distributions-Upgrades.
2. **Hardware identifizieren:** ESP32-Board/Firmware, Lidar-Modell und Kamera
   zuerst am bestehenden Jetson erfassen. Laut Nutzer sind ESP32/Lidar dort bereits
   angeschlossen. Noch nicht auf den Pi umstecken; funktionsfähige Referenz und
   Gerätezuordnung zuerst sichern. Motorversorgung für Tests ausgeschaltet lassen.
3. **Quellen konsolidieren:** Abschnitt 13.3 durchführen; lokale `.env` bleibt
   lediglich Vorlage. Auf dem Pi existieren bereits passende Basis-/Sensorwerte.
4. **Pi-Basis prüfen:** vorhandene Umgebung verwenden. Zunächst Xacro/TF,
   micro-ROS-Verbindungsaufbau, Encoder/IMU und LaserScan getrennt abnehmen.
   Fehlende Abhängigkeiten erst nach Inventarvergleich gezielt ergänzen.
5. **Sichere Motorabnahme:** Firmware-Timeout zuerst prüfen, dann mit angehobenen
   Rädern und niedriger Geschwindigkeit Encoder-Richtung, Radregelung und Stopps
   bei Verbindungsverlust testen. Erst danach Fahrtests am Boden.
6. **Simulation auf Mac:** Ubuntu 22.04 ARM64 / Humble separat einrichten.
   Pi-kompatibles Modell übernehmen, Hardwaretreiber nicht starten. Classic 11
   ist der minimale Änderungsweg; Fortress ist die alternative Simulator-Migration.
   Keine Gazebo-Installation auf dem Pi notwendig.
7. **Navigation auf Pi:** zunächst EKF und Teleop, dann SLAM, Kartenexport und
   schließlich Nav2. Reale Hardware verwendet `use_sim_time=false`; Sensorframes,
   Footprint, Scanfrequenz und eindeutige `odom`-TF verifizieren.
8. **Jetson anbinden:** Jetson-SSH-Ziel/OS zuerst erheben. Bei kompatiblem Humble
   ROS-Topics nutzen, sonst getrennten Ergebnisdienst plus Humble-Adapter auf Pi.
   Datenvertrag mit Zeitstempel, Frame, Klassen, Konfidenz und Koordinaten festlegen.
   Rohbilder zunächst lokal verarbeiten statt unnötig über WLAN übertragen.
9. **Netzwerk und Isolation:** zunächst gleiche ROS-Version, bevorzugt Cyclone
   DDS, ausdrücklich festgelegte `ROS_DOMAIN_ID` und NTP. Realbetrieb und
   Simulation in unterschiedlichen Domains halten; reale/simulierte `/cmd_vel`
   dürfen niemals versehentlich zusammengeführt werden. Keine `/clock`-Bridge
   in den realen Roboter. Ethernet Pi↔Jetson bevorzugen; WLAN-Latenzen messen.
10. **Abnahme:** Navigation ohne Mac, Ausfall Jetson, Ausfall WLAN, Sensorverlust,
    Stopps und Neustarts prüfen; CPU/RAM/Temperatur unter Last dokumentieren.
    Autostart/systemd erst nach erfolgreicher manueller Inbetriebnahme ergänzen.

SSH über `robot.fritz.box` funktioniert; das bestätigt weder DDS-Multicast noch
die Erreichbarkeit von ROS-Nodes in einer Mac-Container-VM. Discovery und UDP-
Verbindungen zwischen Mac-Linux-Umgebung und Pi müssen separat getestet werden.
Für den ersten Onboard-Betrieb bleibt Navigation deshalb vollständig auf dem Pi;
bei Bedarf Visualisierung zunächst per Remote-Desktop im selben Linux-System.

### 13.6 Humble mit Fortress als Alternative zu Classic

Humble/Fortress ist die reguläre moderne ROS/Gazebo-Paarung für Ubuntu 22.04.
Für den Mac zunächst ebenfalls die passende Linux-ARM64-Umgebung bevorzugen;
vorhandenes natives Fortress nur als separate Machbarkeitsoption behandeln.

Migration umfasst Launch/Spawn, Antriebs-/Sensorplugins, Welt und ROS-Bridges.
Abschnitt 7 liefert das konzeptionelle Muster, seine Jazzy-/Harmonic-Namen und
Buildänderungen aber nicht blind übernehmen: Fortress verwendet typischerweise
`ign gazebo` und andere Bibliotheks-/Plugin-Versionen. Humble-kompatiblen Stand
von `ros_gz` und die zu Fortress gehörenden APIs pinnen. Der Pi-Hardwarestack und
seine ROS-Nachrichtentypen müssen für diese Simulator-Migration nicht wechseln.
Wegen Humble-/Fortress-EOL Mai 2027 bleibt eine spätere Migration eine separate
Lebenszyklusentscheidung, ohne dabei das Jetson-OS zwangsweise zu aktualisieren.

### 13.7 Noch vom Nutzer beziehungsweise Jetson zu klären

- Jetson-Benutzer `jetson` bekannt; `nano.fritz.box` antwortet auf SSH, Geräteidentität
   und NVIDIA-Nano-Board bestätigt. NVIDIA-/Vision-Inventar siehe Abschnitt 14.5;
   ROS-/micro-ROS-Stand noch nicht lokalisiert.
- Ursprünglicher Jetson Nano bestätigt; Kamera, ESP32 und Lidar erscheinen derzeit
   nicht in der USB-Liste. Modelle sind teilweise erfasst, siehe Abschnitt 14.5.
- ESP32-Board, aktuelle Firmware/Repository, Transport und konfigurierte Baudrate.
- Firmwarequelle in `ROS_experiments/esp32bot_ws` gefunden (Abschnitt 16);
   Übereinstimmung mit der auf dem ESP32 geflashten Version bleibt unbekannt.
- Exaktes RPLIDAR-Modell und tatsächliche Verbindung/Stromversorgung.
- Reale Roboterabmessungen, Encoder-Auflösung und vorhandener Motor-Leistungstreiber.

Diese Angaben verhindern eine sichere Hardware-/Firmwareabnahme noch, nicht die
Planverfeinerung. Keine Zugangspasswörter oder Schlüssel im Chat bereitstellen.

## 14. Jetson: Erhaltung vor Migration

### 14.1 Stand des Zugriffs und Nutzerangaben

- Nutzer berichtet: bisher funktionierender vollständiger Roboterstack, micro-ROS-
   Agent, ESP32 und Lidar über USB, möglicherweise einzige vollständige Quellen.
- `ssh-copy-id` scheitert laut Nutzer mit „Speicher voll“. `df` bestätigt 0 Bytes
  verfügbar im Root-Dateisystem bei nur 20 % Inode-Auslastung. Datenkapazität ist
  das belegte Problem, nicht erschöpfte Inodes; Dateisystem-/Partitionsgrößen weichen
  stark voneinander ab (Abschnitt 14.6).
- DNS: `nano.fritz.box` → `192.168.0.150` am 04.10.2026. SSH antwortet; vorhandene
  Schlüsselanmeldung wurde abgelehnt. Nach Nutzereingabe konnte die bestehende
  SSH-ControlMaster-Verbindung für weitere lesende Abfragen wiederverwendet werden.
  Board, Docker und Datenträger bestätigt; Host-OS laut Nutzerausgabe Ubuntu 20.04.6.
- Passwortanmeldung nur interaktiv im Terminal durch den Nutzer. Kein Passwort
   in Shellbefehlen, Skripten, Umgebungsvorlagen oder Dokumentation hinterlegen.
- Nicht rebooten, flashen, Paketupdates ausführen, Images/Caches/Logs löschen,
   Git reset/clean/pull ausführen oder USB-Geräte umstecken, bevor Sicherung steht.

### 14.2 Erste Bestandsaufnahme nach Anmeldung

1. Host, Board, OS, Kernel, L4T-/JetPack-/CUDA-Version und installierte ROS-Version
    erheben. Bestehende Systemumgebung erhalten; Humble auf Jetson nicht voraussetzen.
2. `df -h` und `df -i` für Root/Home sowie Mounts und gegebenenfalls Quotas prüfen.
    Große Verzeichnisse nur lesend identifizieren. Volles Dateisystem nicht durch
    ein auf dem Jetson erzeugtes Backup oder neue Pakete zusätzlich belasten.
3. Workspaces, Firmwarequellen, Git-Verzeichnisse, Branches, lokale Commits,
    uncommittete und ignorierte Dateien inventarisieren. Keine Repository-Inhalte
    oder vollständigen Environment-Dumps ausgeben, die Zugangsdaten enthalten könnten.
4. USB-/stabile Serial-Gerätepfade, udev-Regeln, Launch-/EKF-/Nav2-Konfiguration,
    installierte Pakete und Systemdienste erfassen. Keine Ports öffnen und keine
    zusätzlichen Agents/Treiber starten, die mit vorhandenen Prozessen konkurrieren.
5. Vorhandenen funktionierenden Zustand von Quellen und installierten Artefakten
    getrennt aufnehmen; ein aktuelles `src` muss nicht dem letzten Build entsprechen.

### 14.3 Sicherung ohne zusätzlichen Jetson-Speicherbedarf

- Zuerst kleine, unersetzliche Daten direkt per SSH auf den Mac beziehungsweise
   einen externen Datenträger streamen: vollständige Projektquellen einschließlich
   `.git`, unversionierter/ignorierter Dateien, ESP32-Firmware, Karten, Kalibrierungen,
   Launch-/Konfigurationsdateien und Build-/Dependency-Manifeste.
- Anschließend benötigte Modellgewichte, TensorRT-Engines, eigene Bibliotheken,
   installierte Overlays und gegebenenfalls Containerartefakte sichern. Umfang und
   freien Zielspeicher vorher prüfen; Engines allein ersetzen weder Modellquellen
   noch den dazu passenden JetPack-/CUDA-Stand.
- Kein Git-Push als alleinige Sicherung: ignorierte Dateien, installierte Binärdaten
   und lokale Gerätekonfiguration würden fehlen. Geheimnisse gegebenenfalls separat
   geschützt sichern, aber nicht ins Git-Repository oder in den Chat übernehmen.
- Backupstatus erst nach erfolgreichem Transfer, Exitcode-Prüfung, Prüfsummen und
   testweiser Wiederherstellung einer Kopie bestätigen. Lesefehler und fehlende Rechte
   dokumentieren; „nicht gesichert“ nicht als „nicht vorhanden“ interpretieren.
- Zweite unabhängige Kopie anlegen. Danach vollständiges Offline-Abbild des
   tatsächlichen Boot-/Root-Datenträgers mit Partitionstabelle und Bootdaten planen.
   Ein Live-Dateiarchiv ist kein garantiert konsistentes, bootfähiges Systemabbild.
- Vollständiges Abbild nicht blind über das aktive gemountete Root-Dateisystem
   erstellen. Datenträgertyp und sicheres Herunterfahren vorher klären; bei internem
   eMMC den zum Gerät passenden NVIDIA-Sicherungsweg separat prüfen, ohne Flashen.

### 14.4 Erst danach Änderungen und Aufgabenteilung

1. Neueste Jetson-Quellen mit Pi/Mac vergleichen und geschützten Referenzstand
    festschreiben; Wiederherstellbarkeit muss vor Migration nachgewiesen sein.
2. Gegebenenfalls eine gezielte Speicherbereinigung separat abstimmen. Erst dann
    kann das Hinterlegen eines SSH-Public-Keys erfolgen; keine privaten Schlüssel kopieren.
3. ESP32/Lidar erst nach Dokumentation ihrer aktuellen Ports, Firmware-/Baudraten
    und funktionierenden Launchkette auf den Pi übertragen. ESP32 nicht neu flashen,
    solange die vorhandene Firmwarequelle beziehungsweise ein geeigneter Rückweg fehlt.
4. Zielaufteilung Pi-Basis/Navigation und Jetson-Vision bleibt bestehen, aber die
    Migration erfolgt schrittweise mit erhaltener Jetson-Referenz und Rückfallpfad.

**Bestandsaufnahme und priorisierte Sicherung erfolgt; keine Reparatur ausgeführt.**
Den tatsächlich ausgeführten Sicherungsumfang und seine Grenzen beschreibt Abschnitt 15.

### 14.5 Konkreter Jetson-Bestand am 04.10.2026

Die erste Systemausgabe stammt vom Nutzer; Container-/Quellen-/Diskabgleich wurde
anschließend direkt über die vorhandene SSH-Verbindung lesend durchgeführt.

| Bereich | Befund |
| --- | --- |
| Gerät | NVIDIA Jetson Nano Developer Kit, ursprünglicher Nano, nicht Orin |
| Host | `nano`, SSH `jetson@nano.fritz.box`, IPv4 `192.168.0.150` zum Erhebungszeitpunkt |
| OS / Kernel | Ubuntu 20.04.6 ARM64, `4.9.253-tegra`; angepasster Userland-/NVIDIA-Stack |
| NVIDIA | L4T `32.6.1-20210726122000`; kein installiertes JetPack-Metapaket in der Abfrage bestätigt |
| CUDA / TensorRT | CUDA Runtime 10.2.300; TensorRT/libnvinfer 8.0.1 für CUDA 10.2 |
| RAM / Swap | etwa 3,9 GiB RAM; etwa 1,1 GiB verfügbar; zram-Swap etwa 1,9 GiB nahezu voll laut erster Ausgabe |
| Root-FS | ext4, `30.660.661.248` Bytes gemeldete Gesamtkapazität, 0 verfügbar, 100 % belegt |
| Inodes | 20 % belegt; kein Inode-Engpass sichtbar |
| Datenträger | `/dev/mmcblk0`, `255.869.321.216` Bytes, etwa 238,3 GiB; mehrere NVIDIA-Bootpartitionen |
| Root-Partition | `/dev/mmcblk0p1`, `255.854.624.256` Bytes; Partition wesentlich größer als ext4-Dateisystem |
| Host-ROS | kein Eintrag unter `/opt/ros`, `ros2`/`colcon` nicht im PATH; keine Roboterworkspaces in den geprüften lesbaren Pfaden gefunden |
| Aktiver Container | `bold_snyder`, Image `ultralytics/ultralytics:custom`, Runtime `nvidia`, Netzwerk `bridge`, Restart `on-failure` |
| Laufzeit | Container gestartet 23.04.2026; Prozessnamen `sh` und `python3`; Funktion der Anwendung nicht getestet |
| Mount | Host `/home/jetson/video_detection/video_detection` → Container `/video`, beschreibbarer Bind-Mount |
| Images | eigenes Image ca. 5,65 GB; zusätzlich `ultralytics/ultralytics:latest-jetson-jetpack4` ca. 5,48 GB; gemeinsame Layer nicht doppelt zählen |
| Imageidentität | Custom `sha256:aacbddba925945a6593c6b78fffbda9470d0d58efd4ebf8df41731ed38f806ed` |
| Containerlayer | beschreibbarer Layer etwa 100.031 Bytes; trotz kleiner Größe eventuell unersetzliche Änderungen, nicht pauschal ignorieren |
| Docker-Daten | `/var/lib/docker`; keine benannten Volumes laut Abfrage |
| Quellen | `/home/jetson/video_detection`, gesamter Baum etwa 145 MB inklusive `.git` und Dateien |
| Git | Branch `segmentation_containerimages`, Commit `439f0ef074df1df91812bc906106cead76f30396` vom 15.08.2025 |
| Lokale Änderungen | `video_detection/app.py` verändert, eine Zeile ersetzt; `video_detection/logs/` unversioniert |
| Modelle | `yolo12n.onnx`, `yolo12n.pt`, `yolo11n.pt`, `yolo11n.engine` im Projekt vorhanden |
| Buildinformationen | Dockerfiles, Jetson-/Intel-Compose-Dateien, Startup-/Buildskripte, Requirements und Pyproject vorhanden |
| Weitere Daten | `/home/jetson/immich` mit Upload-/Konfigurations-/Cache-Verzeichnissen; nicht ungeprüft als löschbaren Cache behandeln |
| USB | nur Realtek USB2-/USB3-Hubs und Root-Hubs; kein ESP32, Lidar oder Kamera sichtbar, keine seriellen Gerätenodes |
| Zeit | NTP synchronisiert, Europe/Amsterdam; unterschiedliche Zeitzonen zum Pi sind bei synchroner UTC kein Problem |

Der L4T-/CUDA-/TensorRT-Stand ist wichtiger als der Ubuntu-Name allein. Kein
Standard-Ubuntu-Reinstall, NVIDIA-Upgrade oder beliebiger neuer Jetson-Container.
Neue Inferenzfunktionen erst gegen genau diesen erhaltenen Treiber-/Runtime-Stack
testen. Den aktuell funktionierenden Zustand nicht durch Änderung des Images ersetzen.

Die Nutzererinnerung an micro-ROS/ESP32/Lidar ist eine wichtige Suchspur, aber noch
kein bestätigter aktueller Bestand: Die einzige sichtbare Containerinstanz ist
Vision, und die geprüften Pfade enthalten kein Robotersetup. Denkbar sind ein
anderer Datenträger/Bootstand, weitere externe Medien oder nicht geprüfte Pfade.
Nicht als endgültigen Verlust interpretieren und keine Firmware neu flashen.

### 14.6 Speicherproblem: Dateisystem nicht zur Partition passend

`df -B1 /` und `lsblk -b` bestätigen gleichzeitig ungefähr 29 GiB ext4 und
238 GiB Root-Partition. Das spricht stark für ein nicht auf die Partitionsgröße
vergrößertes Dateisystem, beispielsweise nach dem Übertragen eines kleineren Images.
Es erklärt nicht, welche Dateien den vorhandenen Platz belegen, eröffnet aber einen
Weg ohne Löschen und ohne OS-Upgrade.

**Noch keine Größenänderung ausführen.** Erst Sicherung, Datenträgerzustand und
vollständiges Partitionslayout prüfen. NVIDIA-Bootpartitionen nicht verändern und
kein pauschales `growpart`/Neuformatieren: Die Root-Partition wird bereits groß
gemeldet. Ob das Layout konsistent ist, ist separat zu verifizieren.

Wenn konsistent und gesichert, ext4-Vergrößerung mit dem passenden `resize2fs`-
Verfahren planen. Eine Online-Vergrößerung kann bei ext4 möglich sein, wird hier
aber nicht ohne vorherige Prüfung zugesichert. Dateisystemprüfung am gemounteten
Root ist kein Ersatz für eine sichere Offline-Prüfung. Dieser administrative Schritt
muss gesondert freigegeben werden; privilegierte Eingaben direkt durch den Nutzer.
Danach `df`, Bootfähigkeit, Docker und Anwendung prüfen, erst dann SSH-Key ablegen.

Snap-Loop-Dateisysteme mit 100 % Belegung sind normal für schreibgeschützte Images.
Sie belegen zwar Dateien auf Root, ihre 100-%-Anzeige ist aber kein eigenständiger
Defekt und kein Grund, ungeprüft Snap-Versionen zu löschen. Der fast volle Swap ist
hier zram/RAM-Kompression, kein als Löschziel geeigneter großer Swapfile auf Root.

### 14.7 Konkrete Sicherungsreihenfolge und offene Fragen

1. **Quellarchiv zuerst:** gesamtes `/home/jetson/video_detection` direkt auf den
   Mac streamen, einschließlich `.git`, lokaler `app.py`-Änderung, ignorierter
   Modelle und Konfiguration. Größe etwa 145 MB; unversionierte Dateien mitnehmen.
   Archiv auf Mac prüfen und eine Kopie testweise extrahieren, nicht auf den Jetson.
   Anwendung schreibt eventuell weiter: konsistente Quell-/Modellabnahme planen
   und veränderte Dateien nachprüfen; kein vollständiger Live-Snapshot versprochen.
2. **Container sichern:** Original-Custom- und Basisimage möglichst mit exakter ID
   erhalten. **Verifizierte Einschränkung:** `docker image save` benötigt hier auch
   beim Streaming temporären Platz unter `/var/lib/docker/tmp` und scheitert am
   vollen Root. Nicht blind wiederholen. Als ausgeführte Rettung dient ein direkter
   Containerexport plus separat gesicherter Bind-Mount und Laufzeitmetadaten.
   Dieser Export erhält das sichtbare Container-Dateisystem, aber keine ursprüngliche
   Image-Layerhistorie und keine vollständige native `docker load`-Sicherung.
   Originalimages nach sicher gelöstem Platzproblem ergänzen. Kein `docker commit`,
   Pull/Build oder großes Zwischenarchiv auf Root.
3. **Bind-Mount beachten:** `/video` kommt vom Host. Ein Containerexport allein
   sichert diese Daten nicht und ersetzt weder Quellarchiv noch Image-/Metadaten-
   Sicherung. Beschreibbare Layer, Host-Mounts und Images getrennt berücksichtigen.
4. **Gesamtsystemabbild:** tatsächlicher Datenträger ist nominal 256 GB, nicht
   32 GB. Auf dem Mac sind aktuell nur etwa 108 GiB frei. Für ein unkomprimiertes
   vollständiges Abbild externes Ziel mit mindestens 300 GB freiem Platz vorsehen,
   beispielsweise eine ausreichend freie 512-GB-/1-TB-SSD. Nicht auf ausreichende
   Kompression verlassen. Boot-/Firmwareanteile außerhalb der Karte separat prüfen:
   `/dev/mtdblock0` ist sichtbar; Kartenimage allein muss das Gerät nicht vollständig
   wiederherstellen. Dazu passenden NVIDIA-/QSPI-Wiederherstellungsweg dokumentieren.
5. **Roboterstand suchen:** Nutzer fragen, ob seit dem letzten Roboterbetrieb die
   Karte gewechselt/geklont wurde und wo frühere Workspaces lagen. Containerdaten,
   externe Medien und Sicherungen danach gezielt prüfen. Keine pauschale Behauptung,
   der gefundene Vision-Branch sei der letzte vollständige Linorobot-Stand.
6. **USB prüfen:** Datenkabel, Stromversorgung, Hub-Port und Geräteeinschaltung
   kontrollieren, ohne Firmware zu ändern. Container können Geräte nicht einfach
   aus der Host-`lsusb`-Liste verschwinden lassen. Motorversorgung getrennt lassen.

Erst nach verifizierten Sicherungen eine Dateisystemerweiterung oder Bereinigung
planen und die Rollen Pi/Jetson praktisch umsetzen. ROS Humble auf dem Pi ist
bestätigt; die Verbindung zur Jetson-Vision kann unabhängig von einer bisher
nicht gefundenen Jetson-ROS-Version gestaltet werden.

## 15. Ausgeführte Sicherung und verbleibende Blocker

### 15.1 Sicherungsorte und Inhalte

Erste Kopie auf dem Mac:
`/Users/toto/Backups/linobot2/jetson-20261004-174406/`.

Zweite, unabhängig gespeicherte Kopie auf dem persönlichen NAS:
`/Volumes/personal_folder/Backups/linobot2/jetson-20261004-174406/`.

Die Archive liegen außerhalb der Git-Workspaces. Lokaler Zugriff wurde auf den
Benutzer beschränkt; die NAS-Kopie wurde im bereits eingebundenen persönlichen
Share angelegt. Laufzeitmetadaten können sensible Environmentwerte enthalten:
ZIP ist keine Verschlüsselung; Metadaten nicht öffentlich teilen oder einchecken.

| Datei | Inhalt / Prüfergebnis |
| --- | --- |
| `video_detection.tar.gz` | etwa 46 MB, gesamter Projektbaum inklusive `.git`, lokaler Änderungen, Modelle und unversionierter Dateien |
| `bold_snyder-rootfs.tar.gz` | etwa 2,4 GB, direkter Export des laufenden Container-Dateisystems einschließlich sichtbarer Layeränderungen |
| `jetson-metadata.zip` | etwa 38 KB, geschützte Container-/Imageinspektion, Paketliste, OS/NVIDIA-Daten, Mounts, Gerätegrößen, Partitionssektoren und kritische Hashes |
| `SHA256SUMS` | Prüfsummen der drei Archive; auf Mac erfasst und auf NAS erfolgreich geprüft |
| `verification.json` | maschinenlesbare Ergebnisse und ausdrücklich nicht erledigte Punkte |
| `verify_backup.py` | verwendete Standardbibliothek-Prüfung; schreibt nur auf dem Mac, Remote-Inventar lesend |
| `restore-test/` | nur auf Mac: testweise wiederhergestellte reguläre Projektdateien, keine Änderungen am Jetson |

Archivprüfsummen:

- Quellen: `ff6251221865c0067c783324c9df4ccfdb556f304d3047cb45a2e475bafa03ab`.
- Container-FS: `4b75c858e21eec2d0b66d38432b15afba759b4471fcfd62ebbe01c7fb54be7ae`.
- Metadaten: `24d2ce817f9d37f16b2367f3a9c8e402932f9ef61b5baf9077bfc4872e55441d`.

### 15.2 Tatsächlich durchgeführte Prüfungen

- Quellarchiv erfolgreich übertragen, tar-/gzip-Lesbarkeit bestätigt.
- 122 reguläre Quelldateien testweise auf Mac wiederhergestellt; sechs kritische
  SHA-256-Werte stimmen mit den gleichzeitig lesend erfassten Jetson-Dateien überein:
  `app.py`, vier Modell-/Engine-Dateien und `.git/HEAD`.
- Vier Spezial-/Linkeinträge bleiben im Originalarchiv erhalten und wurden bei
  der Sicherheitsprüfung nicht als Links auf dem Mac angelegt. Das ist keine
  behauptete vollständige Symlink-/Hardlink-Wiederherstellung.
- Container-FS-Archiv vollständig durchgelesen: 136.723 Einträge, keine Lesefehler.
- Metadaten-ZIP auf Integrität geprüft, Inhalte mit eventuell sensiblen Werten
  nicht in Terminalausgaben oder Chat ausgegeben.
- Alle drei Archivhashes auf NAS erfolgreich gegen `SHA256SUMS` geprüft.
- 14 Partitionsintervalle aus lesbaren sysfs-Sektordaten geprüft: keine gemeldete
  Überlappung. Dies ersetzt weder die Prüfung des vollständigen Partitionstabellen-
  Zustands noch Dateisystemprüfung oder Datenträgergesundheit.

Dies ist eine geprüfte Rettung der gefundenen Entwicklungsdaten, **kein bootfähiges
Systemabbild und kein nachgewiesener Wiederanlauf der Vision-Anwendung**. Laufende
Anwendungsdaten wurden nicht eingefroren; keine globale Live-Snapshot-Konsistenz
versprechen. NVIDIA-Hosttreiber und QSPI-Bootdaten sind nicht vollständig enthalten.

### 15.3 Fehler beim Originalimage-Export

`docker image save` scheiterte an einem temporären Schreibversuch unter
`/var/lib/docker/tmp/docker-export-...` mit „no space left on device“.
Die lokale Datei `ultralytics-images.tar.gz.partial` ist **kein gültiger
abgeschlossener Imagebackup** und gehört nicht zur verifizierten Sicherung.
Es wurden keine bestehenden Jetson-Nutzerdaten, Images oder Logs gelöscht, um
Platz zu schaffen. Die erfolgreiche Alternative war `docker container export`.

Bei Wiederaufbau: Container-FS und separat archivierten Host-Bind-Mount gemeinsam
verwenden, Laufzeitkonfiguration aus den geschützt gesicherten Metadaten übernehmen
und NVIDIA-Runtime-Kompatibilität beachten. Ein Import kann einen neuen Imagezustand
erzeugen, stellt aber die ursprünglichen Image-IDs/Layers nicht wieder her.
Original-Imagearchive weiterhin ergänzen, sobald temporärer Speicher sicher verfügbar ist.

### 15.4 NAS als Ziel; sicherer Offline-Abbildweg noch offen

Die nicht erkannte 1-TB-SSD wurde vom Nutzer wieder abgezogen. Ihre Einrichtung
und Fehlersuche entfallen; keine weiteren Anschlussprüfungen, USB-Resets,
Treiberänderungen oder Formatierung für diese SSD. Die bereits geprüften Kopien
auf Mac und persönlichem NAS bleiben ausdrücklich erhalten.

Das NAS verfügt laut Abfrage über mehr als 5 TiB freien Platz und kann auch das
vollständige 256-GB-Systemabbild aufnehmen. Die USB-SSD ist keine Voraussetzung.

Der Benutzer `jetson` kann `/dev/mmcblk0` ohne erhöhte Rechte nicht roh lesen.
Die anschließende reine Zugriffsprüfung `sudo -n true` war jedoch erfolgreich:
nichtinteraktiver administrativer Zugriff ist vorhanden, kein Passwort wurde
verarbeitet und keine Rechteumgehung über Docker benötigt. Ob ein konkreter
administrativer Befehl erlaubt ist, ist für diesen Befehl gesondert zu prüfen.
Diese erfolgreiche Prüfung ist noch keine Freigabe für eine Größenänderung.

Für ein konsistentes vollständiges Systemabbild bleibt ein geeigneter Offline-
Zugriff notwendig. Das laufende Root wird nicht unkoordiniert für einen langen
Netzwerktransfer eingefroren oder als garantiert konsistenter Live-Klon ausgegeben.

Sicherer Ablauf vor invasiven Jetson-Arbeiten; kein Startblocker für Simulation:

1. Persönliches NAS als externes Ziel verwenden, Erreichbarkeit und freien Platz
   vor dem nächsten Transfer erneut prüfen. Bestehende NAS-Daten nicht überschreiben.
2. Die bereits gesicherten Rettungsarchive behalten; Original-Imagearchive nach
   sicherer Lösung des Remote-Temporärspeicherproblems ergänzen.
3. Geeigneten Offline-Zugriff auf die Jetson-Bootkarte organisieren: kontrolliertes
   Herunterfahren nach Abstimmung, Kartenleser an einem anderen Rechner, korrektes
   Laufwerk identifizieren und mindestens 255.869.321.216 Bytes plus Reserve sichern.
   Administrativen Zugriff unmittelbar im Nutzerterminal verwenden.
4. Vollständiges Image und QSPI-/Boot-Wiederherstellungsweg prüfen und sichern.
5. Erst danach ext4-Vergrößerung aus Abschnitt 14.6 vornehmen; Partitionsgrenzen,
   NVIDIA-Bootpartitionen und OS unverändert lassen. Kein `fsck` auf aktivem Root.
6. Nachprüfung von Dateisystemkapazität, Bootfähigkeit und Vision; Originalimages
   sichern, danach SSH-Key hinterlegen und Roboterstand weiter lokalisieren.

**Status:** Priorisierte Quellen-/Container-FS-Sicherung und unabhängige zweite
Kopie abgeschlossen. Originalimage-Save durch Remote-Platzproblem blockiert;
vollständiges konsistentes Systemabbild wartet auf einen abgestimmten Offline-Weg.
Speichererweiterung bleibt bis zur Sicherheitsprüfung ausgesetzt. Administrativer
Zugriff ist grundsätzlich bestätigt; USB-SSD als Voraussetzung gestrichen.
Kein vollständiger Abschluss dieser administrativen Schritte behauptet.

## 16. Zusatzwissen aus ROS_experiments

Lesende Analyse vom 04.10.2026. Wurzel:
`/Users/toto/Projects/ROS_experiments`. Etwa 200 MiB belegter Speicher
(`du -sk`: 204316 KiB), sieben Git-Repositories plus Workspaces, Buildartefakte
und Kalibrierungsarchive. Keine Builds, Installation, Kamera-/Portöffnung oder
Firmwareuploads durchgeführt. Die Quellen bleiben unverändert.

### 16.1 Herkunft und lokale Arbeitsstände

Pfade in dieser Tabelle sind relativ zur oben genannten Wurzel. Commits geben
die Git-Basis an; sie enthalten ausdrücklich nicht alle lokalen Änderungen.

| Modul / Pfad | Git-Basis | Lokaler Zustand / Rolle |
| --- | --- | --- |
| `esp32bot_ws/src/linorobot2_hardware_hippo_esp32_fix` | `humble`, `8baaa4a16d6364faf74177c62ab69ad08aa99720`, 09.10.2023 | `firmware/src/firmware.ino` verändert; relevante Motor-/micro-ROS-Quelle |
| `yolo_ros` | `main`, `acc12c7094eb5055eab66077a7d03c4f87c9c7c0`, 10.04.2025 | ROS-YOLO-Pakete 4.2.0; nur unversionierte macOS-Dateien sichtbar |
| `opencv_cam_ws/src/opencv_cam` | `master`, `27eee1ce10f2481f71122b493558438a189ba649`, 03.11.2023 | sauberer Kamera-Quellstand; Parametrisierung außerhalb des Repositories |
| `opencv_cam_ws/src/ros2_shared` | `master`, `662206467488566aea068541fbd48648c7172b89`, 07.12.2022 | saubere zusätzliche Buildabhängigkeit für den Kameratreiber |
| `camera_2d_lidar_calibration` | `master`, `12c2440b8caa387c75becac922e769c1866d0f63`, 16.04.2019 | umfangreiche lokale ROS-1→ROS-2-Portierung, neue Python-/Launch-/Setup-Dateien unversioniert |
| `LiDAR_camera_calibration_tool` | `main`, `a95ec40bc53870ce7dd38b05fb0aec13e5d6842d`, 15.05.2022 | `params.py` und `requirements.txt` verändert; separates PointCloud-/Bild-Overlaywerkzeug |
| `articubot_ws/src/articubot_one` | detached HEAD, `420dc2b4d1a14274d38e3e3c76f7aa0ee7842427`, 28.05.2022 | separates Classic-Beispiel; unversionierte RViz-Konfiguration |

Die Experimente sind zusätzliche Quellen, kein gemeinsamer einsatzfertiger
Workspace und kein Nachweis der zuletzt auf Jetson/ESP32 verwendeten Version.
Insbesondere ROS-Gazebo-Buildartefakte allein sagen nichts über Classic versus
modernes Gazebo aus: `ros_gz` gehört zum modernen Gazebo, `gazebo_ros` zu Classic.

### 16.2 ESP32: konkrete myrobot-Variante

Basis ist `esp32bot_ws/src/linorobot2_hardware_hippo_esp32_fix`:

- `firmware/platformio.ini:34–55`: Environment `myrobot`, `espressif32`,
   `esp32dev`, Arduino, micro-ROS `humble`, Transport `wifi`,
   `-D USE_MYROBOT_CONFIG`. `config/config.h:33` wählt den Custom-Header.
- `config/custom/myrobot_config.h:21–55`: `BAUDRATE=115200`,
   `DIFFERENTIAL_DRIVE`, generischer Zweieingang-Motortreiber, MPU6050,
   I2C SDA 21 / SCL 22, PID P/I/D = 0,6 / 0,8 / 0,5,
   Motor-Maximum 160 RPM, Begrenzungsfaktor 0,8, Motor-/Maximalversorgung 6 V,
   Raddurchmesser 0,0675 m, Radabstand 0,126 m, PWM 8 Bit / 8000 Hz.
- Encoderwerte `COUNTS_PER_REV1/2=975`, `COUNTS_PER_REV3/4=240`.
   Interpretation und tatsächliche gezählte Impulse messen; Übersetzungsverhältnis
   oder Quadraturmodus nicht aus diesen Zahlen allein ableiten.
- `myrobot_config.h:98–100`: `USE_WIFI_TRANSPORT`, Agent `192.168.1.100`,
   UDP-Port 8888. Die erfasste Pi-Adresse liegt in `192.168.0.x`; der konfigurierte
   Agent kann ohne bewusstes Routing/Adressanpassung nicht als passend gelten.
- `monitor_speed=115200` ist ein serieller Monitorwert, kein Nachweis, dass
   micro-ROS seriell läuft. Der Compile-Transport ist aktuell WiFi.
- WLAN-/OTA-Konfiguration geschützt behandeln. Zugangswerte wurden nicht in
   den Plan übernommen; keine privaten Custom-Header öffentlich einchecken.

Aus dem Custom-Header bestätigte 2WD-Pins:

| Motor | PWM | IN A / IN B | Encoder A / B | Invertierung Motor / Encoder |
| --- | --- | --- | --- | --- |
| 1 | 12 | 14 / 27 | 5 / 17 | true / false |
| 2 | 26 | 25 / 33 | 19 / 18 | true / false |

Zusätzliche Motor-3/4-Platzhalter nicht als belegte 4WD-Hardware übernehmen.
Die Firmware instanziiert vier Controller/Encoder auch bei DifferentialDrive;
der Header verwendet für zusätzliche Motoren unter anderem GPIO 34/35, die auf
klassischen ESP32 keine normalen Ausgangspins sind, sowie mehrfach belegte Pins.
Vor Build/Upload Bibliotheksverhalten, deaktivierte Kanäle, Boot-Strapping-Pins
und echte Verdrahtung prüfen; keine Umverdrahtung aus einer Tabelle allein ableiten.

Die gefundenen `.pio`-Artefakte umfassen Bootloader und Partitionstabelle, aber
in der geprüften Firmware-Buildstruktur wurde kein vollständiges `firmware.bin`
oder `firmware.elf` gefunden. **Kein nachgewiesenes Image der laufenden ESP32-
Firmware.** Buildcache und Quellstand erhalten, vor Neu-Flash Rückfallweg klären.
`test_motors`, `test_sensors` und `calibration` sind vorhandene separate Projekte;
ihre Ausführung kann Hardware ansteuern und ist nicht Teil dieser Analyse.

### 16.3 Firmware-Vertrag, Timing und Sicherheit

Belege in `firmware/src/firmware.ino`:

- `setup():162,194–197`: Serial beginnt mit `BAUDRATE`; Transportauswahl
   zwischen Netzwerk und `set_microros_serial_transports(Serial)` ist bedingt.
- `createEntities():291–329`: Publikationen `odom/unfiltered` (`Odometry`),
   `imu/data` (`Imu`), `imu/mag` (`MagneticField`), `battery` (`BatteryState`),
   `ultrasound` (`Range`); Subscription `cmd_vel` (`geometry_msgs/Twist`).
   Ohne `TOPIC_PREFIX` gelten diese relativen Namen im Root-Namespace.
- Timer: Motor-/Odometrieloop 20 ms / 50 Hz, Batterie 2000 ms,
   Ultraschall 100 ms. Daten werden mit Agent-synchronisierten Zeitstempeln publiziert.
- `firmware/lib/odometry/odometry.cpp:22–23`: Frame `odom`, Child
   `base_footprint`; passt konzeptionell zur Pi-EKF-Kette.
- `moveBase():409ff`: nach 200 ms ohne Fahrkommando Geschwindigkeiten auf null;
   `fullStop()` bremst Controller. `AGENT_DISCONNECTED:223ff` ruft ebenfalls
   `fullStop()` vor Entity-Abbau auf. Das sind Codepfade, keine gemessene garantierte
   Sicherheitslatenz; Agent-Pings, Executor und blockierende Sensor-/Netzwerkaufrufe
   beeinflussen den tatsächlichen Verlauf.
- `publishData():465ff` berechnet eine IMU-Orientierung aus Beschleunigung und
   Magnetometer. Sensorwahl/Kalibrierung, Covariances und EKF-Orientierungsfusion
   prüfen; nicht automatisch auf absoluten Yaw als zuverlässig umstellen.
- Fehler-/Init-Schleifen und optionalen Hardware-Watchdog separat abnehmen.
   Lokale Firmwareänderung nicht ungeprüft in die aktive Hardware übertragen.

**Konsequenz:** Der Pi-Agent mit Serial/921600 ist nicht automatisch kompatibel.
Zuerst vorhandenen ESP32-Transport und Firmwarestand bestimmen. Für WiFi braucht
es einen Humble-Agent `udp4` auf dem abgestimmten Port; für seriell müssen Compile-
Transport, Agent-Baudrate und reale Device-ID zusammenpassen. Nur einen Agent pro
Verbindung starten. WLAN-UDP-Agenttransport ist nicht mit ROS-DDS-Domainwahl gleichzusetzen.

### 16.4 Kamera, YOLO und getrennte Jetson-Vision

`opencv_cam_ws/param-file.yaml` stellt Kameraindex 0, 640×480, Frame `my_camera`
und relativen Pfad `calibrationdata/ost.yaml` ein. In
`src/opencv_cam/src/opencv_cam_node.cpp` werden **relative** Topics `image_raw`
und `camera_info` publiziert; sie heißen nicht automatisch
`/camera/rgb/image_raw`. Namespace/Remapping und absoluten Kalibrierpfad festlegen.

`opencv_cam_ws/calibrationdata/ost.yaml` enthält jedoch 1920×1080-Intrinsics
(`fx≈1496,4`, `fy≈1498,7`). Der Treiber lädt CameraInfo, skaliert diese im
betrachteten Code aber nicht automatisch auf die Capturegröße. Kameramodus,
Crop/Resize und Intrinsics gemeinsam abnehmen; vorzugsweise im tatsächlichen
Betriebsmodus kalibrieren. Kein beliebiges Skalieren ohne Prüfung der Bildentstehung.

`yolo_ros` liefert `yolo_msgs`, `yolo_ros` und `yolo_bringup`. Version 4.2.0,
GPL-3-Paketmetadaten, Requirements: `numpy<2`, `opencv-python>=4.8.1.78`,
`ultralytics==8.3.91`, `typing-extensions>=4.4.0`, `lap>=0.5.12`.
Deployment-/Weitergaberegeln einschließlich Ultralytics-Abhängigkeiten gesondert prüfen.

`yolo_bringup/launch/yolo.launch.py` verwendet standardmäßig `cuda:0`,
`yolov8m.pt`, Bildtopic `/camera/rgb/image_raw`, 480×640 und Tracking aktiv.
Der YOLO-Knoten ist ein LifecycleNode und publiziert `DetectionArray` auf
`detections` (default Namespace `yolo`). QoS nicht aus einer Zahl erraten:
Launchwert 1 bedeutet Reliable, 2 Best Effort; der Node-default ohne Launch ist
Best Effort. Kamera-/Bridge-QoS und Lifecycle-Aktivierung ausdrücklich testen.
3D-Ergebnisse benötigen passende Tiefenbilder, CameraInfo und TF; 2D-Detektionen
allein sind keine metrischen Hindernisse für eine Nav2-Costmap.

Die gefundene ROS-YOLO-Quelle und Jetsons `video_detection` sind **zwei getrennte
Implementierungen**. Kein Nachweis, dass yolo_ros bereits auf dem Jetson läuft
oder mit dem Pi-`linorobot2_vision` identisch ist. Den eingefrorenen
CUDA-10.2-/TensorRT-/PyTorch-Stack nicht durch globale Pip-Updates ersetzen.
Aktuelle YOLO-ROS-/Torch-Pakete nicht als auf JetPack-4-kompatibler Native-Install
voraussetzen; gegebenenfalls nur den Datenvertrag beziehungsweise `yolo_msgs`
auf Pi-Seite verwenden und den bestehenden Jetson-Dienst adaptieren.

### 16.5 Kalibrierung: wertvolle Daten, unfertige Portierung

`camera_2d_lidar_calibration` enthält eine lokale ROS-2-Portierung eines älteren
ROS-1-Projekts. Git meldet unter anderem entfernte Original-CMake-/Launch-/Src-
Dateien und unversionierte Pythonpackage-/Setup-/ROS-2-Launch-Dateien. Nicht mit
`git clean` oder frischem Clone ersetzen.

Konkrete Prüfpunkte vor Nutzung:

- `package.xml` exportiert `ament_python`, nennt aber `ament_cmake` als
   Buildtool; Manifest/Setup und echte Python-/ROS-Abhängigkeiten konsolidieren.
- `setup.py` registriert nur `collect_camera_lidar_data`; Kalibrierung wird
   im Launch als `camera_lidar_calibration.py` erwartet, ist so nicht registriert.
- `camera_lidar_calibration.py` initialisiert Argumentpfade nur bei
   `len(sys.argv)==6`, während der Launch drei Dateipfade übergibt. Argument- und
   Einstiegspunktbehandlung prüfen, nicht als bereits funktionierend behandeln.
- `data/` ist aktuell leer; frühere `data.txt`/`calibration_result.txt` sind
   im Arbeitsbaum entfernt. Existenz in Git-Historie/Archiven separat prüfen;
   fehlende Arbeitsergebnisse nicht stillschweigend überschreiben.
- Der Datensammler verwendet RViz-Klickpunkte über `/move_base_simple/goal`
   und `image_raw`, schreibt Korrespondenzen in einen konfigurierten Datenpfad.
   GUI-/Eventloop, QoS, Frames und beschreibbaren Ausgabeort prüfen; Ergebnisse
   nicht in schreibgeschützte Paketinstallationen schreiben.
- `config/config.yaml` enthält einen anderen Intrinsic-Stand (`fx≈606,5`,
   `fy≈607,6`) als obige 1080p-Kamerakalibrierung. Nicht mischen, Kamera/Modus zuordnen.

`LiDAR_camera_calibration_tool/params.py` ist ein separates manuelles
PointCloud-/Kamera-Overlay mit `velodyne_points`, `front`, `front_PCD_overlay`.
Die Matrix hat ebenfalls `fx≈606,5`; Translation/FOV-Werte stammen teils aus
anderen Datensätzen und sind keine bestätigten Maße unseres Roboters.
Ein RPLIDAR-LaserScan ist nicht automatisch ein PointCloud2-Eingang:
Scan-Projektion und Frames oder das passende 2D-Werkzeug auswählen.

`calibrationdata.tar.gz` (~44 MB) enthält Bilder `left-*.png`, `ost.yaml` und
`ost.txt`; zusätzlich gibt es `camera_2d_lidar_calibration.zip` und `ost.txt`
an der Ordnerwurzel. Diese sind Mess-/Referenzdaten, nicht löschbare Buildcaches.
Kameraidentität, Auflösung und verwendete extrinsische Transformation noch zuordnen.

### 16.6 Articubot und Buildtrennung

`articubot_ws/src/articubot_one/launch/launch_sim.launch.py` startet
`gazebo_ros/gazebo.launch.py` und `gazebo_ros/spawn_entity.py`: **Classic**.
Dies ist ein unabhängiges Robotermodell/Beispiel, kein Ersatz für die angepasste
Linorobot-Geometrie. Alte Build-/Installordner und enthaltene `ros_gz`-Artefakte
nicht ungeprüft sourcen oder mit dem aktuellen Workspace vermischen.

PlatformIO-Firmware ist kein Colcon-ROS-Hostpaket. Kameratreiber benötigt auch
`ros2_shared`; YOLO-/Kalibrierungspakete nur gezielt in getrennte Humble-Overlays
übernehmen. Nicht pauschal den gesamten Experimentbaum als `src` hinzufügen:
mehrere verschachtelte Workspaces, alte Artefakte und unterschiedliche APIs.

### 16.7 Konsequenzen für den weiteren Plan und ausgeführte Sicherung

1. Quellenbasis nun aus fünf Bereichen rekonstruieren: Mac-Linorobot,
   Pi-`toto_dev`, Jetson-Vision, `ROS_experiments` inklusive Firmware und
   `JetsonNano` mit zusätzlichem Vision-/Systembuildwissen (Abschnitt 17).
2. Tatsächliches ESP32-Board/Flashstand zuerst zuordnen; Transport und Geometrie
    vergleichen. Keine Firmware-Neuinstallation nur wegen fehlender USB-Erkennung.
3. Agentkonfiguration zum vorhandenen Transport wählen. WiFi-Zieladresse/Port,
    Serial-Baudrate und Stopps gemeinsam abnehmen, bevor Motoren belastet werden.
4. Kamera-/Lidar-Kalibrierungen nach Gerät und Modus katalogisieren und Frames/
    Zeit/QoS abgleichen; erst danach Vision-Ergebnisse für Navigation verwenden.
5. Vorhandene Jetson-Vision erhalten; ROS-Adapter auf Pi bevorzugen, solange
    ein kompatibler Jetson-ROS-/Torch-Stack nicht nachgewiesen ist.
6. ROS-2-Kalibrierungsportierung vor produktiver Nutzung gezielt reparieren und
    testen. In dieser Wissensaufnahme wurden keine Quellcodefehler behoben.

Zusätzliche vollständige Ordnersicherung ausgeführt:

- Mac: `/Users/toto/Backups/linobot2/ros-experiments-20261004-180849/ROS_experiments.tar.gz`.
- NAS: `/Volumes/personal_folder/Backups/linobot2/ros-experiments-20261004-180849/ROS_experiments.tar.gz`.
- Archiv etwa 104 MB; tar-Lesbarkeit geprüft, beide Kopien bytegleich (`cmp`).
- SHA-256 beider Kopien:
   `c0641fb582e1343158eb3e7ac246506a0ed4a0d191372464800fb9e0f52432f6`.
- Enthält lokale/unversionierte Dateien, `.git`, vorhandene Buildartefakte,
   Messdaten und möglicherweise private WLAN-/Gerätekonfiguration. Zugriff
   beschränken; nicht öffentlich hochladen oder als Sourcearchive veröffentlichen.
- Keine Hardwarefunktion oder vollständige Firmware-Reproduzierbarkeit behauptet.
   Bisherige Jetson-Offline-/Speicherblocker bleiben davon getrennte offene Aufgaben.

## 17. Zusatzwissen aus JetsonNano

Lesende Analyse vom 04.10.2026. Wurzel:
`/Users/toto/Projects/JetsonNano`, `du -sk`: 3517124 KiB, etwa 3,35 GiB.
Enthält drei Git-Repositories und einen lokalen Datensatz. Deutlich mehr als
reine Quelltexte: virtuelle Umgebungen, Rootfs, Paketdateien und Trainingsgewichte
sind gesonderte Artefakte. Keine Builds, Installation, Inferenz, Flash-/Resize-
Skripte oder Änderungen am laufenden Jetson ausgeführt.

### 17.1 Repositories, Arbeitsstände und Bezug zum bestehenden Stack

Pfade relativ zur Wurzel:

| Modul | Bestätigter Git-Stand | Lokale Ergänzungen / Größe |
| --- | --- | --- |
| `ai-video-solution` | `segmentation_containerimages`, `b7320bfa74d45a88eb56eacde5c09c360b0ebffb`, 23.04.2026 | `runs/` unversioniert; gesamter Baum ca. 1,8 GiB, darunter Trainingsläufe ca. 400 MiB |
| `build_python` | `main`, `a0eb1c3acf3a343c701c2e914d35194e6dbd6179`, 28.02.2024 | unversioniert `make_apt_repository2.sh` und `python3.11/` mit ARM64-DEBs; ca. 91 MiB |
| `jetson-image` | `master`, `c4bd0251ca11e9d10362f63daf0cdd0bad616467`, 01.02.2025 | `l4t_packages.txt` verändert; gespeicherter `rootfs/` ca. 1,3 GiB |
| `datasets/construction-ppe` | kein eigenes Git-Repository gefunden | ca. 179 MiB, YOLO-Bilder und Labels |

Die gesicherte Jetson-Vision hat Commit
`439f0ef074df1df91812bc906106cead76f30396` vom 15.08.2025 plus eine lokale
`app.py`-Änderung. Dieser Commit ist in der lokalen `ai-video-solution`-Historie
vorhanden und als **Vorfahre** von `b7320bfa...` bestätigt. Es sind nicht einfach
zwei unverbundene Projekte oder identische Arbeitsstände.

Der Git-Diff Jetson-Basis→lokales HEAD umfasst 15 Dateien, darunter `app.py`,
`object_detector.py`, `detection_system.py`, `annotation_worker.py`, Frontend und
Tracking-/Renderingfunktionen. Die zusätzliche Jetson-Arbeitsbaumänderung muss
separat verglichen werden, weil sie nicht im alten Commit enthalten ist.
Neueres Commitdatum ist kein Nachweis, dass der Stand auf CUDA 10.2 oder dem
vorhandenen Container erfolgreich läuft. Aktive Referenz nicht per Git-Pull ersetzen.

### 17.2 Vision-API als Integrationsquelle

`ai-video-solution/video_detection` enthält Flask-/YOLO-Anwendung,
ZeroMQ-Kommunikation und Weboberfläche; außerdem sind ein C#-`Frontend` und
`mcp_server` vorhanden. Diese Zusatzoberflächen gehören nicht in die zeitkritische
Motorregelung. Kein Nachweis einer produktiven ROS-Anbindung in diesen Quellen.

Statisch bestätigte Schnittstellen in `video_detection/app.py`:

- `198–199`: Defaults `tcp://*:5555` für Bilder und `tcp://*:5556` für Ergebnisse.
  Request-/Reply-Protokoll und JSON-Schema vor Integration tatsächlich prüfen.
- `/api/status`, `/api/current_detections_light`,
  `/api/current_detections_unfiltered`, `/api/tracked_objects`,
  `/api/track_history` sowie bildbezogene `/api/detect` und `/v1/vision/detection`.
- Separate Steuerungen für YOLO-Tracking, Rendering, Annotation und Filter.
- `/api/current_detections` wird zweimal registriert (`509` und `1232`) mit
  unterschiedlichen Funktionsnamen. Gültige Antwortstruktur und tatsächlich
  erreichten Handler gezielt prüfen; doppelte URL-Regeln nicht als eindeutigen
  stabilen Datenvertrag voraussetzen. Hier nicht behoben oder ausgeführt.

**Plan:** bestehenden Jetson-Dienst erhalten und einen Humble-Adapter auf dem Pi
gegen den tatsächlich laufenden, versionierten Vertrag entwickeln. Nicht annehmen,
dass der ältere aktive Jetson alle neueren lokalen Endpunkte bereits besitzt.
Ergebnisalter, Kamerazeitstempel, Frame-ID, Bildgröße, Klassen, Konfidenz und
Koordinatensystem festlegen; bei veralteten Daten keine Fahrentscheidung treffen.
2D-Boxen oder ein „closest object“-API-Name allein belegen keine metrische Entfernung.

### 17.3 Python-/Dockerrezepte sind nicht der Nachweis des aktiven Images

`video_detection/pyproject.toml` verlangt Python `>=3.11`,
`ultralytics>=8.3.127`, Flask `>=3.1.0`, `numpy==1.23.5`, LAP, PyZMQ und
OpenCV-headless. `requirements.txt` ist anders: überwiegend unversioniert,
`ultralytics>=8.1.0`, ebenfalls NumPy 1.23.5. Die effektive Installation hängt
vom gewählten Weg ab. Vor einem reproduzierbaren Build einen konsistenten,
plattformverträglichen Lock-/Versionsstand wählen, nicht beide Listen blind mischen.

Vorhanden sind **mehrere echte Dockerfiles/Compose-Dateien** unter
`video_detection/Docker/`. Ein leerer `Docker/` an der Repositorywurzel bedeutet
nicht, dass dem Projekt seine Buildrezepte fehlen.

`Docker/docker-compose.jetson.yml` wählt `Docker/Dockerfile.jetson`, Image
`video-detection:jetson-nano`, NVIDIA-Runtime und Restart `unless-stopped`.
Der Dockerfile verwendet L4T-Base **r32.7.1**, während der erhaltene Host
**r32.6.1** meldet. Die Buildanweisungen enthalten einen NVIDIA-PyTorch-Wheel-
Versuch, CPU-Fallbacks und System-/Pip-OpenCV-Pfade. Ein erfolgreicher Build
würde deshalb nicht automatisch GPU-Inferenz oder die ursprüngliche
Python-/Torch-/CUDA-/NumPy-Kombination nachweisen. NumPy-/Python-Wheelverfügbarkeit
und ABI-Kompatibilität für den gewählten Pythonstand zusätzlich prüfen.

Der Dockerfile nennt HTTP-Port **8082**, ZeroMQ **5555/5556** und Status-Healthcheck.
Dies sind Quell-Defaults, keine jetzt neu bestätigte Live-Portkonfiguration.
Das vorhandene `run-docker.sh` ist ein anderer Startweg: Host-Netzwerk,
NVIDIA-Runtime, 8-GiB-SHM-Limit, Hardware-/Argus-Mounts und Docker-Socket; der
Projektmount heißt dort `/ros_ws`. Der gesicherte reale Container dagegen hat
Bridge-Netzwerk und den Host-Projektmount `/video` (Abschnitt 14.5).

**Folgerung:** Startscripte nicht als exakte Wiederherstellung des laufenden
Containers verwenden. Viele Hardwaremounts und insbesondere ein Docker-Socket
sind nicht für reine Ergebnisinferenz erforderlich; Bedarf und Berechtigungen
prüfen. Ein 8-GiB-SHM-Limit stellt dem Nano mit 4 GB keinen zusätzlichen RAM bereit.
Virtuelle Umgebungen vom Mac nicht als Jetson-ARM64-Deployment kopieren.

### 17.4 Pythonpakete als erhaltene Build-/Wiederherstellungsartefakte

`build_python/README.md` beschreibt DEB-Builds für Python 3.9/3.10/3.11 auf
**Ubuntu 18.04 Bionic / JetPack 4**. Kein nativ auf macOS ausführbarer
Linux-Paketbuild und keine zugesicherte Kompatibilität mit dem angepassten Focal-Host.
Eine neue Host-Pythonversion ist keine Voraussetzung für den vorhandenen
Visioncontainer; dessen tatsächliche Python-/Torch-Version ist separat zu erheben.

`build_python/python3.11/` enthält unter anderem ARM64-DEBs mit Version
`3.11.3-1+bionic2`, `venv`, `dev`, `minimal`, `full`, `stdlib` und TK-Pakete.
Diese unversionierten Artefakte erhalten. Nicht nur das Git-Repository neu klonen.

Die zusätzliche Datei `make_apt_repository2.sh` kopiert Pakete nach
`/opt/debs/python<version>`, schreibt einen lokalen `trusted=yes`-APT-Eintrag
und führt `apt update` aus. Sie erwartet außerdem einen konkreten Pfad unter
`$HOME/build_python`. Das sind Systemänderungen, keine reine Analyse.
Nicht ausführen; OS-Paketbasis und NVIDIA-/Python-Abhängigkeiten zuerst prüfen.

### 17.5 Imagebuilder als Herkunftsspur, nicht als aktuelle Reparaturanweisung

`jetson-image/rootfs/etc/os-release` und `usr/lib/os-release` melden
**Ubuntu 20.04.6**, wie der laufende Jetson. Unter den geprüften `rootfs/home`-
und `rootfs/opt`-Pfaden wurden keine ROS-/PlatformIO-Manifeste gefunden;
`/etc/nv_tegra_release` war in dieser Prüfung nicht vorhanden. Der gespeicherte
Rootfs-Baum ist daher kein bestätigtes Abbild des aktuellen NVIDIA-/Roboterstacks.

`Containerfile.rootfs.20_04` basiert auf Focal, ergänzt aber eine Bionic-
Paketquelle und ältere NVIDIA-Laufzeitabhängigkeiten. Das kann einen angepassten
Userland-Aufbau erklären, beweist aber nicht, dass genau dieser Commit die
vorhandene SD-Karte erzeugt hat. Default-Zugangskonfiguration ist im Builder
vorhanden; keine Zugangswerte im Plan veröffentlichen.

`Containerfile.image.l4t32` lädt den NVIDIA-BSP **32.7.5**, führt
`apply_binaries.sh` aus und installiert CUDA-Toolkit 10.2 ausdrücklich im
Containerrezept. Die lokal geänderte `l4t_packages.txt` enthält aktuell nur
kommentierte Zusatzpakete, darunter Container-/cuDNN-Pakete. CUDA wird im
betrachteten Rezept nicht erst durch Entkommentieren dieser Liste aktiviert.
Ein neu gebautes Image ist wegen BSP 32.7.5 **kein unveränderter Ersatz** für
den aktiven L4T-32.6.1-Stand. Boardvarianten und Containerfiles für neuere
L4T-Versionen sind ebenfalls keine Upgradefreigabe für den ursprünglichen Nano.

Besonders wichtig: `scripts/resizerootfs.sh` ermittelt Root über eine textuelle
`lsblk`-Suche, führt `sfdisk -f -N 1` zur Partitionsänderung und anschließend
`resize2fs` aus. `resizerootfs.service` startet als Oneshot vor
`local-fs-pre.target`; das Rootfs-Rezept aktiviert diesen Dienst.

**Nicht auf der laufenden Karte ausführen.** Root-Partition ist bereits groß
gemeldet, und NVIDIA-Bootpartitionen müssen erhalten bleiben. Dienststatus/
historische Fehlermeldungen können später lesend als Ursache untersucht werden;
kein Start/Restart des Dienstes und keine automatische Partitionserweiterung.
`scripts/flash-jetson-image.sh` schreibt mit `dd` auf das Zielgerät und ist
destruktiv. Es gehört ausschließlich in einen geprüften Wiederherstellungsablauf
auf einer eindeutig identifizierten Ersatzkarte, nicht in die Bestandsaufnahme.

### 17.6 PPE-Datensatz und tatsächlich vorhandene Trainingsresultate

`datasets/construction-ppe/data.yaml` beschreibt elf Klassen, unter anderem
Helmet/Gloves/Vest/Boots/Goggles/Person und fehlende Schutzkleidung.
Erfasste Bildzahlen: **1132 Training, 143 Validierung, 141 Test**.
Dataset-Metadaten nennen AGPL-3.0; Nutzung/Weitergabe von Bildern, Labels und
Modellen separat prüfen. Dies ist kein ROS- oder Kamera-Kalibrierungsdatensatz.

Unter `ai-video-solution/runs/detect/runs/detect/ppe_v1` und `ppe_v1-2` liegen
`weights/best.pt`, `last.pt`, Epoch-Checkpoints und `results.csv`; die gesamte
`runs/`-Struktur ist unversioniert. Daten-/Modellzuordnung und Trainingsparameter
bei späterer Reproduktion aus den Run-Metadaten prüfen; kein Training gestartet.
Die Artefakte sind nicht bloß austauschbare Caches.

Zusätzlich vorhanden: YOLO11n/YOLO11n-seg/YOLO12n/YOLO26s und ONNX-Dateien im
Visionbaum. Dateiname allein beweist weder CUDA-/TensorRT-Kompatibilität noch
die Zuordnung eines aktuell verwendeten Modells. PPE-Klassen nicht als identisch
mit COCO/Yolo-Standardklassen oder einem anderen Modell behandeln; Labelnamen
und Class-ID-Mapping in den Pi-Adaptervertrag aufnehmen.

### 17.7 Ergänzte Prioritäten und verifizierte Sicherung

1. Neuerer Mac-Visionstand als zusätzliche Referenz erhalten; aktive Jetson-
   Version inklusive lokaler Änderung bleibt der geprüfte Wiederherstellungsstand.
2. Vor ROS-Adapter oder Visionupdate Quell-Diff, API-Schema, Modell/Labels und
   tatsächliche Container-Python-/Torch-/CUDA-Version zusammen erfassen.
3. Pi-seitiger Humble-Adapter bleibt der konservative Weg; keine pauschale
   ROS-/Python-/NVIDIA-Neuinstallation auf Jetson aufgrund dieser Rezepte.
4. Imagebuilder und Python-DEBs als geschützte Notfall-/Reproduktionsressourcen
   katalogisieren. Rootfs-Baum ersetzt kein vollständiges Jetson-Systembackup.
5. `resizerootfs.service` bei späterer Ursachenanalyse nur lesen; Partitionstabelle
   nicht aufgrund eines gefundenen älteren Skripts ändern.

Der gesamte Ordner einschließlich Git-Historien, lokalen/unversionierten Dateien,
Rootfs, Python-DEBs, Modelle und Trainingsstände ist zusätzlich archiviert:

- Mac: `/Users/toto/Backups/linobot2/jetsonnano-sources-20261004-182313/JetsonNano.tar.gz`.
- NAS: `/Volumes/personal_folder/Backups/linobot2/jetsonnano-sources-20261004-182313/JetsonNano.tar.gz`.
- Archiv etwa **1,5 GB**, tar-Lesbarkeit geprüft, Kopien bytegleich (`cmp`).
- SHA-256 beider Kopien:
  `cadcc13a56dd97d6c736e78d181c1adeb97b7ef9b46cf2fcf096acfaacad4eb8`.
- Private Konfiguration, mögliche Zugangsdaten, Bilder und Modelle im Archiv
  nicht öffentlich teilen oder ungeprüft ins Git-Repository übertragen.

Diese Wissensaufnahme erweitert die Quellenbasis und Sicherung, führt aber
keine zuvor blockierte Offline-/Speicher-/Hardwaremaßnahme aus und behauptet
keinen getesteten Betrieb der neu gefundenen Rezepte.