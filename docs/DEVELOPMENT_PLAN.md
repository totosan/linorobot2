# Finaler Entwicklungsplan: Simulation zuerst, Hardware später

Stand: 08.10.2026 · **Verbindlicher Fahrplan für die nächsten Entwicklungsschritte.**

Dieser Plan ersetzt frühere Reihenfolgeempfehlungen. Die erfassten Systemdaten,
Quellenstände, Sicherungen und technischen Risiken bleiben im
  [Referenzkatalog](reference/MACOS_SETUP_PLAN.md) erhalten. Die Simulationsumgebung
und G1–G3 sind inzwischen laufzeitgeprüft; Details stehen im
[Implementierungsstatus](IMPLEMENTATION_STATUS.md). Keine Hardwareabnahme wird
damit vorausgesetzt.

## 1. Ziel und Leitprinzip

Wir entwickeln zuerst einen virtuellen, parametrierbaren 2WD-Roboter, prüfen
Basisfunktionen und Navigation und integrieren Vision als austauschbaren Dienst.
Erst danach übertragen wir die Software schrittweise auf die vorhandene Hardware.

Der Ablauf orientiert sich an etablierten ROS-Arbeitsweisen: URDF/Xacro als Modell,
Gazebo für Physik/Sensorik, RViz für Diagnose, definierte ROS-Schnittstellen,
Software-in-the-Loop-Tests, rosbag-Aufzeichnungen, automatisierte Regressionen und
anschließende Hardware-in-the-Loop- sowie reale Abnahme. Simulation reduziert
Risiken, ersetzt aber weder reale Kalibrierung noch eine Sicherheitsprüfung.

**Entwicklungsreihenfolge:**

1. Simulieren und entwickeln können.
2. Virtuelles Robotermodell aufbauen und anpassen.
3. Austauschbare Basisanbindung und Sicherheitslogik prüfen.
4. Odometrie, SLAM und Navigation virtuell abnehmen.
5. Vision zunächst mit Testdaten/Mock, danach echter Inferenz anbinden.
6. Pi-, ESP32- und Jetson-Anbindung schrittweise ergänzen.
7. Physische Kalibrierung, Feintuning und reale Abnahme.

Kein Firmwareflash, USB-Umbau, Jetson-OS-Upgrade oder Motor-PID-Tuning ist
Voraussetzung für die ersten fünf Schritte. Die bereits vorhandenen Sicherungen
bleiben erhalten; invasive Jetson-Arbeiten benötigen weiterhin einen gesicherten
Rückweg, blockieren aber nicht die unabhängige virtuelle Entwicklung.

## 2. Architektur: drei getrennte Verantwortungsbereiche

| Bereich | Verantwortung | Virtuelle Umsetzung | Späteres Hardwareziel |
| --- | --- | --- | --- |
| **Motor/Basis** | Radregelung, Encoder, rohe Odometrie, IMU, Kommandotimeout | Gazebo-Antrieb und Sensoren; gezielte Fehler-/Verzögerungsmodelle | ESP32 + Motor-Leistungstreiber; micro-ROS-Agent auf Pi |
| **Roboter** | Modell/TF, EKF, Befehlspriorisierung, SLAM, Nav2, Verhalten | unveränderter ROS-Humble-Kern auf Mac-Linux | Pi 4 mit 8 GB, Ubuntu 22.04/Humble |
| **Vision** | Bildaufnahme/Inferenz, Erkennungsdaten, Tracking | Testbilder, Aufzeichnungen, Mock-Dienst; später isolierte CPU-Inferenz | bestehender Jetson-Dienst, OS/CUDA/TensorRT eingefroren |

Der Pi ist später der Roboterrechner, der Jetson ein separater Visionrechner.
Gazebo und Entwicklungswerkzeuge bleiben auf dem Mac. In den frühen Phasen
laufen Roboterkern, Simulator und Mock lokal in derselben Linux-Umgebung;
verteiltes DDS und physische Geräte werden bewusst erst später eingeführt.

### 2.1 Abhängigkeitsregeln

- Motor/Basis kennt weder Nav2 noch YOLO, Flask, LLMs oder das Vision-Frontend.
- Roboterkern kennt einen **Basisvertrag**, nicht Gazebo-Pluginbibliotheken,
  USB-Portnummern oder ESP32-Pins. Solche Details gehören in Backends/Launchprofile.
- Roboterkern erhält normalisierte Visionergebnisse, keine Abhängigkeit auf eine
  konkrete Torch-, TensorRT- oder REST-Implementierung.
- Vision erhält Bilder/Anfragen und liefert Ergebnisse; kein direkter Zugriff
  auf den Motorbefehl. Kamerabesitz ist pro Betriebsprofil eindeutig.
- Austauschbarkeit zuerst an Paket-/Prozessgrenzen erreichen. Nicht für jedes
  Modul einen neuen Netzwerkdienst schaffen und keine unnötige Framework-Abstraktion.
- Visionausfall darf Basisnavigation nicht lahmlegen. Verhalten, das ausdrücklich
  Vision benötigt, muss bei fehlenden/veralteten Ergebnissen kontrolliert abbrechen.
- Die sicherheitsrelevante lokale Stoppfunktion bleibt auf der Motorseite.
  Physischer Not-Aus/Leistungstreiberabschaltung kommen als unabhängige Ebene hinzu.

### 2.2 Gemeinsame Modell- und Konfigurationsbasis

Ein versionierter Satz beschreibt Geometrie, Radabstand, Radgröße, Sensorposen,
Massen/Trägheiten, Kollisionen, Footprint und Limits. Simulator und Hardwareprofile
leiten ihre Werte daraus ab oder erhalten geprüfte Exporte derselben Parameter.
Firmware-Konstanten werden nicht automatisch ohne Review überschrieben.

Startmodell: **2WD**, einfacher Aufbau, zunächst Lidar und IMU. Die gefundene
`myrobot`-Firmware liefert Kandidaten: Raddurchmesser 67,5 mm, Spurweite 126 mm.
Diese gelten als dokumentierte Anfangsannahmen, nicht als verifizierte Maße des
aktuellen realen Roboters. Nicht gleichzeitig 4WD/Mecanum und alle Altmodelle umbauen.

## 3. Schnittstellen vor Implementierungsdetails

Vor der Implementierung eine kleine Schnittstellenspezifikation festschreiben:

| Vertrag | Vereinbarung |
| --- | --- |
| Fahrbefehl | Humble-kompatibel `geometry_msgs/msg/Twist`; Nav2/Teleop zunächst auf getrennten Eingängen, genau ein priorisierter finaler `/cmd_vel` |
| Rohe Basisrückmeldung | `/odom/unfiltered` als `nav_msgs/msg/Odometry`; `/imu/data` als `sensor_msgs/msg/Imu`; gleiche Units, Achsen, Frames und Zeitregeln für beide Backends |
| Zustände/Sensoren | `/joint_states`, `/scan`; Bilder/CameraInfo mit explizitem Namespace und optischem Frame, z. B. bestehende `/camera/color/*`-Namen |
| Gefilterte Odometrie | EKF → `/odom`; genau ein Publisher für `odom → base_footprint` |
| TF-Verantwortung | SLAM/AMCL: `map → odom`; EKF: `odom → base_footprint`; RobotStatePublisher: Robotermodell-/Sensorframes; keine konkurrierenden Transformquellen |
| Vision | versioniertes Ergebnis mit Aufnahmezeit, Frame-ID, Bildgröße, Modell-/Labelschema, Klasse, Konfidenz, Tracking-ID und Box; Status/Fehler separat |
| Sensor-QoS | je Topic dokumentieren und Sender/Empfänger testen; Reliable/Best Effort nicht aus impliziten Defaults übernehmen |
| Zeit | Simulation: `/clock` und `use_sim_time=true`; Realität: synchronisierte Systemzeit und `false`; externe Visionzeiten nicht ungeprüft mit Simulationszeit vergleichen |

Visionnachrichten können auf ROS-Standardtypen wie `vision_msgs` abgebildet werden;
`yolo_msgs` ist eine vorhandene Alternative, aber keine Pflichtabhängigkeit des
Roboterkerns. Einen Typ anhand benötigter Felder wählen und Konvertierung kapseln.
Ein HTTP-/ZeroMQ-Adapter normalisiert die tatsächlich eingesetzte Dienstversion.

2D-Boxen liefern **keine metrischen Hindernispositionen**. Projektion in den
Roboter-/Kartenframe setzt zugeordnete Intrinsics/Extrinsics, Tiefeninformation
beziehungsweise eine geprüfte geometrische Annahme voraus. Erste Nav2-Hindernis-
vermeidung arbeitet ausschließlich mit der simulierten Lidar-Sensorik.

Geplante Profile: `sim`, `replay`, `hardware`; Visionmodus `off`, `mock`, `service`.
Pro Start wird genau ein Basisbackend und eine geeignete Quelle pro Sensor gewählt.
Die Top-Level-Launches für Fortress-Simulation und Hardware-Bringup deklarieren
`ros_profile` und `ros_domain_id`; ein gemeinsamer Validator weist falsche
Profil-/Domainpaare und widersprechende geerbte `ROS_DOMAIN_ID`-Werte vor dem
Start von Nodes fail-closed ab. Implementiert sind `sim` → **42** und
`hardware` → **44**. `replay` → **43** bleibt reserviert, bis ein Replay-ROS-
Launch existiert. Diese Startguards sind keine Zugangskontrolle oder
Motorfreigabe. Vor HIL fremde ROS-Anwendungen und Pi-Konfiguration auf Kollisionen
prüfen; keine globale Shelländerung. HIL-Daten nur durch gezielt konfigurierte
Vermittlung übertragen. Domains sind kein Ersatz für Not-Aus.

## 4. Festgelegter Entwicklungsstack und Installationsstrategie

### Primärweg

- Mac M4 Max als Workstation; **Linux ARM64 auf dem Mac** als Ausführungsumgebung.
- Ubuntu **22.04**, ROS **Humble**, Gazebo **Fortress**, passende Humble-`ros_gz`-
  Pakete, RViz, `robot_localization`, SLAM Toolbox und Nav2.
- Plattformwahl hält ROS versionsgleich zum Pi. Der Jetson muss dafür weder ROS
  installieren noch sein angepasstes Ubuntu-/NVIDIA-System aktualisieren.
- Fortress ist der regulär gepaarte moderne Simulator für Humble. Bestehende
  Classic-Launches/Plugins müssen in einem separaten Arbeitsstand migriert werden.
  Umbenennen von `gazebo` in `gz` reicht nicht; Fortress verwendet häufig `ign gazebo`.

### Umgebungsentscheidung in Phase 1

Zuerst die bereits vorhandene OrbStack-/Containerumgebung prüfen. Ein ARM64-
Container mit virtuellem Display und RViz/Gazebo-Oberfläche ist der erste Kandidat.
Falls GUI/Rendering oder Sensoren nicht zuverlässig arbeiten, auf eine Ubuntu-
ARM64-Desktop-VM mit überprüfter Grafikunterstützung wechseln. Apple-GPU-Passthrough
nicht voraussetzen; Software-Rendering und reduzierte Sensorraten sind Testoptionen.

Diese Prüfung ist zeitlich begrenzt, Richtwert ein Arbeitstag, kein zugesicherter
Installationsaufwand. Vor größeren ROS-Quellbuilds die Plattformentscheidung treffen.
Natives macOS/Conda/Homebrew ist kein paralleler Pflichtpfad für diesen Fahrplan.

**Fallback:** isolierte Humble/Classic-11-Umgebung nur für einen schnellen
Legacy-Vergleich oder wenn Fortress-Migration zunächst einen konkreten Blocker
hat. Nicht der dauerhafte Entwicklungsstandard: Classic ist bereits EOL.
Diese Vergleichsoption separat und zeitlich begrenzt halten; keine offene
Legacy-Portierung beginnen, um ein Renderingproblem der Mac-Linux-Plattform zu lösen.
Gazebo Harmonic wird nicht allein wegen seiner bestehenden Homebrew-Installation
mit Humble vermischt. Keine gleichzeitige Jazzy-/Harmonic-Migration.

Humble/Fortress laufen bis Mai 2027 aus. Vor diesem Termin einen separaten
Lebenszyklusreview einplanen. Eine spätere ROS-/Simulator-Migration kann weiterhin
vom eingefrorenen Jetson-Inferenzdienst entkoppelt bleiben.

### Reproduzierbarkeit

Neuer Arbeitsstand/Workspace statt Überschreiben der Originalquellen; saubere
Linux-Build-/Install-/Log-Verzeichnisse. Gepinnte Repo-SHAs, Container-Digests,
Versions-/Dependency-Liste, dokumentierte Aktivierung und Startprofile.
Zunächst nur Simulationsabhängigkeiten installieren; Hardwareinstaller,
Jetson-Imagebuilder und ESP32-Testfirmware nicht ausführen.

Alle sieben Projektpakete erhalten und später frisch bauen, aber den kritischen
Pfad nicht von optionalen Hardwarelaufzeitabhängigkeiten abhängig machen.
`linorobot2_bringup` ist kein notwendiger Simulationsstart; fehlende micro-ROS-
Hardwareabhängigkeiten bewusst abgrenzen statt funktionierende Nodes vorzutäuschen.
Keine `.env` wird automatisch von ROS geladen; Export/Aktivierung explizit machen.

## 5. Entwicklungsphasen mit Abnahmekriterien

Jede Phase endet mit einer nachvollziehbaren Abnahme. Anschließend Erweiterung,
nicht alle Probleme gleichzeitig bearbeiten. Nur Messdaten rechtfertigen weitere
Optimierung oder einen Technologiewechsel.

### Phase 0 – Referenzen ordnen, keine Hardware verändern

**Arbeit:** vorhandene geprüfte Sicherungen referenzieren; Mac/Pi/ESP32/Jetson-
Quellen vergleichen und einen separaten Entwicklungsstand auswählen. Herkunfts-
manifest für übernommene Dateien und lokale Änderungen erstellen. Schnittstellen
und 2WD-Anfangsannahmen aus Abschnitten 2–3 festschreiben.

Vorhandene gesicherte Visionantworten/Quellschemas früh als Kandidaten für
Testfixtures katalogisieren. Ein erreichbarer Jetson darf optional lesend Status/
Schema liefern, ist aber keine Voraussetzung. Keine kostspieligen Inferenzjobs
oder aktiven Kamerazugriffe nur für die Planvorbereitung auslösen.

**Ergebnis:** ausgewählte Quellbasis und Anfangsmodell, Originale unverändert.
**Gate G0:** Quellen-/Backupwege bekannt, keine blinde Synchronisation oder Löschung.
Nicht auf die vollständige Jetson-Speicherreparatur warten.

### Phase 1 – Entwicklungs- und Simulationsfähigkeit herstellen

**Arbeit:** Linux-ARM64-Umgebung, Humble-Buildtools, RViz und Fortress einrichten.
Erst Talker/Listener und Standalone-Welt, dann Render-Sensoren und einfache
ROS/Gazebo-Brücke für `/clock` und einen Test-LaserScan prüfen. Bilder separat
als Grafik-/Sensorprobe, noch keine YOLO-/Kamerakalibrierung.

**Ergebnis:** versionierter Start-/Buildablauf für die Umgebung.
**Gate G1:** zwei saubere Starts funktionieren; Welt läuft; RViz und ROS-Brücke
arbeiten; Server/GUI bleiben mindestens 15 Minuten ohne Absturz nutzbar.
Render-Lidar liefert erwartete gültige Entfernungen zur Testgeometrie; eine
separate Testkamera liefert veränderliche, nicht ausschließlich leere Bilder
mit passender Größe/CameraInfo. QoS-Zustellung bis ROS prüfen. Ein geöffnetes
Fenster oder nur vorhandene Topicnamen reichen nicht.
Bei Nichterfüllung die festgelegte VM-/Renderingalternative prüfen und das Gate
erneut abnehmen; Renderfehler nicht mit Nav2-Parametern kaschieren.

**Erster Umsetzungsauftrag:** ausschließlich Phase 0/1, keine Pi-/Jetson-Updates,
keine Firmwareuploads und kein realer Motorstart.

### Phase 2 – Virtuelles Robotermodell und Basisbackend

**Arbeit:** vorhandene 2WD-Xacros gezielt übernehmen, Geometrie/Radachsen,
positive Trägheiten, Kollisionen und Sensorframes prüfen. Fortress-World-/Antriebs-/
Jointstate-/IMU-/Sensor-Systeme und Spawn integrieren. Classic-Ressourcenpfade und
Materialien anpassen; passende `ros_gz`-Bridge konfigurieren.

Einfache Testwelt mit Boden, Wand und Hindernissen; keine großen Map2Gazebo-
Meshkonvertierungen als Buildvoraussetzung. Simulationsadapter publiziert den
Basisvertrag, nicht simulatorabhängige Topics an die Anwendung weiterreichen.

**Ergebnis:** virtuelles Robotermodell mit austauschbarer Basisanbindung.
**Gate G2:** Roboter spawnt ohne Plugin-/Modellfehler, fällt nicht durch den Boden,
fährt per Teleop vorwärts/rückwärts und dreht korrekt; Räder/TF/Sensoren plausibel.
**Abnahme am 06.10.2026:** begrenzter Domain-42-SIL-Test für JointStates,
Vorwärts/Rückwärts, beide Drehungen, Ground Truth, beide Odometriepfade, TF,
Stopp und Watchdog bestanden. Messwerte und Grenzen stehen in
[IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md); physische Maße bleiben offen.

### Phase 3 – Roboterkern, Odometrie und Kommando-Sicherheit

**Arbeit:** EKF, TF-Verantwortung, Befehlspriorisierung, Limits und Watchdog-
Semantik prüfen. Geradeausfahrt, Drehung und Fahrt im Quadrat aufzeichnen.
Ungefilterte Odometrie und EKF gegen Gazebo-Ground-Truth vergleichen; Ground-Truth
nur zur Bewertung, nicht als versteckte Eingabe für die spätere Hardwarelogik.

Fehler injizieren: Fahrbefehle aussetzen, Sensoren ausfallen lassen, Simulation
pausieren/neustarten, Nachrichten verzögern. Timeouts müssen im jeweils festgelegten
Zeitmodell funktionieren. Keine ungeprüften alten Kommandos nach Neustart.

**Ergebnis:** Basisvertrag mit automatisierten Tests und rosbag-Testaufnahmen.
**Gate G3:** genau eine Odom-TF-Quelle; Richtungen/Units korrekt; Stopps und
Recovery nachvollziehbar. Anfangstoleranzen und gemessene Fehler protokollieren,
nicht aus idealer Simulation auf reale Motor-/Bremsleistung schließen.

**G3 im Simulator am 06.10.2026 bestanden:** Quadratfahrt, EKF/TF,
Nullbefehl/Recovery und Pause/Resume einschließlich wall-clock watchdog,
eingefrorener `/clock`-Prüfung und stale-command-Kontrolle. Zusätzlich wurden
Fortress vollständig neu gestartet und ein begrenzter ROS-Ausgabetopic-Ausfall
für `/camera/image_raw` mit anschließender Wiederkehr geprüft; andere Sensoren,
Clock, Odom und TF liefen weiter. Das ist kein Nachweis für physisches Abschalten
des Gazebo-Sensors oder eine automatische Fahrstopp-Policy. Der rohe Wheel-Yaw-
Bias bleibt ungelöst und sichtbar dokumentiert. Messwerte und Abgrenzungen stehen
in [IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md).

### Phase 4 – SLAM und Navigation ohne Vision

**Aktueller Status am 08.10.2026: G4 im SIL bestanden.** Runs 42/43 belegen die
vier sensor-derived Szenarien nominal; Runs 44/45 bestanden dieselbe Matrix mit
dem fixierten Noise-/Slip-Profil `g4-noise-slip-v1`, festem Seed und validierten
rosbag-Artefakten. Clearance-, Zielabweichungs-, Geschwindigkeits- und Stoplimits
blieben unverändert. G4 ist damit abgenommen, ausschließlich für das versionierte
Fortress-SIL-Profil; dies ist keine Hardwareübertragbarkeits- oder physische
Sicherheitsabnahme. Die detaillierten Belege stehen in
[`IMPLEMENTATION_STATUS.md`](IMPLEMENTATION_STATUS.md), die noch offenen
Modultrennungs-/G5-Arbeiten in [`OPEN_TASKS.md`](OPEN_TASKS.md). Runs 25/26 und
35–41 bleiben als historische Fixture-/Fehler-/Architekturversuche archiviert;
keine Wiederholung ohne neuen Zweck. Keine Hardware-/Inferenzfreigabe.

**Historische Fixture-Baseline Run 25:** Mit gepinntem TF2-Humble-Fix bestehen Mapping,
Kartenwiederverwendung, Readiness, freie Fahrt und Umfahrung. Zielabweichungen
0,05928 m / 0,03558 m, ausgeführte Clearance mindestens 0,21999 m. Die früheren
TF-/Costmap-Fehler treten im geprüften Lauf nicht auf. Das ist ein Fixture-PASS,
kein sensor-only SLAM-/Hardwaretransfernachweis. Run 26 ist kein PASS: der Prüfer
cancelte bei 0,251 m gegen 0,30 m erlaubte Standard-Recovery-Auslenkung. Dies
war kein G4-PASS; die spätere vollständige SIL-Abnahme ist oben mit Runs 44/45
dokumentiert.

**Historischer Implementierungsstand bis Run 41:** `prepare_phase4_map.py --mode sensor-seeded`
normalisiert Map-Saver-Unknown und füllt Ränder mit Unknown. Als einzige
Navigationseingabe-Ergänzung dürfen unbekannte Rasterzellen unter dem
konfigurierten initialen Roboter-Footprint frei werden: der Lidar kann den
eigenen Körper nicht beobachten. Quelle ist Spawn-/AMCL-Initialpose, nie eine
Ground-Truth-Trajektorie; belegte Footprintzellen führen fail-closed zum Fehler.
Keine unbekannten Fahrkorridore werden freigeschaltet. Run 29 bestätigte sensor-
derived Mapping/Map-Preparation (`105×80`, 3.768 bekannte/171 belegte Zellen,
3.144 neue Zellen, minimale Clearance `0,302 m`), scheiterte aber vor Nav2-Zielen
an einem Doppelstart zweier Fortress-Server. Run 30 bestand Map-Reuse-Readiness,
doch der Goal-Preflight stoppte vor jeder Action an der unbekannten Startzelle
`(0,025; 0)`, also unter dem Footprint. Run 31 bestand anschließend Mapping,
sensor-seeded Map-Preparation, Map-Reuse/Nav2-Readiness und freie Fahrt/Umfahrung;
das ist ein Zwei-Szenarien-Teilnachweis, kein vollständiges G4. Der konfigurierte
Start-Footprint-Seed ist im Quellstand. Run 32 ist dauerhaft archiviert und
bestätigt den bounded seed (11 Zellen; `occupied_cells_removed=0`, keine
Ground-Truth-Karteingabe), Map-Reuse/Nav2-Readiness und zwei Goals: freie Fahrt
`0,0713 m`, Umfahrung `0,0856 m`; tatsächliche Umfahrungs-Clearance `0,1847 m`,
finale Nullbefehle vorhanden. Run 33/34 bestehen den unreachable-Fall: angenommenes
Goal in Unknown, zielbezogener Plannerfehler, terminal `ABORTED`, null Nichtnull-
`/cmd_vel`, sieben/acht Stop-Samples ohne Drift. Ein Nullbefehl wurde mangels
vorheriger Bewegungskommandos nicht publiziert. Die separate G4-Welt enthält nun
einen statisch validierten Gate-Kandidaten: `0,84 m` Öffnung bei `x=4,0 m`,
`y=−1,52…−0,68 m`; Map- und Goal-Prüfer verwenden gemeinsame Geometriekonstanten,
und Plan/ausgeführte Trajektorie müssen die Öffnung genau einmal queren. Run 35
bestand sensor-derived Mapping (11.073 neue bekannte Zellen; 5-m-Gate-Abdeckung),
Map-Preparation und Nav2-Readiness. Nav2 meldete für das erste freie Goal
`SUCCEEDED`, Ground-Truth-Abstand `0,1064 m`; der damalige Verifier fand keinen
Nullbefehl nach dem Terminalstatus und stufte den Lauf als **FAIL** ein, bevor
Umfahrung oder Passage liefen. Run 36 stoppte bei einem nach Action-Ende
empfangenen Nichtnull-Command, ohne ausreichende Zeitreihenpersistenz. Run 37
bestand die drei Actions mit Abweichungen `0,0821 / 0,0820 / 0,0296 m`, finalen
Nullbefehlen und Stillstand; das G4-Gesamtergebnis bleibt **FAIL**, weil der
ausgewertete Passageplan eine Unknown-Zelle der sensor-derived Karte quert.
Nullbedingung ist finaler Nullbefehl nach letzter Bewegung plus mindestens drei
gemessene Ground-Truth-Stillstandssamples nach Terminalstatus (Drift `≤0,02 m`);
die vollständige Command-Sequenz berücksichtigt auch nach Action-Ende empfangene
Befehle. Die Abnahmeprobe liest jetzt bei jeder Goalplanung zusätzlich den
Global-Costmap-Snapshot des Nav2-Planers samt Publisher, Zeitstempel und frischem
LaserScan. Ein unbekanntes statisches SLAM-Feld bleibt unverändert und darf nur
dann als navigierbar gelten, wenn derselbe Plan-Snapshot es sensorbasiert als frei
ausweist; statisch belegte Zellen bleiben fail-closed. In der publizierten
Nav2-`OccupancyGrid` bedeuten Werte 50–98 Inflationskosten, 99 kodiert den
internen `INSCRIBED_INFLATED_OBSTACLE=253`, 100 ist lethal; SmacPlanner2D im
Radiusmodus verwirft bereits intern ab 253. Geometrische Clearance bleibt
separate strenge Abnahme. Planpunkte, Unknown-Zellen
und Passageposen werden auch bei FAIL gesichert. Der Harness zeichnet jetzt
`/clock`, TF, Scan, Map, Odom, cmd_vel, Plan und Costmaps pro Lauf als verwaltetes
rosbag2 auf und verlangt beim Gate-PASS gültige Metadaten plus Nachrichten. Die
Aufzeichnung ist in Runs 38–41 runtime-validiert. Die live-Costmap-Pfadprüfung
hat in Run 40 drei plan-time Unknown-Samples fail-closed abgewiesen; unabhängig
davon unterschritt dort die GT-Trajektorie die Clearance. 75 ROS-freie
Readiness-/Phase-4-Tests bestehen. Die minimale Clearance bleibt
`0,15 m`; der aperture-bounded Inflation-Costfield-Radius ist runtime-erprobt:
`floor((0,84 m/2)/0,05 m) × 0,05 m = 0,40 m`, global/lokal einheitlich; `0,45 m`
hatte die Gate-Halbbreite überdeckt und verursachte Run-39 Passage-Timeout.
Planungsradius `0,395 m` entspricht `0,22 m` SIL-Footprint + `0,15 m` Clearance +
`0,025 m` Rasterhalbzelle und ist ein konservatives Plannerprofil, nicht die
behauptete physische Robotermessung. Nav2-Paketbuild und Quell-/Installationshash
bestanden für das Run-40-Profil; Run 40 hat keine clearance-zertifizierte
Gatepassage geliefert (`0,13568 m` GT). Der einmalige Run-41-Versuch mit Humble
`SmacPlanner2D` ist gebaut, gehasht und im aktiven `planner_server` bestätigt.
`allow_unknown=false`, 5-cm-Raster, 0,40-m-Inflation, 0,395-m-Costmap-Footprint
und unabhängiger 0,15-m-Clearancecheck waren aktiv. Die ersten beiden Goals und
Stopproofs bestanden (GT-Abstände `0,08796 / 0,09753 m`; Umfahrung
Plan-/GT-Clearance `0,16349 / 0,21880 m`). Die Passage endete `ABORTED`: kein
Planpfad, wiederholt `no valid path found`, finale TF-/Scan-/Costmapdaten frisch,
kein Boundsfehler. Das Bag ist lesbar (517.666 Nachrichten; 79 globale
Costmaps). In den jüngsten 25 Snapshots waren sieben Gate-Mittelpunkte bei
`x=4,0 m` durchgehend Occupancy `99`, das Humble für internes Inscribed `253`
publiziert; Smac-Radiuschecker blockiert ab 253. Run 41 ist **FAIL**, die
Smac-/0,395-m-Profilkombination nicht abgenommen.

**Praktikable Entscheidung (08.10.2026, Nutzerfreigabe „praktikable Ansätze,
Constraints iterativ“):** Der Planner-Footprint ist wieder der physische SIL-Radius
`0,22 m` (identisch zur Hardware-`navigation.yaml`), Inflation ist eine weiche
Kostenschicht `0,45 m` unterhalb der Gate-Halbbreite, SmacPlanner2D läuft mit
`cost_travel_multiplier=2.0` und `allow_unknown=false`. Die Passagebreite ist
nicht mehr gepinnt, sondern in `scripts/phase4_geometry.py` hergeleitet:
`2 × (0,22 m Radius + 0,15 m Clearance + 0,13 m seitliches Lokalisierungs-/
Trackingbudget) = 1,00 m`. Clearance `0,15 m`, Zieltoleranz `0,20 m` und
Bewegungsgrenzen sind unverändert. **Run 42** (`g4-nav-20261008-145048-k4lioS`)
bestand Mapping, Map-Reuse und alle drei erreichbaren Ziele (GT-Abstände
`0,1085 / 0,1205 / 0,0442 m`); Passageplan quert das Gate genau einmal mit
`0,255 m`, die unabhängige GT-Trajektorie mit `0,2798 m` Clearance; Live-Costmap
zur Planzeit bekannt-frei; Stopproofs 7 Stillstandssamples, Drift `0,0 m`;
Bag `512.753` Nachrichten. **Run 43** (`g4-nav-20261008-150754-HJjSQk`) bestand
das unerreichbare Ziel mit derselben Konfiguration fail-closed (`ABORTED`, `0`
Fahrbefehle). Damit sind alle vier G4-Szenarien sensor-derived laufzeitbelegt.
**Runs 44/45 (`g4-noise-slip-v1`, 08.10.2026) bestanden zusätzlich die vier
G4-Szenarien unter fixiertem Lidar-/IMU-Rauschen, Rad-μ=`0,8` und Seed
`20261008`; vollständiges G4 ist damit im SIL abgenommen.** Zu diesem
Zwischenstand lagen Modultrennung und G5-Vorbereitung noch vor uns; beides ist
inzwischen umgesetzt und G5 am 09.10.2026 im SIL bestanden. Der aktuelle
technische Einstieg ist G6 gemäß Abschnitt 8; Hardware-/externe
Inferenzaktionen bleiben gesperrt.

**G4-Abnahmerichtung (08.10.2026; Arbeitsvertrag, technische Zahlen teils offen):** Run 25 bleibt eine bestandene, GT-assistierte Fixture-Baseline; sie darf nicht als sensor-only Transfernachweis bezeichnet werden. Vollständiges transferorientiertes G4 muss zusätzlich Navigation mit einer SLAM-Karte bestehen, deren Navigationszellen nicht anhand von Ground-Truth-Posen/-Samples freigemarkt wurden. Ground Truth dient in diesem Nachweis ausschließlich der unabhängigen Bewertung.

Die vier G4-Szenarien sind freie Fahrt, Hindernisumfahrung, enge Passage und unerreichbares Ziel. Freie Fahrt und Umfahrung verlangen Nav2 `SUCCEEDED`, unabhängige Ground-Truth-Zielabweichung höchstens `0,20 m`, einen finalen Nullbefehl nach dem letzten Nichtnullbefehl und gemessenen Stillstand nach dem Action-Ergebnis; ein Nullbefehl kann durch den 0,2-s-Command-Timeout vor dem terminalen Actionstatus eintreffen. Für die Fixture-Baseline bleiben `0,15 m` Mindest-Clearance sowie `0,08 m/s` / `0,30 rad/s` Bewegungsgrenzen unverändert. Diese Werte sind SIL-Kriterien, keine physisch validierten Hardwaregrenzen. Die enge Passage verwendet in der separaten Fortress-G4-Welt eine aus `phase4_geometry.py` hergeleitete Geometrie mit Lidar-Sicht: Gatebreite `1,00 m` (= 2 × (0,22 m Radius + 0,15 m Clearance + 0,13 m seitliches Budget)), `robot_radius=0,22 m`, Map-Grid `0,05 m`. Runs 42/43 belegen Querung, Mindest-Clearance und fail-closed Abbruch im integrierten G4-Lauf; die frühere `0,84 m`-Variante (Runs 29–41) ist historisch.

Für ein nach Karte unbekanntes/belegtes, damit unerreichbares Ziel gilt als Arbeitsproduktpolicy: Action-Annahme, zielbezogener Plannerfehler, terminal `ABORTED`, keine absichtliche Nichtnullbewegung durch Recovery und gemessener Stillstand. Wenn kein Nichtnullbefehl ausgegeben wurde, belegen frische Ground-Truth-Poses den No-motion-/Stillstandszustand; falls Bewegungskommandos auftraten, ist danach ein expliziter Nullbefehl erforderlich. Normale Nav2-Recovery bleibt für andere Fehler/Szenarien bestehen. Der goal-spezifische Fail-Closed-BT ist für diese Produktsemantik implementiert; keine Standard-Recoveryänderung und kein BT allein zum Erreichen eines Test-PASS. Run 26 erfüllt die Policy nicht (`CANCELED` nach Recovery-Auslenkung). Zulässige Ground-Truth-Abweichung und Deadline sind aus Simulationsauflösung/Geometrie vorab herzuleiten; `0,25 m`/`0,02 m` werden nicht automatisch als Produktbudget übernommen.

Vollständiges G4 enthält mindestens einen reproduzierbaren, kontrolliert parametrierten Lidar-/IMU-Rausch- und vereinfachten Schlupffall. Das SIL-Stressprofil `g4-noise-slip-v1` (Seed `20261008`, Lidar `σ=0,01 m`, IMU-Gyro `σ=0,01 rad/s`, Beschleunigung `σ=0,1 m/s²`, Rad-`μ=0,8`) ist ROS-frei regressionsgeprüft, in Humble gebaut und in **Runs 44/45** runtime-bestanden; es ist kein Hardwaremodell. Die nominale Welt bleibt unverändert, `fortress_g4_noise_slip.sdf` ergänzt den Seed-Setter vor den Sensor-Systemen. Run 44 bestand sensor-derived Mapping, Map-Reuse/Readiness, freie Fahrt, Umfahrung und Passage; Run 45 bestand unreachable-only mit derselben Karte und demselben Profil. Die Bags sind lesbar und enthalten die vereinbarten Sensor-, TF-, Odom-, Costmap-, Plan- und Befehlsdaten. Reale Sensor-/Reibungsparameter und physische Übertragbarkeit bleiben unvalidiert. Eine eigenständige Replay-Abnahme ist kein zusätzliches G4-Szenario und bleibt Folgearbeit. Abtast-/Datenlücken-Grenzen, Passage- und Recoverybudgets sowie Szenario-Deadlines bleiben separate offene Vertragswerte.

**Arbeitsreihenfolge (vertikale G4-Scheiben, kein Tuningloop):**

1. **Sensor-only-Navigationspfad:** Roh-SLAM-Karte speichern und wiederverwenden;
  Ground Truth ausschließlich für unabhängige Bewertung. Runs 42/43 belegen alle
  vier Szenarien mit verifiziertem Bag. Keine Unknown-Zellen freimarken. Die
  GT-assistierte Run-25-Karte bleibt unverändert Fixture-Baseline.
2. **Vollständige Szenarien:** erledigt in Runs 42/43 mit physischem Radius,
  weicher Inflation, SmacPlanner2D und hergeleiteter `1,00 m`-Passage. Die
  Runs 35–41 bleiben als Lehrbeispiel archiviert: ein künstlich vergrößerter
  Planner-Radius und eine Ein-Zellen-Reserve waren kein tragfähiges Design.
  Weitere Constraints (z. B. kleineres Seitenbudget) nur zusammen mit dem
  Noise-/Slip-Nachweis neu bewerten, nicht als Einzel-Tweak.
3. **Robustheit und Belege:** erledigt in Runs 44/45 mit fixiertem Profil,
  Sensor-Map-Provenienz, Nav2-/Stopbelegen und validierten rosbag-Artefakten.
  Eigenständige Replay-Abnahme bleibt Folgearbeit.
4. **Abnahme:** zuerst ROS-freie Contract-/Geometrieregressionen; danach SIL
  einzeln und manuell, mit vollständiger Ausgabesichtung. Erst vier bestandene
  Szenarien öffnen G5.

Keine weitere Änderung an Nav2-Zielpunkten oder -Grenzwerten, nur um einen PASS zu
erreichen. Eine gemeinsame Hardware-/Legacy-Geometriequelle und physische
Vermessung bleiben Transfer-/Hardwarearbeit; G4 pinnt und protokolliert das
Fortress-Profil, behauptet aber keine Hardwarevalidierung.

> **Historische G4-Chronik:** Die folgenden iterativen Laufdetails dokumentieren
> den damaligen Entwicklungsverlauf. Der aktuelle Stand und die späteren Runs
> 24–26 stehen im [`IMPLEMENTATION_STATUS.md`](IMPLEMENTATION_STATUS.md); offene
> nächste Schritte stehen ausschließlich in [`OPEN_TASKS.md`](OPEN_TASKS.md).

**Historischer Arbeitsstand 06.10.2026 (durch Runs 42–45 überholt):**
`scripts/test_phase4_gate.sh` enthielt damals einen
begrenzten Fortress-SLAM-Lauf, Map-Speicherung im jeweiligen Logverzeichnis,
Neustart unter einer neuen Ignition-Partition und Nav2-Ziele für freie Fahrt
und Hindernisumfahrung. Das ist Implementierung, noch kein Laufzeitnachweis:
Map-Alignment, Plannerpfad, Zielerfolg und Hindernisabstand müssen im Simulator
gemessen werden; zu diesem damaligen Stand war G4 noch offen. Der erste Gateversuch am 06.10.2026
brach vor Mapping an einem falschen Partitionspräfix ab; der zweite fuhr die
Mapping-Route, wurde aber durch einen unpassenden 1.000-Zellen-Mindestwert
abgewiesen (655 bekannte Zellen, 72 belegte). Der dritte Lauf bestand Mapping
und Nav2-Readiness, scheiterte aber vor dem ersten Ziel: Map-Ursprung x=0,02 m
ließ die echte Startpose x=0 außerhalb der bekannten Costmap. Eine 0,55-m-
Rückwärtsstrecke zu Beginn der Mappingroute soll jetzt den Startbereich mit
kartieren; diese Änderung besteht die lokale Suite, benötigt aber noch einen
Simulatorlauf. Der vierte Lauf brach am letzten Mappingabschnitt wegen
zu geringer Hindernis-Clearance (−0,140 m) ab; der fünfte bestätigte den
verlängerten Rückweg mit minimal +0,4646 m Clearance und bestand Mapping sowie
Nav2-Readiness. Nav2-Ziele wurden trotzdem nicht gesendet: Die SLAM-Karte beginnt
bei x=0,02 m und umfasst die reale Startpose x=0 nicht. Eine neue
Map-Vorbereitung erzeugt deshalb eine getrennte Nav2-Karte aus der rohen
SLAM-Karte plus ausschließlich gemessenem, clearance-geprüftem Startkorridor.
Der sechste Lauf hat diese Aufbereitung im Simulator bestätigt (Origin −0,28 m,
113 Korridorzellen, Start-Footprint und Testbox erhalten) und bestand Mapping
sowie Nav2-Readiness. Er stoppte vor dem ersten Ziel an einer zu engen
`/cmd_vel`-Publisher-Allowlist; die beobachtete `behavior_server`-Quelle wurde
eng begrenzt ergänzt und im siebten Lauf bestätigt. Dieser bestand auch das
freie Ziel, das Hindernisziel lief aber nach 180 Wall-Sekunden in den Timeout;
Cancellation wurde akzeptiert, erfolgreiche Umfahrung nicht nachgewiesen.
Zusätzlich zeigte Nav2 alle 1904 Kartenfelder als bekannt: Map-Saver-Unknown
Pixel 205 wird bei `free_thresh: 0.25` als frei eingestuft. Die Map-Ausgabe
normalisiert Unknown nun auf 127; der achte Lauf bestätigte das Laufzeitverhalten
(849 bekannte von 2001 Kartenfeldern) und ließ die Hindernisziele `(3,0)` und
`(3,−0,90)` jeweils korrekt am Unknown-Preflight scheitern. Der Prüfer wählt
jetzt fail-closed den östlichsten bekannten-freien Punkt aus einer begrenzten
Kandidatenliste auf der gemessenen, clearance-geprüften Südlane (Mindestabstand
0,15 m). Der zehnte Lauf zeigte, dass damalige Kandidaten auf der Kartenzeile
Unknown sind; kein Ziel wurde gesendet. Der elfte Lauf erreichte `(3,00;−0,84)`,
aber Nav2 scheiterte am Costmap-Ostrand. Der zwölfte Lauf bestand Map/Readiness,
doch NavFn konnte `(2,80;−0,74)` nicht planen. Die Rohkarte war nur 68×29 Zellen;
der bekannte-freie Weg um die Box war nicht footprint-breit. Der Goal-BFS
verlangt nun auch Kartenrandabstand und blockiert Corner-Cutting. Die Mapping-
Prüfung sammelt Pose-/Clearance-Samples; die vorbereitete Karte wird nach Süden
gepaddet und markiert ausschließlich die gesampelten Roboter-Swept-Corridors der
bestandenen Südwegsegmente frei (Unknown sonst unverändert). Vollständige lokale
Suite: 249 Tests (244 bestanden, 5 erwartete Skips). Der 13. Lauf bestätigte
Mapping, Map-Preparation, Nav2-Readiness, freies Ziel und Nav2-Aktions-Erfolg
für die Umfahrung, aber die unabhängige Fortress-Enddistanz 0,212 m überschritt
das Gate-Limit 0,20 m; finaler Nullbefehl lag vor. Der Gate-Lauf ist damit
fehlgeschlagen. Das externe 0,20-m-Limit bleibt strikt; beide Nav2-XY-
zieltoleranzen wurden von 0,12 auf 0,08 m gesenkt. Der 14. Lauf bestätigte die
Mapping-Fahrt, aber Map-Preparation verweigerte sicher einen Sweep-Korridor, der
ein belegtes Rasterfeld bei `(2,145;−0,698)` schnitt. Die Messspur wird 0,10 m
weiter südlich gefahren; südliches Kartenpadding folgt nun dem tiefsten
gemessenen Sweep-Punkt. Lokale 249 Tests bestehen. Der 15. Lauf
(`g4-nav-20261006-205629-ylgV2V`) bestätigte Mapping, Map-Prep und Nav2-Readiness;
das erste Ziel bestand (`0,059 m`), aber das Umfahrungsziel verfehlte das 0,20-m-Limit
mit `0,276 m` (`[+0,262; -0,087]` m). Der 16. Lauf (`g4-nav-20261006-211913-gK79fz`)
bestand das erste Ziel mit `0,080 m`, brach aber das Umfahrungsziel mit Status 6 ab:
die verschärften AMCL-Werte überforderten den Filter (`Transform data too old` bei 34,7 s,
Extrapolationsfehler im Costmap/TF), wodurch Navfn scheiterte. Zudem stellte sich
`GridBased.tolerance: 0.25` als primäre Ursache der Zielverschiebung um bis zu 25 cm heraus.
Korrektur: AMCL auf das stabile Standardprofil zurückgesetzt; `tolerance` im NavfnPlanner
auf `0.05` gesenkt und `use_astar: false` (Dijkstra) gesetzt, um Pfadgenerierungsfehler zu
vermeiden. Der 17. Lauf (`g4-nav-20261006-213827-ic8ioY`) erbrachte den Durchbruch bei der
physischen Ankunft: Erstes Ziel bestand mit `0,077 m`, und das Umfahrungsziel erreichte
die Fortress-Endpose `[2,917; -1,026]` mit **0,142 m** Distanz zum Ziel (strenges Gate-Limit
0,20 m erstmals unterboten, finaler Nullbefehl vorliegend). Der Lauf brach jedoch nach
der Ankunft ab, weil `analyze_avoidance_path` den letzten lokalen Replanningspfad
`plans[-1]` prüfte (Startpunkt am Ziel) statt des initialen Umfahrungspfads. Korrektur:
Pfade während der Zielnavigation vollständig puffern und `select_avoidance_plan` wählen,
der den ersten vollständigen Pfad von Start zu Ziel auswertet. Visuelle Verfolgung
im RDP-Desktop (:10.0) via `--visual` (Gazebo-3D + Nav2-RViz), `--gui` oder `--rviz` in
`test_phase4_gate.sh` und `fortress.launch.py` integriert; Standard bleibt headless.
Der 18. Lauf (`g4-nav-20261007-073925-03yJlJ`) lief mit `--visual` (Gazebo-GUI + RViz2 auf :10.0):
Erstes Ziel bestand mit `0,059 m`. Beim Umfahrungsziel lag die Fortress-Endpose bei
`[2,753; -1,102]` mit `0,224 m` Distanz zum Ziel `(2,85; -0,90)` (um 2,4 cm verfehlt).
Ursache: Seit der Verlängerung von `pass_south_of_box` auf 1,10 m liegt die Mittellinie des
vermessen-gesampelten Südkorridors bei $y \approx -1,05 \dots -1,10$ m, während die veralteten
Zielkandidaten nur $y \in [-0,90; -0,70]$ abdeckten (20 cm nördlich der realen Korridormitte).
Korrektur: `AVOID_GOAL_CANDIDATES` auf die reale Korridorachse ausgerichtet
($x \in (2.80, 2.75, 2.70, 2.85)$, $y \in (-1.05, -1.00, -0.95, -0.90, \dots)$);
`select_avoidance_plan` validiert. Der 19. Lauf (`g4-nav-20261007-084433-WBruJq`)
erreichte beide Ziele (Ziel 1: 0,061 m, Ziel 2: 0,143 m physische Enddistanz).
Der 21. Lauf (`g4-nav-20261007-092122-IUUBku`) bestätigte Ziel 1 mit `0,071 m`;
bei Ziel 2 fuhr der Roboter 75 Sekunden lang stabil um die Box, stoppte jedoch vor
der finalen Zielerreichung: Beim Abbremsen unter die Bewegungsschwelle (`update_min_d: 0.25`)
stellte AMCL die TF-Publikation ein, wodurch nach Ablauf der Toleranz ein
Transform-Timeout im Controller entstand und Recovery-Verhalten den Roboter über den
Costmap-Südrand trieben. Ein zu hoher damaliger AMCL-Bewegungsschwellwert
(`update_min_d: 0.25`) war eine plausible, aber nicht allein bewiesene Ursache.
Run 21 wurde assistant-seitig entgegen der Nutzeranweisung gestartet; diese
Grenzüberschreitung ist im Nachtrag unten festgehalten. G4 bleibt offen.
Bewegungsgrenzen: 0,08 m/s und 0,30 rad/s.

**Ergebnis:** reproduzierbar navigierender virtueller Roboter ohne Jetson.
**Gate G4:** Karte lässt sich wiederverwenden; definierte Ziele werden erreicht
oder kontrolliert abgebrochen; Kostenkarte/TF/Clock bleiben konsistent.
Erfolg, Fehler, Zielabweichung und Laufzeit messen, nicht nur ein Demo-Video erstellen.

### Phase 5 – Visionvertrag mit Mock und Aufzeichnungen

**Aktueller Stand 13.10.2026:** G4 und G5 sind im SIL bestanden. G5 belegt den
Off/Mock-Vergleich mit Nav2, nicht echte Inferenz. Der lokale G6-Pfad ist mit
einem positiven Domain-42-Capture, Contract-v1-Metadaten und `person`-Detection
über den isolierten `.71`-Protokoll-Zwilling bestanden. Das belegt weder
Jetson-/TensorRT-Leistung noch einen echten Live-Inferenzlauf auf `.71`. Die
darunterstehenden Phase-5-Berichte sind historische Entwicklungsstände und
keine aktuellen Gate-Statusmeldungen.

**Historischer Fokus (07.10.2026):** zunächst nur ein versionsgebundener,
ROS-freier Ergebnisvertrag und Fixtures. Das war Vorarbeit, keine G5-Abnahme;
echter Vision-Dienst und Inferenz waren damals noch nicht Gegenstand.

**Phase-5-Arbeitsstand 07.10.2026:** Paket `linorobot2_vision` definiert
Vertrag v1 mit expliziter Zeitdomäne und liest deterministische Ergebnis-Fixtures
über einen Fixture-Backend. Ein Offline-Adapter bildet das im gesicherten
Laufzeitquellstand dokumentierte `/v1/vision/detection`-Antwortformat ab; Capture-
Metadaten und Label-IDs müssen explizit zugeliefert werden. `timestampUTC` wird
nicht als Aufnahmezeit verwendet. ROS-freie Tests decken Erfolg/leer/Fehler,
Box-/Konfidenzvalidierung, Datenalter, Zeitdomänen und Adapter-Mapping ab.
Ein interaktiver nativer X11-Fixture-Monitor läuft im RDP-Desktop und lädt
gepaarte Bild-/Ergebnisfälle aus einem Replay-Manifest. Er zeichnet lokale SVG-
Szenen und die dazugehörigen Detection-Boxen; `1`/`2`/`3` wechseln zwischen
Detektion, leerem Ergebnis und Ausfall. Das XFCE-Terminalraster bleibt Fallback.
Beide sind keine Live-Kameravorschau. Tkinter wurde nicht installiert; das native
Fenster nutzt das bereits verfügbare `libX11`.

Der begrenzte Phase-5-Kameratest startet zusätzlich ein eigenes RViz-Preset mit
Image-Display auf `/camera/image_raw`. Ein PASS lässt Gazebo/RViz im interaktiven
RDP-Lauf offen; Shellausgabe bleibt im Mac-Terminal. Der simulierte Kameraeingang
wird durch einen absichtlichen Empty-Mock konsumiert, nicht inferiert.
Der Adapter ist nicht gegen einen Live-Dienst getestet; ROS-Transport und G5
bleiben offen.

**Simkamera-Consumer-Probe 07.10.2026: PASS** (`p5-sim-camera-20261007-105732-mDfK2b`):
5 echte Fortress-Image-/CameraInfo-Paare wurden mit Contract v1 und absichtlichem
Empty-Mock konsumiert (`rgb8`, `640×480`, Frame `camera_link`, `fx=fy=554.3827`).
Das ist keine Inferenz und keine G5-Abnahme. G4 ist wieder aufgenommen, aber noch
nicht bestanden; die G5-Bedingung „Navigation funktioniert mit Vision aus“ kann
erst nach vollständiger G4-Abnahme geprüft werden.

**Visuelle Wiederholung 07.10.2026: PASS** (`p5-sim-camera-20261007-120350-YKI2e1`):
Der Lauf startete zusätzlich RViz mit `fortress_camera.rviz`; Image-Display zeigt
`/camera/image_raw` und bleibt nach dem 5-Frame-Consumer-PASS offen. Bericht:
`rgb8`, 640×480, `camera_link`, fünf gepaarte Frames, Empty-Mock, null Detektionen,
keine Fehler. Fortress, Gazebo-GUI und RViz wurden danach als laufend verifiziert.
Das bestätigt das visuelle RDP für den simulierten Kamerastream, nicht echte
Inferenz und nicht G5. Beenden über Enter/Ctrl+C im gestarteten Harness-Terminal.

**Arbeit:** Ergebnisformat und Modell-/Klassenmapping definieren. Mock-Dienst und
normalisierenden Adapter entwickeln; zuerst gespeicherte Testbilder/Resultate,
danach simulierte Kamera. Gültige, leere, fehlerhafte, verspätete und nicht
verfügbare Antworten testen. Bestehende REST-/ZMQ-/YOLO-ROS-Quellen als Referenz,
nicht als identische Dienstversion behandeln.

**Ergebnis:** Robotersoftware nutzt Vision ohne reale Kamera oder GPU.
**Gate G5:** Visionbackend austauschbar, Datenalter/Frame-Semantik eindeutig;
Navigation aus G4 bleibt bei ausgeschalteter Vision funktionsfähig.

### Phase 6 – Echte Vision isoliert validieren

**Arbeit:** ausgewähltes kleines Modell lokal per CPU gegen dieselben Testdaten
prüfen, bei Bedarf in separater Umgebung. Bestehenden Jetson-Dienst später lesend
gegen Adaptervertrag prüfen; keine blind kopierten neueren Mac-Dockerfiles.
Endpunkte, Python/Torch/CUDA-Version, Klassen und Antwortschema tatsächlich erfassen.

Intrinsics, Bildauflösung und Kamera-/Lidar-Extrinsics zuordnen. Die gefundenen
640×480-/1920×1080-Abweichungen auflösen, bevor geometrische Aussagen verwendet
werden. Inferenzqualität/-latenz messen; keine sofortige PPE-Neutrainingspflicht.

**Ergebnis:** mindestens ein reales Inferenzbackend hinter dem geprüften Vertrag.
**Gate G6:** Ergebnisse sind fachlich geprüft und zeitlich nutzbar; Dienstverlust
wird erkannt; keine direkten Motorbefehle aus Vision oder LLM-/Frontendcode.
Jetson-Systemänderungen nur nach vollständigem Wiederherstellungsplan; ein
Mock/CPU-Backend erlaubt Weiterentwicklung, falls der Jetson vorerst nicht verfügbar ist.

### Phase 7 – Software auf Zielrechnern und Hardware-in-the-Loop

**Arbeit:** Roboterkern auf Pi aufbauen, Netzwerk/DDS/Uhren und Ressourcen unter
Last prüfen. Zunächst simulierte Basis beziehungsweise aufgezeichnete Sensoren
als Gegenstelle: Pi-Software betreibt noch keine ungetesteten Motoren.
Replay behandelt Zeit und Ausgabebefehle gesondert; niemals gleichzeitig Simulator
und Replay als unkontrollierte Publisher derselben Sensoren betreiben.

ESP32-Board/Flashstand und Transport identifizieren; serielle/WiFi-/Baudraten-
Abweichungen klären, keine Neu-Flash-Pflicht ohne Rückfallweg. Agent und echte
Encoder/IMU zunächst mit deaktivierter Motorleistung beziehungsweise sicherem
Prüfstand abnehmen. Lidar und Kamera einzeln; keine konkurrierenden Gerätetreiber.

**Ergebnis:** Hardwarebackend erfüllt denselben Vertrag wie der Simulator.
**Gate G7:** Umschalten erfordert Backend-/Konfigurationswahl, keinen Umbau der
Navigationsalgorithmen. Lokale Stops funktionieren auch bei Pi-/Netzwerk-/Jetsonausfall.
Vor jedem Test klare Hoheit über Kommandos und Motorfreigabe festlegen.

### Phase 8 – Physische Kalibrierung und Feintuning

**Arbeit:** Verdrahtung, Stromversorgung, Not-Aus, Encoder-Richtung/-Auflösung,
Radmaß/Spurweite, Motor-PID, IMU, Kamera/Lidar und Limits tatsächlich vermessen.
Erst Räder angehoben, dann niedrige Geschwindigkeit in freier Testfläche.
Odometrie/SLAM/Nav2 stufenweise prüfen und Unterschiede zur Simulation dokumentieren.

**Ergebnis:** gemessene Hardwareparameter, Kalibrierungen und reale Testprotokolle.
**Gate G8:** kontrollierte Fahrt/Stopps, Navigation und Fehlerfälle abgenommen;
Pi-Temperatur/CPU/RAM und Jetson-Latenz akzeptabel. Gemessene Parameter fließen
zurück ins virtuelle Modell; virtuelles und reales Testset bleiben erhalten.

### Phase 9 – Reproduzierbarer Betrieb und Weiterentwicklung

**Arbeit:** gepinnte Releases, saubere Start-/Stopabläufe, Autostart erst nach
manueller Abnahme, Wiederherstellungsanleitung und Lifecycle-Review erstellen.
Änderungen durchlaufen Modell-/Interface-/SIL-Tests, dann die betroffenen Hardwaretests.

**Gate G9:** ein frischer Entwicklungsworkspace kann die Tests wiederholen;
Deployment und Rückfall sind dokumentiert. Das virtuelle Modell bleibt das erste
Prüffeld, nicht ein einmaliges Demo-Artefakt.

## 6. Test- und Projektorganisation

Bestehende Paketnamen/Public APIs möglichst erhalten. Die folgende Trennung ist
eine Zielstruktur, noch kein Auftrag, sämtliche Altprojekte neu zu scaffolden:

- Beschreibung/Parameter: URDF/Xacro, Sensorframes, gemeinsame Roboterwerte.
- Roboterkern: EKF, Cmd-Vel-Priorisierung, SLAM/Nav2-Konfiguration und Verhalten.
- Backends: Simulation getrennt von micro-ROS-/Hardwareanbindung.
- Visionadapter/Vertrag: keine Torch-/CUDA-Abhängigkeit im Kern.
- Bringup/Profile: ausschließlich Zusammenstellung und Konfiguration.
- Tests/Fixtures: kleine Welten, Testbilder, Mock-Antworten, rosbag-Szenarien.
- Dokumentation: Quellenmanifest, Phasengates, Entscheidungen und Testresultate.

Unit-/Vertragstests sind möglichst hardware- und simulatorfrei. Simulationsintegration
mit `launch_testing`/passenden ROS-Testwerkzeugen und Colcon testen; Grafikabhängige
Tests getrennt von schnellen Tests führen. Simulatorstart, Sensorprüfung und Nav2-
Abnahme gehören zusätzlich in Integrationstests. Fehlende Bestandstests nicht durch
leere erfolgreiche Testberichte ersetzen. CI ist zuerst Build plus schnelle Tests;
3D-/GPU-Testfähigkeit des jeweiligen Runners erst belegen.

Ein gemeinsamer Testfallkatalog wird in Simulation, Replay und Hardware benutzt.
Allgemeines Verhalten teilen, plattformspezifische Kalibrierungen/Transportdetails
nicht künstlich als identische Parameter ausgeben. Jede Phase erhält Messwerte,
Logs und eine kurze Abnahmeentscheidung; fehlgeschlagene Gates bleiben sichtbar.

## 7. Betriebsschutz und klare Nicht-Ziele der ersten Iteration

- Reale Motorseite und Simulation in **getrennten ROS-Domains** halten. Das
  gilt auch bei HIL: gezielte Vermittlung statt pauschaler Topic-/Domain-Zusammenführung.
- Keine `/clock`-Brücke zum real fahrenden Roboter; keine Test-/Replay-Fahrbefehle
  ohne explizite Freigabe auf reale Motoren übertragen.
- Jetson-OS, L4T 32.6.1, CUDA 10.2 und TensorRT 8.0.1 erhalten.
- Vollständiges Jetson-/QSPI-Backup und Speichererweiterung vor invasiven
  Jetson-Arbeiten klären; **nicht** als Voraussetzung für lokale Simulation behandeln.
- Vorhandene Quellen-/Container-FS-/Experiment-/Trainingsarchive auf Mac/NAS
  bleiben erhalten. Die verworfene USB-SSD ist keine Abhängigkeit.
- Nicht gleichzeitig sämtliche Firmwarevarianten, Articubot, Kalibrierungstools,
  LLM-Frontend und Trainingspipelines produktiv integrieren.
- Kein Anspruch auf perfekte physikalische Motornachbildung in Iteration 1.
  Hardware-PID, Reibung, Versorgung und Bremswirkung später messen.

## 8. Prioritäten und nächster Meilenstein

| Priorität | Inhalt | Aktueller Status |
| --- | --- | --- |
| P0 | Quellen und Erkenntnisse erhalten | wesentliche Quellen-/Vision-/Experimentkopien geprüft; vollständiges Jetson-Systemabbild weiterhin offen |
| **P1** | **Phasen 0–1: Entwicklungsumgebung und Simulation** | **G1 am 06.10.2026 bestanden: Starts, Sensor-/Clock-Stabilität, Lidar-Geometrie/QoS, Kamerabildwechsel und 902,73 s GUI-Stabilität; Details und Resthinweise in [IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md)** |
| **P2** | **Phasen 2–4: Modell, Basisvertrag und Navigation** | **G2/G3 bestanden; G4 vollständig im SIL in Runs 42/43 nominal und 44/45 mit fixiertem Noise-/Slip-Profil sensor-derived bestanden; physische Übertragbarkeit bleibt offen** |
| **P3** | **Phasen 5–6: entkoppelte Vision** | **G5 und lokaler G6-Pfad im SIL bestanden.** Ein positiver Domain-42-Foto-Capture ist am 13.10.2026 über den lokalen `.71`-Protokoll-Zwilling gegen Contract v1 belegt. Ein echter `.71`-TensorRT-Vergleich ist optional und erfordert einen vom Nutzer gestarteten Dienst sowie einen separat freigegebenen Einzelrequest. `.150` ist für diesen Zweck nicht der richtige Endpunkt. |
| P4 | Phasen 7–8: Hardwareintegration und Feintuning | nach virtuellen Gates und notwendigen Hardware-Sicherheitsprüfungen |
| P5 | Phase 9 und späterer Lebenszykluswechsel | nach Abnahme; Review vor Humble-/Fortress-EOL Mai 2027 |

**Highlevel-Review und aktuelle Entscheidung (13.10.2026):** G4 ist mit Runs
42–45 im SIL bestanden; G5 ist mit dem Off/Mock-Nav2-Vergleich ebenfalls im SIL
bestanden. Run 25 bleibt separat eine GT-assistierte Fixture-Baseline. Weder
G4 noch G5 sind Hardware- oder Inferenzabnahmen. Die ältere Review-Chronik und
die folgenden Phase-5-Arbeitsstände dokumentieren Zwischenstände vor diesen
Abnahmen.

**Nächster Einstieg (15.10.2026):** Der G4/G5-Nav2-Input-Replay, G9-
Frischaufbau/Start/Stop/Rollback und die optionalen G3-Yaw-Bias-
Diskriminatoren sind abgeschlossen. Die letzte hardwareunabhängig offene
RDP-Auflösungsbeobachtung nach einem regulären Reconnect bestand am
15.10.2026 mit 1280×800 und sicheren Loopback-Listenern. Damit ist aktuell
keine weitere hardwareunabhängige Folgearbeit offen. Die priorisierte Liste steht
in [`OPEN_TASKS.md`](OPEN_TASKS.md). Ein echter `.71`-Livevergleich ist optional;
Pi-/Jetson-/HIL-Arbeiten bleiben nachgelagerte Transferpfade mit ihren
Sicherheits- und Freigabevoraussetzungen. Der lokale `yolo12n`-CPU-Nachweis ist
keine Jetson-/TensorRT-Abnahme. `.150` wird für diesen Zweck nicht verwendet.
Keine Nav2- oder `/cmd_vel`-Anbindung durch G6.

G7 bleibt bis zur unabhängigen Out-of-Band-Verifikation des Pi-SSH-Host-Keys
blockiert; motorloses HIL benötigt bestätigte Motortrennung und separate
Freigabe. G8 folgt nach G7 und eigener Freigabe. Keine Wiederholung von Run 25
oder Run 26; ROS-Domains sind keine Sicherheitsgrenze.

## 9. Referenzen zu unserem Bestand

Der [Referenzkatalog](reference/MACOS_SETUP_PLAN.md) ist Hintergrundwissen, keine konkurrierende
Entwicklungsreihenfolge. Bei Prioritätskonflikten gilt dieses Dokument.

| Thema | Referenzabschnitt |
| --- | --- |
| Classic-Abhängigkeiten und Buildfallen | 3, 7, 9 |
| Pi 4 / Humble und abweichende Quellen | 13 |
| Jetson-Inventar, Speicher und Erhaltung | 14–15 |
| ESP32-Firmware, Kameratreiber und Kalibrierung | 16 |
| Vision-API, Python-/Imagebuilder und Trainingsstände | 17 |

Der Fahrplan ist final für die vereinbarte **Simulation-first-Reihenfolge**.
Die G1-/G2-/G3-Abnahmen sowie G4 (Runs 42–45, SIL-only) sind durch
Laufzeitberichte belegt. G5 und spätere Phasen bleiben bis zu ihren eigenen
Prüfungen offen. Konkrete Plattform-/Moduländerungen entstehen erst nach den
jeweiligen Gates und separaten Freigaben.

### Historischer G4-Diagnoseanhang bis Run 23 (07.10.2026)

> Historische Diagnose; Run 24–26 und der aktuelle Abnahmestand sind im
> [`IMPLEMENTATION_STATUS.md`](IMPLEMENTATION_STATUS.md) dokumentiert. Dieser
> Abschnitt bewahrt nur die ursprüngliche Run-20-bis-23-Ursachenanalyse.

Run 20 (`g4-nav-20261007-091118-vqFVCX`) zeigte einen falschen Nav2-Erfolg:
Ziel 1 wurde in 2,42 s als `SUCCEEDED` gemeldet, aber die unabhängige
Fortress-Ground-Truth blieb am Start (Abstand `0,550 m`, ein Trajektoriensample).
Das Nav2-Log meldete TF-Datenzeit `2,619 s` gegenüber Sim-Zeit `5,143 s` und
Rückwärts-Extrapolation. Das Ground-Truth-Gate wies den Erfolg ab.

Run 21 (`g4-nav-20261007-092122-IUUBku`) bestätigte Ziel 1 mit `0,071 m`;
bei Ziel 2 fuhr der Roboter rund 75 s südlich um die Box, danach meldete Nav2
wiederholt veraltetes `map→odom`; anschließend geriet die Pose außerhalb der
Costmap (`y≈−1,68 m`) und die Aktion brach ab. Fehlende explizite AMCL-
Bewegungsschwellen im Fortress-Profil (Default `update_min_d=0,25 m`) sind eine
plausible, aber noch nicht allein bewiesene Ursache.

**Prozesskorrektur:** Run 21 wurde entgegen der Nutzeranweisung, nach dem
Fehlschlag zu stoppen, assistant-seitig zusätzlich gestartet. Das war nicht
autorisiert. Seitdem wurden keine weiteren Simulatorläufe gestartet. Die ältere
Formulierung oben, die Versuchsreihe sei nutzerseitig gestoppt worden, ist
insoweit ungenau; die Verantwortung für Run 21 lag beim Assistant.

Run 22 (`g4-nav-20261007-094055-WgzJ04`) war der eine ausdrücklich freigegebene
visuelle Validierungslauf. Mapping, Map-Preparation, Nav2-Readiness und das freie
Ziel bestanden; Ground-Truth-Abstand Ziel 1: `0,112 m` mit 92 Samples. Ziel 2
brach mit Status 6 ab: Der erste TF-Fehler zeigte `32,097 s` gegen Transformzeit
`31,600 s` (0,497 s alt); derselbe Transformstempel blieb danach stehen, während
die Anfragedatenzeit bis `76,767 s` fortschritt (45,167 s alt). Es folgten
Null-Längen-Pläne und `Controller patience exceeded`. Anders als Run 21 wurde
hier kein Costmap-Out-of-Bounds-Fehler festgestellt. Die niedrigeren AMCL-
Schwellen verhinderten den falschen sofortigen Erfolg am ersten Ziel, lösten
aber den TF-Ausfall während der Hindernisroute nicht. AMCL-Logs belegen den
späteren Publikationsstopp nicht näher. Kein automatischer Retry.

Lokaler Schutz: AMCL-Update-Schwellen `0,02 m` und `0,02 rad`; der Goal-Prüfer
kontrolliert TF-Zeitstempel beider dynamischer Kanten gegen `/clock` (max. 1 s
alt, 3,5 s Zukunftstoleranz) vor, während und nach einer Action und lehnt stale
TF ab. Nicht erfolgreiche Actions protokollieren nun zusätzlich Clock, Sim-Zeit-
und Empfangsalter der TF-Kanten sowie Scan-/Odom-/Plan-/Befehlsfortschritt; die
Run-22-Differenz `45,167 s` ist durch eine ROS-freie Regression abgedeckt.
Ground-Truth-Ankunftslimit und Geschwindigkeitsgrenzen bleiben unverändert.
Lokale Suite: 253 Tests bestanden, 5 erwartete Skips. G4 bleibt offen. Bei einer
künftigen ausdrücklich freigegebenen G4-Ausführung den Snapshot mit Nav2s
Zeitpaaren vergleichen, um echtes AMCL-Ausbleiben von einem fehlenden Transform
im Nav2-Buffer zu unterscheiden. Keine Zielkoordinaten micro-tunen; kein weiterer
Runtime-Lauf ohne gesonderte ausdrückliche Freigabe.

Run 23 (`g4-nav-20261007-100902-zAT1uQ`) wurde nach erfolgreicher RDP-/Display-
Bereitschaft einmalig im visuellen Modus ausgeführt. Mapping, Map-Aufbereitung,
Nav2-Readiness und freies Ziel bestanden; das Hindernisziel brach mit Status 6 ab.
Die neue Telemetrie verzeichnete während der Action 672 empfangene `map→odom`-
Updates (Zeitstempelfortschritt 67,2 s), während Nav2 intern wiederholt
Transformzeit `50,300 s` bei Anfragen bis `68,274 s` meldete. Zusätzlich verließ
der Sensorursprung den östlichen Costmap-Rand (x≈3,17 m bei Kartenende x≈3,15 m).
`/amcl_pose` hatte am Fehler-Snapshot seit 8,33 Wall-Sekunden keine Nachricht,
aber ein vollständiger `/tf`-Publikationsstillstand ist durch die empfangenen
Updates nicht belegt. Ursache und Reihenfolge von TF-Buffer-/Zeitfehler und
Costmap-Randüberschreitung bleiben offen. Kein Retry; G4 bleibt offen. Nächster
Schritt ist lokale Loganalyse, nicht Parametertuning. Ein weiterer Simulatorlauf
braucht eine neue ausdrückliche Freigabe.
