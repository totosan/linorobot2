# Offene Arbeiten: G6 und spätere Transfer-Gates

Stand: 14.10.2026. Diese Datei enthält offene Arbeit und klar gekennzeichnete
Abschlussnachweise. Die aktuelle Reihenfolge zieht alle lokal ausführbaren,
hardwareunabhängigen Arbeiten vor Hardware-/Gerätezugriffen. Sicherheits-
Vorbedingungen für spätere Transferarbeiten bleiben verbindlich. Der
autoritative Ablauf steht in
[`DEVELOPMENT_PLAN.md`](DEVELOPMENT_PLAN.md); geprüfte Einzelnachweise und
Laufprotokolle stehen in [`IMPLEMENTATION_STATUS.md`](IMPLEMENTATION_STATUS.md).

## Aktuelle Reihenfolge und Aufwand

Aufwand ist eine grobe Planungsschätzung: XS = wenige Stunden, S = bis etwa ein
Tag, M = mehrere Arbeitstage, L = mehrere Tage bis Wochen. Zuerst stehen die
ohne Pi, Jetson, physische Sensoren oder externe Inferenz ausführbaren Arbeiten.
Sicherheitsbedingungen für einen späteren Hardwarepfad werden dadurch nicht
abgeschwächt.

### Jetzt: hardwareunabhängige Arbeit

| Rang | Offener Punkt | Priorität | Aufwand | Einordnung |
| ---: | --- | --- | --- | --- |
| 1 | G9: Releaseaktivierung und Rollback-Drill vervollständigen | P1 – Software-Lifecycle | S | Remote-Frischcheckout, isolierter Build, 463 ROS-freie Tests und bewegungsloser Headless-Smoke bestehen. Offen sind zweiter validierter Release, atomare Aktivierung und Rückfall. Humble-/Fortress-EOL-Review bleibt ein eigener späterer Teilschritt. |
| 2 | G3-Yaw-Bias mit den vorgeschlagenen Physikschritt- und Kontaktvarianten-Proben weiter eingrenzen | P2 – optionale SIL-Untersuchung | M | Nur nach bestehendem Analyseplan; keine Änderung von Hardwareparametern und keine Übertragung des Simulatorfaktors. |
| 3 | RDP-Auflösung nach dem nächsten regulären Reconnect verifizieren | P3 – Betriebskomfort | XS–S | Nur beobachten; keine erzwungene RandR-/Framebuffer-Änderung. Gazebo-Autostart bleibt optional und wird nicht ohne Bedarf aktiviert. |

### Danach: externe und hardwareabhängige Transferarbeiten

Diese beginnen nur bei entsprechendem Projektbedarf. Die Sicherheits- und
Freigabebedingungen sind Blocker, keine durch die Umpriorisierung
abgeschwächten Empfehlungen.

| Rang | Offener Punkt | Priorität | Aufwand | Vorbedingung / Einordnung |
| ---: | --- | --- | --- | --- |
| 4 | Optional einen einzelnen echten Inferenzlauf auf Jetson `.71` prüfen | P2 – optional, nur bei gewünschtem Hardware-Inferenzvergleich | S–M | Nutzer kann `.71` bei Bedarf starten. Danach Dienst-/Engine-Hash und Labelmapping gegen den archivierten Nachweis verifizieren und genau einen freigegebenen Request mit klarem Bildursprung ausführen. Der frühere externe Einbildlauf lieferte null Detektionen; positive Erkennung und ROS-Contract-v1-Transport sind damit noch nicht belegt. Kein Motor-/Nav2-Test. |
| 5 | Vollständiges Jetson-/QSPI-Systemabbild sichern | P1 – Werterhalt vor invasiven Jetson-Arbeiten | M | Kein Blocker für lokale SIL-Arbeit; vor invasiven Änderungen erforderlich. |
| 6 | Jetson-Hardwarevoraussetzungen verifizieren: Firmwaretransport/Baudrate, Middleware/RMW, Geräteanwesenheit und Alias-/Kabelrollen | P1 – vor physischem Betrieb | M–L | Software-Preflight erkennt keinen Kabeltausch identischer UART-Geräte. Kein sensorführender Bringup vor Klärung. |
| 7 | Pi-SSH-Host-Key unabhängig out-of-band verifizieren | P0 – Sicherheitsblocker bei Pi-Zugriff | XS, sobald ein vertrauenswürdiger Vergleichswert vorliegt; derzeit blockiert | Vor jeder Pi-Verbindung. Bis zur unabhängigen Prüfung keine Verbindung und keine Änderung an `known_hosts`. |
| 8 | G7: motorloses Pi-/ESP32-HIL vorbereiten und prüfen | P1 – Transfergate | L | Erst nach Host-Key-Verifikation, bestätigter physischer Motortrennung und separater Freigabe. Keine Firmware- oder Motoraktion. |
| 9 | G8: physische Abnahme von Geometrie/Sensorposen, Not-Aus, Stoppverhalten und Limits | P2 – spätere Hardwareabnahme | L | Erst nach G7, auf sicherem Prüfstand und mit eigener Freigabe. SIL-Werte nicht als Hardwaremesswerte übernehmen. |

Der Host-Key bleibt ein absoluter Sicherheitsblocker für Pi-Zugriff, auch wenn
seine Prüfung in der Arbeitsreihenfolge erst unmittelbar vor dem Hardwarepfad
ansteht. `.150` ist für diesen Inferenzzweck nicht der richtige Endpunkt und
wird nicht als offener Kandidat geführt. Der frühere externe `.71`-Einbildlauf
vom 13.10.2026 lieferte null Detektionen; er ist kein positiver Nachweis. Ein
erneuter Request an `.71` erfolgt nur, wenn der Nutzer den Dienst gestartet und
den einzelnen Test ausdrücklich freigegeben hat.

### Abgeschlossen / kein offener Punkt

- **G4-/G5-Nav2-Input-Replay – abgeschlossen am 15.10.2026:** G5-Off,
  G5-Mock und beide Segmente des erfolgreichen G4-Run 42 bestanden den
  isolierten Domain-43-Gate. Der G4-Quellbag enthält einen echten
  Clock-Rücksprung; beide hash-geprüften Segmente sind einzeln lesbar, Segment
  1 übernimmt Map und statische TF. Jeder PASS beobachtete alle sechs Inputs,
  beide Costmaps, genau einen `/clock`-Publisher, keinen `/cmd_vel`-Subscriber
  und sauberen Prozess-/Graph-Cleanup. Die korrigierten Publisher- und
  Topictyp-Discovery-Races sind regressionsgeprüft. Scope bleibt
  Input-/Costmap-Replay ohne Ziele, G4/G5-Neuabnahme oder Hardware.
- **Dokumentstatus:** G4/G5, der lokale G6-PASS und aktuelle Gates sind
  synchronisiert; widersprechende ältere Formulierungen sind historische
  Zwischenstände.
- **G6 lokal – PASS am 13.10.2026:** Genau ein Domain-42-Gazebo-Fotocapture
  lieferte eine `person`-Detektion durch Contract v1. Bericht:
  `log/person_runtime/g6_oneshot_20261013_1759/vision-probe.json`.
  Modell `yolo12n`/CPU-PyTorch; keine Nav2-, `/cmd_vel`- oder Hardwareaktion.
  Der Mac-gehostete `sim_detector` ist dabei ein lokaler CPU-Inferenzdienst,
  der das ZeroMQ-Protokoll von `.71` nachbildet; er ist weder ein bloßer Mock
  noch der echte entfernte Jetson-Dienst. Der ROS-`backend:=mock` bleibt
  fixturebasiert und führt keine Inferenz aus. Der lokale Kamera→Detector→
  Contract-v1-Pfad ist damit positiv geprüft.
  Der CC0-Kandidat unter `log/test_assets/` samt Manifest ist nur für lokale
  Tests bestimmt und wird nicht weiterverteilt. Der Fixture-Generator prüft
  Lizenz, Hash und Maße, erhält das Seitenverhältnis und überschreibt keine
  bestehende Weltdatei. Details und bekannte Einschränkungen stehen in
  [`IMPLEMENTATION_STATUS.md`](IMPLEMENTATION_STATUS.md).

### Jetson-Container: Paketbuild bestanden, Hardwareprüfung offen

- Manuelles Starten durch den Nutzer ist die gewünschte Betriebsart; kein OS-Autostart erforderlich. Read-only geprüft: Image `l4t-ros2:humble-v2.4`, Ubuntu 22.04/Jammy, ROS 2 Humble, gemountetes Jetson-Workspace-Overlay und installierte Robot-/Nav2-/SLAM-/Sensorpakete.
- Im Mac-Workspace ergänzt: `src/linorobot2/docker/docker-compose.jetson.yaml`, `.env.jetson.example`, `start_jetson_container.sh` und `docker/JETSON.md`. Der Wrapper verweigert den Start bei fehlenden/gleichen seriellen Geräten oder fehlendem Overlay; Compose reicht nur die zwei Aliase explizit durch und mappt ESP32→`ttyUSB0`, LiDAR→`ttyUSB1`. Die Launchkette propagiert beide Portargumente und micro-ROS-Transport/Port/Baudrate; Laser-Treiber lesen das Portargument. ROS-freie Regressionen: `src/linorobot2/tests/test_jetson_deployment_profile.py`.
- Auf `.71` wurden die gezielten Quelländerungen synchronisiert; der bestehende lokale Xacro-Edit blieb erhalten. Der isolierte Builder bestand beide Pakete, Domain-Import/Installationspfad sowie `ros2 launch ... --show-args`. Der no-scan-LiDAR-Test bestand ebenfalls: nach 10-s Startup-Grace exitete der Monitor wie erwartet mit Status 1, der Launch-Handler beobachtete das Prozessende und die Testlaunch wurde innerhalb von 25 s beendet. Das ist ein isolierter Softwaretest ohne physische Sensoren oder Robot-Service.
- **Vor physischem Betrieb offen:** Firmwaretransport und Baudrate, Middleware/RMW, Geräteanwesenheit und korrekte physische Alias-/Kabelrollen verifizieren. Preflight erkennt keinen Kabeltausch identischer UARTs; Ports labeln. Kein sensorführender Bringup-, Motor- oder Firmwaretest erfolgte.

## Abgeschlossene SIL-Gates als Referenz

- **Erledigt:** G1–G4 im SIL bestanden; TF2-Humble-Fix manuell per A/B und 11/11 Upstream-Buffer-Tests belegt.
- **Run 25:** PASS für zwei Szenarien (freie Fahrt und Umfahrung) mit der vorhandenen GT-assistierten Map-Fixture. Dies ist kein sensor-only SLAM-/Hardwaretransfernachweis.
- **Run 26:** FAIL. Der Prüfer cancelt bei ca. 0,251 m; Nav2-Standard-Recovery erlaubt BackUp bis 0,30 m. Kein natürlicher `ABORTED`-Nachweis und keine G4-Abnahme.
- **Runs 42/43 (08.10.2026):** alle vier nominalen G4-Szenarien sensor-derived **PASS** mit gemeinsamer Nav2-Konfiguration (Radius `0,22 m`, Inflation `0,45 m`, SmacPlanner2D, Gate `1,00 m`).
- **Runs 44/45 (08.10.2026): vollständiges G4 robustheitsbelegt – PASS.** Run 44 (`log/g4_runtime/g4-nav-20261008-154438-w67Hqz/`) bestand Mapping, Sensor-Map-Preparation/Nav2-Readiness und freie Fahrt, Umfahrung sowie enge Passage mit dem eingefrorenen Profil `g4-noise-slip-v1` (Seed `20261008`, Lidar σ=`0,01 m`, Gyro σ=`0,01 rad/s`, Beschleunigung σ=`0,1 m/s²`, Rad-μ=`0,8`). Run 45 (`log/g4_runtime/g4-nav-20261008-163511-FBup3W/`) bestand mit derselben Karte/demselben Profil das unreachable-Szenario fail-closed. Beide Bags/Reports sind validiert; keine Prozesse übrig. G4 ist abgenommen; Grenzen bleiben unverändert.
- **G5:** Off/Mock-Adapter und Nav2-Vergleich im SIL bestanden (siehe Punkt 3). Der lokale G6-Foto-Capture ist ebenfalls bestanden; externe Live-Inferenz bleibt separat gesperrt. Keine Motoraktion durch G5/G6 freigegeben.
- **Historischer G6-Zwischenstand vor dem lokalen Foto-Capture am 13.10.2026,
  17:59:** Der lokale `.71`-Zwilling war im Loopback und mit einem bekannten
  Referenzbild geprüft; die Simulationsperson war sensorisch sichtbar. Zu
  diesem Zeitpunkt fehlte ein positiver Detektionsnachweis aus der
  Simulationskamera. Der spätere Einbild-PASS und der aktuelle Gate-Status
  stehen in der priorisierten Restliste unter Punkt 5. Der separate externe
  `.71`-Lauf auf einem archivierten Standardweltbild lieferte null Detektionen;
  er bleibt kein positives Personen-Gate und kein Nachweis für `.150`.
- **G7/G8:** nicht freigegeben. G7 wartet mindestens auf eine unabhängige
  Host-Key-Verifikation; physische Tests benötigen eigene Sicherheitsprüfung und
  Freigabe. SIL-Geometrie/-Limits sind keine Hardwaremesswerte.

## Historischer G4-Abnahmevertrag (für Nachvollziehbarkeit)

Dieser Abschnitt hält den G4-Vertrag und die vor G4 identifizierten Produktfragen historisch fest; G4 ist mit Runs 42–45 im SIL bestanden. Noch nicht definierte Budgets sind keine offene SIL-Abnahme und dürfen nicht als Hardware-Sicherheitsgrenzen interpretiert werden. Das eingefrorene Noise-/Slip-Profil ist SIL-only.

| Szenario | Vereinbarter Sollausgang | Karten-/Bewertungsquelle | Damals offene Produktfestlegungen |
| --- | --- | --- | --- |
| Freie Fahrt | Nav2 `SUCCEEDED`; unabhängige Ground-Truth-Endabweichung höchstens `0,20 m`; finaler Nullbefehl und gemessener Stillstand | Run-25-Fixture bleibt reproduzierbare Baseline. Für einen Transfernachweis muss die Navigation zusätzlich auf einer sensorbasierten SLAM-Karte ohne Ground-Truth-Freimarkierung belegt werden. | `0,20 m`, `0,08 m/s` und `0,30 rad/s` sind aktuelle Harnessgrenzen. Szenario-Deadline und genaue Stoppbeobachtung für diesen Erfolgsfall sind noch nicht als Produktanforderung festgelegt. |
| Hindernisumfahrung | Wie freie Fahrt; zusätzlich unabhängige Trajektorien-/Clearance-Auswertung und belegter Umweg um die Hindernisgeometrie | Fixture-Baseline und sensorbasierter Transferfall getrennt berichten; Ground Truth nur zur Bewertung, nicht zum Freischalten von Kartenfeldern | `0,15 m` Mindest-Robot-Clearance ist im aktuellen Prüfer verwendet; Run 25 maß `0,21999 m`. Hindernis-/Footprintquelle, Sicherheitsmarge und Zeitbudget müssen als Anforderungen bestätigt werden. |
| Enge Passage | Nav2 `SUCCEEDED`, keine Kollision, unabhängige Pose-/Trajektorienmessung und gemessene Mindest-Clearance | G4-eigene Fortress-Welt plus sensor-derived SLAM-/Nav2-Karte; Ground Truth ausschließlich zur Bewertung | Geometrie seit 08.10.2026 hergeleitet statt gepinnt: `W = 2 × (0,22 m Radius + 0,15 m Clearance + 0,13 m seitliches Lokalisierungs-/Trackingbudget) = 1,00 m`, `y=−1,60…−0,60 m`. Die frühere Ein-Zellen-Reserve (`0,84 m`) ließ nur ±0,05 m seitliche Abweichung zu und war in Runs 37–41 nicht erreichbar. Budget ist praktikabler Startwert; es wird bei Noise-/Slip-Arbeit neu bewertet. |
| Unerreichbares Ziel | Action angenommen, zielbezogener Plannerfehler und terminal `ABORTED`; fail-closed ohne absichtliche Nichtnullbewegung durch Recovery. Gemessener Stillstand; falls Nichtnullbefehle ausgegeben wurden, ist finaler Nullbefehl Pflicht. | Karte muss das physische Ziel als unbekannt/belegt ausweisen; kein Out-of-map- oder TF-/Costmap-Fehler als Ersatz für den Negativfall | Run 26 belegt `CANCELED` nach Überschreiten des Guards `0,25 m`, während Nav2-Recovery BackUp bis `0,30 m` und bis zu sechs Wiederholungen erlaubt. `0,25 m` nicht erhöhen oder übernehmen. Zulässige Ground-Truth-Abweichung und Deadline vor Implementierung festlegen; bestehende `0,02 m` Stillstandstoleranz ist nur Kandidat. |

### Übergreifende Vertragsentscheidungen

1. **Kartenbeweis:** Run 25 bleibt als GT-assistierte Fixture-Baseline gültig, aber nicht als sensor-only Transferbeweis. Vollständiges transferorientiertes G4 verlangt zusätzlich einen getrennten SLAM-/Nav2-Nachweis, dessen Navigationskarte nicht mit Ground-Truth-Posen oder -Samples ergänzt wurde. Rohkarte, Ableitung, Hashes und Bewertungsdaten getrennt dokumentieren.
2. **Recovery:** Ein laut Karte unbekanntes/belegtes Ziel endet goal-spezifisch fail-closed ohne Bewegungs-Recovery; normale Nav2-Recovery bleibt für andere Fehler/Szenarien erhalten. Die Probe verlangt Nullbefehle, falls Bewegungskommandos ausgegeben wurden; bei keinem ausgegebenen Bewegungsbefehl belegen frische Ground-Truth-Stillstandssamples den No-motion-Fall. |
3. **Rauschen/Schlupf:** Der reproduzierbare Lidar-/IMU-Rausch- und vereinfachte Schlupffall ist in Runs 44/45 mit eingefrorenem Modell/Seed bestanden. Das Profil ist SIL-only; ein Hardwaremodell oder physischer Parameternachweis folgt daraus nicht.
4. **Aufzeichnung/Replay:** `test_phase4_gate.sh` startet einen verwalteten rosbag2-Prozess mit G4-Sensor-/TF-/Odom-/Costmap-/Plan-/Befehlsdaten. Metadaten und Nachrichteninhalt sind in Runs 44/45 runtime-validiert. Eigenständige Replay-Abnahme bleibt Folgearbeit und ist kein zusätzliches G4-Szenario.
5. **Budgets:** Ziel-Deadlines, erlaubte Recovery-Auslenkung, Clearance und Stoppnachweis pro Szenario vorab aus Produktanforderung und vermessener Simulationsgeometrie ableiten. Vorhandene Harness-Timeouts sind Schutzfristen, nicht automatisch Leistungsziele. Kein Grenzwert darf nach Sichtung eines neuen Ergebnisses angepasst werden.

### Historische G4-Arbeitsrichtung (seit Runs 42–45 abgeschlossen)

- **Recovery:** Ein Ziel, das laut Karte in unbekanntem oder belegtem Raum liegt, endet fail-closed ohne Bewegungs-Recovery (`ABORTED` nach zielbezogenem Plannerfehler). Normale Nav2-Recovery bleibt für andere Fehler/Szenarien erhalten. Kein BT allein zum Erreichen eines PASS. Bei keinem Nichtnullbefehl gilt gemessener Stillstand als Stop-Beleg; falls ein Nichtnullbefehl auftritt, muss danach explizit Null folgen. Die zulässige Ground-Truth-Abweichung wird anhand der Simulationsauflösung festgelegt.
- **Kartenbeweis:** Sensor-only SLAM-/Nav2 ist Voraussetzung für transferorientiertes vollständiges G4. Run 25 bleibt als Fixture-Baseline gültig und separat ausgewiesen.
- **Robustheit:** Ein reproduzierbarer, kontrolliert parametrierter Lidar-/IMU-Rausch- und Schlupffall gehört zu G4. Parameter und Seed müssen vor dem Lauf begründet und eingefroren sein.
- **Replay:** rosbag-Aufzeichnung pro Lauf ist im Harness implementiert; der nächste integrierte Lauf muss ein valides Bag beweisen. Eine eigenständige Replay-Abnahme ist kein zusätzliches G4-Szenario und bleibt Folgearbeit.
- **Unveränderte Grenzen:** Die vorhandenen `0,20 m` Zielabweichung, `0,15 m` Robot-Clearance und `0,08 m/s` / `0,30 rad/s` Geschwindigkeitslimits bleiben für die bestehende Simulationsbaseline eingefroren, nicht als physische Hardware-Sicherheitswerte. Noch nicht begründete Passagegeometrie, Recovery-Bewegungsgrenze, Noise-/Slip-Parameter und Szenario-Deadlines werden nicht erfunden.

### Historischer G4-Quellenabgleich und Versuchsstand

- Die allgemeine Fortress-Testwelt enthält unverändert eine statische Box mit Größe `1,0 × 1,0 × 1,0 m` bei `(2,0; 0; 0,5) m`. Eine separate `fortress_g4_test.sdf` ergänzt den Westanker und zwei Wände, die ein nur frontal durchfahrbares Gate bei `x=4,0 m` bilden (seit 08.10.2026 `1,00 m`, `y=−1,60…−0,60 m`; Runs 29–41 nutzten `0,84 m`, `y=−1,52…−0,68 m`). Zwei-Wand-Geometrie und Modellnameneindeutigkeit bestehen Fortress-SDF-Parsing und ROS-freie Regression; die Testsuite prüft SDF-Wandkanten gegen `phase4_geometry.py`. Der Fortress-Roboter hat im Xacro eine Basis von `0,22 × 0,18 m`; Nav2 verwendet einen Kreisradius von `0,22 m`. Das sind SIL-Konfigurationswerte, keine physisch vermessenen Maße.
- Die Run-25-Rohkarte hat `origin.x=0,02 m`; damit liegt die Startpose `x=0` außerhalb, während die Zellen ab etwa `x=0,10 m` entlang der Startlinie bekannt/frei sind. Die Fixture schiebt den Origin auf `−0,28 m` und markiert den fehlenden Korridor GT-assistiert frei. Die damalige G4-Fixture `fortress_g4_test.sdf` ergänzt einen statischen, vom Lidar sichtbaren Westanker bei `x=−2,0 m`; `fortress_test.sdf` wurde bei dieser G4-Änderung nicht verändert. Am 13.10.2026 wurde dort separat der Standard-Personenproxy ergänzt. Das soll echte Messstrahlen zum Kartieren der westlichen Freifläche liefern, ist aber noch nicht runtime-bestätigt.
- Nav2-/SLAM-Kartenauflösung ist `0,05 m`; die Mindest-Clearance ist `0,15 m`. Der frühere Geometrie-Kandidat `W = 2 × (0,22 + 0,15 + 0,05) = 0,84 m` ließ bei idealer Mittellinie `0,20 m` Clearance, also nur eine Rasterzelle seitliche Toleranz. Runs 40/41 zeigten, dass AMCL-/Controller-Abweichung (`≈5–6 cm`) plus Rasterung dieses Budget sprengt. Seit 08.10.2026 gilt `scripts/phase4_geometry.py` als einzige Quelle: `NARROW_PASSAGE_LATERAL_BUDGET = 0,13 m` ⇒ Halbbreite `0,50 m`, Gate `1,00 m`; SDF-Wände, Verifier und Tests leiten sich daraus ab. Nav2-Costmaps verwenden den physischen Radius `0,22 m` wie `navigation.yaml` (Hardware) und eine weiche Inflation `0,45 m` unterhalb der Halbbreite; Clearance wird weiterhin unabhängig geprüft, nicht über einen künstlich vergrößerten Planner-Radius. Das Passage-Fixture muss Umfahrung außen verhindern, ausreichend lange Ein-/Ausläufe für SLAM und Lidar-Sicht enthalten und GT-Clearance unabhängig auswerten.
- **Robustheitsprofil runtime-validiert (Runs 44/45):** Fortress loggte `G4_RANDOM_SEED=20261008`; beide Laufmanifeste stimmen bytegleich mit dem Profil überein. Run 44 Mapping: 63 Map-Updates, 1.026→12.013 bekannte Zellen, 513 belegt, alle acht Segmente bestanden, Mindest-Clearance `0,1809 m`. Start-Footprint-Seed: 3 Unknown-Zellen, 0 belegte entfernt, keine GT-Navigationseingabe. Ziele: frei `SUCCEEDED`/GT `0,1100 m`; Umfahrung `SUCCEEDED`/`0,1091 m`, Plan-/GT-Clearance `0,2177/0,2534 m`, maximale Samplelücke `0,00672 m`; Passage `SUCCEEDED`/`0,1129 m`, Plan-/GT-Clearance `0,2550/0,2045 m`, genau eine Gatequerung, Samplelücke `0,00650 m`. Alle drei mit finalem Null und je 8 Nach-Result-Stillstandsproben (max. Drift `0,00251/0,00207 m`, Toleranz `0,02 m`). Run 44 Bag: `88,6 MiB`, `505.889` Nachrichten. Run 45: Action angenommen, GridBased-Fehler zum physischen Ziel, `ABORTED`, 0 Plan-/`/cmd_vel`-Samples, 7 GT-Stillstandsproben, maximale Bewegung `0 m`, Clearance `1,28 m`; Bag `2,4 MiB`, `13.498` Nachrichten. Quellen zur Sensor-/Reibungs-/Seed-Semantik: [SDFormat-Sensorschema](https://sdformat.org/spec/1.8/sensor/), [SDFormat-Kontakt/Reibung](https://sdformat.org/spec/1.8/collision/), [Fortress-Systemreihenfolge](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/SystemManager.cc), [Fortress-CLI-Optionen](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/cmd/cmdgazebo.rb.in), [GaussianNoise](https://github.com/gazebosim/gz-sensors/blob/ign-sensors6/src/GaussianNoiseModel.cc), [Ignition-Math-RNG](https://github.com/gazebosim/gz-math/blob/ign-math6/src/Rand.cc). Profil ist SIL-only, kein Hardwaremodell; G3-Yaw-Bias bleibt getrennte historische Evidenz.
- **Run 29 (08.10.2026, `g4-nav-20261008-101751-gRI4L7`):** SLAM-Mapping **PASS** (3.144 neue bekannte Zellen, 3.768 bekannte Zellen, 171 belegte Zellen; alle acht Mappingsegmente bestanden, Mindest-Clearance `0,302 m`). Map-Preparation **PASS**. Map-Reuse/Nav2-Readiness **FAIL**: sechs Lifecycle-Nodes blieben inaktiv; keine Goals. Ursache: Default- und G4-Fortress starteten gleichzeitig; Nav2 protokollierte TF-Zeitsprünge. Doppelstart entfernt und regressionsgeprüft.
- **Run 30 (08.10.2026):** Mit einem Fortress-Server bestanden G4-World-Readiness, Mapping und sensor-derived Map-Preparation. Goal-Preflight brach vor Nav2-Goal/Fahrbefehl ab: `(0,025; 0)` war Unknown, weil die Zelle unter dem Roboter-Footprint vom Lidar nicht beobachtbar ist. Kein G4-PASS. Run-Dateien waren in späteren VM-Shells nicht verfügbar; Terminalbericht bewahrt Fehlerursache und Mappingzusammenfassung.
- **Implementiert/runtime-geprüft:** `prepare_phase4_map.py --mode sensor-seeded` normalisiert Map-Saver-Unknown und füllt Ränder mit Unknown. Nur unbekannte Zellen im konfigurierten Start-Footprint dürfen frei werden; Quelle Spawn-/AMCL-Initialpose, nie Ground Truth. Occupied-Zellen führen fail-closed zum Fehler; kein Fahrkorridor wird ergänzt. Unreachable-only BT enthält keine Recovery-Bewegungen; Verifier lehnt jedes Nichtnull-`/cmd_vel` ab. Harness nutzt Seed standardmäßig, BT nur für unreachable, GT-Fixture nur explizit. Run 33/34 runtime-pass belegen die No-motion-Semantik. 74 Readiness-/Phase-4-Tests, XML-/Shellsyntax, Paketbuilds und Fortress-Parser bestehen.

- **Run 31 (08.10.2026, `g4-nav-20261008-103921-jsh2nR`):** Der `sensor-seeded`-Pfad bestand Mapping, Map-Preparation, Map-Reuse/Nav2-Readiness und beide erreichbaren Goals; Harness `PASS [G4_SIMULATION_ONLY]`. JSONs gingen mit Ende der VM-Session verloren; nicht als persistente Evidenz verwenden.
- **Run 32 (08.10.2026, archiviert unter `log/g4_runtime/g4-nav-20261008-105004-6Z7nA4/`): PASS für die zwei erreichbaren Szenarien.** TF2, World-/Nav2-Readiness, Mapping und Map-Preparation bestanden. Rohkarte `106×80`, 3.727 bekannte/165 belegte Zellen, 3.103 neu bekannt, min Mapping-Clearance `0,2947 m`. Prepared map `116×90`, Origin `[-2,23; -2,23]`, 11 Start-Footprint-Unknowns aus Spawn-/AMCL-Pose ergänzt; `ground_truth_used_for_navigation_map=false`, `occupied_cells_removed=0`. Freies Ziel `(0,55;0)` `SUCCEEDED`, GT-Abweichung `0,0713 m`, 106 Samples, finaler Nullbefehl. Umfahrung `(2,80;-1,05)` `SUCCEEDED`, GT-Abweichung `0,0856 m`, 547 Samples, finaler Nullbefehl; global-plan-Clearance `0,1713 m`, ausgeführte Clearance `0,1847 m`, max. Trajektorienlücke `0,0071 m`. Geschwindigkeiten `0,08 m/s`/`0,30 rad/s`, keine Fehler. Sensor-derived Zwei-Szenarien-Teil-PASS, nicht vollständiges G4.
- **Run 33/34 (08.10.2026): PASS für das unerreichbare Ziel.** Run 33 ist archiviert unter `log/g4_runtime/g4-nav-20261008-110604-t6UVIt/`; Run 34 bestätigte dasselbe fail-closed Verhalten. Action angenommen; physisches Boxziel `(2,0)` in der sensor-derived Map Unknown; spezifischer `GridBased`-Plannerfehler; terminal `ABORTED`. Fail-Closed-BT gab null `/cmd_vel`-Samples aus; gemessene Maximalbewegung `0 m`, sieben/acht Stop-Samples ohne Drift (Toleranz `0,02 m`), Clearance `1,280 m`; keine TF-/Costmap-Störung. Ein finaler Nullbefehl war nicht nötig, weil kein Bewegungsbefehl gesendet wurde. Damit drei Szenarien sensor-derived bestanden; enge Passage fehlt.
- **Run 35 (08.10.2026, `log/g4_runtime/g4-nav-20261008-112045-GhzX6B/): FAIL, kein Retry.** Sensor-derived Mapping aller acht Segmente **PASS**, 11.073 neue bekannte Zellen, 63 Map-Updates; 5-m-Fernseitenfahrt durch die vorgesehene Gate-Linie; Map-Aufbereitung und Map-Reuse/Nav2-Readiness **PASS**, zwei footprint-Unknown-Zellen aus der initialen Spawn-/AMCL-Pose freigegeben, keine Occupied-Zelle entfernt, alle Lifecycle-Nodes aktiv. Das erste freie Nav2-Ziel meldete `SUCCEEDED`, unabhängige Enddistanz `0,1064 m` (Limit `0,20 m`), 95 Trajektoriensamples, TF/Footprint frisch; Report sah jedoch null Nullbefehle **nach** Action-Terminalstatus und stoppte vor Umfahrung/Passage. Ursache der Timingbedingung: Command-Timeout publiziert Null nach `0,2 s` ohne neuen Bewegungsbefehl, sodass Null vor dem Action-Ergebnis liegen kann; Run 35 enthält keine vollständige Befehlsreihenfolge-/Nach-Ergebnis-Stillstandsserie, daher ist die Stop-Evidenz nicht rückwirkend beweisbar. Verifier-/Regression geändert: finales Null nach dem letzten empfangenen Nichtnullbefehl plus mindestens drei Ground-Truth-Stillstandssamples nach Terminalstatus (Drift höchstens `0,02 m`); auch nach Action-Ende empfangene Nichtnullsamples müssen von final Null gefolgt werden. Keine Ziel-/Grenzänderung. Folge-Integration läuft nicht automatisch.
- **Run 36 (08.10.2026, `log/g4_runtime/g4-nav-20261008-115346-Iz7h2q/): FAIL, keine Ziele 2/3.** Sensor-derived Mapping (11.010 neue bekannte Zellen; acht Segmente), Map-Preparation und Nav2-Readiness **PASS**. Nav2 meldete für das freie Goal `SUCCEEDED`; der damalige Verifier brach an einem nach Action-Ende **empfangenen** Nichtnull-`/cmd_vel`-Sample ab. Da der Code bei dieser Prüfung weder die Befehlszeitreihe noch Stop-Samples in den Fehlerreport kopierte, lässt sich nicht mehr unterscheiden, ob das ein nachlaufender Transport-Sample oder ein echter späterer Befehl war. Keine Umfahrung/Passage; alle Prozesse danach beendet. Für Run 37 wurden die letzten 20 Befehle und post-result Ground-Truth-Samples persistiert; Stop-PASS verlangt Null nach letztem empfangenen Nichtnullbefehl und drei Stillstandssamples (≤`0,02 m` Drift). Ein Nichtnull-Sample nach Action-Ende ist zulässig, wenn danach finales Null und unabhängiger Stillstand belegt sind. ROS-freie Suite: 68 Tests; keine Ziel-/Grenzänderung.
- **Run 37 (08.10.2026, `log/g4_runtime/g4-nav-20261008-120221-smzYbc/): FAIL an der Kartenbeweisprüfung, alle drei Actions erfolgreich.** Sensor-derived Mapping **PASS** (11.078 neue bekannte Zellen, 63 Map-Updates, alle acht Segmente; 5-m-Gate-Abdeckung); Map-Preparation, Reuse und Nav2-Readiness **PASS**. Freies Ziel / Umfahrung / Passage jeweils Nav2 `SUCCEEDED`, unabhängige GT-Abweichungen `0,0821 / 0,0820 / 0,0296 m`. Stopproof für alle drei bestanden: finales Null nach der letzten empfangenen Bewegung; 7–10 post-result Nullsamples und 7–8 Stillstandsposen; maximale Drift `0,00069 m`. Der Gesamtverifier stoppte danach, weil der ausgewählte globale Passageplan bei `(3,989; −1,103)` eine Zelle der gespeicherten sensor-derived Karte mit Occupancy `−1` quert. Die Passage-Plan-/ausgeführte-Clearance-Auswertung wurde dadurch nicht abgeschlossen; Nav2-Ankunft ist **kein** vollständiger Passage-PASS. Kein aktueller Map-Korridor wurde GT-seitig freigeschaltet, keine Ziel-/Geometrie-/Grenzwertänderung und kein Prozessrest. Gespeicherte Karte zeigt benachbarte Zeile `y≈−1,075 m` als bekannt-frei, aber einzelne rasterisierte Planzelle darunter Unknown; zu klären ist, ob SLAM-Coverage oder Abweichung zwischen statischer Karte und live sensor-cleared Global-Costmap vorliegt. Nicht Unknown wegcarven und nicht den Clearance-/Passagecheck lockern.
- **Run 38 (08.10.2026, `log/g4_runtime/g4-nav-20261008-122822-xD45du/): FAIL an der Umfahrungs-Plan-Clearance, rosbag2 validiert.** Sensor-derived Mapping (11.080 neue bekannte Zellen), Map-Preparation und Nav2-Readiness **PASS**. Freies Goal `SUCCEEDED` (`0,0689 m`), Umfahrungs-Goal `SUCCEEDED` (`0,1148 m`) und beide Stopproofs bestanden. Globalplan war `2,744 m` lang, Minimum-Clearance `0,14136 m` bei `(1,3744;−0,8388)`: um `0,00864 m` unter der unveränderten G4-Forderung `0,15 m`; Gesamtgate korrekt **FAIL vor Goal 3**. Keine Nav2-TF-/Costmapfehler. Bag `75,6 MiB`, `438.753` Nachrichten, inklusive 106 Plans, 43 Global- und 90 Local-Costmap-Snapshots; Metadaten/Topics lesbar. Read-only Bag-Analyse (Bagzeit statt zurückgesetzter Sim-Clock gepaart) zeigt am Planzeitpunkt globalen Costmap-Snapshot `+0,071 s`, mit Publisher `global_costmap`; statische SLAM-Unknown-Zellen entlang Teilen der Route waren im Live-Costmap tatsächlich `free` (sensor-cleared). Die zu geringe geometrische Clearance bleibt der reale Blocker; keine Grenz-/Goal-/Geometrieänderung. Systemischer Kostenfeldradius nun abgeleitet: `ceil((0,22 + 0,15 + 0,0354)/0,05) × 0,05 = 0,45 m`; Global- und Local-Inflation einheitlich auf `0,45 m`, Goal-/Clearancegrenzen unverändert. Humble-Paketbuild **PASS**, Quell-/Installationshash identisch; 72 ROS-freie G4-Tests bestehen. Einmaliger Integrationsnachweis dieses Kostenfeldprofils samt Bag/Costmap bleibt offen.
- **Run 39 (08.10.2026, `log/g4_runtime/g4-nav-20261008-130020-GBYDdw/): FAIL an Passagezeit/Costmap-Fehlplanung.** Mapping (10.953 neue Zellen), Map-Preparation, Nav2-Readiness und Bag-Pfad liefen; Ziele 1–2 `SUCCEEDED` (`0,0996 / 0,1305 m`) samt Stop. Passage nach 180 s **Timeout**; GT-Fortschritt `0,970 m` über `1,089 m` Pfadlänge, drei Controller-`Failed to make progress`-/Costmap-Clear-Zyklen, aber frische Clock/TF/Scan/Footprints (keine TF-/Out-of-bounds-Störung). rosbag2 `109,7 MiB`, `614.912` Nachrichten, Metadaten valide. Das Gate ist `0,84 m` breit (`halbe Öffnung=0,42 m`); Run-39 Global-/Local-Inflation `0,45 m` überdeckt rechnerisch die Halböffnung. Gatequerschnitt im planzeitlichen Costmap hatte keine niedrigen Kosten durch die Mitte (Kosten ca. `67–100`), was mit dem fehlenden Controllerfortschritt zusammenfällt. Die Passage-Trajektorie blieb vor dem Ziel; kein PASS. Korrektur aus fixierter Geometrie/Raster, nicht Mikrotuning: `inflation_radius = floor(0,42/0,05) × 0,05 = 0,40 m` lokal und global, `robot_radius=0,22 m` und Clearance `0,15 m` bleiben separat/unverändert. Costmap-Gridsemantik regressionssicher präzisiert: 50–99 sind nichtlethale Inflationskosten, `100` lethal, `−1` Unknown; geometrischer Clearancecheck bleibt maßgeblich. Humble-Paketbuild/hash und 73 ROS-freie Tests bestehen; einmaliger Runtime-Beleg offen.
- **Run 40 (08.10.2026, `log/g4_runtime/g4-nav-20261008-132408-bXGOOq/): FAIL, G4 nicht abgenommen.** Mapping (11.063 neue bekannte Zellen), Map-Preparation, Map-Reuse/Nav2-Readiness und rosbag2 **PASS**. Alle drei Nav2-Actions `SUCCEEDED` (GT-Zielabstände `0,0509 / 0,0727 / 0,0269 m`), alle Null-/Stillstandsprüfungen bestanden. Tatsächliche Ground-Truth-Passagequerung genau einmal bei `(4,0; −1,1643)` hatte nur `0,13568 m` clearance `<0,15 m`; der globale Plan kreuzte bei `y=−1,14962` mit `0,15038 m` am rechnerischen Limit. Außerdem lagen 3 Planabtastungen (`x≈3,688; y≈−1,218`) im zur Planzeit empfangenen Global-Costmap noch Unknown (`−1`; Costmapalter `1,001 s`, Scanalter `0,002 s`), so kein bekannter-freier Planbeleg. Kein TF-/boundsfehler. rosbag2 **PASS**: `92,6 MiB`, `523.565` msgs, 199 `/plan`, 83 Global-/160 Local-Costmap-Snapshots; Prozesse bereinigt. Die konservative Nav2-Hülle `robot_radius=0,395 m` und Inflation `0,40 m` waren gebaut, garantierten aber keinen clearance-zertifizierten Pfad. 74 ROS-freie Tests bestanden vor Planerwechsel. `0,15 m` Clearance bleibt unverändert.
- **Run 41 (08.10.2026, `log/g4_runtime/g4-nav-20261008-141555-jFeEaH/): FAIL, keine Passage, keine Wiederholung.** Sensor-derived Mapping/Preparation, Map-Reuse/Nav2-Readiness und Smac-Plugin-Laden **PASS**. Freies Ziel `SUCCEEDED`, GT-Abstand `0,08796 m`, 7 Nullsamples nach Resultat. Umfahrung `SUCCEEDED`, GT-Abstand `0,09753 m`, 8 Nullsamples; Pfad-Clearance `0,16349 m`, GT-Trajektorien-Clearance `0,21880 m`, Max-Samplelücke `0,00653 m`. Passageziel `(4,80;−1,10)` endete nach Recovery/Timeout als Action `ABORTED`; kein `/plan`-Sample während des Passagegoals (`planner_path_samples_during_goal=0`), Plannerlog wiederholt `no valid path found`. Finaldiagnostik meldete frische Scan-/Costmap-/TF-Updates, keine Bounds-/Hardwarestörung. rosbag2 ist lesbar: `517.666` Nachrichten, `79` globale Costmap-Snapshots; read-only Inspector fand 8 zum Umfahrungsziel passende Plans. In den letzten 25 Costmap-Snapshots vor Laufende waren 7 geprüfte Zellen bei `x=4,0 m`, `y=−1,25…−0,95 m` durchgehend Occupancy-Wert `99`. Humble `Costmap2DPublisher` bildet internen `INSCRIBED_INFLATED_OBSTACLE=253` exakt auf Occupancy `99` ab; Smac `GridCollisionChecker` verwirft im Radiusmodus Kosten `≥253`. Das konservative `0,395 m`-Radiusmodell hatte damit in der Gate-Costmap keinen traversierbaren Mittelstreifen. Diese Evidenz erklärt den fehlenden Pfad im getesteten Profil; sie rechtfertigt weder eine Geometrie-/Grenzlockerung noch den nächsten Zahlenversuch. Kein Plan-/Trajektorien-Clearance-PASS für die Passage.

**Ergebnis Architekturversuch (Run 41):** SmacPlanner2D mit künstlich vergrößertem Planner-Radius `0,395 m` war nicht abnehmbar: Humble publiziert internen Cost `253` (`INSCRIBED`) als Occupancy `99`, der Radius-Checker verwirft ihn, und der `0,84 m`-Gatekorridor war durchgehend `99`. Daraus abgeleitete, praktikable Entscheidung (08.10.2026, Nutzer: „praktikable Ansätze, Constraints können weiterentwickelt werden“): Planner-Footprint = physischer Radius `0,22 m` (identisch mit `navigation.yaml` für Hardware), Inflation als weiche Kostenschicht `0,45 m` unterhalb der Gate-Halbbreite, SmacPlanner2D mit `cost_travel_multiplier=2.0` (Humble-Node2D-Default) und `allow_unknown=false`; Passagebreite aus Radius + Clearance + explizitem seitlichem Budget `0,13 m` hergeleitet (`1,00 m`). Clearance `0,15 m` und Zieltoleranz `0,20 m` blieben unverändert.

- **Run 42 (08.10.2026, `log/g4_runtime/g4-nav-20261008-145048-k4lioS/`): PASS, drei erreichbare Szenarien.** Sensor-derived Mapping (Mindest-Clearance `0,193 m`), Map-Preparation, Map-Reuse/Nav2-Readiness, SmacPlanner2D geladen. Freies Ziel `SUCCEEDED` GT `0,1085 m`; Umfahrung `SUCCEEDED` GT `0,1205 m`, Plan-Clearance `0,2011 m`, GT-Trajektorie `0,2405 m`; Passage `SUCCEEDED` GT `0,0442 m`. Plan quert das Gate genau einmal bei `y=−1,075` (Clearance `0,255 m`), planzeitlicher Live-Costmap bekannt-frei (`0` Unknown, 164 freie/12 inflationskostige Samples, Costmapalter `0,66 s`, Scan `0,24 s`). GT-Querung nachgerechnet: genau einmal bei `y=−1,1002`, Clearance `0,2798 m`; GT-Mindest-Clearance der ganzen Passagefahrt `0,2727 m`. Alle drei Stopproofs: finales Null, 7 Stillstandssamples, Drift `0,0 m`. rosbag2 `512.753` Nachrichten / `89 MiB`. Keine Prozessreste. Build-Hashes: SDF `2bfec5fb5ec9f49e…`, Config `8495c1bf7144f888…`.
- **Run 43 (08.10.2026, `log/g4_runtime/g4-nav-20261008-150754-HJjSQk/`): PASS, unerreichbares Ziel mit derselben Konfiguration und der Run-42-Karte.** Ziel `(2,0; 0)` in der Karte Unknown; Action angenommen, zielbezogener `GridBased`-Plannerfehler, terminal `ABORTED`; `0` `/cmd_vel`-Samples, `0` Pläne, 7 GT-Samples ohne Bewegung; TF/Footprints frisch. rosbag2 `13.518` Nachrichten.

**Nächste umsetzbare Arbeit:** siehe die untenstehende sequenzielle Restliste. G4 und der reproduzierbare Robustheitsnachweis sind abgeschlossen; kein weiteres G4-Tuning.

Runs 35–41 bleiben als FAIL archiviert; Run 25 bleibt Fixture-Baseline. Quellen: [SmacPlanner2D (Humble)](https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/smac_planner_2d.cpp), [CollisionChecker (Humble)](https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/collision_checker.cpp), [Node2D weighted cost (Humble)](https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/node_2d.cpp), [Costmap publisher mapping 253→99 (Humble)](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_2d_publisher.cpp).

## Historische Arbeits- und Gate-Nachweise

Die folgenden Abschnitte halten frühere Schritte, Runs und Befunde fest. Sie
sind kein alternativer Startplan; die aktuelle offene Reihenfolge steht oben.

1. **Nav2-Konfiguration aufteilen — erledigt (09.10.2026).** Gemeinsame Quelle `navigation_common.yaml` und profil-vorrangige Launch-Zusammenführung sind implementiert. Geteilt werden nur der Planner-Alias `GridBased` und belegte Costmap-Werte; konkrete Planner, Geschwindigkeit, Frames, `use_sim_time`, Inflation und Unknown-Policy bleiben profilabhängig. Effektives Hardwareprofil ist strukturell identisch zum Vorher-Stand. TF2-Overlay wurde unter dem historischen Verzeichnisnamen aktiviert und per Commit-/Library-Preflight bestätigt. Zwei erste SIL-Versuche schlugen vor Nav2-Readiness mit `FileNotFoundError` für Launch-/Profildateien fehl; behoben durch Komposition in der Haupt-Launchdatei, Kopieren der Nav2-YAMLs in den Linux-Buildbaum, absoluten Share-Default und explizite Common-Datei im G4-Harness. **Erfolgreicher Lauf** `g4-nav-20261009-131721-SRrY89`: Mapping **PASS** (64 Updates), alle zehn Nav2-Lifecycle-Nodes **active**, freie Fahrt/Umfahrung/enge Passage jeweils `SUCCEEDED` (GT-Zielabweichung `0,0670/0,0840/0,0693 m`), Stopproofs bestanden (max. Drift `0,00103 m`), rosbag2 validiert (`91,6 MiB`, `521.328` Nachrichten), keine Prozesse übrig. Profil-SIL-Regression ohne Ziel-/Grenztuning bestanden; dies ist kein neuer unreachable-/Noise-Slip-Voll-G4-Nachweis. Humble-Build und 354 ROS-freie Tests bestanden.
2. **G5-ROS-Adapter mit Mock — Implementierung/Transport-Smoke erledigt (09.10.2026).** `linorobot2_vision` enthält `vision_adapter_node` mit `off`/Fixture-Mock, gepaartem Image-/CameraInfo-Eingang, Capture-Zeit-/Frame-/Dimensions-/Labelvalidierung, Contract-v1-JSON auf `/vision/result` und separatem Fehlerstatus auf `/vision/status`. Default-Mock ist eine gültige leere Fixture; keine Inferenz und kein `/cmd_vel`. Humble-Build, `ros2 run`-Discovery, Off-/Mock-Start/Shutdown und ROS-Transporttests bestanden; Gesamtsuite 362 Tests. Vollständige G5-Off/Mock-Abnahme: Punkt 3 (PASS).
3. **G5-Abnahme und Replay-Grundlage — G5 PASS und initiales Replay ausgeführt (09.10.2026).** Der sequentielle SIL-Vergleich `g5-nav-20261009-143456-QVT85c` bestand mit identischer sensor-derived Map sowie identischen Fortress-/Nav2-Profilhashes: Vision `off` meldete `unavailable`, Mock lieferte valide Fixture-Detections; freie Fahrt, Umfahrung und enge Passage jeweils `SUCCEEDED` in beiden Modi. Zielabweichungen Off/Mock: frei `0,07393/0,07337 m`, Umfahrung `0,09485/0,07248 m`, Passage `0,06702/0,03538 m`; finale Nullbefehle, bestehende Stop-/Clearance-Gates im Verifier bestanden. Bags gültig: `35,2 MiB/181.657` und `35,5 MiB/182.685` Nachrichten. Keine Inferenz, Hardware oder Vision-`/cmd_vel`-Publisher. **Replay ausgeführt:** `scripts/create_replay_segment.py` erzeugte und validierte das synthetische Segment `vision-transport-v1` (91 Nachrichten, Manifest mit SHA-256); `scripts/test_replay_gate.sh` bestand Domain-43-Replay je einmal mit `mock` (`replay-20261009-145320-19HHgD`, 3 Captures, 1 Detection) und `off` (`replay-20261009-145336-JRS6Nb`, `unavailable`); einziger `/clock`-Publisher war `rosbag2_player` (RELIABLE), keine Prozessreste. Vertrag: `docs/REPLAY_ROS_CONTRACT.md`. Verbleibende Folgearbeit: echte Segmentierung großer G4-/G5-Bags und ein späterer separater Nav2-Replay sind nicht damit abgenommen.
4. **G3-Rohodometrie-Yaw-Bias — eingegrenzt (09.10.2026).** Analyse in `docs/G3_YAW_BIAS_ANALYSIS.md`: konstanter multiplikativer Fehler 1,2143 nur in Scrub-Drehungen, richtungsunabhängig, IMU=GT, Gelenke tracken Kommando, Geradeausfahrt unverzerrt. Probe `wheel_mu=2.0` ohne Effekt; Caster mu=0,01. Hauptthese: Fortress/DART-Kontaktconstraint-Artefakt bei Skid-Turns. Faktor ist SIL-Artefakt-Kandidat und darf nicht auf Hardware übertragen werden. Verbleibende optionale Diskriminatoren (Physikschritt, Kontaktvariante) sind im Analysedokument benannt; neue SIL-Proben nur mit diesem Rahmen.
5. **G6 lokal bestanden; optionaler echter `.71`-Vergleich (Stand 14.10.2026).** Der positive lokale SIL-Nachweis über genau einen Domain-42-Fotocapture ist unter `log/person_runtime/g6_oneshot_20261013_1759/vision-probe.json` dokumentiert (Contract v1, ein `person`-Result, Modell-/Gewichts-Hash und Capture-Korrelation; Details in [`IMPLEMENTATION_STATUS.md`](IMPLEMENTATION_STATUS.md)). Der Mac-gehostete CPU-Detector als `.71`-Protokollzwilling hat somit den lokalen Kamera→Inference→Contract-Pfad positiv geprüft. Nur wenn ein echter TensorRT-Vergleich gewünscht wird, kann der Nutzer den Jetson `.71` starten; dann sind aktueller Engine-/Labelnachweis, ein klar definierter Bildursprung und separate Freigabe für genau einen Request nötig. Der historische externe `.71`-Einbildlauf lieferte null Detektionen und belegte weder positive Erkennung noch ROS-/Contract-v1-Transport. `.150` ist nicht der passende Endpunkt und wird nicht weiterverfolgt. Kein Nav2-/`/cmd_vel`-Anschluss oder Motoraktion.
6. **G7 motorloses Pi-/ESP32-HIL vorbereiten (separate Freigabe).** Der Nutzer meldet den Pi wieder online. Vor jeder Verbindung ist der angebotene Host-Key out-of-band mit dem vertrauenswürdigen Schlüssel abzugleichen: der letzte Kandidat `192.168.0.100` präsentierte einen abweichenden Schlüssel, `known_hosts` wurde nicht geändert. Zuerst nur freigegebene Read-only-Inventur/Build; anschließend ROS Domain `44`, Backend-/Stop-/Fallbacktests nur bei physisch bestätigter Motortrennung und expliziter Testfreigabe. Keine Firmware-/Motoraktion aus dem Online-Hinweis ableiten.
7. **G8 physische Abnahme (später, separate Freigabe).** Erst nach G7 und sicherem Prüfstand: Rad-/Spurweite, Sensorposen, Not-Aus, Brems-/Stopverhalten und Limits physisch messen; Simulationswerte nicht übernehmen.
8. **G9 Reproduzierbarkeit/Lifecycle.** Frischen Workspace-Aufbau, Start-/Stop-/Rollback-Anleitung und Humble/Fortress-EOL-Review vor Mai 2027 vervollständigen.

**G6 `.71`-Nachtrag 13.10.2026:** Nutzer startete `ros_detection` manuell und
gab genau einen simulierten Inferenzlauf frei. ZeroMQ `5555/5556` ist erreichbar;
Flask nutzt Defaultport `5000`, nicht `8082`. Startup-Log plus Engine-Hash und
Exportmetadaten belegen `yolo12n.engine` (YOLOv12n, 80 Labels, `0=person`).
Genau ein archiviertes Standardwelt-Personenbild wurde verarbeitet:
**Transport PASS, null Detektionen, positives Personen-Gate FAIL**.
Erster Request einschliesslich TensorRT-Lazy-Loading: `54.223,4 ms`;
keine Warm-Latenz und kein Contract-v1-/ROS-Nachweis. Keine Jetson-Aenderung,
kein Nav2/Motorstart, kein Retry. Nachweise und Grenzen stehen im
[Implementierungsstatus](IMPLEMENTATION_STATUS.md), Bericht:
[`jetson71-single-image-20261013.json`](../log/person_runtime/jetson71-single-image-20261013.json).
Historischer Stand: Dieser externe `.71`-Lauf war ein begrenzter
Inferenzversuch ohne ROS-/Contract-v1-Nachweis und ohne positive Detection.
Nur wenn ein echter TensorRT-Vergleich gewünscht wird, kann der Nutzer `.71`
starten; dann den Dienstzustand und aktuellen Engine-/Labelnachweis prüfen, den
Bildursprung festlegen und genau einen Request separat freigeben. `.150` wird
für diesen Zweck nicht verwendet.

**G6 lokaler `.71`-Protokollzwilling (historischer Zwischenstand, 13.10.2026):**
Die damals offenen Fixture- und Kamera-Capture-Punkte sind mit dem späteren,
einzelnen lokalen Domain-42-Lauf im vorstehenden Abschnitt geschlossen. Der
externe `.71`-Einzelbildlauf lieferte null Detektionen und bleibt ein
unabhängiger historischer Befund; er ist keine Freigabe für einen weiteren
Request; `.150` ist für den aktuellen Zweck nicht der richtige Endpunkt.

**G6-Simulationsintegration (Vorstart-Stand, 12.10.2026; historisch):**
`vision_overlay.launch.py` unterstützte `backend:=service`, war auf den damals
vorgesehenen `.150`-Endpoint gepinnt und prüfte Modell-/Labelversion sowie
Class-ID-Mapping vor dem Start. Dieser HTTP-Pfad ist nicht der aktuelle
`.71`-Inferencepfad und wird dafür nicht verwendet.
Configtests (5/5), isolierter Build, Sim-Overlay-Build und Negativ-Preflight
bestanden. Vor dem Runtime-Wechsel lief in Domain 42 noch der Mock ohne neuen
Service-Node oder echten Request. Der strikte Startmodus verlangt verifizierte
Metadaten; der explizite `unverified`-Opt-in wird im folgenden Runtime-Nachtrag
dokumentiert.

**Runtime-Nachtrag 12.10.2026:** Die obige Vorstart-Aussage ist durch den danach
erfolgten Live-Wechsel überholt. Der alte Mock-Overlay wurde beendet; Fortress,
Kamera und `/clock` liefen weiter. Domain 42 nutzt jetzt den laufenden
`vision_adapter_sim` mit `backend=service` und dem gepinnten Endpoint
`http://192.168.0.150:8082/v1/vision/detection`; Ergebnis-Ownership ist eindeutig
(ein Publisher). Eine vollständige Live-Antwort kam mit `status=ok`, ohne Fehler,
aber leer (`detection_count=0`). Die Simulation sendet höchstens ein Bild pro
Sekunde; kein Requestlimit. Versionen bleiben `unverified`, Mapping `{}`; positive
Detection/Personenabnahme und aktive Gewichts-/Klassenprovenienz sind weiterhin
offen. Ein nicht abgebildeter Klassen-ID-Wert wird fail-closed verworfen. Es wurden
weder Nav2 noch `/cmd_vel` oder Hardware gestartet.
weder Nav2 noch `/cmd_vel` oder Hardware gestartet. Das aktive `fortress_test.sdf`
enthält nur Boden und den roten Hinderniskasten, keine Person; daher entsprechen
die fehlerfreien leeren Resultate der Szene und sind kein Launch-/Transportfehler.
Die separate Geometrie-Personenfixture wurde noch nicht runtime-validiert. Eine
positive Erkennung bleibt offen, bis Modell-/Labelversion und Klassen-ID geprüft
sind.

**Standardwelt-Person, 13.10.2026:** `fortress_test.sdf` enthält `sim_person_proxy` mit Visual- und Collision-Geometrien; finaler Humble-Build, Fortress-SDF-Parser und Headless-Sensorcheck bestanden. Der verifizierte Standard-Start liefert ein gepaartes Kamerabild, sichtbares Ziel im Bild und sechs LiDAR-Returns zum Ziel; die Person liegt bei `(4.0, -1.8, 0)` neben dem Kasten. Das belegt Sensor-Sichtbarkeit im SIL, nicht echte Erkennung oder Fusion. Bericht und Bild: [`sensor-report.json`](../log/person_runtime/default-person-final-20261013-103044-B8AfRj/sensor-report.json), [`camera.png`](../log/person_runtime/default-person-final-20261013-103044-B8AfRj/camera.png).

**Vision-Kameraansicht:** Der Standard-Fortress-Launch unterstützt `vision_view:=true`. Damit startet er den vorhandenen Vision-Overlay-/Fusion-Launch samt RViz-Kamerabild; die Haupt-RViz-Instanz wird nicht doppelt geöffnet. Standardmäßig wird ausschließlich der synthetische Mock verwendet, keine externe Inferenz. Details und Aufruf stehen in der [`linorobot2_vision`-README](../src/linorobot2/linorobot2_vision/README.md).

**Nebenläufiger RDP-Restpunkt:** `LP_NUM_THREADS=2` ist im Gazebo/RViz-Launcher und die CPU-Reduktion wurde im SIL beobachtet. `.rdp` fordert 1280×800, die laufende Sitzung zeigt bis zum nächsten Reconnect weiterhin 1920×1481. Der XDG-Gazebo-Autostart wurde vorbereitet, aber noch nicht aktiviert.

**Scope-Grenze:** Eine gemeinsame Geometriequelle für Fortress, Legacy und Hardware sowie physische Vermessung sind wichtige spätere Transfer-/HIL-Arbeit, aber kein Vorwand für eine G4-Großrefaktorierung. Für G4 reicht ein versioniertes, gehashtes Fortress-Profil; es darf nicht als Hardwaremodell ausgegeben werden.

## Transfer-Gates — erst nach hardwareunabhängiger Arbeit

- **G6 lokal:** Contract-v1-Foto-Capture bestanden (siehe oben). Externe
  Inferenz bleibt nur bei Bedarf und nach Provenienzprüfung sowie separater
  Freigabe offen.
- **G7:** motorloses HIL/Pi- und ESP32-Backend mit Domain-/Stop-/Rückfallnachweis; nur nach separater Freigabe.
- **G8:** reale Geometrie, Sensorposen, Not-Aus und Stopps physisch messen und abnehmen; keine ungeprüfte Übernahme der Simulationskandidaten.

## Arbeitsgrenzen

- Keine Run-25-Wiederholung ohne neuen Testzweck; Run 26 nicht unverändert wiederholen.
- Kein automatischer Retry nach einem fehlgeschlagenen manuellen Lauf; erst Ausgabe sichten.
- Simulation-first bleibt aktiv: lokale ROS-freie, Modell-, Schnittstellen- und SIL-Arbeit kann weitergehen.
- Keine Hardware-, Firmware-, echten Motor- oder Inferenzaktionen ohne separate Freigabe.
