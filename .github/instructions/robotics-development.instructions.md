---
description: "Use when planning, coding, testing, debugging, reviewing, or documenting robotics software in this workspace: ROS 2, simulation, robot models, navigation, sensors, safety, system integration, or AI/vision. Covers project evidence, safe development workflow, current-technology research, and optional perception-specific constraints."
---

# Robotikentwicklung: Projektwissen und Arbeitsregeln

Stand: 14.10.2026. Diese projektbezogene Instruction unterstützt unterschiedliche Entwicklungsaufgaben; sie ersetzt weder Nutzerfreigaben noch die maßgebliche Projektdokumentation.

## 1. Projektkontext und maßgebliche Quellen

- Der autoritative Fahrplan steht im [Development Plan](../../docs/DEVELOPMENT_PLAN.md), belegte Ergebnisse und Grenzen im [Implementation Status](../../docs/IMPLEMENTATION_STATUS.md), offene Arbeit in [OPEN_TASKS](../../docs/OPEN_TASKS.md). Die [aktuelle Übergabe](../../docs/HANDOFF.md) ist nur ein Kurzüberblick.
- Git-Root ist `src/linorobot2`, nicht die Workspace-Wurzel. Der Checkout enthält lokale Änderungen: kein `git clean`, Reset, pauschales Überschreiben oder Entfernen von Dateien ohne ausdrücklichen Auftrag.
- Zielarchitektur: Motor/Basis (Regelung und lokaler Stopp), Roboterkern (ROS, TF, EKF, SLAM/Nav2) und optionale Vision-/AI-Dienste mit klaren Verträgen. Austauschbarkeit an Paket-/Prozessgrenzen erreichen; keine unnötigen Netzwerkdienste oder Frameworks einführen.
- Simulation-first bleibt die verbindliche Reihenfolge. OrbStack Linux ARM64 mit Ubuntu 22.04, ROS 2 Humble und Gazebo Fortress ist der geprüfte SIL-Pfad. G1–G5 sind im SIL bestanden; G6 lokal ist am 13.10.2026 bestanden. G4/G5-PASS und ihr Abnahmeumfang nicht durch spätere Replay-Arbeit umdeuten oder ändern. Der Nav2-Eingabe-Replay in Domain 43 hat am 14.10.2026 für genau ein G5-Off-Bag-Segment bestanden; Mehrsegment-/Clock-Rewind, G4- und G5-Mock-Segmentierung bleiben offen. G7/G8 bleiben durch fehlende Hardware-Vorbedingungen und separate Freigaben blockiert.
- Projektprofile: Simulation 42, Replay 43, Hardware 44. Diese Profile und Harnesses sind implementiert, aber ROS-Domains sind keine Sicherheitsgrenze und belegen keine sichere Domainisolation gegen fremde Prozesse.
- Kein Pi-/Jetson-/ESP32-/Motor-/Firmware-/Inferenzbetrieb ohne separate Freigabe. Hardware-Geometrie und reale Stopps sind unvermessen, sofern im aktuellen Nachweis nicht anders belegt.

## 2. Planung und Vorgehen für Änderungen

1. Nutzerziel, geltenden Plan, Scope und Safety-Auswirkung klären. Relevante Code-/Config-Verwendungen und vorhandene Tests untersuchen, bevor Dateien geändert werden.
2. Bei mehrstufiger Arbeit eine kurze, verifizierbare Reihenfolge festlegen. Vor dem Edit den vollständigen relevanten Kontext lesen; vorhandenen Stil und öffentliche Schnittstellen erhalten.
3. Kleine, nachvollziehbare Änderungen bevorzugen. Keine Test-, Grenzwert-, Zielkoordinaten- oder Messmethodenänderung allein, um einen PASS zu erzeugen. Unbekanntes ausdrücklich als unbekannt markieren.
4. Änderungen auf der niedrigsten sinnvollen Ebene prüfen: Syntax/Format und schnelle Unit-/Contract-Tests zuerst; anschließend Build und Integration; Runtime/Hardware nur innerhalb freigegebener Gates.
5. Ergebnisse evidenzbasiert berichten: **geplant**, **im Quellstand vorhanden**, **lokal installiert**, **im Test simuliert** und **am Zielsystem gemessen** sind verschiedene Zustände. Testzahl oder erfolgreicher Build allein beweist keine Laufzeitfähigkeit.
6. Fehlerursache statt Symptom beheben. Bei fehlgeschlagenem Runtime-Test zuerst alle Ausgaben/Logs sichten; in diesem Projekt keine automatischen Retries. Keine Folgeschritte starten, bevor ein manueller Lauf ausgewertet ist.
7. Dokumentation nach Zuständigkeit pflegen: Roadmap/Gates in `DEVELOPMENT_PLAN.md`, Nachweise und Fehlschläge in `IMPLEMENTATION_STATUS.md`, offene Schritte in `OPEN_TASKS.md`. Keine parallele Laufchronik oder widersprüchliche Statuskopie erstellen.
8. Bei einem Blocker sofort eine konkrete Meldung geben: fehlende Pakete/Tools/Dateien/Installationsartefakte, erwarteter Zielpfad und ggf. Versions-/Overlaykontext, beobachteter Istzustand, betroffener nächster Schritt sowie sichere Handlungsoptionen vollständig nennen. Quell-Datei vorhanden, gebaut, im richtigen Overlay installiert und zur Laufzeit geladen sind getrennte Zustände; prüfen statt annehmen.
9. Wenn eine Nachinstallation möglich ist, eine minimale, gezielte Strategie empfehlen und angeben, welche Umgebung/Dateien sie ändert. Vor Änderungen an VM, Overlay, Abhängigkeiten oder Geräten um Erlaubnis bitten, sofern sie nicht bereits ausdrücklich freigegeben sind. Nach Freigabe den Zielpfad und die erwartete Konfiguration/Funktion verifizieren; keine irrelevanten Pakete nachziehen. Nach Runtime-Fehlern Logs und Cleanup prüfen und für einen neuen Lauf eine frische Freigabe einholen.

## 3. ROS-, Simulations- und Systemschnittstellen

- ROS-Verträge explizit dokumentieren: Topic-/Service-/Actionname, Nachricht, Einheiten, Achsen/Frames, Zeitdomäne, QoS, Ownership, Rate, Timeout und Fehlerverhalten. Standardnachrichten bevorzugen; projektspezifische Abstraktion nur bei konkretem Bedarf.
- TF-Verantwortung eindeutig halten. Im Zielmodell verwaltet EKF `odom → base_footprint`; SLAM/AMCL `map → odom`; RobotStatePublisher die Modell- und Sensorframes. Keine konkurrierenden Publisher oder stillen Ersatztransforms.
- In Simulation `/clock` und `use_sim_time=true` konsistent verwenden. ROS-/Wall-Clock, Simulationszeit, Aufnahmezeit und Empfangszeit nicht gleichsetzen. Stale, zukünftige, fehlende oder inkonsistente Daten explizit behandeln.
- QoS aus tatsächlich entdeckten DDS-Endpunkten und erfolgreicher Zustellung prüfen; nicht aus Gazebo-Transport, RViz-Konfiguration oder Defaults ableiten.
- Ground Truth ist eine Bewertungsquelle, keine unbemerkte Laufzeiteingabe. Die Run-25-Karte wird mit Ground-Truth-Samples zu freien Korridoren ergänzt und ist daher eine Fixture, kein sensor-only SLAM-/Hardwaretransfernachweis. Rohkarte und abgeleitete Karte getrennt halten.
- Robotermaße, Footprint, Rad-/Sensorposen und Limits nur aus nachvollziehbarer, versionierter Quelle beziehen. Die unterschiedlichen 2WD-Geometriewerte in Fortress- und Legacy-Konfigurationen sind nicht physisch vermessen; nicht automatisch angleichen oder in Firmware exportieren.
- Safety-Pfad unabhängig von Netzwerk, Vision und Cloud halten. Ein Nullbefehl, Nav2-Abbruch oder Software-Watchdog ist nicht automatisch ein physischer Not-Aus- oder Bremsnachweis.

## 4. Tests, Gate-Abnahme und Betrieb

- Testvertrag vor Implementation definieren: Erfolg/Abbruchsemantik, Clearance, Zeit-/Bewegungsbudget, Stoppnachweis, Map-/Sensorquelle und erlaubte Recovery. Grenzen müssen aus Anforderungen und Messung begründet sein.
- Unit-/ROS-freie Tests decken Geometrie, Verträge, Fehler-/Zeitfälle und Parser ab. SIL Integration prüft echte ROS-/Simulatorpfade und protokolliert Laufzeitdaten. HIL/Hardware erfordert eigene Freigabe, sichere Motorzustände, Rückfallweg und unabhängige Stoppprüfung.
- Simulationserfolg nicht mit Hardwaretauglichkeit gleichsetzen. Test-fixture, aufgezeichnete Daten, Sensor-SIL, HIL und physischer Test deutlich unterscheiden.
- Bei Robotik-Runtime die relevanten Observables sichern: Clock/TF/Frames, Sensorfrische, tatsächliche Odom-/Ground-Truth-Bewegung, ausgegebene Befehle, Action-Endstatus, Kostenkarte/Plan, Clearance, Ressourcen und Prozess-Cleanup.
- Sensorrauschen, Schlupf, Verzögerungen, Ausfälle und Replay nur kontrolliert, reproduzierbar und passend zur Abnahmematrix hinzufügen. Die geprüften aktuellen G3/G4-Harnesses belegen kein explizites generisches Rauschmodell oder rosbag-Aufzeichnung.
- Bei Fehlern keine Gate-Umetikettierung, nachträgliche Lockerung oder automatische Wiederholung. Sicher abbrechen, Bericht erhalten, Root Cause prüfen, Tests aktualisieren und erneute Runtime-Freigabe einholen.

## 5. Aktuelle Technologie recherchieren

Bei Fragen nach „neuesten“ Technologien, Paketen, Modellen, Runtimes, APIs oder Gerätefähigkeiten nicht allein auf Trainingswissen oder Versionsnamen vertrauen. Recherche muss zur konkreten Distribution und Zielhardware passen.

1. Primärquellen zuerst: offizielle ROS-2-/REP-Dokumentation und getaggte Quellstände; Nav2-/Gazebo-Dokumentation für die verwendeten Releases; Hersteller-Supportmatrizen und Runtime-Dokumentation; offizielle Modell-Repositorys, Model Cards und Lizenzen.
2. Versionen/Kompatibilität abgleichen: ROS-Distribution, Ubuntu/JetPack/L4T, CPU-Architektur, Compiler/Python-ABI, CUDA/cuDNN/TensorRT, API/Message-Schema, Speicher, Echtzeit- und Leistungsbudget. `latest` dokumentiert nicht automatisch Humble/Fortress oder Jetson Nano.
3. Technische Aussagen mit konkreter Version, Datum und Quelle belegen. Paper zeigen Methoden, nicht zwingend Wartung, Integration, Lizenz, Echtzeitfähigkeit oder Zielgeräteunterstützung. Benchmarks auf Testgerät, Daten, Auflösung und Messmethode prüfen.
4. Behauptungen als **unterstützt**, **verfügbar**, **installiert**, **implementiert** oder **gemessen** klassifizieren. Bei Konflikten Quelle/Release nennen; fehlende Evidenz als offene Frage ausweisen.
5. Alternativen gegen Aufgabe und Messziele vergleichen: Funktion, Genauigkeit/Fehlerrisiko, p50/p95-Latenz, Durchsatz, CPU/GPU/Speicher, Energie/Thermik, Offlinebetrieb, Lizenz, Wartung und Rückfall. Erst isoliert prototypisieren; eingefrorene Zielgeräte nicht für Rechercheexperimente verändern.
6. Ergebnisse mit Links in Plan, Status oder Entscheidungseintrag sichern. Keine unbelegten „aktuellsten“ Empfehlungen oder Kompatibilitätsversprechen.

### Verlässliche Recherche-Einstiege

- ROS REP 103 (Einheiten/Achsen): https://www.ros.org/reps/rep-0103.html
- ROS REP 105 (Frames): https://www.ros.org/reps/rep-0105.html
- ROS 2 Humble: https://docs.ros.org/en/humble/
- Nav2: https://docs.nav2.org/ — verwendeten Distributionstag zusätzlich prüfen.
- Gazebo Fortress: https://gazebosim.org/docs/fortress/
- NVIDIA Jetson-Dokumentation: https://docs.nvidia.com/jetson/ — exakt passende L4T-Version heranziehen.
- ONNX Runtime Execution Providers: https://onnxruntime.ai/docs/execution-providers/

## 6. AI-, Vision- und Wahrnehmungsregeln

Diese Regeln sind zusätzlich anzuwenden, wenn eine Aufgabe Bildverarbeitung, AI-Inferenz, Detection/Segmentation, Tracking, Sensorfusion oder AI-ROS-Integration betrifft.

- `linorobot2_vision` bietet Contract v1, Fixture-/Off-Backends, einen ROS-freien Consumer und einen Offline-Adapter für die im gesicherten Jetson-Laufzeitquellstand gefundene `/v1/vision/detection`-Antwort. Es gibt keinen bestätigten Live-API-Aufruf und keine produktive ROS-/HTTP-Integration.
- Die Legacy-Antwort `success`/`predictions` enthält `timestampUTC`, das zur Antwortzeit erzeugt wird und **keine** Capture-Zeit ist. Modellversion und stabile Label-IDs fehlen. Capture-Metadaten, Modellversion und explizites Label-Mapping zuführen; keine Werte aus `moduleId`, Empfangszeit oder Namen erraten.
- Contract v1 markiert Capture-Zeit mit `ros_sim` oder `unix_utc`; diese Zeitdomänen nicht vergleichen. `frame_id`, Bildgröße, Modell-/Label-Schema und Pixelboxen `[x_min, y_min, x_max, y_max]` (exklusive rechte/untere Grenze) erhalten. Fehler, `empty`, fehlend, `stale`, future und unavailable getrennt behandeln.
- Die vorhandenen Fixtures sind synthetisch. Fünf Fortress-Image-/CameraInfo-Paare wurden mit einem absichtlichen Empty-Mock konsumiert. Das belegt den Consumer-Pfad, keine Detektionsqualität, echte Inferenz, Live-Transport oder G5.
- Eine 2D-Box ist keine metrische Hindernisposition. Projektion benötigt passende Intrinsics/Extrinsics und Depth/PointCloud oder eine begründete Geometrieannahme samt Unsicherheit. Keine AI-Box direkt als Costmap-Hindernis oder Motorbefehl verwenden.
- Auflösung, Encoding, Crop/Resize/Letterbox, optischer Frame und CameraInfo/Kalibrierungs-ID gemeinsam versionieren. Bekannte Projektstände umfassen 640×480 und 1920×1080; Intrinsics nicht ungeprüft übertragen.
- Modellqualität nicht nur mit mAP oder Demo-Frames beschreiben. Je nach Aufgabe Precision/Recall, klassenspezifische Fehler, False-Negative-Risiko, Robustheit und p50/p95-Latenz sowie Rechen-/Energie-/Thermikbudget messen. Konfidenz ist nicht automatisch eine kalibrierte Wahrscheinlichkeit.
- Generative/Vision-Language-Modelle können Offline-Analyse oder semantische Bedienung unterstützen, sind ohne strengen Determinismus-, Latenz- und Fehlermodusnachweis kein Ersatz für lokalen Kollisionsschutz, Motorregelung oder Not-Aus.
- Die gesicherte Jetson-Laufzeitreferenz ist Nano/L4T 32.6.1/CUDA 10.2/TensorRT 8.0.1. Nichts daran aktualisieren oder als kompatibel annehmen, bevor offizielle Versionsmatrix und Zielgerätetest es bestätigen. TensorRT-Engines sind versions-/hardwareabhängig.
