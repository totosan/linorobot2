# Quell- und Schnittstellenmanifest (Phase 0)

Stand: 04.10.2026
Referenz: [DEVELOPMENT_PLAN.md](../DEVELOPMENT_PLAN.md), Abschnitt 2–3 und Phase 0; [MACOS_SETUP_PLAN.md](MACOS_SETUP_PLAN.md).

## 1. Übersicht der Quellbestände und Repositories

| Komponente | Quelle / Repository | Zweig / Commit-SHA | Bemerkung |
|---|---|---|---|
| **Mac Linorobot2 Basis** | `https://github.com/linorobot/linorobot2.git` | `humble` (`6b7ae2ebd5e3782560363dca136c3a92864f5a46`) | Upstream Humble, Stand 09.02.2025. Enthält Classic-Gazebo-Plugins. |
| **Pi Linorobot2 Stand** | `https://github.com/totosan/linorobot2.git` | `toto_dev` (`3f5fb12a555cd4d4209a924b056b14a6636eda88`) | Pi-Entwicklungsstand mit angepassten 2WD-Maßen, Webcam-URDF, echten Raumkarten, `linorobot2_vision`. |
| **Linorobot2 RViz / Viz** | `https://github.com/linorobot/linorobot2_viz.git` | `master` (`d59955364177b09d0cb0313c05c317ff6909fa0b`) | Visualisierungskonfigurationen für RViz2. |
| **ESP32 Firmware** | `ROS_experiments/esp32bot_ws/...` | `humble` (`8baaa4a16d6364faf74177c62ab69ad08aa99720` + lokale Modifikationen) | Environment `myrobot`, PlatformIO, micro-ROS Humble WiFi-Transport. |
| **Jetson Vision (gesicherte Laufzeitreferenz)** | `/Users/toto/Backups/linobot2/jetson-20261004-174406/video_detection.tar.gz` + `bold_snyder-rootfs.tar.gz` + geschützte Metadaten | `segmentation_containerimages` (`439f0ef074df1df91812bc906106cead76f30396`) plus lokale Änderung in `app.py` | Als zuletzt auf dem Jetson laufender Visionstand vorrangig für Phase 5/6 behandeln. Containerexport ist kein Image-Layer-Backup; Host-Bind-Mount separat im Quellarchiv. |
| **Jetson Vision (neuere Entwicklungsreferenz)** | `/Users/toto/Projects/JetsonNano/ai-video-solution` (ebenfalls im `JetsonNano.tar.gz` gesichert) | `segmentation_containerimages` (`b7320bfa74d45a88eb56eacde5c09c360b0ebffb`) | Nachfahre des Laufzeitcommits, aber nicht als auf dem Jetson ausgeführt verifiziert. Nur nach Diff/API-/Modellabgleich übernehmen. |

## 2. 2WD-Geometrieparameter im Vergleich

| Parameter | Upstream `humble` | Pi `toto_dev` | ESP32 `myrobot_config.h` | Gewählte Simulationsannahme (Startmodell) |
|---|---|---|---|---|
| **Basis-Länge** | 0.24 m | 0.22 m | – | 0.22 m (Pi Hardware-Stand) |
| **Basis-Breite** | 0.275 m | 0.18 m | – | 0.18 m (Pi Hardware-Stand) |
| **Basis-Höhe** | 0.003 m | 0.06 m | – | 0.06 m (Pi Hardware-Stand) |
| **Basis-Masse** | 1.0 kg | 1.0 kg | – | 1.0 kg |
| **Raddurchmesser / Radius** | r = 0.045 m (d = 90 mm) | r = 0.035 m (d = 70 mm) | d = 0.0675 m (r = 33.75 mm) | r = 0.03375 m (ESP32-Firmwarekandidat, nicht vermessen) |
| **Radbreite** | 0.01 m | 0.027 m | – | 0.027 m |
| **Radposition X** | 0.0 m | -0.07 m | – | -0.07 m |
| **Radabstand / Spurweite** | 0.26 m (y = ±0.13) | 0.21 m (y = ±0.105) | 0.126 m | 0.126 m (ESP32-Firmwarekandidat, nicht vermessen) |
| **Caster-Räder** | vorn: true, hinten: true | vorn: true, hinten: false | – | vorn: true, hinten: false |
| **Lidar-Pose (xyz, rpy)** | `0.12 0 0.33`, `0 0 0` | `0.02 0 0.085`, `0 0 0` | – | `0.02 0 0.085`, `0 0 0` |
| **Kamera-Pose (xyz, rpy)** | `0.14 0 0.045`, `0 0 0` | `0.14 0 0.045`, `0 0 0` | – | `0.14 0 0.045`, `0 0 0` |

**Wichtig zur Herkunft:** Das Jetson-Backup ist die maßgebliche Referenz für den
zuletzt laufenden Visiondienst, nicht für die Robotermodell-/EKF-/Nav2-Konfiguration.
Die geprüften Jetson-Quell-, Containerexport- und JetsonNano-Archive enthalten
keine `2wd`-URDF, `ekf.yaml`, `navigation.yaml` oder ESP32-Firmwaredatei. Deshalb
ist das Fortress-Modell ein ausdrücklich vorläufiger Hybrid: Basis-/Caster-/Sensor-
Anordnung aus dem Pi-`toto_dev`-URDF; Radmaß/Spurweite aus den ESP32-`myrobot`-
Firmwarekonstanten. Nichts davon ist durch das Jetson-Backup als reale Geometrie
bestätigt. Vor G2 müssen die Jetson-Quellen außerhalb dieser Sicherungsarchive
gefunden oder Rad-/Sensormaße am Roboter vermessen werden.

## 3. Schnittstellen- und Topic-Vertrag

Die Module kommunizieren über standardisierte ROS-2-Nachrichten:

| Vertrag / Topic | ROS-Nachrichtentyp | Sender | Empfänger | QoS / Bemerkung |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 / Teleop | Basis / Simulator | Reliable / Transient Local (Nav2) bzw. Default |
| `/odom/unfiltered` | `nav_msgs/msg/Odometry` | Simulator / micro-ROS | `robot_localization` (EKF) | 50 Hz, frame `odom`, child `base_footprint` |
| `/imu/data` | `sensor_msgs/msg/Imu` | Simulator / micro-ROS | `robot_localization` (EKF) | frame `imu_link` |
| `/odom` | `nav_msgs/msg/Odometry` | `ekf_node` | Nav2 / SLAM | gefilterte Odometrie, publiziert TF `odom -> base_footprint` |
| `/scan` | `sensor_msgs/msg/LaserScan` | Simulator / RPLIDAR | SLAM / Nav2 Costmap | frame `laser` |
| `/joint_states` | `sensor_msgs/msg/JointState` | Simulator / micro-ROS | `robot_state_publisher` | Radgelenke `left_wheel_joint`, `right_wheel_joint` |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulator (Bridge) | Alle Nodes | aktiv bei `use_sim_time:=true` |

### TF-Hierarchie
- `map -> odom`: Verwaltet durch `slam_toolbox` oder `amcl`.
- `odom -> base_footprint`: Genau ein Publisher (`ekf_node`).
- `base_footprint -> base_link -> [laser, imu_link, camera_link, wheels]`: Verwaltet durch `robot_state_publisher` via URDF.

### ROS-Domain-Zuordnung
- Simulation: `ROS_DOMAIN_ID=42`
- Replay: `ROS_DOMAIN_ID=43`
- Reale Hardware: `ROS_DOMAIN_ID=44`

## 4. Sicherungsorte (Verifiziert)

1. **Jetson-Archive (04.10.2026):**
   - Mac: `/Users/toto/Backups/linobot2/jetson-20261004-174406/`
   - NAS: `/Volumes/personal_folder/Backups/linobot2/jetson-20261004-174406/`
   - Archive: `video_detection.tar.gz`, `bold_snyder-rootfs.tar.gz`, `jetson-metadata.zip`
2. **ROS_experiments (04.10.2026):**
   - Mac: `/Users/toto/Backups/linobot2/ros-experiments-20261004-180849/ROS_experiments.tar.gz`
   - NAS: `/Volumes/personal_folder/Backups/linobot2/ros-experiments-20261004-180849/ROS_experiments.tar.gz`
   - SHA-256: `c0641fb582e1343158eb3e7ac246506a0ed4a0d191372464800fb9e0f52432f6`
3. **JetsonNano (04.10.2026):**
   - Mac: `/Users/toto/Backups/linobot2/jetsonnano-sources-20261004-182313/JetsonNano.tar.gz`
   - NAS: `/Volumes/personal_folder/Backups/linobot2/jetsonnano-sources-20261004-182313/JetsonNano.tar.gz`
   - SHA-256: `cadcc13a56dd97d6c736e78d181c1adeb97b7ef9b46cf2fcf096acfaacad4eb8`

## 5. Vorbereitete TF2-Dependency-Reparatur (07.10.2026)

- Quelle: `https://github.com/ros2/geometry2.git`, offizieller Humble-Backport
   [#990](https://github.com/ros2/geometry2/pull/990), gepinnter Commit
   `9997e969556f5fe5d32ad498d02b3b3473da70b6` (BSD-3-Clause; upstream Lizenzheader
   bleiben unverändert). Kein eigener Ersatz des Navigationsalgorithmus.
- Anlass: installierte tf2/tf2_ros 0.25.23 mit bekanntem Async-TF-Sperrzyklus;
   Run 24 zeigt passenden lokalen Costmap-Posezeit-Rückstand. Upstream A/B
   reproduziert den Deadlock; Run 25 bestätigt erfolgreiche Navigation mit Fix.
- Reproduktion/Build: `scripts/setup_tf2_fix.sh` und
   `scripts/tf2_regression/CMakeLists.txt`; vorgesehener Linux-Root
   `$HOME/linobot2_tf2_fix`. Quellen müssen exakt gepinnt und unverändert sein.
- **Status: vom Nutzer manuell gebaut; Deadlock-A/B und Buffer-Suite bestanden.**
   Agent-Ausführung war blockiert. Checkout-SHA lesend bestätigt; nach Korrektur
   der optionalen colcon-Option tf2 und tf2_ros erfolgreich gebaut. Nachweis:
   `/home/toto/linobot2_tf2_fix/evidence-20261007-133055-x2SBto/` (Exit 0).
   Baseline reproduziert Deadlock, Overlay besteht dessen Regression.
   Nach Korrektur des im vorherigen Lauf fehlenden CycloneDDS-Suchpfads bestehen
   alle 11 upstream Buffer-Tests. `LD_LIBRARY_PATH` bewahrt ROS-Architekturpfade.
   `/opt/ros`, Pi, Jetson und Firmware unverändert. Lokale Regressionen:
   Linux 303 bestanden, Mac 298 + 5 Skips. Nav2-Zwei-Szenarien-PASS mit Fix:
   `/home/toto/linobot2_sim/log/g4-nav-20261007-134435-LJ0KXV/`.
   `tf2-overlay.json` enthält tatsächlich verwendete Pfade und Bibliothekshashes.
- Nach erfolgreichem manuellem Build Nachweisverzeichnis mit Quell-SHA,
   Bibliotheksauflösung und Baseline-/Fix-Logs referenzieren; erst danach als
   Runtime-Dependency im Simulationsprofil und späteren Hardwareprofil bewerten.

### Transferprüfung des gepinnten Fixes

Der [versionsgebundene upstream Vergleich](https://api.github.com/repos/ros2/geometry2/compare/0.25.23...9997e969556f5fe5d32ad498d02b3b3473da70b6)
zeigt genau einen zusätzlichen Commit und zwei geänderte Dateien:
`tf2/src/buffer_core.cpp` und `tf2_ros/test/test_buffer.cpp`. Öffentliche Header,
Nachrichten-/Service-Schemas, Paketmanifeste und Buildabhängigkeiten sind gegenüber
0.25.23 unverändert. Die Reparatur betrifft Callback-/Sperrreihenfolge, nicht
Gazebo-Geometrie, Motorparameter oder Navigationsalgorithmen.

Das ist ein überprüfter **quellseitiger Schnittstellenbefund**, ergänzt durch
lokalen Buffer-A/B-Erfolg und Nav2-Baselinelauf; kein umfassender ABI-Test und keine
G7-Hardwareabnahme. Der ursprünglich blockierte
Agent-Build bedeutet nicht, dass der Fix prinzipiell unübertragbar wäre.

Für eine spätere Übertragung nach Phasen 7–8:

1. Identischen Quell-SHA auf dem Zielsystem bauen oder eine geprüfte offizielle
   Humble-Version mit demselben Fix verwenden; keine Mac-/VM-Binärdateien blind
   auf den Pi kopieren.
2. Bibliotheksauflösung und upstream Regression auf dem Zielsystem prüfen;
   anschließend Basisvertrag, TF-Ownership und Stopps im motorlosen HIL testen.
3. Hardwaredomain 44 und `use_sim_time=false` verwenden; Simulations-Clock und
   G4-Fahrbefehle nicht auf reale Motoren übertragen.
4. Physisch gemessene Geometrie/Sensorposen im gemeinsamen Modell zurückführen;
   danach SIL wiederholen. Firmwarekonstanten nur nach separatem Review ändern.
5. Ursprünglichen ROS-/Roboteroverlay als Rückfallweg erhalten. Änderungen am
   Pi/ESP32/Jetson benötigen weiterhin separate Nutzerfreigabe.
