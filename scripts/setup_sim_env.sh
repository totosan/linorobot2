#!/usr/bin/env bash
# setup_sim_env.sh - Einrichtung der Linux-ARM64 Simulationsumgebung (Ubuntu 22.04 / ROS Humble / Fortress)
# Ausführung innerhalb der OrbStack-Maschine 'ros-humble' oder eines Ubuntu 22.04 ARM64 Containers.
set -euo pipefail

[[ "$(uname -s)" == Linux ]] || { echo "Nur in der Linux-VM ausführen."; exit 1; }
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 && "$(uname -m)" == aarch64 ]] || {
    echo "Erwartet: Ubuntu 22.04 ARM64."; exit 1;
}
sudo -n true

# Die offizielle ROS-Quellenvorlage verwendet HTTP; APT prüft weiterhin die
# signierten Repository-Metadaten und daraus die Hashes der Pakete.
ROS_APT_SOURCE="deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${VERSION_CODENAME} main"

# Repariert unseren Eintrag aus einem früheren fehlgeschlagenen Setup bereits
# vor dem ersten apt-get update. Andere Quellen werden nicht verändert.
if [[ -f /etc/apt/sources.list.d/ros2.list ]] && \
    grep -q 'https://packages.ros.org/ros2/ubuntu' /etc/apt/sources.list.d/ros2.list; then
    echo "$ROS_APT_SOURCE" | sudo -n tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

echo "=========================================================="
echo " Linorobot2: Setup Simulationsumgebung (Humble + Fortress)"
echo " Datum: $(date)"
echo " Host: $(hostname), Architektur: $(uname -m)"
echo "=========================================================="

# 1. Paketquellen und Grundwerkzeuge
echo "--> [1/5] Konfiguriere Paketquellen..."
sudo -n apt-get -o APT::Update::Error-Mode=any update
sudo -n apt-get install -y --no-install-recommends \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-zmq

sudo -n add-apt-repository universe -y

# 2. ROS 2 Humble Repository
echo "--> [2/5] Richte ROS 2 Humble Repository ein..."
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --dearmor \
        | sudo -n tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null
fi
echo "$ROS_APT_SOURCE" | sudo -n tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. OSRF Gazebo / Ignition Fortress Repository
echo "--> [3/5] Richte OSRF Gazebo / Ignition Fortress Repository ein..."
if [ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then
    curl -fsSL https://packages.osrfoundation.org/gazebo.key \
        | gpg --dearmor \
        | sudo -n tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg > /dev/null
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo -n tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo -n apt-get -o APT::Update::Error-Mode=any update

# 4. ROS 2 Humble Pakete & Ignition Fortress
echo "--> [4/5] Installiere ROS 2 Humble, RViz, Fortress und Nav2..."
sudo -n apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-ros-base \
    ros-humble-demo-nodes-cpp \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-teleop-twist-keyboard \
    ros-humble-robot-localization \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-rmw-cyclonedds-cpp \
    ignition-fortress \
    ros-humble-ros-ign-bridge \
    ros-humble-ros-ign-gazebo \
    ros-humble-ros-ign-interfaces

# 5. rosdep initialisieren
echo "--> [5/5] Initialisiere rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo -n rosdep init
fi
rosdep update --rosdistro humble

# ROS-Setupdateien unterstützen nounset nicht durchgängig.
set +u
source /opt/ros/humble/setup.bash
set -u

if ! command -v ros2 >/dev/null || ! command -v ign >/dev/null; then
    echo "[FEHLER] Installation unvollständig: ros2 oder ign fehlt im PATH."
    exit 1
fi

echo "=========================================================="
echo " ROS 2 und Ignition sind installiert. Simulation und GUI bleiben separat abzunehmen."
echo " Aktiviere die Umgebung mit: source /opt/ros/humble/setup.bash"
echo "=========================================================="
