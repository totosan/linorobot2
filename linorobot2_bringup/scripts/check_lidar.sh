#!/bin/bash

# Check if ydlidar device exists
if [ ! -e "/dev/ydlidar" ]; then
    echo "ERROR: /dev/ydlidar device not found!"
    echo "Please check if the lidar is connected and the device exists."
    exit 1
fi

echo "Lidar device found at /dev/ydlidar"
exec "$@"
