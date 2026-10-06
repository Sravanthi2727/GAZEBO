#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")"
echo "Bridging /robot/cmd_vel, /robot/odometry, /robot/scan"
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$PWD/ros_gz_bridge.yaml"
