#!/usr/bin/env bash
# Launcher script for Problem 5: Agricultural Field Monitoring Robot
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

usage() {
    echo "=========================================================================="
    echo " Problem 5: Agricultural Field Monitoring Robot Launcher"
    echo "=========================================================================="
    echo "Usage: $0 [sim | bridge | node | all]"
    echo ""
    echo "  sim    : Start Gazebo Sim with crop rows and monitoring stations"
    echo "  bridge : Start ROS 2 parameter bridge for agribot sensors"
    echo "  node   : Run row-following navigation, inspection & logging node"
    echo "  all    : Launch simulation, bridge, and agribot node"
    echo "=========================================================================="
}

MODE="${1:-all}"

case "$MODE" in
    sim)
        echo "Starting Gazebo Sim for Problem 5..."
        gz sim -r "$SCRIPT_DIR/problem5_agricultural_monitoring_world.sdf"
        ;;
    bridge)
        echo "Starting ROS 2 <-> Gazebo Bridge for Problem 5..."
        ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$SCRIPT_DIR/problem5_agricultural_monitoring_bridge.yaml"
        ;;
    node)
        python3 "$SCRIPT_DIR/problem5_agricultural_monitoring_node.py"
        ;;
    all)
        echo "[1/3] Launching Gazebo agricultural world in background..."
        gz sim -r "$SCRIPT_DIR/problem5_agricultural_monitoring_world.sdf" &
        SIM_PID=$!
        sleep 4

        echo "[2/3] Launching ROS 2 bridge in background..."
        ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$SCRIPT_DIR/problem5_agricultural_monitoring_bridge.yaml" &
        BRIDGE_PID=$!
        sleep 2

        echo "[3/3] Launching Agricultural Monitoring Navigation Node..."
        trap 'kill $SIM_PID $BRIDGE_PID 2>/dev/null || true' EXIT
        python3 "$SCRIPT_DIR/problem5_agricultural_monitoring_node.py"
        ;;
    *)
        usage
        exit 1
        ;;
esac
