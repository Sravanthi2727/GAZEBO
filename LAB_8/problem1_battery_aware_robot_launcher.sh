#!/usr/bin/env bash
# Launcher script for Problem 1: Battery-Aware Autonomous Robot Agent
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

usage() {
    echo "=========================================================================="
    echo " Problem 1: Battery-Aware Autonomous Robot Agent Launcher"
    echo "=========================================================================="
    echo "Usage: $0 [sim | bridge | node | all]"
    echo ""
    echo "  sim    : Start Gazebo Sim with the warehouse and battery dock world"
    echo "  bridge : Start ROS 2 <-> Gazebo parameter bridge"
    echo "  node   : Run the autonomous battery-aware decision-making ROS 2 node"
    echo "  all    : Launch sim, bridge, and node in separate background processes"
    echo "=========================================================================="
}

MODE="${1:-all}"

case "$MODE" in
    sim)
        echo "Starting Gazebo Simulation for Problem 1..."
        gz sim -r "$SCRIPT_DIR/problem1_battery_aware_robot_world.sdf"
        ;;
    bridge)
        echo "Starting ROS 2 <-> Gazebo Bridge for Problem 1..."
        ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$SCRIPT_DIR/problem1_battery_aware_robot_bridge.yaml"
        ;;
    node)
        echo "Starting Battery-Aware Autonomous Navigation Node..."
        python3 "$SCRIPT_DIR/problem1_battery_aware_robot_node.py"
        ;;
    all)
        echo "Launching all components for Problem 1..."
        echo "[1/3] Launching Gazebo world in background..."
        gz sim -r "$SCRIPT_DIR/problem1_battery_aware_robot_world.sdf" &
        SIM_PID=$!
        sleep 4

        echo "[2/3] Launching ROS 2 bridge in background..."
        ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$SCRIPT_DIR/problem1_battery_aware_robot_bridge.yaml" &
        BRIDGE_PID=$!
        sleep 2

        echo "[3/3] Launching Problem 1 Python agent..."
        trap 'kill $SIM_PID $BRIDGE_PID 2>/dev/null || true' EXIT
        python3 "$SCRIPT_DIR/problem1_battery_aware_robot_node.py"
        ;;
    *)
        usage
        exit 1
        ;;
esac
