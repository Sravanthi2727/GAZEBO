#!/usr/bin/env bash
# Launcher script for Problem 4: Self-Driving Car in a Simulated Road
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

usage() {
    echo "=========================================================================="
    echo " Problem 4: Self-Driving Car (Lane Detection & Obstacle Avoidance)"
    echo "=========================================================================="
    echo "Usage: $0 [sim | bridge | node | all]"
    echo ""
    echo "  sim    : Start Gazebo Sim with road track, markings & obstacle layout"
    echo "  bridge : Start ROS 2 parameter bridge for car topics & camera"
    echo "  node   : Run OpenCV vision lane follower & LiDAR safety controller"
    echo "  all    : Launch simulation, bridge, and self-driving car controller"
    echo "=========================================================================="
}

MODE="${1:-all}"

case "$MODE" in
    sim)
        echo "Starting Gazebo Sim for Problem 4..."
        gz sim -r "$SCRIPT_DIR/problem4_self_driving_car_world.sdf"
        ;;
    bridge)
        echo "Starting ROS 2 <-> Gazebo Bridge for Problem 4..."
        ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$SCRIPT_DIR/problem4_self_driving_car_bridge.yaml"
        ;;
    node)
        python3 "$SCRIPT_DIR/problem4_self_driving_car_node.py"
        ;;
    all)
        echo "[1/3] Launching Gazebo road world in background..."
        gz sim -r "$SCRIPT_DIR/problem4_self_driving_car_world.sdf" &
        SIM_PID=$!
        sleep 4

        echo "[2/3] Launching ROS 2 bridge (camera + LiDAR + cmd_vel) in background..."
        ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$SCRIPT_DIR/problem4_self_driving_car_bridge.yaml" &
        BRIDGE_PID=$!
        sleep 2

        echo "[3/3] Launching Autonomous Self-Driving Car Node..."
        trap 'kill $SIM_PID $BRIDGE_PID 2>/dev/null || true' EXIT
        python3 "$SCRIPT_DIR/problem4_self_driving_car_node.py"
        ;;
    *)
        usage
        exit 1
        ;;
esac
