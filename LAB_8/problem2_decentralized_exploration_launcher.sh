#!/usr/bin/env bash
# Launcher script for Problem 2: Decentralized Multi-Robot Exploration
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

usage() {
    echo "=========================================================================="
    echo " Problem 2: Decentralized Multi-Robot Exploration Launcher"
    echo "=========================================================================="
    echo "Usage: $0 [sim | bridge | node1 | node2 | node3 | all_nodes | all]"
    echo ""
    echo "  sim       : Start Gazebo Sim with multi-room environment & 3 robots"
    echo "  bridge    : Start ROS 2 parameter bridge for robot1, robot2, robot3"
    echo "  node1     : Run decentralized agent for robot1"
    echo "  node2     : Run decentralized agent for robot2"
    echo "  node3     : Run decentralized agent for robot3"
    echo "  all_nodes : Run all 3 decentralized agents together"
    echo "  all       : Launch simulation, bridge, and all agents"
    echo "=========================================================================="
}

MODE="${1:-all}"

case "$MODE" in
    sim)
        echo "Starting Gazebo Sim for Problem 2..."
        gz sim -r "$SCRIPT_DIR/problem2_decentralized_exploration_world.sdf"
        ;;
    bridge)
        echo "Starting ROS 2 <-> Gazebo Bridge for Problem 2..."
        ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$SCRIPT_DIR/problem2_decentralized_exploration_bridge.yaml"
        ;;
    node1)
        python3 "$SCRIPT_DIR/problem2_decentralized_exploration_node.py" --robot robot1
        ;;
    node2)
        python3 "$SCRIPT_DIR/problem2_decentralized_exploration_node.py" --robot robot2
        ;;
    node3)
        python3 "$SCRIPT_DIR/problem2_decentralized_exploration_node.py" --robot robot3
        ;;
    all_nodes)
        python3 "$SCRIPT_DIR/problem2_decentralized_exploration_node.py" --robot all
        ;;
    all)
        echo "[1/3] Launching Gazebo world in background..."
        gz sim -r "$SCRIPT_DIR/problem2_decentralized_exploration_world.sdf" &
        SIM_PID=$!
        sleep 4

        echo "[2/3] Launching multi-robot ROS bridge in background..."
        ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$SCRIPT_DIR/problem2_decentralized_exploration_bridge.yaml" &
        BRIDGE_PID=$!
        sleep 2

        echo "[3/3] Launching 3 decentralized exploration nodes..."
        trap 'kill $SIM_PID $BRIDGE_PID 2>/dev/null || true' EXIT
        python3 "$SCRIPT_DIR/problem2_decentralized_exploration_node.py" --robot all
        ;;
    *)
        usage
        exit 1
        ;;
esac
