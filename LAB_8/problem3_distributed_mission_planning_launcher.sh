#!/usr/bin/env bash
# Launcher script for Problem 3: Distributed Multi-Robot Mission Planning and Fault Recovery
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

usage() {
    echo "=========================================================================="
    echo " Problem 3: Distributed Multi-Robot Mission Planning & Fault Recovery"
    echo "=========================================================================="
    echo "Usage: $0 [sim | bridge | node1 | node2 | node3 | node4 | all_nodes | fail_robot3 | all]"
    echo ""
    echo "  sim          : Start Gazebo Sim with 4 robots & warehouse depots"
    echo "  bridge       : Start ROS 2 parameter bridge for all 4 robots"
    echo "  node1..4     : Run individual distributed robot agent"
    echo "  all_nodes    : Run all 4 fleet agents simultaneously"
    echo "  fail_robot3  : Injects a hardware failure into robot3 to test recovery"
    echo "  all          : Launch simulation, bridge, and fleet agents"
    echo "=========================================================================="
}

MODE="${1:-all}"

case "$MODE" in
    sim)
        echo "Starting Gazebo Sim for Problem 3..."
        gz sim -r "$SCRIPT_DIR/problem3_distributed_mission_planning_world.sdf"
        ;;
    bridge)
        echo "Starting ROS 2 <-> Gazebo Bridge for Problem 3..."
        ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$SCRIPT_DIR/problem3_distributed_mission_planning_bridge.yaml"
        ;;
    node1)
        python3 "$SCRIPT_DIR/problem3_distributed_mission_planning_node.py" --robot robot1
        ;;
    node2)
        python3 "$SCRIPT_DIR/problem3_distributed_mission_planning_node.py" --robot robot2
        ;;
    node3)
        python3 "$SCRIPT_DIR/problem3_distributed_mission_planning_node.py" --robot robot3
        ;;
    node4)
        python3 "$SCRIPT_DIR/problem3_distributed_mission_planning_node.py" --robot robot4
        ;;
    all_nodes)
        python3 "$SCRIPT_DIR/problem3_distributed_mission_planning_node.py" --robot all
        ;;
    fail_robot3)
        echo "Triggering failure injection in robot3..."
        ros2 topic pub /fleet/inject_failure std_msgs/msg/String "{data: 'robot3'}" --once
        ;;
    all)
        echo "[1/3] Launching Gazebo world in background..."
        gz sim -r "$SCRIPT_DIR/problem3_distributed_mission_planning_world.sdf" &
        SIM_PID=$!
        sleep 4

        echo "[2/3] Launching 4-robot ROS bridge in background..."
        ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$SCRIPT_DIR/problem3_distributed_mission_planning_bridge.yaml" &
        BRIDGE_PID=$!
        sleep 2

        echo "[3/3] Launching distributed fleet agents..."
        trap 'kill $SIM_PID $BRIDGE_PID 2>/dev/null || true' EXIT
        python3 "$SCRIPT_DIR/problem3_distributed_mission_planning_node.py" --robot all
        ;;
    *)
        usage
        exit 1
        ;;
esac
