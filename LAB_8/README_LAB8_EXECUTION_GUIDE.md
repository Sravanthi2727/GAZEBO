# Lab 8: Mobile Robot Navigation Using ROS & Gazebo
**Complete Implementation and Experimentation Suite**

This repository contains full implementations for all 5 problems of **Lab 8: Mobile Robot Navigation Using ROS & Gazebo**, configured for **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Sim 8 (Harmonic)**.

---

## Table of Contents & File Organization

All files strictly follow the requested naming convention containing the **question number** and the **task it performs**:

| Problem | World SDF | Bridge YAML | ROS 2 Python Node | Shell Launcher | Technical Report |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Problem 1: Battery-Aware Autonomous Robot** | [problem1_battery_aware_robot_world.sdf](file:///home/sravanthi/GAZEBO/LAB_8/problem1_battery_aware_robot_world.sdf) | [problem1_battery_aware_robot_bridge.yaml](file:///home/sravanthi/GAZEBO/LAB_8/problem1_battery_aware_robot_bridge.yaml) | [problem1_battery_aware_robot_node.py](file:///home/sravanthi/GAZEBO/LAB_8/problem1_battery_aware_robot_node.py) | [problem1_battery_aware_robot_launcher.sh](file:///home/sravanthi/GAZEBO/LAB_8/problem1_battery_aware_robot_launcher.sh) | [problem1_battery_aware_robot_report.md](file:///home/sravanthi/GAZEBO/LAB_8/problem1_battery_aware_robot_report.md) |
| **Problem 2: Decentralized Multi-Robot Exploration** | [problem2_decentralized_exploration_world.sdf](file:///home/sravanthi/GAZEBO/LAB_8/problem2_decentralized_exploration_world.sdf) | [problem2_decentralized_exploration_bridge.yaml](file:///home/sravanthi/GAZEBO/LAB_8/problem2_decentralized_exploration_bridge.yaml) | [problem2_decentralized_exploration_node.py](file:///home/sravanthi/GAZEBO/LAB_8/problem2_decentralized_exploration_node.py) | [problem2_decentralized_exploration_launcher.sh](file:///home/sravanthi/GAZEBO/LAB_8/problem2_decentralized_exploration_launcher.sh) | [problem2_decentralized_exploration_report.md](file:///home/sravanthi/GAZEBO/LAB_8/problem2_decentralized_exploration_report.md) |
| **Problem 3: Distributed Multi-Robot Mission Planning** | [problem3_distributed_mission_planning_world.sdf](file:///home/sravanthi/GAZEBO/LAB_8/problem3_distributed_mission_planning_world.sdf) | [problem3_distributed_mission_planning_bridge.yaml](file:///home/sravanthi/GAZEBO/LAB_8/problem3_distributed_mission_planning_bridge.yaml) | [problem3_distributed_mission_planning_node.py](file:///home/sravanthi/GAZEBO/LAB_8/problem3_distributed_mission_planning_node.py) | [problem3_distributed_mission_planning_launcher.sh](file:///home/sravanthi/GAZEBO/LAB_8/problem3_distributed_mission_planning_launcher.sh) | [problem3_distributed_mission_planning_report.md](file:///home/sravanthi/GAZEBO/LAB_8/problem3_distributed_mission_planning_report.md) |
| **Problem 4: Self-Driving Car on Road** | [problem4_self_driving_car_world.sdf](file:///home/sravanthi/GAZEBO/LAB_8/problem4_self_driving_car_world.sdf) | [problem4_self_driving_car_bridge.yaml](file:///home/sravanthi/GAZEBO/LAB_8/problem4_self_driving_car_bridge.yaml) | [problem4_self_driving_car_node.py](file:///home/sravanthi/GAZEBO/LAB_8/problem4_self_driving_car_node.py) | [problem4_self_driving_car_launcher.sh](file:///home/sravanthi/GAZEBO/LAB_8/problem4_self_driving_car_launcher.sh) | [problem4_self_driving_car_report.md](file:///home/sravanthi/GAZEBO/LAB_8/problem4_self_driving_car_report.md) |
| **Problem 5: Agricultural Monitoring Robot** | [problem5_agricultural_monitoring_world.sdf](file:///home/sravanthi/GAZEBO/LAB_8/problem5_agricultural_monitoring_world.sdf) | [problem5_agricultural_monitoring_bridge.yaml](file:///home/sravanthi/GAZEBO/LAB_8/problem5_agricultural_monitoring_bridge.yaml) | [problem5_agricultural_monitoring_node.py](file:///home/sravanthi/GAZEBO/LAB_8/problem5_agricultural_monitoring_node.py) | [problem5_agricultural_monitoring_launcher.sh](file:///home/sravanthi/GAZEBO/LAB_8/problem5_agricultural_monitoring_launcher.sh) | [problem5_agricultural_monitoring_report.md](file:///home/sravanthi/GAZEBO/LAB_8/problem5_agricultural_monitoring_report.md) |

---

## Problem-by-Problem Architecture & Overview

### Problem 1: Battery-Aware Autonomous Robot Agent
- **Application**: AMR warehouse or hospital delivery vehicle subject to battery discharge constraints.
- **Navigation Problem**: Executing a 4-waypoint mission while continuously estimating required energy to return to the charging station; autonomously interrupting tasks upon reaching low battery ($< 28\%$), returning to the base dock, rapid recharging to $95\%$, and resuming unfinished tasks.
- **Sensors**: DiffDrive odometry (`/problem1/odometry`), Planar LiDAR (`/problem1/scan`).
- **ROS Node & Topics**: `Problem1BatteryAwareRobotNode` publishing `/problem1/cmd_vel`, `/problem1/battery_state`, and `/problem1/task_status`.

### Problem 2: Decentralized Multi-Robot Exploration
- **Application**: Multi-rover search-and-rescue reconnaissance in an unknown partitioned multi-room facility without a central server.
- **Navigation Problem**: 3 robots starting in different quadrants; each robot builds its own local occupancy grid map via LiDAR raytracing; extracts boundary frontiers; calculates cost-utility scores with peer penalties over ad-hoc peer broadcast (`/exploration/peer_broadcast`); gracefully degrades to local exploration if communication is lost.
- **Sensors**: Per-robot LiDAR (`/robot{i}/scan`) and Odometry (`/robot{i}/odometry`).
- **ROS Node & Topics**: `DecentralizedExplorerAgent` supporting standalone invocation per robot or multi-threaded simultaneous launch (`--robot all`).

### Problem 3: Distributed Multi-Robot Mission Planning & Fault Recovery
- **Application**: Classical multi-agent automated factory fleet with 4 autonomous mobile robots.
- **Navigation Problem**: Dynamic mission planning across 6 depot stations; includes:
  - 2D Occupancy-grid representation
  - Global A* path planning
  - Market-based task auction (Contract Net Protocol)
  - Collision prediction and deadlock back-off resolution
  - Heartbeat failure detection ($3.5\text{ s}$ timeout)
  - Dynamic task reassignment to surviving robots when a robot fails
  - Dynamic replanning around path obstacles.
- **Sensors & Fleet Comms**: LiDAR and Odometry on all 4 robots, `/fleet/heartbeats`, `/fleet/tasks`, `/fleet/inject_failure`.

### Problem 4: Self-Driving Car in a Simulated Road
- **Application**: Autonomous vehicle road lane following and collision prevention.
- **Navigation Problem**: 4-wheel vehicle equipped with forward RGB camera and roof LiDAR; detects road centerline yellow dashes and white outer boundary markers using OpenCV HSV segmentation and centroid calculation; implements lateral PID steering; detects road obstacles with LiDAR and regulates speed between Cruise ($0.42\text{ m/s}$), Caution ($0.20\text{ m/s}$), and Emergency Stop/Swerve ($< 1.4\text{ m}$).
- **Sensors**: Forward RGB Camera (`/car/camera/image_raw`), Roof LiDAR (`/car/scan`), Odometry (`/car/odometry`).

### Problem 5: Agricultural Field Monitoring Robot
- **Application**: Precision agricultural phenotyping and field health monitoring.
- **Navigation Problem**: High-clearance mobile robot moving through parallel crop hedges; implements bilateral LiDAR crop row centering; executes headland $180^\circ$ turns; visits 5 designated inspection stations; captures crop images, computes the Excess Green Index ($ExG = 2G - R - B$) and canopy coverage; records timestamped metrics and $(x, y, \theta)$ positions to `problem5_field_monitoring_log.csv`.
- **Sensors**: Planar LiDAR (`/agribot/scan`), Crop inspection camera (`/agribot/camera/image_raw`), Odometry (`/agribot/odometry`).

---

## Execution Instructions

Follow the 3-terminal workflow outlined in [HOW_TO_RUN_LAB8.txt](file:///home/sravanthi/GAZEBO/LAB_8/HOW_TO_RUN_LAB8.txt) or use the self-contained launcher scripts:

```bash
# Problem 1:
./problem1_battery_aware_robot_launcher.sh all

# Problem 2:
./problem2_decentralized_exploration_launcher.sh all

# Problem 3:
./problem3_distributed_mission_planning_launcher.sh all

# Problem 4:
./problem4_self_driving_car_launcher.sh all

# Problem 5:
./problem5_agricultural_monitoring_launcher.sh all
```
