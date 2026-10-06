# Problem 3: Distributed Multi-Robot Mission Planning and Fault Recovery
**Lab 8: Mobile Robot Navigation Using ROS & Gazebo**

---

## 1. Real-World Application Understanding
In modern automated distribution hubs (e.g. Amazon robotics, automated container terminals), a fleet of 4–6 mobile robots must pick and deliver goods across several depot zones. The system must operate without central single-point bottlenecks: robots dynamically distribute tasks, avoid deadlock in narrow aisles, detect teammate hardware failures, and reallocate orphan missions without human intervention.

---

## 2. System Architecture & Requirements Addressed

| Requirement | Implementation Mechanism in Solution |
| :--- | :--- |
| **Occupancy-Grid Mapping** | 2D discretization ($0.5\text{ m}$ grid) with static storage racks and wall boundaries. |
| **Localization** | Continuous $(x, y, \theta)$ tracking from DiffDrive odometry. |
| **A* Path Planning** | Global grid planner using 8-connectivity and Euclidean distance heuristic. |
| **Task Allocation** | Decentralized Contract Net Protocol (auction) based on distance, battery, and work balance. |
| **Collision Prediction** | Spatial-temporal safety radius prediction ($1.2\text{ m}$) with alphabetical priority yield rules. |
| **Deadlock Detection** | Velocity vs position displacement monitor ($> 2.5\text{ s}$ stall triggers evasive back-off). |
| **Inter-Robot Communication** | Peer `/fleet/heartbeats` and `/fleet/tasks` JSON event broadcasting. |
| **Battery Monitoring** | Linear energetic discharge model with low-battery return trips. |
| **Task Reassignment** | Heartbeat loss triggers immediate task reclamation back to the auction pool. |
| **Robot Failure Detection** | Timeout threshold ($3.5\text{ s}$) detects absent peers or injected faults. |
| **Dynamic Replanning** | LiDAR detects obstacles along A* path and triggers local replan. |
| **FSM Decision Structure** | `IDLE` $\to$ `AUCTION` $\to$ `A_STAR_PLANNING` $\to$ `EXECUTING` $\to$ `REPLANNING` $\to$ `FAULT_RECOVERY`. |

---

## 3. Gazebo Environment Design
Defined in `problem3_distributed_mission_planning_world.sdf`:
- $20\text{ m} \times 16\text{ m}$ logistics facility with central aisles and storage racks.
- 6 task depots: Depots A, B, C, D, E, F distributed around the facility.
- 4 autonomous robots (`robot1`, `robot2`, `robot3`, `robot4`) with distinct colors (Blue, Green, Orange, Purple).

---

## 4. Implementation Files
- **World SDF**: [problem3_distributed_mission_planning_world.sdf](file:///home/sravanthi/GAZEBO/LAB_8/problem3_distributed_mission_planning_world.sdf)
- **Bridge Config**: [problem3_distributed_mission_planning_bridge.yaml](file:///home/sravanthi/GAZEBO/LAB_8/problem3_distributed_mission_planning_bridge.yaml)
- **Fleet Planning Node**: [problem3_distributed_mission_planning_node.py](file:///home/sravanthi/GAZEBO/LAB_8/problem3_distributed_mission_planning_node.py)
- **Launcher**: [problem3_distributed_mission_planning_launcher.sh](file:///home/sravanthi/GAZEBO/LAB_8/problem3_distributed_mission_planning_launcher.sh)

---

## 5. How to Run

### Method A: Standard 3-Terminal Execution (Recommended)
Open **THREE separate terminals**. In **every terminal**, run the environment setup:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/GAZEBO/LAB_8
export LIBGL_ALWAYS_SOFTWARE=1
```

1. **Terminal 1 — Launch Gazebo 4-Robot Logistics Simulation**:
   ```bash
   gz sim -r problem3_distributed_mission_planning_world.sdf
   ```
   *Wait until the warehouse racks, 6 depots, and 4 colored robots appear.*

2. **Terminal 2 — Start 4-Robot ROS 2 Bridge**:
   ```bash
   ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=problem3_distributed_mission_planning_bridge.yaml
   ```
   *Verify topics: `ros2 topic list | grep -E "robot1|robot2|robot3|robot4"`.*

3. **Terminal 3 — Run Distributed Fleet Mission Planners**:
   ```bash
   python3 problem3_distributed_mission_planning_node.py --robot all
   ```
   *(Or launch each robot individually in separate terminals: `--robot robot1`, `--robot robot2`, etc.)*

4. **Terminal 4 (Optional) — Inject Hardware Fault into Robot 3**:
   ```bash
   ros2 topic pub /fleet/inject_failure std_msgs/msg/String "{data: 'robot3'}" --once
   ```
   *Observe the surviving robots detect the silence and reassign Robot 3's pending tasks!*

### Method B: Single Command Launcher
```bash
./problem3_distributed_mission_planning_launcher.sh all
```
Or individual launcher components:
```bash
./problem3_distributed_mission_planning_launcher.sh sim          # Terminal 1
./problem3_distributed_mission_planning_launcher.sh bridge       # Terminal 2
./problem3_distributed_mission_planning_launcher.sh all_nodes    # Terminal 3
./problem3_distributed_mission_planning_launcher.sh fail_robot3  # Terminal 4 (Fault test)
```

---

## 6. Fault Injection & Recovery Verification
1. **Normal Multi-Robot Dispatch**: All 4 robots bid on Depots A through D and navigate along collision-free A* trajectories.
2. **Failure Injection**: Run `./problem3_distributed_mission_planning_launcher.sh fail_robot3`.
3. **Detection**: Within $3.5\text{ s}$, surviving peers detect Robot 3's heartbeat silence.
4. **Reassignment**: Robot 3's pending tasks are automatically unassigned and claimed by Robot 1 or 2, proving resilient fault recovery.
