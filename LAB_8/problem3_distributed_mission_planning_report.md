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

## 4. Fault Injection & Recovery Verification
1. **Normal Multi-Robot Dispatch**: All 4 robots bid on Depots A through D and navigate along collision-free A* trajectories.
2. **Failure Injection**: Run `./problem3_distributed_mission_planning_launcher.sh fail_robot3`.
3. **Detection**: Within $3.5\text{ s}$, surviving peers detect Robot 3's heartbeat silence.
4. **Reassignment**: Robot 3's pending tasks are automatically unassigned and claimed by Robot 1 or 2, proving resilient fault recovery.
