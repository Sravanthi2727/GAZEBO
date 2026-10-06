# Problem 1: Battery-Aware Autonomous Robot Agent
**Lab 8: Mobile Robot Navigation Using ROS & Gazebo**

---

## 1. Real-World Application Understanding
In modern autonomous mobile robotics—such as industrial warehousing (e.g., Kiva / AMR systems), automated hospital logistics, and planetary exploration rovers—mobile agents are untethered and operate on finite onboard battery energy. An agent cannot simply focus on shortest-path navigation or greedy waypoint achievement; it must possess **energy-aware autonomy**. If a robot exhausts its battery far from a charging station, it requires human manual rescue, stalling logistics pipelines and potentially creating critical corridor blockages.

This application requires an agent that:
1. Continuously tracks its internal State of Charge (SoC).
2. Dynamically balances task execution against the energy required to safely reach a charging base.
3. Performs graceful preemption: suspending the current task, safely navigating back to base, rapid-docking, and resuming the exact mission sequence once recharged.

---

## 2. Navigation & Autonomy Problem Identification
The core challenges identified are:
- **State Estimation & Odometry**: Real-time localization relative to known charging dock and task stations.
- **Energy Budgeting & Safety Horizons**: Calculating the energetic cost-to-go back to dock, defining safe tripwire thresholds ($E_{thresh} = 28\%$) with dynamic distance margin estimation.
- **Obstacle Avoidance en Route**: Steering around warehouse crates and boundary walls using LiDAR range data.
- **Finite State Machine (FSM) Decision Engine**: Seamlessly transitioning between `IDLE`, `NAVIGATING_TO_TASK`, `EXECUTING_TASK`, `RETURNING_TO_CHARGER`, `CHARGING`, and `RESUMING_TASK` without losing mission context.

---

## 3. Sensors, ROS Topics/Nodes & Navigation Method

### Sensors
- **2D Planar LiDAR (`gpu_lidar`)**: Range 0.25 m to 10.0 m, 180 horizontal samples. Detects obstacle proximity in front, left, and right angular sectors.
- **Wheel Encoders / Odometry**: Integrated diff-drive dead-reckoning supplying $(x, y, \theta)$ position and orientation.

### ROS Topics
| Topic Name | Message Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/problem1/cmd_vel` | `geometry_msgs/msg/Twist` | ROS $\to$ Gazebo | Velocity commands (linear $v_x$, angular $\omega_z$) |
| `/problem1/odometry` | `nav_msgs/msg/Odometry` | Gazebo $\to$ ROS | Continuous pose and velocity feedback |
| `/problem1/scan` | `sensor_msgs/msg/LaserScan` | Gazebo $\to$ ROS | 2D LiDAR range profile |
| `/problem1/battery_state` | `sensor_msgs/msg/BatteryState`| ROS Internal Pub | Publishes simulated battery percentage & voltage |
| `/problem1/task_status` | `std_msgs/msg/String` | ROS Internal Pub | Status string showing FSM state & current task index |

### Navigation Method
- **Proportional Heading & Velocity Regulation**: Calculates angular heading error $e_\theta = \text{atan2}(\Delta y, \Delta x) - \theta_{robot}$, applying proportional steering $\omega = 1.2 \cdot e_\theta$ and distance-scaled forward speed.
- **Reactive Sector LiDAR Repulsion**: If front obstacle distance $< 0.65\text{ m}$, suppresses forward drive and steers away from the nearest side obstacle.
- **Dynamic Battery State Machine**:
  ```mermaid
  stateDiagram-v2
      [*] --> IDLE
      IDLE --> NAVIGATING_TO_TASK : Start Mission
      NAVIGATING_TO_TASK --> EXECUTING_TASK : Arrived at Workstation
      NAVIGATING_TO_TASK --> RETURNING_TO_CHARGER : Battery <= Threshold
      EXECUTING_TASK --> NAVIGATING_TO_TASK : Task Complete (Next Station)
      EXECUTING_TASK --> RETURNING_TO_CHARGER : Battery <= Threshold
      RETURNING_TO_CHARGER --> CHARGING : Arrived at Dock Pad
      CHARGING --> RESUMING_TASK : Battery >= 95%
      RESUMING_TASK --> NAVIGATING_TO_TASK : Navigate to Saved Interrupted Task
      EXECUTING_TASK --> MISSION_COMPLETED : All Tasks Complete
  ```

---

## 4. Gazebo Environment Design
The environment is specified in `problem1_battery_aware_robot_world.sdf`:
- **Dimensions**: Enclosed $16\text{ m} \times 14\text{ m}$ industrial staging floor.
- **Central Charging Dock**: Located at $(0.0, 0.0)$, featuring a distinctive illuminated cyan charging base pad and vertical docking pylon.
- **4 Distributed Workstations**:
  - Task 1: Inspection Station A at $(+4.5, +3.5)$ [Yellow Pad]
  - Task 2: Material Pickup Station B at $(+5.0, -4.0)$ [Cyan Pad]
  - Task 3: Assembly Station C at $(-4.5, -3.5)$ [Magenta Pad]
  - Task 4: Quality Check Station D at $(-4.0, +4.0)$ [Orange Pad]
- **Warehouse Obstacles**: 4 sets of wooden cargo crates strategically placed between corridors to force non-trivial paths and test obstacle avoidance.

---

## 5. Implementation Files
- **World SDF**: [problem1_battery_aware_robot_world.sdf](file:///home/sravanthi/GAZEBO/LAB_8/problem1_battery_aware_robot_world.sdf)
- **Bridge Config**: [problem1_battery_aware_robot_bridge.yaml](file:///home/sravanthi/GAZEBO/LAB_8/problem1_battery_aware_robot_bridge.yaml)
- **Autonomous Node**: [problem1_battery_aware_robot_node.py](file:///home/sravanthi/GAZEBO/LAB_8/problem1_battery_aware_robot_node.py)
- **Launcher**: [problem1_battery_aware_robot_launcher.sh](file:///home/sravanthi/GAZEBO/LAB_8/problem1_battery_aware_robot_launcher.sh)

---

## 6. How to Run

### Method A: Standard 3-Terminal Execution (Recommended)
Open **THREE separate terminals**. In **every terminal**, run the environment setup:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/GAZEBO/LAB_8
export LIBGL_ALWAYS_SOFTWARE=1
```

1. **Terminal 1 — Launch Gazebo Simulation World**:
   ```bash
   gz sim -r problem1_battery_aware_robot_world.sdf
   ```
   *Wait until the warehouse arena, charging dock pylon, workstations, and robot appear.*

2. **Terminal 2 — Start ROS 2 <-> Gazebo Bridge**:
   ```bash
   ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=problem1_battery_aware_robot_bridge.yaml
   ```
   *Verify topics with: `ros2 topic list` and `ros2 topic echo /problem1/odometry --once`.*

3. **Terminal 3 — Run Battery-Aware Autonomous Navigation Node**:
   ```bash
   python3 problem1_battery_aware_robot_node.py
   ```

### Method B: Single Command Launcher
Alternatively, run the automated background launcher script:
```bash
./problem1_battery_aware_robot_launcher.sh all
```
Or start individual components:
```bash
./problem1_battery_aware_robot_launcher.sh sim     # Terminal 1
./problem1_battery_aware_robot_launcher.sh bridge  # Terminal 2
./problem1_battery_aware_robot_launcher.sh node    # Terminal 3
```

---

## 7. Testing Conditions & Observations

### Condition 1: Nominal Mission with Ample Energy
- **Setup**: Start robot with 100% battery.
- **Observation**: Robot successfully navigates to Task 1, executes for 3.0s, moves to Task 2. Battery drains at expected rate (~1.35%/s during transit, ~0.15%/s during task dwell).

### Condition 2: Low-Battery Trigger & Task Preemption
- **Setup**: En route to Task 3, battery drops to $\le 28.0\%$.
- **Observation**: FSM immediately issues preemption warning, records interrupted task ID (`Task 3`), aborts transit, and charts a return course to dock $(0.0, 0.0)$. Obstacles are successfully bypassed.

### Condition 3: Docking, Rapid Recharge, and Mission Resumption
- **Setup**: Robot reaches dock within $0.35\text{ m}$ tolerance.
- **Observation**: Motion ceases ($v=0, \omega=0$). State changes to `CHARGING`. Battery level climbs at $+7.5\%/\text{s}$. Upon hitting $95.0\%$, state transitions to `RESUMING_TASK`, accurately redirecting the robot back to Task 3 and completing Task 4.

---

## 7. Results & Key Takeaways
- The robot never becomes stranded in the field.
- The preemptive return-to-base mechanism ensures $100\%$ task completion reliability even under severe energy constraints.
