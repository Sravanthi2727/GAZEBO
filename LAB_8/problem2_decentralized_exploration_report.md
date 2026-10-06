# Problem 2: Decentralized Multi-Robot Exploration
**Lab 8: Mobile Robot Navigation Using ROS & Gazebo**

---

## 1. Real-World Application Understanding
In search-and-rescue, planetary exploration, or sub-surface tunnel mapping, a single centralized controller presents a single point of failure and suffers from bandwidth bottlenecks. A swarm or team of autonomous rovers must explore an unknown partitioned layout without any central coordinator. Each rover operates as an independent decentralized agent.

---

## 2. Navigation Problem Identification
1. **Dispersed Starting Positions**: Robots spawn in separate rooms with no initial shared map.
2. **Local Sensor Horizons**: Each robot only sees what is directly within its LiDAR range and must construct its own local occupancy map.
3. **Frontier-Based Exploration**: Boundary cells separating known free space from unmapped space are identified and ranked.
4. **Preventing Redundant Exploration**: Without coordination, robots may converge on the same room. A peer-to-peer (P2P) cost-utility broadcast mechanism penalizes frontiers already claimed by peers.
5. **Handling Communication Loss**: Robots must seamlessly cope with radio dropouts or out-of-range situations by falling back to local greedy exploration without stalling.

---

## 3. Required Sensors, ROS Topics/Nodes & Navigation Method

### Sensors
- **2D Planar LiDAR (`/robot{i}/scan`)**: $180^\circ$ scan range, up to $8.0\text{ m}$.
- **Differential Drive Odometry (`/robot{i}/odometry`)**: For local dead-reckoning and occupancy grid projection.

### ROS Topics
| Topic Name | Message Type | Purpose |
| :--- | :--- | :--- |
| `/{robot_id}/cmd_vel` | `geometry_msgs/msg/Twist` | Individual robot velocity |
| `/{robot_id}/odometry` | `nav_msgs/msg/Odometry` | Individual robot position |
| `/{robot_id}/scan` | `sensor_msgs/msg/LaserScan` | Onboard LiDAR measurements |
| `/exploration/peer_broadcast` | `std_msgs/msg/String` (JSON) | Ad-hoc P2P status, pose & claimed frontier |

### Navigation Method
- **Local Occupancy Grid Mapping**: Raytracing via Bresenham algorithm with cell resolution of $0.35\text{ m}$.
- **Frontier Detection**: Free cells adjacent to $-1$ (unknown).
- **Cost-Utility & Anti-Redundancy Scoring**:
  $$\text{Score}(f) = U_{base} - \alpha \cdot \text{dist}(f, \mathbf{x}_{robot}) - \sum_{j \neq i} \beta \cdot \text{penalty}(f, \mathbf{goal}_j)$$
- **Communication Loss Fallback**: If $\text{dist}(\text{peer}) > 7.0\text{ m}$ or no heartbeat for $> 4.0\text{ s}$, peer is considered dropped. Exploration continues safely based on local knowledge.

---

## 4. Gazebo Environment Design
Defined in `problem2_decentralized_exploration_world.sdf`:
- $16\text{ m} \times 16\text{ m}$ multi-room building partitioned into 4 distinct quadrants (Southwest, Southeast, Northwest, Northeast) with central crossing doorways.
- Internal structural columns in each quadrant.
- 3 robots spawned at distant locations:
  - `robot1` at $(-5.5, -5.0)$ [Blue]
  - `robot2` at $(+5.5, -5.0)$ [Green]
  - `robot3` at $(0.0, +5.5)$ [Orange]

---

## 5. Implementation Files
- **World SDF**: [problem2_decentralized_exploration_world.sdf](file:///home/sravanthi/GAZEBO/LAB_8/problem2_decentralized_exploration_world.sdf)
- **Bridge Config**: [problem2_decentralized_exploration_bridge.yaml](file:///home/sravanthi/GAZEBO/LAB_8/problem2_decentralized_exploration_bridge.yaml)
- **Decentralized Node**: [problem2_decentralized_exploration_node.py](file:///home/sravanthi/GAZEBO/LAB_8/problem2_decentralized_exploration_node.py)
- **Launcher**: [problem2_decentralized_exploration_launcher.sh](file:///home/sravanthi/GAZEBO/LAB_8/problem2_decentralized_exploration_launcher.sh)

---

## 6. How to Run

### Method A: Standard 3-Terminal Execution (Recommended)
Open **THREE separate terminals**. In **every terminal**, run the environment setup:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/GAZEBO/LAB_8
export LIBGL_ALWAYS_SOFTWARE=1
```

1. **Terminal 1 — Launch Multi-Robot Multi-Room Simulation World**:
   ```bash
   gz sim -r problem2_decentralized_exploration_world.sdf
   ```
   *Wait until the 4-room building and the 3 mobile robots appear in their respective rooms.*

2. **Terminal 2 — Start Multi-Robot ROS 2 Bridge**:
   ```bash
   ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=problem2_decentralized_exploration_bridge.yaml
   ```
   *Verify topics: `ros2 topic list | grep -E "robot1|robot2|robot3"`.*

3. **Terminal 3 — Run All 3 Decentralized Exploration Agents**:
   ```bash
   python3 problem2_decentralized_exploration_node.py --robot all
   ```
   *(Or launch each robot independently in its own terminal)*:
   ```bash
   # Terminal 3:
   python3 problem2_decentralized_exploration_node.py --robot robot1
   # Terminal 4:
   python3 problem2_decentralized_exploration_node.py --robot robot2
   # Terminal 5:
   python3 problem2_decentralized_exploration_node.py --robot robot3
   ```

### Method B: Single Command Launcher
```bash
./problem2_decentralized_exploration_launcher.sh all
```
Or start components individually:
```bash
./problem2_decentralized_exploration_launcher.sh sim        # Terminal 1
./problem2_decentralized_exploration_launcher.sh bridge     # Terminal 2
./problem2_decentralized_exploration_launcher.sh all_nodes  # Terminal 3
```

---

## 7. Testing Conditions & Observations
1. **Baseline Exploration**: Each robot discovers its local room rapidly without interference.
2. **P2P Goal Negotiation**: When Robot 1 and Robot 2 approach the central doorway, peer broadcast penalties divert Robot 1 toward Northwest and Robot 2 toward Northeast, completely avoiding redundant coverage.
3. **Communication Dropout**: When Robot 3 enters the far northern corridor beyond the $7.0\text{ m}$ radio threshold, it drops peer packets and continues local room exploration seamlessly without crashing or freezing.
