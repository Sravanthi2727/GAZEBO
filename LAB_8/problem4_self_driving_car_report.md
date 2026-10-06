# Problem 4: Self-Driving Car in a Simulated Road
**Lab 8: Mobile Robot Navigation Using ROS & Gazebo**

---

## 1. Real-World Application Understanding
Autonomous passenger vehicles and campus shuttles (e.g. Waymo, Tesla Autopilot, Baidu Apollo) navigate roadways by fusing optical cameras and LiDAR. Optical cameras detect subtle visual road markings (yellow centerlines, solid white road shoulder lines), while LiDAR guarantees millimeter-precision obstacle distance detection in all lighting conditions.

---

## 2. Navigation & Control Problem Identification
- **Vision Lane Detection**: Extract lane boundaries and centerline dashes from raw RGB camera frames against varied backgrounds.
- **Centroid & Error Calculation**: Compute normalized lateral deviation error $e_y$ between vehicle camera center and detected lane track.
- **PID Lateral Steering Control**: Apply proportional-derivative steering to keep the vehicle smoothly centered between lane markers.
- **LiDAR Distance & Obstacle Avoidance**: Monitor forward sector ($\pm 25^\circ$). Implement multi-zone adaptive cruise control:
  1. *Cruise Zone* ($d > 3.2\text{ m}$): Nominal velocity ($0.42\text{ m/s}$).
  2. *Caution Deceleration Zone* ($1.4\text{ m} < d \le 3.2\text{ m}$): Linear braking.
  3. *Emergency Stop / Evasive Swerve Zone* ($d \le 1.4\text{ m}$): Safe stopping and lane change bypass around road obstacles.

---

## 3. Sensors, Topics & Controller Pipeline

### Sensors
- **RGB Road Camera**: Angled downward forward ($35^\circ$ pitch) on front bumper, publishing $640 \times 480$ images at $15\text{ Hz}$.
- **Planar LiDAR**: $180^\circ$ scan range on roof, measuring distances up to $15.0\text{ m}$.

### ROS Topics
| Topic Name | Message Type | Purpose |
| :--- | :--- | :--- |
| `/car/cmd_vel` | `geometry_msgs/msg/Twist` | Steering and throttle output |
| `/car/odometry` | `nav_msgs/msg/Odometry` | Wheel odometry & vehicle pose |
| `/car/scan` | `sensor_msgs/msg/LaserScan` | Forward obstacle range detection |
| `/car/camera/image_raw` | `sensor_msgs/msg/Image` | Forward road video stream |

---

## 4. Gazebo Environment Design
Defined in `problem4_self_driving_car_world.sdf`:
- $32\text{ m}$ asphalt roadway surface with dark gray asphalt material.
- Outer solid white lane boundary lines.
- Center yellow dashed line markers spaced at $4.0\text{ m}$ intervals.
- Concrete road curbs on left and right sides.
- Static obstacles (traffic cones, construction obstacle barriers) placed on the roadway.
- 4-wheel vehicle model with realistic chassis, 4 independent wheels, camera, and roof LiDAR.

---

## 5. Testing Conditions & Observations
1. **Clear Road Lane Tracking**: In the absence of obstacles, the vehicle locks onto the yellow dashes and white boundary lines, maintaining lateral position with error $|e_y| < 0.08$.
2. **Obstacle Deceleration**: When approaching Obstacle 1 at $10.0\text{ m}$, the LiDAR detects the obstacle at $3.2\text{ m}$ and smoothly decreases speed from $0.42\text{ m/s}$ down to $0.20\text{ m/s}$.
3. **Emergency Evasive Bypass**: At $1.4\text{ m}$, the vehicle initiates an evasive swerve into the clear adjacent lane space, avoiding collision and resuming lane tracking.
