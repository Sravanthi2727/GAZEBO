# Problem 5: Agricultural Field Monitoring Robot
**Lab 8: Mobile Robot Navigation Using ROS & Gazebo**

---

## 1. Real-World Application Understanding
Precision agriculture requires autonomous field robots to survey crop rows, monitor soil conditions, detect weed outbreaks, and compute canopy vegetation indexes (e.g., NDVI, Excess Green ExG) at designated monitoring points without damaging crops.

---

## 2. Navigation & Inspection Problem Identification
- **Crop Row Extraction & Centering**: Balancing bilateral distances between left and right crop rows ($d_{left} \approx d_{right}$) using planar LiDAR.
- **Headland Turnaround**: Detecting the transition from crop alleyway to open headland and executing an autonomous $180^\circ$ turnaround into the adjacent crop row.
- **Monitoring Station Identification**: Locating predefined sampling stations along the crop rows.
- **Data Logging & Canopy Analysis**: Extracting RGB image data, computing Excess Green Index ($ExG = 2G - R - B$), recording spatial coordinates, and persisting telemetry to a structured CSV log.

---

## 3. Sensors, Topics & Data Logging

### Sensors
- **Distance Sensor / Planar LiDAR**: Measures perpendicular distances to crop hedges on both sides.
- **Crop Canopy Camera**: Captures plant foliage at monitoring stations.
- **Wheel Odometry**: Spatial coordinates $(x, y, \theta)$.

### ROS Topics
| Topic Name | Message Type | Purpose |
| :--- | :--- | :--- |
| `/agribot/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands |
| `/agribot/odometry` | `nav_msgs/msg/Odometry` | Odometric localization |
| `/agribot/scan` | `sensor_msgs/msg/LaserScan` | Row wall distances |
| `/agribot/camera/image_raw` | `sensor_msgs/msg/Image` | Visual canopy data |
| `/agribot/crop_health_report`| `std_msgs/msg/String` | Real-time inspection telemetry |

### Structured CSV Telemetry Output
Logged to `problem5_field_monitoring_log.csv`:
```csv
Timestamp_ISO,Station_ID,Station_Name,Robot_X,Robot_Y,Robot_Yaw_deg,Left_Row_Dist_m,Right_Row_Dist_m,ExG_Canopy_Index,Green_Canopy_Percent,Health_Status
2026-10-06 14:35:10,Station_1,Row 1 (West),1.02,1.11,0.2,1.08,1.12,24.50,82.4,EXCELLENT
...
```

---

## 4. Gazebo Environment Design
Defined in `problem5_agricultural_monitoring_world.sdf`:
- Brown soil agricultural field ($22\text{ m} \times 11\text{ m}$).
- 4 parallel crop rows (dense green foliage hedges, length $16.0\text{ m}$, row spacing $2.2\text{ m}$).
- Headland turnaround clearance on both east and west ends.
- 5 designated monitoring inspection points with color-coded inspection flagposts (Red, Blue, Yellow, Cyan, Magenta).
- High-clearance agricultural mobile robot equipped with diff-drive, distance sensor, and canopy camera.

---

## 5. Testing Conditions & Observations
1. **Row Centering**: As the robot moves through Row 1, bilateral LiDAR distances are maintained at $1.10\text{ m} \pm 0.05\text{ m}$, preventing any hedge collisions.
2. **Station Dwell & Inspection**: Upon reaching Station 1 $(1.0, 1.1)$, the robot halts for $3.0\text{ s}$, samples the camera feed, calculates $ExG = 24.5$, and appends a record to `problem5_field_monitoring_log.csv`.
3. **Headland Turn**: At the eastern row end ($x > 9.5\text{ m}$), the robot executes a smooth $180^\circ$ turnaround into Row 2 and continues surveying Stations 3 through 5.
