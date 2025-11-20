# System Architecture — Autonomous Grocery & Food Delivery Robot

**Project**: Autonomous Grocery and Food Delivery Robot for IIT Mandi Campus  
**Institution**: Indian Institute of Technology Mandi  
**Target Platform**: Raspberry Pi 4B (4GB/8GB) running Ubuntu 22.04 LTS & ROS 2 Humble  

---

## 1. System Overview

The Autonomous Grocery and Food Delivery Robot is engineered to navigate the pedestrian roads, paved corridors, and outdoor ramps of the IIT Mandi campus. Because the campus is nestled in a steep mountainous river valley (Kamand, Himachal Pradesh), the navigation system must reliably handle:
- Continuous slopes (up to 12°–15° pitch/roll),
- GPS degradation and multipath near steep mountain ridges and multi-story hostel buildings,
- Negative obstacles (descending stairwells, drainage gutters, and cliff edges) that 2D LiDAR cannot reliably detect,
- Dynamic pedestrian and campus shuttle traffic.

```
+-----------------------------------------------------------------------------------+
|                                 SENSORS LAYER                                     |
|  [RPLIDAR A1M8 (2D LiDAR)]    [MPU-6050 (6-DOF IMU)]    [NEO-M9N (Multi-GNSS)]    |
+-----------------------------+-------------------------+---------------------------+
                              |                         |
                              v                         v
                       +-------------+           +-------------+
                       |  /imu/data  |           |  /gps/fix   |
                       +------+------+           +------+------+
                              |                         |
+-----------------------+     |                         |
|  Wheel Encoders (x2)  |     |                         |
+-----------+-----------+     |                         |
            |                 |                         v
            v                 |              +----------------------+
+-----------------------+     |              | navsat_transform_node|
| robot_base_controller |     |              +----------+-----------+
+-----------+-----------+     |                         |
            |                 |                         v (/odometry/gps)
            v (/wheel/odom)   |                         |
+-----------------------------v-------------------------v---------------------------+
|                          ROBOT_LOCALIZATION LAYER                                 |
|                                                                                   |
|  [Local EKF Filter]  --> Fuses: /wheel/odometry + /imu/data                       |
|                          Publishes: odom -> base_footprint (smooth, no jumps)     |
|                                                                                   |
|  [Global EKF Filter] --> Fuses: /wheel/odometry + /imu/data + /odometry/gps       |
|                          Publishes: map -> odom (drift-free campus global pose)   |
+-------------------------------------------+---------------------------------------+
                                            |
                                            v (Fused Pose & Transforms)
+-------------------------------------------+---------------------------------------+
|                             NAV2 NAVIGATION LAYER                                 |
|                                                                                   |
|  - Global Costmap (Static Map + Obstacle Layer + Stairs Keepout Filter Mask)      |
|  - Local Costmap (Rolling window 4x4m + High-frequency LiDAR clearing)           |
|  - Navfn Planner (A* path planning along approved campus walkways)                |
|  - Regulated Pure Pursuit Controller (Speed scaled on slopes & turns)             |
|  - Recovery Behaviors (Wait, Backup, Costmap Clear)                               |
+-------------------------------------------+---------------------------------------+
                                            |
                                            v (/cmd_vel)
+-------------------------------------------+---------------------------------------+
|                             ROBOT BASE LAYER                                      |
|                                                                                   |
|  - cmd_vel Watchdog (0.50s timeout -> safe motor halt)                            |
|  - Acceleration Limiter (0.5 m/s^2 linear, prevents load toppling)                |
|  - Differential Drive Inverse Kinematics                                          |
|  - Hardware Abstraction Layer (PWM/Direction pins or Mock Simulator)             |
+-------------------------------------------+---------------------------------------+
                                            |
                                            v (PWM & Direction Signals)
+-------------------------------------------+---------------------------------------+
|                               HARDWARE LAYER                                      |
|  [Cytron / BTS7960 Driver] --> [12V High-Torque Geared DC Motors] --> [Wheels]   |
+-----------------------------------------------------------------------------------+
```

---

## 2. Logical Architectural Layers

### Layer 1: Hardware Layer
- **Actuation**: 2x 12V Planetary Geared DC Motors (120:1 reduction) delivering ~18 kg.cm torque each.
- **Feedback**: Integrated optical/magnetic quadrature encoders (16 CPR on motor shaft $\times 120 = 1920$ ticks/revolution of the 150mm wheel).
- **Drive Electronics**: High-efficiency dual H-bridge motor driver with hardware PWM isolation.
- **Safety Interlock**: Physical, latching red Emergency Stop mushroom switch connected in series with an automotive 12V 30A power isolation relay.
- **Sensing Peripherals**:
  - RPLIDAR A1M8 on USB UART (`/dev/rplidar`)
  - Holybro NEO-M9N GNSS receiver on USB/UART (`/dev/ttyGPS`)
  - MPU-6050 6-DOF IMU on I2C bus 1 (`0x68`)

### Layer 2: Robot Base Layer (`robot_base`)
- Implements pure mathematical differential-drive kinematics without third-party ROS bloat.
- **Velocity Watchdog**: If `/cmd_vel` stops publishing for more than `0.50` seconds (due to network latency, planner timeout, or crash), motor outputs are immediately clamped to zero.
- **Acceleration Limiting**: Ramps linear velocity by at most $0.50\text{ m/s}^2$ to protect the hot food and grocery cargo from sliding or tipping during slope transitions.
- **Odometry Computation**: Uses 2nd-order Runge-Kutta midpoint integration to track pose from wheel tick deltas.
- **Hardware Abstraction Layer (HAL)**:
  - `RealHardwareDriver`: Interacts directly with Raspberry Pi 4B GPIO hardware PWM and edge-triggered encoder interrupts.
  - `MockHardwareDriver`: Full in-memory kinematic and physics simulation for testing on non-Pi workstations.

### Layer 3: Sensor Processing Layer (`robot_sensors`)
- **LiDAR Driver**: Publishes calibrated 360-degree laser range scans (`sensor_msgs/LaserScan`) on `/scan` at 7.0 Hz.
- **IMU Node (`mpu6050_node`)**: Reads acceleration and angular velocity registers over I2C, applies calibrated zero-rate bias corrections, executes a complementary orientation filter, and evaluates slope safety thresholds (alerts if pitch $>16^\circ$ or roll $>12^\circ$).
- **GNSS Node (`gps_node`)**: Parses NMEA `$GNGGA` / `$GNRMC` sentences from Holybro NEO-M9N, enforces quality filters (requiring $\ge 6$ satellites and $\text{HDOP} \le 2.5$), and publishes `sensor_msgs/NavSatFix`.
- **Sensor Health Supervisor (`sensor_health_monitor`)**: Tracks message arrival rates and trips an emergency stop if odometry, IMU, or LiDAR streams cease.

### Layer 4: Localization Layer (`robot_localization`)
Employs an industrial dual-EKF architecture:
1. **Local EKF (`ekf_filter_node_odom`)**:
   - Fuses continuous wheel odometry (`/wheel/odometry`) and IMU angular rate / linear acceleration (`/imu/data`).
   - Produces the continuous, jump-free dynamic transform: `odom -> base_footprint`.
   - Never fuses GPS directly, preventing discrete jumps that would de-stabilize local motion controllers.
2. **NavSat Transform Node (`navsat_transform_node`)**:
   - Computes local East-North-Up (ENU) coordinates from latitude/longitude using a fixed campus datum reference (Kamand Campus: $31.7815^\circ\text{ N}, 76.9943^\circ\text{ E}$).
   - Publishes `/odometry/gps`.
3. **Global EKF (`ekf_filter_node_map`)**:
   - Fuses wheel odometry, IMU absolute yaw, and `/odometry/gps`.
   - Publishes the dynamic transform: `map -> odom`.

### Layer 5: Navigation & Planning Layer (`robot_navigation`)
- **Nav2 Stack**:
  - Global Planner: `NavfnPlanner` using $A^*$ search over 2D campus costmaps.
  - Local Controller: `RegulatedPurePursuitController` configured with speed-scaling around tight corners and near dynamic obstacles.
  - Costmap Keepout Filter (`nav2_costmap_2d::CostmapFilterInfoServer`): Injects keepout zones defined by `stairs_keepout_mask.pgm` directly into the global costmap to prevent the robot from planning paths across stairwells, cliffs, or unpaved mountain edges.

---

## 3. Coordinate Frame Architecture (TF Tree)

All coordinate frames strictly adhere to **REP-105** (Coordinate Frames for Mobile Platforms):

```
       [map]                       (Global coordinate system fixed to campus datum)
         |
         |  Dynamic transform (published by ekf_filter_node_map or slam_toolbox)
         v
       [odom]                      (Local, smooth, continuous odometric reference)
         |
         |  Dynamic transform (published by ekf_filter_node_odom)
         v
 [base_footprint]                  (Robot 2D projection on ground surface)
         |
         |  Static transform (joint: 0, 0, wheel_radius)
         v
    [base_link]                    (Chassis geometric center)
   /     |     \       \
  /      |      \       \
 v       v       v       v
[laser_frame] [imu_link] [gps_link] [left_wheel_link] ...
```

### Transform Publishers
| Transform Link | Publishing Node | Frequency | Purpose |
| :--- | :--- | :--- | :--- |
| `map -> odom` | `ekf_filter_node_map` | 30 Hz | Eliminates accumulated long-term drift via GNSS |
| `odom -> base_footprint` | `ekf_filter_node_odom` | 50 Hz | Provides high-frequency, smooth local dead-reckoning |
| `base_footprint -> base_link` | `robot_state_publisher` | Static / 30 Hz | Elevation offset from ground contact |
| `base_link -> laser_frame` | `robot_state_publisher` | Static | Fixed position of RPLIDAR on chassis |
| `base_link -> imu_link` | `robot_state_publisher` | Static | Fixed position of MPU-6050 |
| `base_link -> gps_link` | `robot_state_publisher` | Static | Fixed position of GPS mast |
