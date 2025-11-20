# Software Architecture & ROS 2 Pipeline

**Project**: Autonomous Grocery and Food Delivery Robot for IIT Mandi Campus  
**Distribution**: ROS 2 Humble Hawksbill  
**Base OS**: Ubuntu 22.04 LTS  

---

## 1. ROS 2 Workspace & Package Directory

The ROS 2 workspace is organized into modular packages adhering to the single-responsibility principle:

```
ros2_ws/src/
├── robot_interfaces/          # Custom ROS 2 message definitions (MotorVels, SafetyStatus, GpsStatus)
├── robot_description/         # URDF/Xacro models, visual meshes, and RViz2 configurations
├── robot_base/                # Kinematics engine, motor driver HAL, odometry & watchdog node
├── robot_sensors/             # Drivers for MPU-6050 (I2C), Holybro GNSS (UART), & Health Supervisor
├── robot_localization_config/ # Dual EKF and NavSat transform configurations and launch files
├── robot_navigation/          # Nav2 parameters, costmap keepout filter for stairs, SLAM Toolbox
└── robot_bringup/             # Master startup orchestration and launch hierarchy
```

---

## 2. Topic & Interface Registry

### Primary Topic Graph

| Topic Name | Message Type | Publisher Node | Subscriber Node(s) | QoS |
| :--- | :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `nav2_controller` / `teleop` | `robot_base_controller` | Reliable / Volatile |
| `/wheel/odometry` | `nav_msgs/msg/Odometry` | `robot_base_controller` | `ekf_filter_node_odom`, `ekf_filter_node_map` | Reliable / 10 |
| `/scan` | `sensor_msgs/msg/LaserScan` | `rplidar_node` | `local_costmap`, `global_costmap`, `slam_toolbox` | Best Effort / SensorData |
| `/imu/data` | `sensor_msgs/msg/Imu` | `mpu6050_node` | `ekf_filter_node_odom`, `ekf_filter_node_map`, `navsat_transform` | Best Effort / SensorData |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | `gps_node` | `navsat_transform_node` | Reliable / Transient Local |
| `/odometry/gps` | `nav_msgs/msg/Odometry` | `navsat_transform_node` | `ekf_filter_node_map` | Reliable / 10 |
| `/odometry/filtered/local` | `nav_msgs/msg/Odometry` | `ekf_filter_node_odom` | `nav2_controller`, `navsat_transform` | Reliable / 10 |
| `/odometry/filtered/global`| `nav_msgs/msg/Odometry` | `ekf_filter_node_map` | `nav2_planner`, High-level mission supervisor | Reliable / 10 |
| `/motor_vels` | `robot_interfaces/msg/MotorVels` | `robot_base_controller` | Diagnostic logger / UI | Reliable / 10 |
| `/safety_status` | `robot_interfaces/msg/SafetyStatus`| `robot_base_controller` / `mpu6050_node` | UI dashboard / Health monitor | Reliable / 10 |
| `/emergency_stop` | `std_msgs/msg/Bool` | `sensor_health_monitor` / E-Stop UI | `robot_base_controller` | Reliable / Transient Local |
| `/stairs_keepout_mask`| `nav_msgs/msg/OccupancyGrid`| `filter_mask_server` | `nav2_costmap_2d` (Costmap Filter) | Transient Local |

---

## 3. Node Specifications

### 3.1 `robot_base_controller` (Package: `robot_base`)
- **Execution Rate**: 50 Hz
- **Inputs**: `/cmd_vel` (`Twist`), `/emergency_stop` (`Bool`), Quadrature encoder interrupts.
- **Outputs**: `/wheel/odometry` (`Odometry`), `/motor_vels` (`MotorVels`), `/safety_status` (`SafetyStatus`), Hardware PWM signals.
- **Key Algorithms**:
  - 2nd-order Runge-Kutta midpoint odometry integration.
  - Linear and angular acceleration clamping ($a_v \le 0.50\text{ m/s}^2, a_\omega \le 1.00\text{ rad/s}^2$).
  - Watchdog timer: If `/cmd_vel` timestamp is older than `0.50 s`, speed is immediately forced to zero.

### 3.2 `mpu6050_node` (Package: `robot_sensors`)
- **Execution Rate**: 50 Hz
- **Bus**: Linux I2C (`/dev/i2c-1`), address `0x68`.
- **Outputs**: `/imu/data` (`sensor_msgs/Imu`), `/safety_status` on hazard.
- **Key Algorithms**:
  - Zero-bias gyro subtraction and accelerometer scale calibration.
  - Complementary pitch/roll estimation ($\alpha = 0.96$).
  - Slope hazard monitor: Warns and triggers safety status if pitch $> 16^\circ$ or roll $> 12^\circ$.

### 3.3 `gps_node` (Package: `robot_sensors`)
- **Execution Rate**: 5 Hz
- **Bus**: Serial UART (`/dev/ttyGPS`), baud rate 38400.
- **Outputs**: `/gps/fix` (`NavSatFix`), `/gps/status` (`GpsStatus`).
- **Key Algorithms**:
  - NMEA sentence checksum verification and `$GNGGA` / `$GNRMC` parsing.
  - Fix quality filter: Fixes with fewer than 6 satellites or $\text{HDOP} > 2.5$ are rejected to eliminate multipath jumps near campus buildings.

### 3.4 `sensor_health_monitor` (Package: `robot_sensors`)
- **Execution Rate**: 2 Hz
- **Supervises**: Arrival timestamps of `/scan`, `/imu/data`, `/gps/fix`, `/wheel/odometry`.
- **Safety Interlock**: If any essential navigation sensor fails for longer than its declared timeout, the node issues a diagnostic alert and fires `/emergency_stop: true` to prevent unguided runaway behavior.

---

## 4. Hardware/Software Separation Architecture

To ensure testability and prevent dependency on physical hardware during development:
- Every hardware node exposes a `use_mock_hardware: true/false` parameter.
- The base controller uses a polymorphic interface (`BaseHardwareInterface`), selecting between `RealHardwareDriver` (RPi GPIO) and `MockHardwareDriver` (physics simulation).
- The sensor drivers generate synthetic yet physically plausible telemetry when mock mode is enabled.
- **No fake hardware claims**: Simulation mode is explicitly separated and declared across launch files and documentation.
