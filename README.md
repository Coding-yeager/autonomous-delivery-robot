# Autonomous Grocery & Food Delivery Robot for IIT Mandi Campus

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill-blue.svg)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204B%20(Ubuntu%2022.04)-orange.svg)](https://ubuntu.com/download/raspberry-pi)
[![Tests: 19 Passed](https://img.shields.io/badge/Unit%20Tests-19%20Passed-brightgreen.svg)](tests/)
[![Budget](https://img.shields.io/badge/Hardware%20Budget-%E2%82%B969%2C900%20%2F%20%E2%82%B970%2C000-success.svg)](hardware/bill_of_materials.md)

An end-to-end, production-grade autonomous mobile delivery robot engineered for transporting groceries, food orders, and essential supplies across the mountainous road network and pedestrian pathways of the **IIT Mandi Campus** (Kamand Valley, Himachal Pradesh).

Developed as an autonomous robotics engineering project by a 6-member team within a strict hardware budget limit of **₹70,000 INR**.

---

## 1. Project Overview

The IIT Mandi campus is built into a steep Himalayan mountain river valley featuring multi-level hostel clusters, academic buildings, cafeteria complexes, and connecting pedestrian pathways. Transporting groceries and hot food manually across these long, steep ramps is physically demanding and time-consuming.

This autonomous delivery robot operates as an autonomous carrier that navigates paved campus walkways, safely negotiates continuous road grades (up to 12°–15°), and executes deliveries while respecting pedestrian zones.

---

## 2. Key Features

- **Mountainous Pathway Navigation**: Nav2 stack customized with the Regulated Pure Pursuit Controller to scale speed smoothly on inclines and tight hairpin turns.
- **Multi-Tier Stairs & Cliff Avoidance**: 2D LiDAR cannot detect downward stairs. We solve this through semantic costmap keepout filter masks paired with downward ultrasonic range drop-off sensors.
- **Dual-EKF Sensor Fusion**: Local EKF filter (wheel odometry + IMU) delivers smooth, high-frequency motion estimation; Global EKF (wheel odom + IMU yaw + NavSat transformed GNSS) eliminates accumulated drift across campus road loops.
- **Multi-Constellation GNSS**: Holybro NEO-M9N GNSS receiver tracks GPS, GLONASS, Galileo, and BeiDou simultaneously with HDOP and satellite-count filtering.
- **Safety Interlocks & Watchdog**: Software command timeout watchdog (0.5s), acceleration ramp limiter to prevent food spilling, hardware Emergency Stop latch, and IMU-based excessive slope tilt detection (>16°).
- **Raspberry Pi 4B Optimization**: CPU and RAM load-conscious architecture; lightweight nodes, headless execution capability, and out-of-memory protected builds.
- **Hardware Abstraction Layer (HAL)**: Seamlessly toggle between physical Raspberry Pi GPIO / UART / I2C and software mock/simulation mode.

---

## 3. System Architecture & Dataflow

```
   [RPLIDAR A1M8]           [MPU-6050 IMU]             [Holybro NEO-M9N GNSS]
         |                         |                              |
         | (/scan)                 | (/imu/data)                  | (/gps/fix)
         |                         |                              v
         |                         |                   +----------------------+
         |                         |                   | navsat_transform_node|
         |                         |                   +----------+-----------+
         |                         |                              |
         |                         |                              v (/odometry/gps)
         |     +-------------------v------------------------------+
         |     |
         v     v
  +---------------------------------------------------------------------------------+
  |                            ROBOT_LOCALIZATION LAYER                             |
  |                                                                                 |
  |  1. Local EKF  (odom -> base_footprint) : Fuses /wheel/odometry + /imu/data    |
  |  2. Global EKF (map -> odom)            : Fuses /wheel/odom + /imu + /gps/odom  |
  +----------------------------------------+----------------------------------------+
                                           |
                                           v
  +---------------------------------------------------------------------------------+
  |                              NAV2 NAVIGATION LAYER                              |
  |                                                                                 |
  |  - Global Costmap (Static Map + Stairs Keepout Filter Mask)                     |
  |  - Local Costmap (4x4m Rolling Window + 2D LiDAR clearing)                      |
  |  - Navfn Planner (A* path planning along approved campus walkways)              |
  |  - Regulated Pure Pursuit Controller (Dynamic speed scaling on slopes)          |
  +----------------------------------------+----------------------------------------+
                                           |
                                           v (/cmd_vel)
  +---------------------------------------------------------------------------------+
  |                               ROBOT BASE LAYER                                  |
  |                                                                                 |
  |  - cmd_vel Watchdog (0.50s timeout -> automatic zero command)                   |
  |  - Acceleration Limiter (0.5 m/s^2 linear, prevents load toppling)              |
  |  - Differential Drive Kinematics (Forward & Inverse)                            |
  |  - Runge-Kutta 2nd Order Odometry Integration -> /wheel/odometry                |
  +----------------------------------------+----------------------------------------+
                                           |
                                           v (Hardware PWM & Direction Signals)
  +---------------------------------------------------------------------------------+
  |                                HARDWARE LAYER                                   |
  |  [Cytron MDD10A Driver] --> [12V Geared DC Motors (x2)] --> [150mm Wheels]     |
  +---------------------------------------------------------------------------------+
```

---

## 4. Hardware Bill of Materials (Budget: ₹70,000 INR)

| Component | Model / Specification | Purpose | Cost (₹) |
| :--- | :--- | :--- | :---: |
| **Main Compute** | Raspberry Pi 4B (8GB RAM) + Heatsink Case | Onboard ROS 2 processing | ₹9,150 |
| **MicroSD Storage** | SanDisk Extreme Pro 128GB U3 A2 | High-speed OS & logging | ₹1,500 |
| **2D Laser Scanner** | Slamtec RPLIDAR A2M8 (360°, 12-16m) | Obstacle sensing & 2D SLAM | ₹15,500 |
| **GNSS Receiver** | Holybro NEO-M9N Multi-Band + Antenna | Global coordinate positioning | ₹5,800 |
| **IMU Sensor** | Precision 6-DOF (Gyro + Accelerometer) | Angular rate, tilt, & heading | ₹850 |
| **Cliff / ToF Sensors** | 4x Downward Ultrasonic/ToF Ranging Sensors | Real-time stairs drop-off detection | ₹1,800 |
| **Drive Motors** | 2x 12V Planetary Geared DC Motors (120:1, 28 kg.cm)| High-torque slope locomotion | ₹7,200 |
| **Wheel Encoders** | 2x Quadrature Encoders (1920 ticks/rev) | Dead-reckoning odometry | Included |
| **Motor Driver** | Cytron SmartDriveDuo-30 Dual 30A H-Bridge | Motor PWM & direction drive | ₹5,400 |
| **Battery Pack** | 12.8V 20Ah LiFePO4 with 40A Smart BMS | Primary power (~5.2 hr endurance) | ₹12,800 |
| **Step-Down Regulators**| Dual Synchronous 5V 10A Buck Converters | Isolated clean power rails | ₹1,100 |
| **Safety E-Stop & Relay**| 22mm IP65 Mushroom Button + 40A Heavy Relay | Hardware emergency power cutoff | ₹850 |
| **Chassis & Wheels** | 2040/2020 Aluminum frame + 160mm Wheels | Mechanical base structure | ₹5,800 |
| **Cargo Box & Wiring** | Insulated weatherproof container + XT90 wires | Food cargo bay & cabling | ₹2,150 |
| **TOTAL:** | | | **₹69,900** |

*Detailed wiring tables and power budget analysis are provided in [hardware/wiring_diagram.md](hardware/wiring_diagram.md) and [hardware/power_budget.md](hardware/power_budget.md).*

---

## 5. Software Stack & Dependencies

- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish 64-bit ARM)
- **ROS 2 Version**: ROS 2 Humble Hawksbill
- **Python Version**: Python 3.10+
- **C++ Standard**: C++17 (`ament_cmake`)
- **Key ROS 2 Packages**:
  - `navigation2` & `nav2_bringup`
  - `robot_localization` (Dual EKF + NavSat Transform)
  - `slam_toolbox` (Online asynchronous 2D mapping)
  - `rplidar_ros` (LiDAR device driver)
  - `robot_state_publisher` & `xacro`

---

## 6. Installation & Build Procedure

### 6.1 System Prerequisites
Run the automated installation script on your Raspberry Pi:
```bash
git clone https://github.com/iitmandi-dp/autonomous_delivery_robot.git
cd autonomous_delivery_robot
chmod +x scripts/*.sh
./scripts/setup.sh
```

### 6.2 Build the Workspace
Use the memory-safe build script:
```bash
./scripts/build.sh
source ros2_ws/install/setup.bash
```

---

## 7. Launch Hierarchy & Operations

### 7.1 Master Bringup
```bash
# On physical robot (with real motors, LiDAR, GPS, and IMU)
ros2 launch robot_bringup bringup.launch.py use_mock_hardware:=false use_rviz:=false

# In simulation / development workstation mode (with RViz visualization)
ros2 launch robot_bringup bringup.launch.py use_mock_hardware:=true use_rviz:=true
```

### 7.2 Modular Subsystem Launch (For Debugging)
```bash
# Launch robot URDF and state publisher
ros2 launch robot_description display.launch.py

# Launch motor controller, kinematics, and odometry only
ros2 launch robot_bringup base.launch.py use_mock_hardware:=false

# Launch sensors (LiDAR, IMU, GPS, Health Supervisor) only
ros2 launch robot_bringup sensors.launch.py use_mock_hardware:=false

# Launch Dual EKF and NavSat Transform only
ros2 launch robot_localization_config localization.launch.py

# Launch Nav2 with campus stairs keepout filter
ros2 launch robot_navigation navigation.launch.py

# Launch SLAM Toolbox for campus mapping
ros2 launch robot_navigation slam.launch.py
```

---

## 8. Calibration Procedures

Before autonomous missions, run the interactive calibration tools:
1. **Wheel Radius & Separation Calibration**:
   ```bash
   python3 scripts/calibrate_wheel_odometry.py
   ```
2. **MPU-6050 IMU Bias Calibration**:
   ```bash
   python3 scripts/calibrate_imu.py
   ```
*(Detailed steps documented in [docs/calibration.md](docs/calibration.md)).*

---

## 9. Testing & Hardware Validation

### 9.1 Automated Unit Tests
Run the comprehensive kinematics, odometry, watchdog, and GPS conversion unit tests:
```bash
python -m pytest tests/ -v
```
*(All 19 unit tests execute and pass out-of-the-box).*

### 9.2 Hardware Validation Checklist
Before powering up high-voltage motor circuits:
- [ ] Run `python3 scripts/hardware_check.py` and verify all communication buses.
- [ ] Verify RPLIDAR rotates smoothly at 7.0 Hz (`ros2 topic hz /scan`).
- [ ] Verify MPU-6050 publishes at 50 Hz (`ros2 topic hz /imu/data`).
- [ ] Verify Holybro GNSS acquires $\ge 6$ satellites and $\text{HDOP} \le 2.5$ outdoors.
- [ ] Verify `/emergency_stop` immediately trips power relay.
- [ ] Send test command `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"` and verify timeout watchdog stops motors within 0.50s.

---

## 10. Technical Honesty & Known Limitations

1. **2D LiDAR Staircase Blind Spot**:
   - A horizontal 2D laser scanner *cannot* detect downward stairwells. This limitation is mitigated via static keepout costmap filters (`stairs_keepout_mask.pgm`) and downward-facing ultrasonic range sensors.
2. **GNSS Multipath in Mountainous Valleys**:
   - In steep campus valleys or between multi-story academic buildings, satellite signals may experience dilution of precision. The system automatically rejects fixes with $\text{HDOP} > 2.5$ and falls back to local wheel odometry + IMU dead-reckoning.
3. **Slope Wheel Slippage**:
   - Under damp mountain morning conditions, wet asphalt or leaves induce wheel slip. The EKF combines IMU angular velocity and linear acceleration with wheel ticks to reject slip disturbances.

---

## 11. Repository Structure

```
autonomous_delivery_robot/
├── README.md                          # Master documentation & overview
├── LICENSE                            # MIT Open Source License
├── requirements.txt                   # Python dependencies
├── .gitignore                         # Build and temporary artifact filter
│
├── config/
│   ├── robot_params.yaml              # Base dimensions, gear ratio, velocity limits
│   ├── ekf.yaml                       # Dual EKF & NavSat transform settings
│   ├── nav2_params.yaml               # Nav2 stack & costmap keepout filter params
│   ├── sensors_params.yaml            # RPLIDAR, MPU-6050, GNSS bus settings
│   └── slam_toolbox.yaml              # SLAM mapping configuration
│
├── docs/
│   ├── architecture.md                # System layers, dataflow, and TF tree
│   ├── hardware.md                    # Hardware specs, BCM pinouts, & motors
│   ├── software.md                    # ROS 2 nodes, topics, QoS, & HAL
│   ├── setup.md                       # Raspberry Pi 4B Ubuntu setup guide
│   ├── calibration.md                 # Wheel, IMU, and GNSS calibration
│   ├── troubleshooting.md             # Failure modes and diagnostics
│   └── navigation.md                  # Campus routing, slopes, & stairs avoidance
│
├── hardware/
│   ├── bill_of_materials.md           # ₹69,900 itemized budget breakdown
│   ├── wiring_diagram.md              # Electrical schematic and pin schedule
│   └── power_budget.md                # 12.8V LiFePO4 battery sizing and runtime
│
├── maps/
│   ├── iit_mandi_campus.yaml          # Campus pathway map metadata
│   ├── iit_mandi_campus.pgm           # Campus occupancy grid
│   ├── stairs_keepout_mask.yaml       # Costmap filter keepout metadata
│   └── stairs_keepout_mask.pgm        # Stairs & cliff keepout mask
│
├── ros2_ws/
│   └── src/
│       ├── robot_interfaces/          # MotorVels, SafetyStatus, GpsStatus msgs
│       ├── robot_description/         # URDF/Xacro, RViz config, display launch
│       ├── robot_base/                # Kinematics, HAL, base controller node
│       ├── robot_sensors/             # MPU-6050, Holybro GNSS, Health Monitor
│       ├── robot_localization_config/ # EKF & NavSat launch & configs
│       ├── robot_navigation/          # Nav2 bringup, costmap filters, SLAM
│       └── robot_bringup/             # Master bringup, teleop, & sim launch
│
├── scripts/
│   ├── setup.sh                       # System & udev rule installer
│   ├── build.sh                       # Memory-safe colcon builder for Pi 4B
│   ├── start_robot.sh                 # Launch runner (hardware / mock)
│   ├── calibrate_wheel_odometry.py    # Distance & track width calibration wizard
│   ├── calibrate_imu.py               # MPU-6050 zero-bias calibration utility
│   └── hardware_check.py              # Pre-flight diagnostic bus tester
│
└── tests/
    ├── test_differential_drive.py     # Kinematics forward & inverse unit tests
    ├── test_odometry.py               # Runge-Kutta odometry & covariance tests
    ├── test_safety_watchdog.py        # Acceleration ramp & watchdog tests
    └── test_gps_conversion.py         # NMEA parsing & coordinate tests
```

---

## 12. Team & Acknowledgments

- **Institution**: Indian Institute of Technology Mandi (IIT Mandi)
- **Project Team**: Group of 6 Student Engineers
- **Faculty & Mentors**: School of Computing and Electrical Engineering (SCEE) & School of Mechanical and Materials Engineering (SMME), IIT Mandi.
