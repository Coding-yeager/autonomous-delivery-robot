# Troubleshooting & Diagnostic Guide

**Project**: Autonomous Grocery and Food Delivery Robot for IIT Mandi Campus  

---

## 1. Quick Diagnostic Commands

Run these ROS 2 commands to quickly isolate system issues:

```bash
# 1. Verify Active ROS 2 Nodes
ros2 node list

# 2. Check Topic Publishing Rates
ros2 topic hz /scan               # Should be ~7.0 Hz
ros2 topic hz /imu/data           # Should be ~50.0 Hz
ros2 topic hz /gps/fix            # Should be ~5.0 Hz
ros2 topic hz /wheel/odometry     # Should be ~50.0 Hz
ros2 topic hz /cmd_vel            # Active during teleop or nav

# 3. Inspect Live Data
ros2 topic echo /cmd_vel
ros2 topic echo /safety_status
ros2 topic echo /gps/status

# 4. Check Complete TF Tree
ros2 run tf2_tools view_frames
# Inspect frames.pdf generated in current working directory
```

---

## 2. Common Failure Modes & Solutions

### 2.1 RPLIDAR Not Spinning or "Device Cannot Be Opened"
- **Symptom**: `rplidar_node` logs `Error: cannot open serial port /dev/rplidar`.
- **Cause 1**: Udev rule not loaded or CP2102 adapter on different USB port.
  - *Fix*: Check `ls -l /dev/rplidar` or `ls -l /dev/ttyUSB*`. If missing, run:
    ```bash
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```
- **Cause 2**: Insufficient USB power from Raspberry Pi.
  - *Fix*: The RPLIDAR motor requires 5V @ ~600mA surge upon spinup. If powered directly from Pi USB alongside other devices, use an externally powered USB hub or tap 5V directly from the high-current buck converter.

### 2.2 MPU-6050 "Errno 121: Remote I/O Error"
- **Symptom**: `mpu6050_node` crashes with I2C bus communication error.
- **Cause**: I2C bus lockup due to electrical noise from adjacent motor wiring.
  - *Fix 1*: Ensure twisted-pair or shielded wiring for SDA/SCL lines.
  - *Fix 2*: Add $4.7\text{ k}\Omega$ pull-up resistors to 3.3V on SDA and SCL lines if not already present on breakout module.
  - *Fix 3*: Reset the I2C bus without rebooting:
    ```bash
    sudo rmmod i2c_bcm2835
    sudo modprobe i2c_bcm2835
    ```

### 2.3 GNSS / GPS Not Publishing Valid Fixes
- **Symptom**: `/gps/fix` status is `STATUS_NO_FIX` or topic is silent.
- **Cause 1**: Indoor testing or mountain valley obstruction.
  - *Note*: Kamand Valley has steep hills blocking low-elevation satellites. The robot requires direct sky exposure.
  - *Fix*: Verify antenna is outdoors with a clear $360^\circ$ view of the sky. Check `/gps/status`: satellites visible must be $\ge 6$ and $\text{HDOP} \le 2.5$.
- **Cause 2**: Serial console claiming UART port.
  - *Fix*: Disable the Linux serial getty service:
    ```bash
    sudo systemctl stop serial-getty@ttyAMA0.service
    sudo systemctl disable serial-getty@ttyAMA0.service
    ```

### 2.4 EKF Filter Drift or Conflicting Transforms
- **Symptom**: Robot jumps in RViz or `tf2_ros` reports "Multiple transform authorities for frame base_footprint".
- **Cause**: Both `robot_base_controller` and `ekf_filter_node_odom` are publishing `odom -> base_footprint`.
  - *Fix*: In `config/robot_params.yaml`, ensure `publish_tf: false` for `robot_base_controller`. The local EKF must be the sole authority publishing `odom -> base_footprint`.

### 2.5 Nav2 Path Planning Failure
- **Symptom**: Nav2 rejects goal pose with error: `Failed to create plan`.
- **Cause 1**: Goal is inside an obstacle or inside the stairs keepout mask.
  - *Fix*: Ensure the goal waypoint is on the clear path and not inside a black pixel zone on `stairs_keepout_mask.pgm`.
- **Cause 2**: Costmap inflation radius too large for narrow campus path.
  - *Fix*: In `config/nav2_params.yaml`, lower `inflation_radius` from `0.60` to `0.45` if navigating narrow hostel doorways or single-track sidewalks.

### 2.6 Motors Stop Unexpectedly (Watchdog Timeout)
- **Symptom**: Robot drives briefly then abruptly stops; `/safety_status` reports `Watchdog Timeout`.
- **Cause**: High Wi-Fi latency or CPU load causes `/cmd_vel` messages from Nav2 or teleop to arrive slower than the `0.50 s` timeout threshold.
  - *Fix*: Check Raspberry Pi CPU load using `htop`. Ensure RViz is running on a remote workstation, NOT on the Pi itself. In `config/robot_params.yaml`, you can slightly increase `cmd_vel_timeout: 0.75` during Wi-Fi testing.

### 2.7 Avoiding Stairs: Hardware & Algorithmic Safety
- **Limitation**: A single horizontal 2D planar LiDAR at $z = 0.27\text{ m}$ cannot see downward stairs or cliffs until the wheels have already begun to drop over the edge!
- **Engineering Solutions Deployed**:
  1. **Strict Costmap Keepout Filter**: All campus stairwells are permanently painted as impassable keepout masks in `maps/stairs_keepout_mask.pgm`.
  2. **Ultrasonic / ToF Ground Proximity Sensors**: Front bumper downward-facing range sensors detect sudden ground drops $> 100\text{ mm}$ and publish to `/emergency_stop`.
  3. **Pitch Hazard Protection**: If the robot tilts by $> 16^\circ$ on an unmapped incline, the IMU node triggers a safety halt.
