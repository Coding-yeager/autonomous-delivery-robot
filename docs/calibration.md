# Sensor & Actuator Calibration Procedures

**Project**: Autonomous Grocery and Food Delivery Robot for IIT Mandi Campus  

Accurate calibration is essential for reliable autonomous dead-reckoning and navigation on outdoor roads and ramps. Follow these procedures systematically before attempting autonomous navigation.

---

## 1. Wheel Odometry Calibration

Differential drive odometry depends on two core parameters in `config/robot_params.yaml`:
1. `wheel_radius` (nominal: $0.075\text{ m}$)
2. `wheel_separation` (nominal: $0.400\text{ m}$)

Because rubber tires compress under payload weight and tire tread dynamics vary across asphalt and paver tiles, effective values must be calibrated empirically.

### Step 1: Linear Distance (Wheel Radius) Calibration
1. Place the robot on a flat, level corridor or outdoor paved pathway.
2. Lay down a 2.00 or 5.00 meter tape measure along the ground.
3. Align the wheel axle center with the 0.00 m mark.
4. Launch the interactive calibration wizard:
   ```bash
   python3 scripts/calibrate_wheel_odometry.py
   ```
5. Follow prompts to drive or push the robot forward exactly to the tape mark.
6. The script reads accumulated encoder ticks:
   $$\text{ticks\_per\_meter} = \frac{\Delta \text{ticks}_{avg}}{D_{\text{actual}}}$$
   $$r_{\text{calibrated}} = \frac{N}{2 \pi \cdot \text{ticks\_per\_meter}}$$
   *(where $N = 1920$ ticks/rev).*

### Step 2: Angular Rotation (Wheel Separation / Track Width) Calibration
1. Align the robot's heading with a chalk line or laser pointer dot on the wall.
2. Using the calibration script, rotate the robot in place for exactly 10 complete revolutions ($3600^\circ$ or $20 \pi\text{ radians}$).
3. When the robot returns exactly to the initial heading, press Enter to record ticks.
4. The script computes the effective wheel separation:
   $$L_{\text{calibrated}} = \frac{D_{\text{left\_arc}} + D_{\text{right\_arc}}}{2 \cdot \theta_{\text{total}}}$$
5. Update `config/robot_params.yaml` with the calibrated `wheel_radius` and `wheel_separation`.

---

## 2. MPU-6050 IMU Calibration

IMU gyroscopes drift if zero-rate biases are not removed, and accelerometers require static gravity calibration.

### Procedure
1. Place the robot on a strictly level surface (verify with a spirit bubble level on the chassis plate).
2. Ensure motors are powered down and no mechanical vibration is present.
3. Run the automated IMU calibration tool:
   ```bash
   python3 scripts/calibrate_imu.py
   ```
4. The script collects 1,000 samples over 5 seconds and calculates:
   - Gyroscope zero-bias vector: $(b_{gx}, b_{gy}, b_{gz})$
   - Accelerometer offsets: $(a_x - 0, a_y - 0, a_z - 9.80665\text{ m/s}^2)$
5. Copy the generated YAML snippet into `config/sensors_params.yaml`:
   ```yaml
   mpu6050_node:
     ros__parameters:
       accel_offset_x: 0.018230
       accel_offset_y: -0.009410
       accel_offset_z: 0.045120
       gyro_offset_x: 0.001210
       gyro_offset_y: -0.002340
       gyro_offset_z: 0.000850
   ```

---

## 3. Holybro NEO-M9N GNSS Antenna & Magnetic Declination

### Antenna Placement
- The active patch antenna must be mounted on the elevated mast at least $200\text{ mm}$ above the main chassis plate.
- Keep the antenna at least $150\text{ mm}$ away from the Raspberry Pi 4B CPU heatsink and motor cables to minimize RF interference.
- Ground plane: Ensure the metal ground disc underneath the patch antenna is securely fastened.

### Campus Reference Datum & Magnetic Declination
For the IIT Mandi campus (Kamand Valley, Himachal Pradesh):
- **Latitude**: $31.7815^\circ\text{ N}$
- **Longitude**: $76.9943^\circ\text{ E}$
- **Elevation**: $\approx 1040\text{ m}$ MSL
- **Magnetic Declination**: $+0^\circ 51'$ East ($\approx +0.0150\text{ radians}$).

Verify that `config/ekf.yaml` has these values configured in `navsat_transform`:
```yaml
navsat_transform:
  ros__parameters:
    magnetic_declination_radians: 0.0150
    yaw_offset: 0.0
    datum: [31.7815, 76.9943, 0.0, map, base_footprint]
```

---

## 4. RPLIDAR A1M8 Alignment & Frame

- The RPLIDAR A1M8 is positioned at $x = +0.18\text{ m}$ (front overhang) and $z = 0.27\text{ m}$ above ground.
- Ensure the USB wire exit faces backward (toward $-x$ body frame) so that $0^\circ$ scan angle points directly forward along the robot's heading axis.
- Verify scan direction:
  ```bash
  ros2 topic echo /scan --field angle_increment
  ```
  Angle increment must be positive (~0.017 rad or 1.0 deg).
