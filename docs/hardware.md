# Hardware Specification & Interfaces

**Project**: Autonomous Grocery and Food Delivery Robot for IIT Mandi Campus  
**Institution**: Indian Institute of Technology Mandi  
**Hardware Budget Limit**: ₹70,000 INR  

---

## 1. Hardware Architecture Overview

The robot's physical design prioritizes stability on mountainous slopes, high low-end torque for carrying up to 12 kg of food/groceries, and complete electrical isolation between noisy motor drivers and sensitive compute hardware.

```
+----------------------------------------------------------------------------------+
|                     12.8V 20Ah LiFePO4 Battery Pack (with BMS)                   |
+-------------------+--------------------------------------+-----------------------+
                    |                                      |
         Main Fuse (40A) + E-Stop Pushbutton               |
                    |                                      |
                    v                                      v
       +-------------------------+            +-------------------------+
       |   12V 40A Auto Relay    |            | Dual Synchronous 5V 10A |
       +------------+------------+            | Step-Down Buck Converter|
                    |                         +------------+------------+
                    v                                      | 5.1V clean DC
       +-------------------------+                         v
       | Cytron SmartDriveDuo-30 |            +-------------------------+
       | Dual 30A Motor Driver   |            |   Raspberry Pi 4B (8GB) |
       +----+---------------+----+            +----+---------+--------+-+
            |               |                      |         |        |
            v               v                      v         v        v
      [Left Motor]   [Right Motor]             [RPLIDAR]   [IMU]    [GNSS]
     (12V Planetary) (12V Planetary)           (USB/UART)  (I2C-1)  (UART)
```

---

## 2. Raspberry Pi 4B GPIO Pinout Mapping

All pin numbers below refer to the **BCM (Broadcom) GPIO numbering**, not physical header pins.

| Function | Signal | BCM GPIO | Physical Pin | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Left Motor PWM** | PWM0 | `GPIO 12` | Pin 32 | Hardware PWM channel 0 |
| **Left Motor DIR** | Direction | `GPIO 23` | Pin 16 | HIGH = Forward, LOW = Reverse |
| **Right Motor PWM** | PWM1 | `GPIO 13` | Pin 33 | Hardware PWM channel 1 |
| **Right Motor DIR** | Direction | `GPIO 24` | Pin 18 | HIGH = Forward, LOW = Reverse |
| **Left Encoder A** | Phase A | `GPIO 17` | Pin 11 | Edge interrupt (Pull-Up) |
| **Left Encoder B** | Phase B | `GPIO 27` | Pin 13 | Direction sense (Pull-Up) |
| **Right Encoder A**| Phase A | `GPIO 22` | Pin 15 | Edge interrupt (Pull-Up) |
| **Right Encoder B**| Phase B | `GPIO 10` | Pin 19 | Direction sense (Pull-Up) |
| **MPU-6050 I2C** | SDA | `GPIO 2` | Pin 3 | I2C-1 Data line |
| **MPU-6050 I2C** | SCL | `GPIO 3` | Pin 5 | I2C-1 Clock line |
| **NEO-M9N GNSS** | TX -> RX | `GPIO 15` | Pin 10 | Pi RX (Receive NMEA data) |
| **NEO-M9N GNSS** | RX <- TX | `GPIO 14` | Pin 8 | Pi TX (Send UBX configuration) |
| **E-Stop Sense** | ESTOP_IN | `GPIO 25` | Pin 22 | Normally Closed; LOW when pressed |
| **Power Relay** | RELAY_CTRL| `GPIO 18` | Pin 12 | HIGH = Relay closed (Motor power enabled) |

---

## 3. Component Details & Specifications

### 3.1 Main Compute: Raspberry Pi 4B
- **Processor**: Broadcom BCM2711, Quad-core Cortex-A72 (ARM v8) 64-bit SoC @ 1.5GHz
- **RAM**: 8 GB LPDDR4
- **Operating System**: Ubuntu 22.04 LTS (64-bit ARM Server) + ROS 2 Humble Hawksbill
- **Cooling**: Aluminum Armour Heatsink Case with Dual Active 5V Cooling Fans (prevents thermal throttling during continuous hill climbs).

### 3.2 2D LiDAR: Slamtec RPLIDAR A2M8
- **Technology**: Optical triangulation, 360-degree laser range scanner
- **Range**: 0.15 m to 16.0 m
- **Sampling Frequency**: 8,000–16,000 samples/sec
- **Rotation Rate**: Configured to 10.0 Hz (high-speed outdoor scanning)
- **Interface**: USB UART via Silicon Labs CP2102 adapter (mapped to `/dev/rplidar`)

### 3.3 GNSS / GPS: Holybro NEO-M9N
- **Chipset**: u-blox M9 high-sensitivity GNSS engine
- **Constellations**: Simultaneous reception of 4 GNSS (GPS, GLONASS, Galileo, BeiDou)
- **Antenna**: Active patch antenna mounted on a 200 mm elevated mast to clear chassis electronics
- **Interface**: UART / USB (mapped to `/dev/ttyGPS` at 38,400 baud)
- **Horizontal Position Accuracy**: ~1.5 m CEP (with SBAS differential correction)

### 3.4 IMU: MPU-6050 6-DOF
- **Sensors**: 3-Axis Gyroscope ($\pm 250^\circ/\text{s}$ range) + 3-Axis Accelerometer ($\pm 2g$ range)
- **Interface**: I2C bus 1 (`0x68`)
- **Mounting**: Rigidly secured at chassis center-of-gravity with anti-vibration rubber standoffs.

### 3.5 Motors & Encoders
- **Motor Type**: 12V High-Torque Planetary Geared DC Motors
- **Gear Ratio**: 120:1
- **Output Speed**: 45 RPM (corresponds to $v_{max} = 0.35$–$0.60\text{ m/s}$ with 150 mm wheels)
- **Stall Torque**: 22 kg.cm per motor (provides ample torque to climb 15° campus slopes)
- **Encoders**: 16 Counts Per Revolution (CPR) on the motor shaft $\times 120$ gear ratio = **1920 ticks/revolution** on output wheel.

---

## 4. Ground Contact & Chassis Geometry
- **Chassis Dimensions**: $600\text{ mm (Length)} \times 400\text{ mm (Width)} \times 250\text{ mm (Height)}$
- **Drive Wheels**: 2x 150 mm (radius $r = 0.075\text{ m}$) rugged rubber off-road wheels located on the central axle.
- **Track Width (Wheel Separation)**: $L = 0.400\text{ m}$ (400 mm)
- **Support Casters**: 2x 70 mm heavy-duty spring-loaded swivel ball casters (one mounted at the front center, one at the rear center) to prevent chassis grounding when entering inclined pedestrian ramps.
- **Ground Clearance**: 40 mm (sufficient for asphalt cracks, storm drainage grates, and small pebbles).
