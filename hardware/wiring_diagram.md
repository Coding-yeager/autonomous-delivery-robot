# Electrical Wiring & Hardware Pinout Guide

**Project**: Autonomous Grocery and Food Delivery Robot for IIT Mandi Campus  

---

## 1. Power Distribution Schematic

```
[12.8V 20Ah LiFePO4 Battery]
       |
       +--- [40A Blade Fuse]
       |
       +--- [Physical E-Stop Switch (NC)]
       |
       +---------------------------------------------+
       |                                             |
       v                                             v
[12V 40A Relay (NO contacts)]               [Dual 5V 10A Synchronous Buck]
       |                                             |
       | Switched 12V Motor Rail                     v Regulated 5.1V
       v                                    +--------------------------------+
[Cytron SmartDriveDuo-30 Driver]            |  Raspberry Pi 4B (USB-C / Pin) |
       |                                    |  RPLIDAR A2M8 (USB 5V)         |
       +---> Left Motor (12V)               |  Holybro NEO-M9N (5V Pin)      |
       +---> Right Motor (12V)              |  Precision IMU (3.3V Pin)      |
                                            +--------------------------------+
```

---

## 2. Raspberry Pi 4B Wiring Schedule

### 2.1 Motor Driver (Cytron MDD10A) Interface
| Pi 4B Physical Pin | BCM GPIO | Signal Name | Cytron MDD10A Terminal | Description |
| :---: | :---: | :---: | :---: | :--- |
| **Pin 32** | `GPIO 12` | `LEFT_PWM` | `PWM 1` | Left motor speed control (20 kHz PWM) |
| **Pin 16** | `GPIO 23` | `LEFT_DIR` | `DIR 1` | Left motor direction (0=Rev, 1=Fwd) |
| **Pin 33** | `GPIO 13` | `RIGHT_PWM`| `PWM 2` | Right motor speed control (20 kHz PWM)|
| **Pin 18** | `GPIO 24` | `RIGHT_DIR`| `DIR 2` | Right motor direction (0=Rev, 1=Fwd)|
| **Pin 6**  | `GND`     | `GND`      | `GND`   | Common logic ground reference |

### 2.2 Quadrature Wheel Encoders Interface
| Pi 4B Physical Pin | BCM GPIO | Signal Name | Encoder Wire Color | Description |
| :---: | :---: | :---: | :---: | :--- |
| **Pin 1**  | `3.3V`    | `VCC`      | Red | Encoder logic power (3.3V DC) |
| **Pin 9**  | `GND`     | `GND`      | Black | Encoder logic ground |
| **Pin 11** | `GPIO 17` | `ENC_L_A`  | Yellow | Left wheel Phase A (Interrupt) |
| **Pin 13** | `GPIO 27` | `ENC_L_B`  | Green | Left wheel Phase B (Direction) |
| **Pin 15** | `GPIO 22` | `ENC_R_A`  | Yellow | Right wheel Phase A (Interrupt)|
| **Pin 19** | `GPIO 10` | `ENC_R_B`  | Green | Right wheel Phase B (Direction)|

### 2.3 MPU-6050 6-DOF IMU Interface
| Pi 4B Physical Pin | BCM GPIO | Signal Name | MPU-6050 Pin | Notes |
| :---: | :---: | :---: | :---: | :--- |
| **Pin 17** | `3.3V`    | `VCC`      | `VCC` | Do not connect to 5V (protects Pi GPIO)|
| **Pin 14** | `GND`     | `GND`      | `GND` | Ground |
| **Pin 3**  | `GPIO 2`  | `SDA`      | `SDA` | I2C-1 Data line |
| **Pin 5**  | `GPIO 3`  | `SCL`      | `SCL` | I2C-1 Clock line |
| --         | --        | `GND`      | `AD0` | Tie AD0 to GND to select address 0x68|

### 2.4 Holybro NEO-M9N GNSS Receiver Interface
| Pi 4B Physical Pin | BCM GPIO | Signal Name | Holybro Pin | Notes |
| :---: | :---: | :---: | :---: | :--- |
| **Pin 2**  | `5.0V`    | `5V`       | `Pin 1 (VCC)`| Active antenna requires 5V supply |
| **Pin 10** | `GPIO 15` | `UART_RXD` | `Pin 2 (TXD)`| Pi receives NMEA sentences |
| **Pin 8**  | `GPIO 14` | `UART_TXD` | `Pin 3 (RXD)`| Pi sends UBX configuration |
| **Pin 20** | `GND`     | `GND`      | `Pin 6 (GND)`| Ground |

### 2.5 Safety Relay & Emergency Stop Sensing
| Pi 4B Physical Pin | BCM GPIO | Signal Name | Connection | Notes |
| :---: | :---: | :---: | :---: | :--- |
| **Pin 12** | `GPIO 18` | `RELAY_EN` | Relay Driver In | HIGH enables 12V motor power rail |
| **Pin 22** | `GPIO 25` | `ESTOP_SENSE`| E-Stop Aux Contact | Pulled HIGH internally; LOW = Pressed |

---

## 3. Grounding & Noise Suppression Rules
1. **Star Grounding**: All high-current grounds (battery negative, motor driver GND) must join at a single central copper busbar to prevent ground loops.
2. **Flyback & Snubber Diodes**: Cytron MDD10A has internal flyback protection; additional snubber capacitors (0.1 µF ceramic) are soldered across motor terminals to suppress RF interference.
3. **Logic Isolation**: The 5V step-down buck converter supplies the Raspberry Pi, RPLIDAR, and GNSS modules through isolated filtering to prevent motor voltage dips from resetting the Pi.
