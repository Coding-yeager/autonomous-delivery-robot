# Installation & Deployment Guide

**Target Computer**: Raspberry Pi 4B (4GB or 8GB RAM)  
**Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish) 64-bit Server/Desktop  
**ROS Version**: ROS 2 Humble Hawksbill  

---

## 1. Raspberry Pi 4B Operating System Setup

1. Download and install the official **Raspberry Pi Imager** on your workstation.
2. Select **OS**: `Other general-purpose OS` -> `Ubuntu` -> `Ubuntu Server 22.04.4 LTS (64-bit)`.
3. In OS Customization settings (gear icon):
   - Set hostname: `delivery-robot-pi`
   - Set username & password: `robot` / `<your_password>`
   - Enable SSH with password authentication.
   - Configure local Wi-Fi credentials (e.g. `IIT_MANDI_WIFI` or mobile hotspot).
4. Flash onto a Class 10 U3 MicroSD card (minimum 32GB recommended).
5. Insert card into the Raspberry Pi 4B, attach Ethernet or verify Wi-Fi connection, and boot up.

---

## 2. Hardware Interfaces Configuration

SSH into the Raspberry Pi:
```bash
ssh robot@delivery-robot-pi.local
```

### 2.1 Enable I2C and UART Hardware Buses
Edit `/boot/firmware/usercfg.txt` (or `/boot/firmware/config.txt`):
```bash
sudo nano /boot/firmware/usercfg.txt
```
Add the following lines to enable I2C at 400kHz and enable the hardware mini-UART:
```ini
# Enable I2C-1 bus at fast mode (400 kHz)
dtparam=i2c_arm=on
dtparam=i2c1_baudrate=400000

# Enable Primary Hardware UART
enable_uart=1
dtoverlay=miniuart-bt
```
Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`).

Disable the Linux serial console so the Holybro NEO-M9N GNSS module can access `/dev/ttyAMA0`:
```bash
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service
```

---

## 3. Clone Repository & Run Automated Setup

Clone the repository into your home directory:
```bash
cd ~
git clone https://github.com/iitmandi-dp/autonomous_delivery_robot.git
cd autonomous_delivery_robot
```

Make the setup script executable and run it:
```bash
chmod +x scripts/*.sh
./scripts/setup.sh
```

The script will:
- Install ROS 2 Humble core packages, Nav2, Robot Localization, and SLAM Toolbox.
- Install Python requirements (`numpy`, `pyserial`, `smbus2`, `scipy`, `pytest`).
- Install persistent udev rules for RPLIDAR A1M8 (`/dev/rplidar`) and Holybro GNSS (`/dev/ttyGPS`).
- Grant user access to the `dialout`, `i2c`, and `gpio` groups.

Reboot the Raspberry Pi to apply group and hardware changes:
```bash
sudo reboot
```

---

## 4. Building the ROS 2 Workspace

After rebooting and logging back in:
```bash
cd ~/autonomous_delivery_robot
./scripts/build.sh
```

*(Note: The build script automatically detects if available RAM is $\le 4\text{ GB}$ and restricts parallel build workers to 2, preventing system freezing).*

Once the build finishes successfully, source the workspace overlay:
```bash
source ros2_ws/install/setup.bash
```
*(Tip: Add `source ~/autonomous_delivery_robot/ros2_ws/install/setup.bash` to your `~/.bashrc` file for automatic sourcing).*

---

## 5. Pre-Flight Hardware Validation

Before powering the motor battery, run the diagnostic script:
```bash
python3 scripts/hardware_check.py
```

Expected output:
```
=================================================================
     PRE-FLIGHT HARDWARE DIAGNOSTIC SUITE (IIT MANDI)
=================================================================
--- 1. Serial & USB Peripheral Interfaces ---
Detected serial devices: ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/rplidar', '/dev/ttyGPS']
  [ OK ] RPLIDAR A1M8 (/dev/rplidar or /dev/ttyUSB0)
  [ OK ] Holybro NEO-M9N GNSS (/dev/ttyGPS or /dev/ttyACM0)

--- 2. I2C Bus & MPU-6050 Interface ---
  [ OK ] MPU-6050 detected at address 0x68 (WHO_AM_I: 0x68)

--- 3. Raspberry Pi GPIO & Motor Control Pins ---
  [ OK ] RPi.GPIO library loaded and BCM mode configured.
=================================================================
```
If any check fails, refer to `docs/troubleshooting.md`.
