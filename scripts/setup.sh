#!/usr/bin/env bash
# ==============================================================================
# Autonomous Delivery Robot - Environment & Hardware Setup Script
# Target: Raspberry Pi 4B / Ubuntu 22.04 LTS (Jammy) / ROS 2 Humble
# ==============================================================================

set -e

echo "=== [1/5] Updating Package Lists ==="
sudo apt update

echo "=== [2/5] Installing ROS 2 Humble Core & Navigation Packages ==="
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-rplidar-ros \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    i2c-tools

echo "=== [3/5] Installing Python Peripherals & Calibration Libraries ==="
pip3 install --upgrade pip
pip3 install numpy pyserial smbus2 pyyaml scipy pytest pytest-cov

echo "=== [4/5] Configuring Hardware Udev Rules for RPLIDAR and GNSS ==="
sudo bash -c 'cat > /etc/udev/rules.d/99-delivery-robot.rules << "EOF"
# RPLIDAR A1M8 (Silicon Labs CP2102)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="rplidar"

# Holybro NEO-M9N GNSS (u-blox GNSS receiver)
KERNEL=="ttyACM*", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE:="0666", SYMLINK+="ttyGPS"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0666", SYMLINK+="ttyGPS"
EOF'

sudo udevadm control --reload-rules
sudo udevadm trigger
echo "Udev rules installed: /dev/rplidar and /dev/ttyGPS will be automatically mapped."

echo "=== [5/5] Adding Current User to Peripheral Groups (dialout, i2c, gpio) ==="
sudo usermod -aG dialout,i2c $USER
if getent group gpio > /dev/null 2>&1; then
    sudo usermod -aG gpio $USER
fi

echo "=============================================================================="
echo "Setup complete! Please REBOOT or log out and log back in for group changes."
echo "=============================================================================="
