#!/usr/bin/env python3
"""
Pre-Flight Hardware Validation Script.
Validates hardware communication buses and peripherals on Raspberry Pi 4B:
- RPLIDAR A1M8 serial connectivity
- Holybro NEO-M9N GNSS UART interface
- MPU-6050 I2C sensor detection
- GPIO controller availability
"""

import os
import sys
import glob


def check_serial_ports():
    print("\n--- 1. Serial & USB Peripheral Interfaces ---")
    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/rplidar') + glob.glob('/dev/ttyGPS')
    print(f"Detected serial devices: {ports if ports else 'None'}")
    
    lidar_present = os.path.exists('/dev/rplidar') or '/dev/ttyUSB0' in ports
    gps_present = os.path.exists('/dev/ttyGPS') or '/dev/ttyACM0' in ports or '/dev/ttyUSB1' in ports
    
    print(f"  [ {'OK' if lidar_present else 'FAIL'} ] RPLIDAR A1M8 (/dev/rplidar or /dev/ttyUSB0)")
    print(f"  [ {'OK' if gps_present else 'FAIL'} ] Holybro NEO-M9N GNSS (/dev/ttyGPS or /dev/ttyACM0)")
    return lidar_present, gps_present


def check_i2c_bus():
    print("\n--- 2. I2C Bus & MPU-6050 Interface ---")
    i2c_dev = "/dev/i2c-1"
    if not os.path.exists(i2c_dev):
        print("  [ FAIL ] /dev/i2c-1 does not exist. (Enable I2C in raspi-config)")
        return False

    try:
        import smbus2
        bus = smbus2.SMBus(1)
        # Try reading WHO_AM_I register (0x75) from MPU6050 (default address 0x68)
        who_am_i = bus.read_byte_data(0x68, 0x75)
        bus.close()
        if who_am_i in [0x68, 0x72]:
            print(f"  [  OK  ] MPU-6050 detected at address 0x68 (WHO_AM_I: 0x{who_am_i:02x})")
            return True
        else:
            print(f"  [ WARN ] Device responded at 0x68 with unexpected ID: 0x{who_am_i:02x}")
            return False
    except Exception as e:
        print(f"  [ FAIL ] Cannot read from 0x68: {e}")
        return False


def check_gpio():
    print("\n--- 3. Raspberry Pi GPIO & Motor Control Pins ---")
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        print("  [  OK  ] RPi.GPIO library loaded and BCM mode configured.")
        return True
    except ImportError:
        print("  [ WARN ] RPi.GPIO library not installed. Running in Mock/Sim mode.")
        return False
    except Exception as e:
        print(f"  [ FAIL ] GPIO error: {e}")
        return False


def main():
    print("=================================================================")
    print("     PRE-FLIGHT HARDWARE DIAGNOSTIC SUITE (IIT MANDI)")
    print("=================================================================")
    lidar_ok, gps_ok = check_serial_ports()
    imu_ok = check_i2c_bus()
    gpio_ok = check_gpio()

    print("\n=================================================================")
    print("                   SUMMARY STATUS MATRIX")
    print("=================================================================")
    print(f"  RPLIDAR A1M8:         {'PASSED' if lidar_ok else 'CHECK CABLING/UDEV'}")
    print(f"  Holybro NEO-M9N GNSS: {'PASSED' if gps_ok else 'CHECK CABLING/UDEV'}")
    print(f"  MPU-6050 6-DOF IMU:   {'PASSED' if imu_ok else 'CHECK I2C WIRING'}")
    print(f"  RPi 4B GPIO Driver:   {'PASSED' if gpio_ok else 'OPTIONAL / MOCK'}")
    print("=================================================================\n")


if __name__ == '__main__':
    main()
