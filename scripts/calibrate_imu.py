#!/usr/bin/env python3
"""
MPU-6050 IMU Calibration Script.
Places robot on a flat, level surface and samples 1000 measurements to calculate
gyro bias and accelerometer offset vectors.
"""

import sys
import time
import math


def calibrate_imu(bus_num=1, address=0x68, samples=1000):
    print("=================================================================")
    print("           MPU-6050 6-DOF IMU CALIBRATION UTILITY")
    print("=================================================================")
    print("Ensure the robot is on a strictly LEVEL, stationary surface.")
    print("DO NOT move, bump, or vibrate the robot during sampling.")
    input("Press ENTER to begin sampling...")

    try:
        import smbus2
    except ImportError:
        print("Error: smbus2 is not installed. Run: pip3 install smbus2")
        sys.exit(1)

    try:
        bus = smbus2.SMBus(bus_num)
        bus.write_byte_data(address, 0x6B, 0x00)  # Wake up MPU6050
        time.sleep(0.1)
    except Exception as e:
        print(f"Error connecting to I2C bus {bus_num} address 0x{address:02x}: {e}")
        sys.exit(1)

    def read_word_2c(reg):
        high = bus.read_byte_data(address, reg)
        low = bus.read_byte_data(address, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            val = -((65535 - val) + 1)
        return val

    ax_sum, ay_sum, az_sum = 0.0, 0.0, 0.0
    gx_sum, gy_sum, gz_sum = 0.0, 0.0, 0.0

    print(f"\nCollecting {samples} samples...")
    for i in range(samples):
        # Accelerometer raw
        ax = (read_word_2c(0x3B) / 16384.0) * 9.80665
        ay = (read_word_2c(0x3D) / 16384.0) * 9.80665
        az = (read_word_2c(0x3F) / 16384.0) * 9.80665

        # Gyroscope raw
        gx = math.radians(read_word_2c(0x43) / 131.0)
        gy = math.radians(read_word_2c(0x45) / 131.0)
        gz = math.radians(read_word_2c(0x47) / 131.0)

        ax_sum += ax
        ay_sum += ay
        az_sum += az
        gx_sum += gx
        gy_sum += gy
        gz_sum += gz

        if (i + 1) % 100 == 0:
            sys.stdout.write(f"\rProgress: [{i+1}/{samples}]")
            sys.stdout.flush()
        time.sleep(0.005)

    print("\nCalculation complete!")

    # Averages
    ax_mean = ax_sum / samples
    ay_mean = ay_sum / samples
    az_mean = az_sum / samples
    gx_bias = gx_sum / samples
    gy_bias = gy_sum / samples
    gz_bias = gz_sum / samples

    # Expected gravity is 9.80665 on Z axis
    ax_offset = ax_mean
    ay_offset = ay_mean
    az_offset = az_mean - 9.80665

    print("\n=================================================================")
    print("CALIBRATION RESULTS (Paste into config/sensors_params.yaml):")
    print("=================================================================")
    print(f"    accel_offset_x: {ax_offset:.6f}")
    print(f"    accel_offset_y: {ay_offset:.6f}")
    print(f"    accel_offset_z: {az_offset:.6f}")
    print(f"    gyro_offset_x:  {gx_bias:.6f}")
    print(f"    gyro_offset_y:  {gy_bias:.6f}")
    print(f"    gyro_offset_z:  {gz_bias:.6f}")
    print("=================================================================\n")


if __name__ == '__main__':
    calibrate_imu()
