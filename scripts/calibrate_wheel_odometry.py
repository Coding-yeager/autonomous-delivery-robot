#!/usr/bin/env python3
"""
Wheel Odometry Calibration Script for Differential Drive Delivery Robot.
Computes:
1. Effective Wheel Radius (linear distance calibration)
2. Effective Track Width / Wheel Separation (rotation calibration)
"""

import sys
import math
import time


def calibrate_linear():
    print("\n--- 1. Linear Wheel Radius Calibration ---")
    print("Mark a starting line on a flat floor and lay down a 2.00 meter tape measure.")
    print("Align the robot's front axle exactly at 0.00 m.")
    input("Press ENTER when robot is ready at the starting line...")

    test_distance_m = float(input("Enter target test distance in meters [default: 2.00]: ") or "2.00")
    print(f"\nPush the robot straight forward along the tape measure until front axle is at {test_distance_m:.2f} m.")
    
    start_ticks_l = int(input("Enter initial left encoder tick count: ") or "0")
    start_ticks_r = int(input("Enter initial right encoder tick count: ") or "0")
    
    input(f"Move robot {test_distance_m:.2f} m, then press ENTER...")
    
    end_ticks_l = int(input("Enter final left encoder tick count: "))
    end_ticks_r = int(input("Enter final right encoder tick count: "))

    delta_l = abs(end_ticks_l - start_ticks_l)
    delta_r = abs(end_ticks_r - start_ticks_r)
    avg_ticks = (delta_l + delta_r) / 2.0

    print(f"\nResults:")
    print(f"  Left ticks:  {delta_l}")
    print(f"  Right ticks: {delta_r}")
    print(f"  Average:     {avg_ticks:.1f} ticks for {test_distance_m:.2f} m")

    ticks_per_meter = avg_ticks / test_distance_m
    print(f"  Ticks per meter: {ticks_per_meter:.2f}")

    ticks_per_rev = int(input("Enter encoder ticks per motor revolution [default: 1920]: ") or "1920")
    # Distance per rev = 2 * pi * r = ticks_per_rev / ticks_per_meter
    circumference = ticks_per_rev / ticks_per_meter
    calibrated_radius = circumference / (2.0 * math.pi)

    print(f"\n==> CALIBRATED WHEEL RADIUS: {calibrated_radius:.5f} m ({calibrated_radius*1000.0:.2f} mm)")
    return calibrated_radius


def calibrate_angular(calibrated_radius: float):
    print("\n--- 2. Angular Wheel Separation (Track Width) Calibration ---")
    print("Mark robot heading on the floor.")
    num_rotations = int(input("Enter number of in-place rotations [default: 10]: ") or "10")
    
    print(f"Rotate the robot in-place by hand or teleop for EXACTLY {num_rotations} full turns (3600 deg).")
    input("Press ENTER when ready to record initial ticks...")
    
    start_ticks_l = int(input("Enter initial left encoder ticks: ") or "0")
    start_ticks_r = int(input("Enter initial right encoder ticks: ") or "0")
    
    input(f"Rotate {num_rotations} complete rotations and align heading, then press ENTER...")
    
    end_ticks_l = int(input("Enter final left encoder ticks: "))
    end_ticks_r = int(input("Enter final right encoder ticks: "))

    delta_l = abs(end_ticks_l - start_ticks_l)
    delta_r = abs(end_ticks_r - start_ticks_r)

    # Total distance moved by wheels = 2 * pi * (wheel_separation / 2) * num_rotations for each wheel
    # Distance = pi * wheel_separation * num_rotations
    ticks_per_rev = 1920
    dist_l = (delta_l / ticks_per_rev) * (2.0 * math.pi * calibrated_radius)
    dist_r = (delta_r / ticks_per_rev) * (2.0 * math.pi * calibrated_radius)
    avg_arc_length = (dist_l + dist_r) / 2.0

    total_angle_radians = num_rotations * 2.0 * math.pi
    calibrated_separation = (dist_l + dist_r) / total_angle_radians

    print(f"\nResults:")
    print(f"  Left wheel traveled:  {dist_l:.4f} m")
    print(f"  Right wheel traveled: {dist_r:.4f} m")
    print(f"\n==> CALIBRATED WHEEL SEPARATION (Track Width): {calibrated_separation:.5f} m ({calibrated_separation*1000.0:.2f} mm)")
    return calibrated_separation


def main():
    print("=================================================================")
    print("  AUTONOMOUS DELIVERY ROBOT - ODOMETRY CALIBRATION WIZARD")
    print("=================================================================")
    radius = calibrate_linear()
    separation = calibrate_angular(radius)

    print("\n=================================================================")
    print("Update your config/robot_params.yaml with these calibrated values:")
    print(f"  wheel_radius: {radius:.5f}")
    print(f"  wheel_separation: {separation:.5f}")
    print("=================================================================\n")


if __name__ == '__main__':
    main()
