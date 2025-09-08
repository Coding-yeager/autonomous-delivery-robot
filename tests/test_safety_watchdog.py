"""
Unit tests for Velocity Clamping, Acceleration Limits, and Watchdog Protection.
"""

import math
import sys
import os
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'ros2_ws', 'src', 'robot_base')))

from robot_base.kinematics import DifferentialDriveKinematics
from robot_base.hardware_interface import MockHardwareDriver


def test_velocity_clamping_acceleration_limits():
    # Attempting to instantly step from 0.0 to 1.0 m/s with max_accel = 0.50 m/s^2 over dt = 0.1s
    # Max delta allowed = 0.50 * 0.1 = 0.05 m/s
    new_v, new_w = DifferentialDriveKinematics.clamp_velocity(
        target_v=1.00,
        target_w=0.00,
        current_v=0.00,
        current_w=0.00,
        max_v=0.60,
        min_v=-0.30,
        max_w=0.80,
        max_accel_v=0.50,
        max_accel_w=1.00,
        dt=0.10
    )
    assert pytest.approx(new_v, rel=1e-5) == 0.05
    assert pytest.approx(new_w, abs=1e-5) == 0.00


def test_velocity_clamping_max_bound():
    # If currently at 0.58 m/s and requested 2.0 m/s, clamped to max_v = 0.60
    new_v, _ = DifferentialDriveKinematics.clamp_velocity(
        target_v=2.00,
        target_w=0.00,
        current_v=0.58,
        current_w=0.00,
        max_v=0.60,
        min_v=-0.30,
        max_w=0.80,
        max_accel_v=1.00,
        max_accel_w=1.00,
        dt=0.10
    )
    assert pytest.approx(new_v, rel=1e-5) == 0.60


def test_mock_hardware_estop():
    hw = MockHardwareDriver()
    hw.set_motor_speeds(left_pwm=80.0, right_pwm=80.0, left_dir=True, right_dir=True)
    assert hw.left_target_speed_mps > 0.0

    # Trigger emergency stop
    hw.set_estop_simulated(True)
    assert hw.read_estop() is True
    assert hw.left_target_speed_mps == 0.0
    assert hw.right_target_speed_mps == 0.0


def test_mock_hardware_relay():
    hw = MockHardwareDriver()
    hw.set_motor_speeds(left_pwm=50.0, right_pwm=50.0, left_dir=True, right_dir=True)
    assert hw.left_target_speed_mps > 0.0

    # Cut relay
    hw.set_relay(False)
    assert hw.left_target_speed_mps == 0.0
    assert hw.right_target_speed_mps == 0.0
