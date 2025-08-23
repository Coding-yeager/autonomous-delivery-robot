"""
Unit tests for Differential Drive Kinematics.
Verifies forward kinematics, inverse kinematics, RPM calculations, and angle normalization.
"""

import math
import sys
import os
import pytest

# Add package to path for direct testing
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'ros2_ws', 'src', 'robot_base')))

from robot_base.kinematics import DifferentialDriveKinematics


@pytest.fixture
def diff_drive():
    # 150mm wheels (r = 0.075m), 400mm separation (L = 0.40m), 1920 ticks/rev
    return DifferentialDriveKinematics(wheel_radius=0.075, wheel_separation=0.40, encoder_ticks_per_rev=1920)


def test_invalid_parameters():
    with pytest.raises(ValueError):
        DifferentialDriveKinematics(wheel_radius=0.0, wheel_separation=0.40)
    with pytest.raises(ValueError):
        DifferentialDriveKinematics(wheel_radius=0.075, wheel_separation=-0.10)
    with pytest.raises(ValueError):
        DifferentialDriveKinematics(wheel_radius=0.075, wheel_separation=0.40, encoder_ticks_per_rev=0)


def test_forward_kinematics_straight(diff_drive):
    # Both wheels moving at 0.50 m/s forward
    v, omega = diff_drive.forward_kinematics(left_wheel_vel=0.50, right_wheel_vel=0.50)
    assert pytest.approx(v, rel=1e-5) == 0.50
    assert pytest.approx(omega, abs=1e-5) == 0.0


def test_forward_kinematics_pure_rotation(diff_drive):
    # Left wheel -0.2 m/s, right wheel +0.2 m/s -> pure counter-clockwise rotation
    v, omega = diff_drive.forward_kinematics(left_wheel_vel=-0.20, right_wheel_vel=0.20)
    assert pytest.approx(v, abs=1e-5) == 0.0
    # omega = (0.2 - (-0.2)) / 0.4 = 0.4 / 0.4 = 1.0 rad/s
    assert pytest.approx(omega, rel=1e-5) == 1.00


def test_inverse_kinematics_straight(diff_drive):
    # Target: 0.40 m/s linear, 0.0 rad/s angular
    vl, vr, rpml, rpmr = diff_drive.inverse_kinematics(linear_velocity=0.40, angular_velocity=0.0)
    assert pytest.approx(vl, rel=1e-5) == 0.40
    assert pytest.approx(vr, rel=1e-5) == 0.40

    # Circumference = 2 * pi * 0.075 = 0.471238 m
    # RPM = (0.40 / 0.471238) * 60 = 50.9295 RPM
    expected_rpm = (0.40 / (2.0 * math.pi * 0.075)) * 60.0
    assert pytest.approx(rpml, rel=1e-4) == expected_rpm
    assert pytest.approx(rpmr, rel=1e-4) == expected_rpm


def test_inverse_kinematics_turning(diff_drive):
    # Target: 0.30 m/s linear, 0.5 rad/s angular
    # vl = 0.30 - (0.5 * 0.4 / 2) = 0.30 - 0.10 = 0.20 m/s
    # vr = 0.30 + (0.5 * 0.4 / 2) = 0.30 + 0.10 = 0.40 m/s
    vl, vr, _, _ = diff_drive.inverse_kinematics(linear_velocity=0.30, angular_velocity=0.50)
    assert pytest.approx(vl, rel=1e-5) == 0.20
    assert pytest.approx(vr, rel=1e-5) == 0.40


def test_ticks_to_distance(diff_drive):
    # Exactly one revolution = 1920 ticks
    # Distance = 2 * pi * 0.075 = 0.471238898 m
    dist = diff_drive.ticks_to_distance(1920)
    expected_dist = 2.0 * math.pi * 0.075
    assert pytest.approx(dist, rel=1e-5) == expected_dist


def test_angle_normalization(diff_drive):
    assert pytest.approx(diff_drive.normalize_angle(0.0)) == 0.0
    assert pytest.approx(diff_drive.normalize_angle(math.pi)) == -math.pi
    assert pytest.approx(diff_drive.normalize_angle(3.0 * math.pi)) == -math.pi
    assert pytest.approx(diff_drive.normalize_angle(-3.0 * math.pi)) == -math.pi
    assert pytest.approx(diff_drive.normalize_angle(math.pi / 2.0)) == math.pi / 2.0
    assert pytest.approx(diff_drive.normalize_angle(2.5 * math.pi)) == math.pi / 2.0
