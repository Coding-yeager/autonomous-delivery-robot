"""
Unit tests for Odometry Integration and Covariance Generation.
Verifies Runge-Kutta 2nd order numerical integration across linear and arc trajectories.
"""

import math
import sys
import os
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'ros2_ws', 'src', 'robot_base')))

from robot_base.kinematics import DifferentialDriveKinematics


@pytest.fixture
def diff_drive():
    return DifferentialDriveKinematics(wheel_radius=0.075, wheel_separation=0.40, encoder_ticks_per_rev=1920)


def test_integrate_stationary(diff_drive):
    # Robot does not move
    x, y, yaw, vx, wz = diff_drive.integrate_odometry(
        current_x=1.0, current_y=2.0, current_yaw=0.5,
        delta_left_ticks=0, delta_right_ticks=0, dt=0.02
    )
    assert x == 1.0
    assert y == 2.0
    assert yaw == 0.5
    assert vx == 0.0
    assert wz == 0.0


def test_integrate_straight_forward(diff_drive):
    # One revolution of both wheels = 1920 ticks
    # Traveled distance = 2 * pi * 0.075 = 0.4712389 m
    # Initial pose: (0, 0, 0)
    dt = 1.0  # 1 second
    x, y, yaw, vx, wz = diff_drive.integrate_odometry(
        current_x=0.0, current_y=0.0, current_yaw=0.0,
        delta_left_ticks=1920, delta_right_ticks=1920, dt=dt
    )

    expected_dist = 2.0 * math.pi * 0.075
    assert pytest.approx(x, rel=1e-4) == expected_dist
    assert pytest.approx(y, abs=1e-5) == 0.0
    assert pytest.approx(yaw, abs=1e-5) == 0.0
    assert pytest.approx(vx, rel=1e-4) == expected_dist / dt
    assert pytest.approx(wz, abs=1e-5) == 0.0


def test_integrate_circular_arc(diff_drive):
    # Pure 90-degree in-place turn (left wheel backwards, right wheel forwards)
    # Arc length of each wheel = (separation / 2) * angle = 0.20 * (pi / 2) = 0.10 * pi = 0.314159 m
    # Ticks = 0.314159 / (2 * pi * 0.075 / 1920) = 1280 ticks
    target_angle = math.pi / 2.0
    wheel_dist = (diff_drive.wheel_separation / 2.0) * target_angle
    ticks = int(round(wheel_dist / diff_drive.meters_per_tick))

    x, y, yaw, vx, wz = diff_drive.integrate_odometry(
        current_x=0.0, current_y=0.0, current_yaw=0.0,
        delta_left_ticks=-ticks, delta_right_ticks=ticks, dt=1.0
    )

    # Position should remain centered at origin for pure in-place turn
    assert pytest.approx(x, abs=1e-3) == 0.0
    assert pytest.approx(y, abs=1e-3) == 0.0
    assert pytest.approx(yaw, rel=1e-2) == target_angle


def test_build_covariance():
    pose_cov, twist_cov = DifferentialDriveKinematics.build_covariance_matrices(delta_dist=1.5, delta_yaw=0.2)
    assert len(pose_cov) == 36
    assert len(twist_cov) == 36

    # Positional uncertainty increases with distance traveled
    assert pose_cov[0] > 0.01  # Var(x)
    assert pose_cov[7] > 0.01  # Var(y)
    assert pose_cov[35] > 0.001 # Var(yaw)

    # Unobserved z, roll, pitch have high variance (1e6)
    assert pose_cov[14] == 1e6
    assert pose_cov[21] == 1e6
    assert pose_cov[28] == 1e6
