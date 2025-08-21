"""
Differential Drive Kinematics & Odometry Module
Clean mathematical implementation of forward/inverse kinematics,
Runge-Kutta 2nd order odometry integration, velocity ramp limits,
and covariance estimation.
"""

import math
from typing import Tuple, List


class DifferentialDriveKinematics:
    """
    Mathematical kinematics model for differential-drive mobile base.
    """

    def __init__(self, wheel_radius: float = 0.075, wheel_separation: float = 0.40,
                 encoder_ticks_per_rev: int = 1920):
        """
        :param wheel_radius: Drive wheel radius in meters (default 0.075m = 75mm)
        :param wheel_separation: Distance between left and right wheel contact points (track width) in meters
        :param encoder_ticks_per_rev: Encoder ticks per full output wheel revolution
        """
        if wheel_radius <= 0.0:
            raise ValueError("Wheel radius must be strictly positive.")
        if wheel_separation <= 0.0:
            raise ValueError("Wheel separation must be strictly positive.")
        if encoder_ticks_per_rev <= 0:
            raise ValueError("Encoder ticks per rev must be positive.")

        self.wheel_radius = float(wheel_radius)
        self.wheel_separation = float(wheel_separation)
        self.encoder_ticks_per_rev = int(encoder_ticks_per_rev)
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.encoder_ticks_per_rev

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize angle into [-pi, pi) radians.
        """
        while angle >= math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def forward_kinematics(self, left_wheel_vel: float, right_wheel_vel: float) -> Tuple[float, float]:
        """
        Calculate robot linear and angular velocity from individual wheel velocities.
        :param left_wheel_vel: Left wheel linear velocity in m/s
        :param right_wheel_vel: Right wheel linear velocity in m/s
        :return: (linear_velocity_m_s, angular_velocity_rad_s)
        """
        v = (right_wheel_vel + left_wheel_vel) / 2.0
        omega = (right_wheel_vel - left_wheel_vel) / self.wheel_separation
        return v, omega

    def inverse_kinematics(self, linear_velocity: float, angular_velocity: float) -> Tuple[float, float, float, float]:
        """
        Calculate individual wheel linear velocities and RPMs from robot cmd_vel.
        :param linear_velocity: Desired robot linear velocity in m/s
        :param angular_velocity: Desired robot angular velocity in rad/s
        :return: (left_vel_m_s, right_vel_m_s, left_rpm, right_rpm)
        """
        v_left = linear_velocity - (angular_velocity * self.wheel_separation / 2.0)
        v_right = linear_velocity + (angular_velocity * self.wheel_separation / 2.0)

        # Convert linear wheel speed to wheel RPM
        # RPM = (v / (2 * pi * r)) * 60
        left_rpm = (v_left / (2.0 * math.pi * self.wheel_radius)) * 60.0
        right_rpm = (v_right / (2.0 * math.pi * self.wheel_radius)) * 60.0

        return v_left, v_right, left_rpm, right_rpm

    def ticks_to_distance(self, delta_ticks: int) -> float:
        """
        Convert encoder ticks to linear ground distance traveled in meters.
        """
        return delta_ticks * self.meters_per_tick

    def integrate_odometry(self, current_x: float, current_y: float, current_yaw: float,
                           delta_left_ticks: int, delta_right_ticks: int, dt: float) -> Tuple[float, float, float, float, float]:
        """
        Integrate wheel encoder displacement using 2nd Order Runge-Kutta (midpoint).
        :param current_x: Previous X pose (m)
        :param current_y: Previous Y pose (m)
        :param current_yaw: Previous Yaw orientation (rad)
        :param delta_left_ticks: Change in left encoder ticks over interval dt
        :param delta_right_ticks: Change in right encoder ticks over interval dt
        :param dt: Time interval in seconds
        :return: (new_x, new_y, new_yaw, linear_velocity, angular_velocity)
        """
        if dt <= 0.0:
            return current_x, current_y, current_yaw, 0.0, 0.0

        delta_left_m = self.ticks_to_distance(delta_left_ticks)
        delta_right_m = self.ticks_to_distance(delta_right_ticks)

        delta_dist = (delta_right_m + delta_left_m) / 2.0
        delta_yaw = (delta_right_m - delta_left_m) / self.wheel_separation

        # Midpoint Runge-Kutta angle
        midpoint_yaw = current_yaw + (delta_yaw / 2.0)

        new_x = current_x + delta_dist * math.cos(midpoint_yaw)
        new_y = current_y + delta_dist * math.sin(midpoint_yaw)
        new_yaw = self.normalize_angle(current_yaw + delta_yaw)

        vx = delta_dist / dt
        wz = delta_yaw / dt

        return new_x, new_y, new_yaw, vx, wz

    @staticmethod
    def clamp_velocity(target_v: float, target_w: float,
                       current_v: float, current_w: float,
                       max_v: float, min_v: float, max_w: float,
                       max_accel_v: float, max_accel_w: float,
                       dt: float) -> Tuple[float, float]:
        """
        Ramps velocity to stay within acceleration and speed bounds (prevents tipping/skidding).
        """
        # Hard limits
        clamped_target_v = max(min_v, min(max_v, target_v))
        clamped_target_w = max(-max_w, min(max_w, target_w))

        # Acceleration limits
        max_delta_v = max_accel_v * dt
        max_delta_w = max_accel_w * dt

        delta_v = clamped_target_v - current_v
        delta_w = clamped_target_w - current_w

        if abs(delta_v) > max_delta_v:
            new_v = current_v + math.copysign(max_delta_v, delta_v)
        else:
            new_v = clamped_target_v

        if abs(delta_w) > max_delta_w:
            new_w = current_w + math.copysign(max_delta_w, delta_w)
        else:
            new_w = clamped_target_w

        return new_v, new_w

    @staticmethod
    def build_covariance_matrices(delta_dist: float, delta_yaw: float) -> Tuple[List[float], List[float]]:
        """
        Generate 6x6 pose and twist covariance arrays (flattened 36-element lists)
        proportional to distance traveled.
        """
        # Base uncertainties
        var_x = 0.0001 + 0.01 * abs(delta_dist)
        var_y = 0.0001 + 0.01 * abs(delta_dist)
        var_yaw = 0.0005 + 0.02 * abs(delta_yaw)

        # 36 element row-major covariance matrix
        pose_cov = [0.0] * 36
        pose_cov[0] = var_x          # x
        pose_cov[7] = var_y          # y
        pose_cov[14] = 1e6           # z (unobservable 2D)
        pose_cov[21] = 1e6           # roll
        pose_cov[28] = 1e6           # pitch
        pose_cov[35] = var_yaw       # yaw

        twist_cov = [0.0] * 36
        twist_cov[0] = 0.002         # vx
        twist_cov[7] = 0.002         # vy
        twist_cov[14] = 1e6          # vz
        twist_cov[21] = 1e6          # vroll
        twist_cov[28] = 1e6          # vpitch
        twist_cov[35] = 0.005        # vyaw

        return pose_cov, twist_cov
