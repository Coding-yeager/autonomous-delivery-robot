"""
Robot Base Package
Handles differential-drive kinematics, encoder odometry, hardware abstraction,
safety watchdog, and motor speed control.
"""

from .kinematics import DifferentialDriveKinematics

__all__ = ['DifferentialDriveKinematics']
