#!/usr/bin/env python3
"""
Base Controller ROS 2 Node for Differential Drive Delivery Robot.
Subscribes to /cmd_vel, enforces acceleration limits & cmd_vel watchdog,
drives hardware or mock motors, computes encoder odometry, and publishes /wheel/odometry.
"""

import time
import math
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from geometry_msgs.msg import Twist, TransformStamped, Quaternion
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, Header
    from tf2_ros import TransformBroadcaster
except ImportError:
    rclpy = None
    class Node:  # type: ignore
        def __init__(self, *args, **kwargs):
            pass
    Time = None  # type: ignore
    Twist = None  # type: ignore
    TransformStamped = None  # type: ignore
    Quaternion = None  # type: ignore
    Odometry = None  # type: ignore
    Bool = None  # type: ignore
    Header = None  # type: ignore
    TransformBroadcaster = None  # type: ignore

try:
    from robot_interfaces.msg import MotorVels, SafetyStatus
except ImportError:
    # Graceful fallback for non-ROS or pre-build linting environments
    MotorVels = None  # type: ignore
    SafetyStatus = None  # type: ignore

from .kinematics import DifferentialDriveKinematics
from .hardware_interface import BaseHardwareInterface, MockHardwareDriver, RealHardwareDriver


class BaseControllerNode(Node):
    def __init__(self):
        super().__init__('robot_base_controller')

        # Declare ROS 2 Parameters
        self.declare_parameter('wheel_radius', 0.075)
        self.declare_parameter('wheel_separation', 0.40)
        self.declare_parameter('encoder_ticks_per_rev', 1920)
        self.declare_parameter('max_linear_velocity', 0.60)
        self.declare_parameter('min_linear_velocity', -0.30)
        self.declare_parameter('max_angular_velocity', 0.80)
        self.declare_parameter('max_linear_acceleration', 0.50)
        self.declare_parameter('max_angular_acceleration', 1.00)
        self.declare_parameter('cmd_vel_timeout', 0.50)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('use_mock_hardware', True)

        # Retrieve Parameter Values
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').get_parameter_value().integer_value
        self.max_v = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.min_v = self.get_parameter('min_linear_velocity').get_parameter_value().double_value
        self.max_w = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.max_accel_v = self.get_parameter('max_linear_acceleration').get_parameter_value().double_value
        self.max_accel_w = self.get_parameter('max_angular_acceleration').get_parameter_value().double_value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.use_mock = self.get_parameter('use_mock_hardware').get_parameter_value().bool_value

        # Initialize Kinematics Model
        self.kinematics = DifferentialDriveKinematics(
            wheel_radius=self.wheel_radius,
            wheel_separation=self.wheel_separation,
            encoder_ticks_per_rev=self.ticks_per_rev
        )

        # Initialize Hardware Interface
        self.hw: BaseHardwareInterface
        if self.use_mock:
            self.get_logger().info("Initializing base controller in MOCK HARDWARE mode.")
            self.hw = MockHardwareDriver(
                ticks_per_rev=self.ticks_per_rev,
                wheel_radius=self.wheel_radius,
                wheel_separation=self.wheel_separation
            )
        else:
            self.get_logger().info("Initializing base controller with REAL Raspberry Pi GPIO.")
            try:
                self.hw = RealHardwareDriver()
            except Exception as e:
                self.get_logger().error(f"Failed to initialize real hardware: {e}. Falling back to Mock.")
                self.hw = MockHardwareDriver(
                    ticks_per_rev=self.ticks_per_rev,
                    wheel_radius=self.wheel_radius,
                    wheel_separation=self.wheel_separation
                )

        # State Variables
        self.target_v = 0.0
        self.target_w = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        self.last_cmd_vel_time = self.get_clock().now()
        self.software_estop = False
        self.watchdog_tripped = False

        # Odometry Integration State
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.last_odom_time = self.get_clock().now()

        # Publishers & Subscriptions
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        if MotorVels is not None:
            self.motor_vels_pub = self.create_publisher(MotorVels, '/motor_vels', 10)
        else:
            self.motor_vels_pub = None

        if SafetyStatus is not None:
            self.safety_pub = self.create_publisher(SafetyStatus, '/safety_status', 10)
        else:
            self.safety_pub = None

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.estop_sub = self.create_subscription(Bool, '/emergency_stop', self.estop_callback, 10)

        # High Frequency Control & Odometry Timer
        timer_period = 1.0 / self.publish_rate
        self.control_timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info("Robot Base Controller node started successfully.")

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Record velocity command and reset watchdog.
        """
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z
        self.last_cmd_vel_time = self.get_clock().now()

    def estop_callback(self, msg: Bool) -> None:
        """
        Software emergency stop command.
        """
        self.software_estop = msg.data
        if self.software_estop:
            self.get_logger().warn("Software Emergency Stop TRIGGERED!")
            self.hw.stop_all()

    def control_loop(self) -> None:
        """
        Periodic control loop executing at publish_rate Hz.
        Handles watchdog timeout, velocity ramping, motor actuation, and odometry integration.
        """
        now = self.get_clock().now()
        dt_sec = (now - self.last_odom_time).nanoseconds / 1e9

        if dt_sec <= 0.0:
            return

        # 1. Watchdog & E-Stop Check
        time_since_cmd = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        is_hardware_estop = self.hw.read_estop()

        if is_hardware_estop or self.software_estop:
            self.target_v = 0.0
            self.target_w = 0.0
            self.current_v = 0.0
            self.current_w = 0.0
            self.hw.stop_all()
            status_msg = "Emergency Stop Active"
        elif time_since_cmd > self.cmd_vel_timeout:
            if not self.watchdog_tripped:
                self.get_logger().warn(f"cmd_vel watchdog timeout ({time_since_cmd:.2f}s > {self.cmd_vel_timeout}s). Halting motors.")
                self.watchdog_tripped = True
            self.target_v = 0.0
            self.target_w = 0.0
            status_msg = "Watchdog Timeout"
        else:
            self.watchdog_tripped = False
            status_msg = "Normal Operation"

        # 2. Velocity Ramping (Acceleration limits)
        self.current_v, self.current_w = self.kinematics.clamp_velocity(
            target_v=self.target_v,
            target_w=self.target_w,
            current_v=self.current_v,
            current_w=self.current_w,
            max_v=self.max_v,
            min_v=self.min_v,
            max_w=self.max_w,
            max_accel_v=self.max_accel_v,
            max_accel_w=self.max_accel_w,
            dt=dt_sec
        )

        # 3. Inverse Kinematics -> Motor PWM
        v_left, v_right, left_rpm, right_rpm = self.kinematics.inverse_kinematics(
            self.current_v, self.current_w
        )

        left_dir = v_left >= 0.0
        right_dir = v_right >= 0.0
        # Map velocity to 0..100% PWM duty cycle
        left_pwm = min(100.0, (abs(v_left) / self.max_v) * 100.0) if self.max_v > 0 else 0.0
        right_pwm = min(100.0, (abs(v_right) / self.max_v) * 100.0) if self.max_v > 0 else 0.0

        if not is_hardware_estop and not self.software_estop:
            self.hw.set_motor_speeds(left_pwm, right_pwm, left_dir, right_dir)
        else:
            self.hw.stop_all()

        # 4. Encoder Odometry Integration
        curr_left_ticks, curr_right_ticks = self.hw.read_encoder_ticks()
        delta_left = curr_left_ticks - self.prev_left_ticks
        delta_right = curr_right_ticks - self.prev_right_ticks
        self.prev_left_ticks = curr_left_ticks
        self.prev_right_ticks = curr_right_ticks

        (self.odom_x, self.odom_y, self.odom_yaw,
         odom_vx, odom_wz) = self.kinematics.integrate_odometry(
            current_x=self.odom_x,
            current_y=self.odom_y,
            current_yaw=self.odom_yaw,
            delta_left_ticks=delta_left,
            delta_right_ticks=delta_right,
            dt=dt_sec
        )

        self.last_odom_time = now

        # 5. Publish /wheel/odometry
        self.publish_odometry(now, self.odom_x, self.odom_y, self.odom_yaw, odom_vx, odom_wz,
                              delta_dist=self.kinematics.ticks_to_distance((delta_left + delta_right) // 2),
                              delta_yaw=odom_wz * dt_sec)

        # 6. Publish Diagnostic & Safety Interfaces
        if self.motor_vels_pub is not None:
            m_msg = MotorVels()
            m_msg.header.stamp = now.to_msg()
            m_msg.left_velocity_mps = float(v_left)
            m_msg.right_velocity_mps = float(v_right)
            m_msg.left_rpm = int(left_rpm)
            m_msg.right_rpm = int(right_rpm)
            m_msg.left_pwm = int(left_pwm)
            m_msg.right_pwm = int(right_pwm)
            self.motor_vels_pub.publish(m_msg)

        if self.safety_pub is not None:
            s_msg = SafetyStatus()
            s_msg.header.stamp = now.to_msg()
            s_msg.is_estop_active = bool(is_hardware_estop or self.software_estop)
            s_msg.is_watchdog_timed_out = bool(self.watchdog_tripped)
            s_msg.slope_warning = False
            s_msg.pitch_deg = 0.0
            s_msg.roll_deg = 0.0
            s_msg.status_message = status_msg
            self.safety_pub.publish(s_msg)

    def publish_odometry(self, now: Time, x: float, y: float, yaw: float,
                         vx: float, wz: float, delta_dist: float, delta_yaw: float) -> None:
        """
        Assemble and publish ROS 2 Odometry message and optional TF.
        """
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        # Position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        # Orientation quaternion from 2D yaw
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy

        # Covariance matrices
        pose_cov, twist_cov = self.kinematics.build_covariance_matrices(delta_dist, delta_yaw)
        odom.pose.covariance = pose_cov

        # Twist
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        # Dynamic TF broadcast (if configured)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self.hw.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
