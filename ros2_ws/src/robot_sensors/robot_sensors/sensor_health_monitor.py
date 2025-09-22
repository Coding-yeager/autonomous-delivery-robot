#!/usr/bin/env python3
"""
Sensor Health Monitor Node.
Supervises heartbeat rates of LiDAR, IMU, GNSS, and Wheel Odometry.
Triggers emergency stops or warnings if critical navigation sensors fail.
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class SensorHealthMonitor(Node):
    def __init__(self):
        super().__init__('sensor_health_monitor')

        # Parameter Declarations
        self.declare_parameter('check_rate_hz', 2.0)
        self.declare_parameter('timeout_lidar_sec', 1.0)
        self.declare_parameter('timeout_imu_sec', 0.5)
        self.declare_parameter('timeout_gps_sec', 3.0)
        self.declare_parameter('timeout_odom_sec', 0.5)
        self.declare_parameter('emergency_stop_topic', '/emergency_stop')

        self.check_rate = self.get_parameter('check_rate_hz').get_parameter_value().double_value
        self.to_lidar = self.get_parameter('timeout_lidar_sec').get_parameter_value().double_value
        self.to_imu = self.get_parameter('timeout_imu_sec').get_parameter_value().double_value
        self.to_gps = self.get_parameter('timeout_gps_sec').get_parameter_value().double_value
        self.to_odom = self.get_parameter('timeout_odom_sec').get_parameter_value().double_value
        self.estop_topic = self.get_parameter('emergency_stop_topic').get_parameter_value().string_value

        # Timestamps of last received messages
        now_mono = time.monotonic()
        self.last_lidar = now_mono
        self.last_imu = now_mono
        self.last_gps = now_mono
        self.last_odom = now_mono

        self.lidar_received = False
        self.imu_received = False
        self.gps_received = False
        self.odom_received = False

        # Subscriptions
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/data', 10)
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix', 10)
        self.sub_odom = self.create_subscription(Odometry, '/wheel/odometry', 10)

        # Publishers
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.estop_pub = self.create_publisher(Bool, self.estop_topic, 10)

        self.timer = self.create_timer(1.0 / self.check_rate, self.check_health)
        self.get_logger().info("Sensor Health Monitor initiated.")

    def lidar_cb(self, _):
        self.last_lidar = time.monotonic()
        self.lidar_received = True

    def imu_cb(self, _):
        self.last_imu = time.monotonic()
        self.imu_received = True

    def gps_cb(self, _):
        self.last_gps = time.monotonic()
        self.gps_received = True

    def odom_cb(self, _):
        self.last_odom = time.monotonic()
        self.odom_received = True

    def check_health(self):
        now = time.monotonic()
        d_lidar = now - self.last_lidar
        d_imu = now - self.last_imu
        d_gps = now - self.last_gps
        d_odom = now - self.last_odom

        # Flag critical sensors
        lidar_ok = self.lidar_received and (d_lidar <= self.to_lidar)
        imu_ok = self.imu_received and (d_imu <= self.to_imu)
        odom_ok = self.odom_received and (d_odom <= self.to_odom)
        gps_ok = self.gps_received and (d_gps <= self.to_gps)

        critical_fault = False
        fault_reasons = []

        if not odom_ok:
            critical_fault = True
            fault_reasons.append(f"Odom dropout ({d_odom:.1f}s)")
        if not imu_ok:
            critical_fault = True
            fault_reasons.append(f"IMU dropout ({d_imu:.1f}s)")
        if not lidar_ok:
            critical_fault = True
            fault_reasons.append(f"LiDAR dropout ({d_lidar:.1f}s)")

        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        stat = DiagnosticStatus()
        stat.name = "Sensor Health Supervisor"
        stat.hardware_id = "RaspberryPi4B_Peripherals"

        if critical_fault:
            stat.level = DiagnosticStatus.ERROR
            stat.message = "CRITICAL: " + ", ".join(fault_reasons)
            self.get_logger().error_throttle(2.0, stat.message)

            # Trigger emergency stop to halt motion
            estop_msg = Bool()
            estop_msg.data = True
            self.estop_pub.publish(estop_msg)
        elif not gps_ok:
            stat.level = DiagnosticStatus.WARN
            stat.message = f"WARNING: GNSS signal degraded/dropped ({d_gps:.1f}s). Continuing on local odometry."
            self.get_logger().warn_throttle(5.0, stat.message)
        else:
            stat.level = DiagnosticStatus.OK
            stat.message = "All navigation sensors functioning within expected tolerances."

        stat.values = [
            KeyValue(key="lidar_age_sec", value=f"{d_lidar:.2f}"),
            KeyValue(key="imu_age_sec", value=f"{d_imu:.2f}"),
            KeyValue(key="gps_age_sec", value=f"{d_gps:.2f}"),
            KeyValue(key="odom_age_sec", value=f"{d_odom:.2f}"),
        ]

        diag_array.status.append(stat)
        self.diag_pub.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)
    node = SensorHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
