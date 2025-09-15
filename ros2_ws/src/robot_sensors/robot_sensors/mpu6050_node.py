#!/usr/bin/env python3
"""
MPU-6050 6-DOF IMU ROS 2 Node.
Communicates via I2C (smbus2) or runs in Mock mode.
Publishes sensor_msgs/Imu with calibrated accelerometer and gyroscope data,
computes roll/pitch/yaw orientation, and monitors slope limits for campus terrain.
"""

import math
import time
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Header
    from geometry_msgs.msg import Quaternion
except ImportError:
    rclpy = None
    class Node:  # type: ignore
        def __init__(self, *args, **kwargs):
            pass
    Imu = None  # type: ignore
    Header = None  # type: ignore
    Quaternion = None  # type: ignore

try:
    from robot_interfaces.msg import SafetyStatus
except ImportError:
    SafetyStatus = None  # type: ignore


class MPU6050Driver:
    """Low-level I2C registers and conversion for MPU6050."""
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43

    ACCEL_SCALE = 16384.0  # LSB/g for +/- 2g
    GYRO_SCALE = 131.0     # LSB/(deg/s) for +/- 250 deg/s
    GRAVITY = 9.80665      # m/s^2

    def __init__(self, bus_num: int = 1, address: int = 0x68):
        self.bus_num = bus_num
        self.address = address
        self.bus = None
        self._init_i2c()

    def _init_i2c(self):
        import smbus2  # type: ignore
        self.bus = smbus2.SMBus(self.bus_num)
        # Wake up MPU-6050 (reset sleep bit)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        # Set sample rate divider (1kHz / (1 + 7) = 125 Hz)
        self.bus.write_byte_data(self.address, self.SMPLRT_DIV, 0x07)
        # DLPF 44Hz bandwidth
        self.bus.write_byte_data(self.address, self.CONFIG, 0x03)
        # Gyro +/- 250 deg/s
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
        # Accel +/- 2g
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

    def _read_word(self, reg: int) -> int:
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            val = -((65535 - val) + 1)
        return val

    def read_raw_data(self):
        ax_raw = self._read_word(self.ACCEL_XOUT_H)
        ay_raw = self._read_word(self.ACCEL_XOUT_H + 2)
        az_raw = self._read_word(self.ACCEL_XOUT_H + 4)
        gx_raw = self._read_word(self.GYRO_XOUT_H)
        gy_raw = self._read_word(self.GYRO_XOUT_H + 2)
        gz_raw = self._read_word(self.GYRO_XOUT_H + 4)

        ax = (ax_raw / self.ACCEL_SCALE) * self.GRAVITY
        ay = (ay_raw / self.ACCEL_SCALE) * self.GRAVITY
        az = (az_raw / self.ACCEL_SCALE) * self.GRAVITY

        gx = math.radians(gx_raw / self.GYRO_SCALE)
        gy = math.radians(gy_raw / self.GYRO_SCALE)
        gz = math.radians(gz_raw / self.GYRO_SCALE)

        return ax, ay, az, gx, gy, gz

    def close(self):
        if self.bus:
            self.bus.close()


class Mpu6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        # Declare parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('use_mock_sensor', True)
        self.declare_parameter('accel_offset_x', 0.0)
        self.declare_parameter('accel_offset_y', 0.0)
        self.declare_parameter('accel_offset_z', 0.0)
        self.declare_parameter('gyro_offset_x', 0.0)
        self.declare_parameter('gyro_offset_y', 0.0)
        self.declare_parameter('gyro_offset_z', 0.0)
        self.declare_parameter('linear_acceleration_stdev', 0.08)
        self.declare_parameter('angular_velocity_stdev', 0.005)
        self.declare_parameter('orientation_stdev', 0.02)
        self.declare_parameter('max_allowable_pitch_deg', 16.0)
        self.declare_parameter('max_allowable_roll_deg', 12.0)

        # Retrieve parameters
        self.i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        self.i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.use_mock = self.get_parameter('use_mock_sensor').get_parameter_value().bool_value

        self.off_ax = self.get_parameter('accel_offset_x').get_parameter_value().double_value
        self.off_ay = self.get_parameter('accel_offset_y').get_parameter_value().double_value
        self.off_az = self.get_parameter('accel_offset_z').get_parameter_value().double_value
        self.off_gx = self.get_parameter('gyro_offset_x').get_parameter_value().double_value
        self.off_gy = self.get_parameter('gyro_offset_y').get_parameter_value().double_value
        self.off_gz = self.get_parameter('gyro_offset_z').get_parameter_value().double_value

        self.accel_var = self.get_parameter('linear_acceleration_stdev').get_parameter_value().double_value ** 2
        self.gyro_var = self.get_parameter('angular_velocity_stdev').get_parameter_value().double_value ** 2
        self.orient_var = self.get_parameter('orientation_stdev').get_parameter_value().double_value ** 2

        self.max_pitch_deg = self.get_parameter('max_allowable_pitch_deg').get_parameter_value().double_value
        self.max_roll_deg = self.get_parameter('max_allowable_roll_deg').get_parameter_value().double_value

        # Initialize hardware or mock
        self.driver = None
        if not self.use_mock:
            try:
                self.driver = MPU6050Driver(bus_num=self.i2c_bus, address=self.i2c_address)
                self.get_logger().info(f"Connected to MPU6050 on I2C bus {self.i2c_bus} address 0x{self.i2c_address:02x}")
            except Exception as e:
                self.get_logger().error(f"Cannot initialize MPU6050: {e}. Falling back to MOCK mode.")
                self.use_mock = True

        if self.use_mock:
            self.get_logger().info("Operating MPU-6050 node in MOCK SENSOR mode.")

        # Orientation state
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        if SafetyStatus is not None:
            self.safety_pub = self.create_publisher(SafetyStatus, '/safety_status', 10)
        else:
            self.safety_pub = None

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu)

    def publish_imu(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        if not self.use_mock and self.driver is not None:
            try:
                ax, ay, az, gx, gy, gz = self.driver.read_raw_data()
                ax -= self.off_ax
                ay -= self.off_ay
                az -= self.off_az
                gx -= self.off_gx
                gy -= self.off_gy
                gz -= self.off_gz
            except Exception as e:
                self.get_logger().warn(f"I2C read error: {e}")
                return
        else:
            # Mock stationary sensor resting horizontally on Earth
            ax, ay, az = 0.0, 0.0, 9.80665
            gx, gy, gz = 0.0, 0.0, 0.0

        # Estimate Pitch and Roll from Accelerometer
        accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        accel_roll = math.atan2(ay, az)

        # Complementary filter (96% gyro integration, 4% accel gravity vector)
        alpha = 0.96
        self.pitch = alpha * (self.pitch + gy * dt) + (1.0 - alpha) * accel_pitch
        self.roll = alpha * (self.roll + gx * dt) + (1.0 - alpha) * accel_roll
        self.yaw += gz * dt

        pitch_deg = math.degrees(self.pitch)
        roll_deg = math.degrees(self.roll)

        # Campus Mountainous Terrain Slope Warning
        slope_danger = abs(pitch_deg) > self.max_pitch_deg or abs(roll_deg) > self.max_roll_deg
        if slope_danger and self.safety_pub is not None:
            s_msg = SafetyStatus()
            s_msg.header.stamp = now.to_msg()
            s_msg.is_estop_active = False
            s_msg.is_watchdog_timed_out = False
            s_msg.slope_warning = True
            s_msg.pitch_deg = float(pitch_deg)
            s_msg.roll_deg = float(roll_deg)
            s_msg.status_message = f"DANGER: Steep campus slope exceeded! Pitch: {pitch_deg:.1f}deg, Roll: {roll_deg:.1f}deg"
            self.safety_pub.publish(s_msg)
            self.get_logger().warn_throttle(2.0, s_msg.status_message)

        # Convert Euler to Quaternion
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sp = math.sin(self.pitch * 0.5)
        cr = math.cos(self.roll * 0.5)
        sr = math.sin(self.roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        # Build sensor_msgs/Imu
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = self.frame_id

        imu.orientation = q
        imu.orientation_covariance = [
            self.orient_var, 0.0, 0.0,
            0.0, self.orient_var, 0.0,
            0.0, 0.0, self.orient_var
        ]

        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)
        imu.angular_velocity_covariance = [
            self.gyro_var, 0.0, 0.0,
            0.0, self.gyro_var, 0.0,
            0.0, 0.0, self.gyro_var
        ]

        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        imu.linear_acceleration_covariance = [
            self.accel_var, 0.0, 0.0,
            0.0, self.accel_var, 0.0,
            0.0, 0.0, self.accel_var
        ]

        self.imu_pub.publish(imu)

    def destroy_node(self):
        if self.driver:
            self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Mpu6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
