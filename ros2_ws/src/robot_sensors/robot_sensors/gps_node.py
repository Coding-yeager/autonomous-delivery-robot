#!/usr/bin/env python3
"""
Holybro NEO-M9N GNSS Receiver Node for ROS 2.
Parses NMEA sentences ($GNGGA, $GNRMC) over UART serial,
filters fixes by satellite count and HDOP, and publishes sensor_msgs/NavSatFix.
Supports Mock GNSS simulation for testing.
"""

import math
import time
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix, NavSatStatus
    from geometry_msgs.msg import TwistWithCovarianceStamped
except ImportError:
    rclpy = None
    class Node:  # type: ignore
        def __init__(self, *args, **kwargs):
            pass
    NavSatFix = None  # type: ignore
    NavSatStatus = None  # type: ignore
    TwistWithCovarianceStamped = None  # type: ignore

try:
    from robot_interfaces.msg import GpsStatus
except ImportError:
    GpsStatus = None  # type: ignore


def parse_nmea_coord(coord_str: str, direction: str) -> float:
    """
    Convert NMEA lat/lon string (ddmm.mmmm or dddmm.mmmm) to decimal degrees.
    """
    if not coord_str or not direction:
        return 0.0

    dot_pos = coord_str.find('.')
    if dot_pos < 0:
        return 0.0

    deg_len = dot_pos - 2
    degrees = float(coord_str[:deg_len])
    minutes = float(coord_str[deg_len:])
    decimal = degrees + (minutes / 60.0)

    if direction.upper() in ['S', 'W']:
        decimal = -decimal

    return decimal


class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyGPS')
        self.declare_parameter('serial_baudrate', 38400)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('use_mock_sensor', True)
        self.declare_parameter('min_satellites', 6)
        self.declare_parameter('max_hdop', 2.5)

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('serial_baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.use_mock = self.get_parameter('use_mock_sensor').get_parameter_value().bool_value
        self.min_sats = self.get_parameter('min_satellites').get_parameter_value().integer_value
        self.max_hdop = self.get_parameter('max_hdop').get_parameter_value().double_value

        self.serial_conn = None
        if not self.use_mock:
            try:
                import serial  # type: ignore
                self.serial_conn = serial.Serial(self.serial_port, self.baudrate, timeout=0.5)
                self.get_logger().info(f"Opened GNSS UART port {self.serial_port} at {self.baudrate} baud.")
            except Exception as e:
                self.get_logger().error(f"Failed to open GNSS serial port: {e}. Falling back to MOCK mode.")
                self.use_mock = True

        if self.use_mock:
            self.get_logger().info("Operating GNSS receiver in MOCK SENSOR mode (IIT Mandi Campus coordinates).")

        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.vel_pub = self.create_publisher(TwistWithCovarianceStamped, '/gps/vel', 10)
        if GpsStatus is not None:
            self.status_pub = self.create_publisher(GpsStatus, '/gps/status', 10)
        else:
            self.status_pub = None

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.update_loop)

        # Mock trajectory state around IIT Mandi South Campus (31.7815 N, 76.9943 E)
        self.mock_lat = 31.78150
        self.mock_lon = 76.99430
        self.mock_alt = 1040.0  # meters altitude in Kamand valley
        self.mock_step = 0

    def parse_gga(self, parts) -> dict:
        """
        Parse $GNGGA sentence parts.
        """
        try:
            time_utc = parts[1]
            lat = parse_nmea_coord(parts[2], parts[3])
            lon = parse_nmea_coord(parts[4], parts[5])
            fix_quality = int(parts[6]) if parts[6] else 0
            num_sats = int(parts[7]) if parts[7] else 0
            hdop = float(parts[8]) if parts[8] else 99.9
            altitude = float(parts[9]) if parts[9] else 0.0

            return {
                'valid': fix_quality > 0 and num_sats >= self.min_sats and hdop <= self.max_hdop,
                'latitude': lat,
                'longitude': lon,
                'altitude': altitude,
                'fix_quality': fix_quality,
                'satellites': num_sats,
                'hdop': hdop
            }
        except (ValueError, IndexError):
            return {'valid': False}

    def update_loop(self):
        now = self.get_clock().now()

        if not self.use_mock and self.serial_conn is not None:
            data = None
            try:
                line = self.serial_conn.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                    parts = line.split(',')
                    data = self.parse_gga(parts)
            except Exception as e:
                self.get_logger().warn(f"UART read error: {e}")
                return

            if data and data.get('valid', False):
                self.publish_fix(now, data['latitude'], data['longitude'], data['altitude'],
                                 data['satellites'], data['hdop'], data['fix_quality'])
        else:
            # Generate realistic campus trajectory in mock mode
            self.mock_step += 1
            # Slight slow drift simulating robot roaming on path
            lat = self.mock_lat + 0.00001 * math.sin(self.mock_step * 0.05)
            lon = self.mock_lon + 0.00001 * math.cos(self.mock_step * 0.05)
            self.publish_fix(now, lat, lon, self.mock_alt, satellites=14, hdop=0.8, fix_quality=2)

    def publish_fix(self, now, lat: float, lon: float, alt: float,
                    satellites: int, hdop: float, fix_quality: int):
        fix = NavSatFix()
        fix.header.stamp = now.to_msg()
        fix.header.frame_id = self.frame_id

        if fix_quality > 0:
            fix.status.status = NavSatStatus.STATUS_FIX
        else:
            fix.status.status = NavSatStatus.STATUS_NO_FIX

        fix.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_GALILEO

        fix.latitude = float(lat)
        fix.longitude = float(lon)
        fix.altitude = float(alt)

        # Position covariance from HDOP (approximate 2.5m CEP * HDOP)
        pos_accuracy = (hdop * 2.5) ** 2
        alt_accuracy = (hdop * 5.0) ** 2
        fix.position_covariance = [
            pos_accuracy, 0.0, 0.0,
            0.0, pos_accuracy, 0.0,
            0.0, 0.0, alt_accuracy
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.fix_pub.publish(fix)

        if self.status_pub is not None:
            s = GpsStatus()
            s.header.stamp = now.to_msg()
            s.fix_valid = bool(fix_quality > 0)
            s.satellites_visible = int(satellites)
            s.hdop = float(hdop)
            s.latitude = float(lat)
            s.longitude = float(lon)
            s.altitude = float(alt)
            s.fix_type = "DGPS/SBAS Fix" if fix_quality == 2 else "Standard GPS Fix"
            self.status_pub.publish(s)

    def destroy_node(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
