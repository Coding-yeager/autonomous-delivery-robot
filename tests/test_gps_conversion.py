"""
Unit tests for GNSS NMEA sentence parsing and coordinate conversion.
"""

import sys
import os
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'ros2_ws', 'src', 'robot_sensors')))

from robot_sensors.gps_node import parse_nmea_coord


def test_parse_nmea_coord_latitude():
    # NMEA: 3146.8900, N -> 31 deg + (46.8900 / 60) = 31.7815 deg North
    coord_str = "3146.8900"
    direction = "N"
    deg = parse_nmea_coord(coord_str, direction)
    assert pytest.approx(deg, rel=1e-5) == 31.7815


def test_parse_nmea_coord_longitude():
    # NMEA: 07659.6580, E -> 76 deg + (59.6580 / 60) = 76.9943 deg East
    coord_str = "07659.6580"
    direction = "E"
    deg = parse_nmea_coord(coord_str, direction)
    assert pytest.approx(deg, rel=1e-5) == 76.9943


def test_parse_nmea_coord_southern_western():
    # South -> negative
    coord_lat = "3146.8900"
    deg_south = parse_nmea_coord(coord_lat, "S")
    assert pytest.approx(deg_south, rel=1e-5) == -31.7815

    # West -> negative
    coord_lon = "07659.6580"
    deg_west = parse_nmea_coord(coord_lon, "W")
    assert pytest.approx(deg_west, rel=1e-5) == -76.9943


def test_parse_nmea_empty_or_malformed():
    assert parse_nmea_coord("", "N") == 0.0
    assert parse_nmea_coord("invalid", "N") == 0.0
