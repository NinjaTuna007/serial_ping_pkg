"""Tests for the shared on-air ASCII decimal catalog.

Exercises ``serial_ping_pkg.common.on_air`` directly (no ROS, no hardware):
constants, formatters, clamping, and that the centimetre-scale position axes
share one grain.
"""

import pytest

from serial_ping_pkg.common.on_air import (
    DEPTH_DECIMALS,
    DEPTH_MAX,
    HEADING_DECIMALS,
    LATLON_DECIMALS,
    RANGE_DECIMALS,
    SPEED_DECIMALS,
    SVS_DECIMALS,
    format_depth,
    format_depth_padded,
    format_heading,
    format_latlon,
    format_range,
    format_speed,
    format_svs,
)


def test_catalog_values():
    """Locked on-air grain: ~1 cm position, 0.1 m/s SVS, integer heading."""
    assert LATLON_DECIMALS == 7
    assert DEPTH_DECIMALS == 2
    assert HEADING_DECIMALS == 0
    assert RANGE_DECIMALS == 2
    assert SVS_DECIMALS == 1
    assert SPEED_DECIMALS == 2


def test_format_latlon_seven_places():
    """Lat/lon print at 7 decimal places (rounded)."""
    assert format_latlon(59.12345678, 18.87654321) == '59.1234568,18.8765432'


def test_format_latlon_precision_override():
    """Beacon may drop digits without editing the catalog."""
    assert format_latlon(59.1, 18.8, decimals=4) == '59.1000,18.8000'


def test_format_depth_unpadded_and_padded():
    """Beacon depth is unpadded; relay depth is DDD.DD."""
    assert format_depth(12.3) == '12.30'
    assert format_depth_padded(42.5) == '042.50'
    assert format_depth_padded(0.0) == '000.00'


def test_format_depth_padded_clamps():
    """Relay depth clamps to [0, DEPTH_MAX]."""
    assert format_depth_padded(-5.0) == '000.00'
    assert format_depth_padded(1500.0) == f'{DEPTH_MAX:06.2f}'
    assert DEPTH_MAX == pytest.approx(999.99)


def test_format_heading_clamps_and_pads():
    """Heading is integer degrees in 000..359."""
    assert format_heading(270) == '270'
    assert format_heading(0) == '000'
    assert format_heading(-10) == '000'
    assert format_heading(400) == '359'


def test_format_range_svs_speed():
    """Range and speed are centimetres; SVS is 0.1 m/s."""
    assert format_range(468.75) == '468.75'
    assert format_range(1500.0) == '1500.00'
    assert format_svs(1481.6) == '1481.6'
    assert format_speed(1.2) == '1.20'
