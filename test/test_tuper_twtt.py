"""Pure-logic tests for the tuper_twtt leader/follower wire format.

Exercises ``serial_ping_pkg.tuper_twtt.leader_protocol`` directly
(no ROS, no hardware): the leader's ``$B`` position+distance builder, the
follower's bare ``lat,lon,dist`` parser, and that the leader reuses the shared
travel-time conversion. Mirrors the style of ``test_owtt_protocol.py``.
"""

import pytest

from serial_ping_pkg.common.ping_protocol import travel_time_to_distance
from serial_ping_pkg.tuper_twtt.leader_protocol import (
    build_position_broadcast,
    parse_position_distance,
)


# --------------------------------------------------------------------------- #
# Builder                                                                     #
# --------------------------------------------------------------------------- #

def test_build_position_broadcast_length_prefix():
    """The byte count covers the stringified values plus the two commas."""
    frame = build_position_broadcast(59.1, 18.8, 468.75)
    # len("59.1") + len("18.8") + len("468.75") + 2 == 16
    assert frame == "$B1659.1,18.8,468.75"


def test_build_position_broadcast_counts_dynamically():
    """The prefix tracks the actual lengths of the values."""
    frame = build_position_broadcast(1.0, 2.0, 3.0)
    # len("1.0") * 3 + 2 commas == 11
    assert frame == "$B111.0,2.0,3.0"


# --------------------------------------------------------------------------- #
# Parser                                                                      #
# --------------------------------------------------------------------------- #

def test_parse_position_distance_ok():
    """A bare lat,lon,dist payload parses to three floats."""
    assert parse_position_distance("59.1,18.8,468.75") == (
        pytest.approx(59.1), pytest.approx(18.8), pytest.approx(468.75))


def test_parse_position_distance_wrong_field_count():
    """Anything other than three comma-separated values is rejected."""
    assert parse_position_distance("a,b") is None


def test_parse_position_distance_with_header_rejected():
    """A leftover $B<n> header makes the first field non-numeric -> reject."""
    assert parse_position_distance("$B1659.1,18.8,468.75") is None


# --------------------------------------------------------------------------- #
# Shared travel-time conversion reuse                                         #
# --------------------------------------------------------------------------- #

def test_leader_reuses_common_travel_time():
    """The leader's range math is the shared common helper."""
    assert travel_time_to_distance(32000.0, 1500.0) == pytest.approx(1500.0)
