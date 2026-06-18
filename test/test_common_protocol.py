"""Pure-logic tests for the common ping/broadcast protocol helpers.

Exercises ``serial_ping_pkg.common.ping_protocol`` directly (no ROS, no
hardware): the Succorfish travel-time -> range conversion, the response
completeness gate, and the ``#B00`` leader-broadcast parser. Mirrors the style
of ``test_owtt_protocol.py``.
"""

import pytest

from serial_ping_pkg.common.ping_protocol import (
    parse_leader_broadcast,
    parse_ping_distance,
    ping_response_complete,
    travel_time_to_distance,
)


# --------------------------------------------------------------------------- #
# Travel-time -> distance                                                     #
# --------------------------------------------------------------------------- #

def test_travel_time_round_number():
    """32000 ticks * 1500 m/s * (1/32000) == 1500 m."""
    assert travel_time_to_distance(32000.0, 1500.0) == pytest.approx(1500.0)


def test_travel_time_matches_estimator_constant():
    """Same constant the ping_estimator_action uses: ticks * c * 3.125e-5."""
    assert travel_time_to_distance(12345, 1500.0) == pytest.approx(578.671875)


def test_parse_ping_distance_from_buffer():
    """The travel-time after the first 'T' drives the range."""
    assert parse_ping_distance("garbleT32000", 1500.0) == pytest.approx(1500.0)


def test_parse_ping_distance_raises_on_garbage():
    """A buffer without a numeric travel-time raises (callers catch it)."""
    with pytest.raises((IndexError, ValueError)):
        parse_ping_distance("no-delimiter-here", 1500.0)


# --------------------------------------------------------------------------- #
# Completeness gate                                                           #
# --------------------------------------------------------------------------- #

def test_response_complete_true():
    """A 'T' plus >=5 trailing chars is a complete reply."""
    assert ping_response_complete("fooT12345") is True


def test_response_complete_too_few_digits():
    """Fewer than 5 chars after 'T' is still incomplete."""
    assert ping_response_complete("fooT1234") is False


def test_response_complete_no_marker():
    """No 'T' at all is incomplete."""
    assert ping_response_complete("123456") is False


# --------------------------------------------------------------------------- #
# Leader broadcast parser                                                     #
# --------------------------------------------------------------------------- #

def _leader_frame(leader_id, data):
    return f"#B00{leader_id}{len(data):02d}{data}"


def test_parse_leader_broadcast_ok():
    """A well-formed #B00 frame yields (leader_id, lat, lon, dist)."""
    frame = _leader_frame('1', '1.0,2.0,3.0')
    assert parse_leader_broadcast(frame) == ('1', 1.0, 2.0, 3.0)


def test_parse_leader_broadcast_wrong_prefix():
    """Non-#B00 lines are rejected."""
    assert parse_leader_broadcast('#B007foo') is None
    assert parse_leader_broadcast('hello') is None


def test_parse_leader_broadcast_length_mismatch():
    """A length field longer than the payload is rejected."""
    assert parse_leader_broadcast('#B00199,18') is None


def test_parse_leader_broadcast_wrong_field_count():
    """A payload that is not exactly three values is rejected."""
    frame = _leader_frame('1', '1.0,2.0')
    assert parse_leader_broadcast(frame) is None


def test_parse_leader_broadcast_non_numeric():
    """Non-numeric fields are rejected."""
    frame = _leader_frame('2', 'a,b,c')
    assert parse_leader_broadcast(frame) is None
