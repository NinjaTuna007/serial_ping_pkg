"""Pure-logic tests for the acoustic-relay position broadcast codec.

Exercises ``serial_ping_pkg.acoustic_relay.pos_protocol`` directly (no ROS, no
hardware): the outbound ``$B`` builder (with depth/heading clamping + field
widths), the inbound ``#B`` parser (2/3/4-field payloads and rejects), and an
encode -> parse round-trip. Mirrors the style of ``test_owtt_protocol.py``.
"""

import pytest

from serial_ping_pkg.acoustic_relay.pos_protocol import (
    build_pos_broadcast,
    parse_pos_broadcast,
)


# --------------------------------------------------------------------------- #
# Builder                                                                     #
# --------------------------------------------------------------------------- #

def test_build_basic_frame():
    """lat/lon at 8dp, depth as ,DDD.D, heading as ,DDD, with byte count."""
    frame = build_pos_broadcast(59.12345678, 18.87654321, 42.5, 270)
    assert frame == "$B3359.12345678,18.87654321,042.5,270"


def test_build_zero_pads_depth_and_heading():
    """Zero depth/heading still occupy the fixed field widths."""
    frame = build_pos_broadcast(1.0, 2.0, 0.0, 0)
    assert frame == "$B311.00000000,2.00000000,000.0,000"


def test_build_clamps_high_values():
    """Depth clamps to 999.9 and heading to 359."""
    frame = build_pos_broadcast(1.0, 2.0, 1500.0, 400)
    assert frame.endswith(",999.9,359")


def test_build_clamps_negative_values():
    """Negative depth/heading clamp to zero."""
    frame = build_pos_broadcast(1.0, 2.0, -5.0, -10)
    assert frame.endswith(",000.0,000")


# --------------------------------------------------------------------------- #
# Parser                                                                      #
# --------------------------------------------------------------------------- #

def _inbound(modem_id, data):
    return f"#B{modem_id}{len(data):02d}{data}"


def test_parse_four_fields():
    """A 4-field payload yields modem id, lat, lon, depth (float), heading (int)."""
    line = _inbound('007', '59.10000000,18.80000000,012.3,270')
    modem_id, lat, lon, depth, heading = parse_pos_broadcast(line)
    assert modem_id == '007'
    assert lat == pytest.approx(59.1)
    assert lon == pytest.approx(18.8)
    assert depth == pytest.approx(12.3)
    assert heading == 270


def test_parse_three_fields_depth_only():
    """A 3-field payload carries depth but no heading."""
    line = _inbound('111', '59.1,18.8,12.3')
    assert parse_pos_broadcast(line) == ('111', pytest.approx(59.1), pytest.approx(18.8),
                                         pytest.approx(12.3), None)


def test_parse_two_fields_latlon_only():
    """A 2-field payload carries neither depth nor heading."""
    line = _inbound('007', '59.1,18.8')
    assert parse_pos_broadcast(line) == ('007', pytest.approx(59.1), pytest.approx(18.8),
                                         None, None)


def test_parse_rejects_non_broadcast():
    """Lines that are not #B frames are rejected."""
    assert parse_pos_broadcast('hello') is None


def test_parse_rejects_length_mismatch():
    """A length field that does not match the payload is rejected."""
    assert parse_pos_broadcast('#B00733short') is None


def test_parse_rejects_non_numeric_latlon():
    """A non-numeric lat/lon is rejected."""
    line = _inbound('007', 'a,b')
    assert parse_pos_broadcast(line) is None


# --------------------------------------------------------------------------- #
# Round-trip                                                                  #
# --------------------------------------------------------------------------- #

def test_build_parse_round_trip():
    """The outbound payload survives a trip through the inbound parser.

    The modem prepends ``#B<modem_id>`` to the broadcast; the length + payload
    are identical, so swapping the ``$B`` prefix reconstructs the received line.
    """
    out = build_pos_broadcast(59.1, 18.8, 12.3, 270)        # "$B<n><payload>"
    inbound = "#B007" + out[2:]                              # "#B007<n><payload>"
    modem_id, lat, lon, depth, heading = parse_pos_broadcast(inbound)
    assert modem_id == '007'
    assert lat == pytest.approx(59.1)
    assert lon == pytest.approx(18.8)
    assert depth == pytest.approx(12.3)
    assert heading == 270
