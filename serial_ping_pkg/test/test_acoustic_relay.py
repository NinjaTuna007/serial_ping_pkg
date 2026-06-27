"""Pure-logic tests for the acoustic-relay position broadcast codec.

Exercises ``serial_ping_pkg.acoustic_relay.pos_protocol`` directly (no ROS, no
hardware): the outbound ``$B`` builder (with depth/heading clamping + field
widths), the inbound ``#B`` parser (2/3/4-field payloads and rejects), and an
encode -> parse round-trip. Mirrors the style of ``test_owtt_protocol.py``.
"""

import time

import pytest

from serial_ping_pkg.acoustic_relay.pos_protocol import (
    build_pos_broadcast,
    parse_pos_broadcast,
)

from full_stack_harness import DRIVER_AVAILABLE, Stack, exe, wait_until


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


# --------------------------------------------------------------------------- #
# Full-stack pty integration (Succorfish USB modem profile)                  #
#                                                                             #
# fake modem <--pty--> succorfish_driver <--ROS--> relay node. Exercises the  #
# real $B broadcast emit and #B receive path end to end.                      #
# --------------------------------------------------------------------------- #

_skip = pytest.mark.skipif(
    not DRIVER_AVAILABLE, reason="succorfish_driver not built/sourced")


@_skip
@pytest.mark.skipif(exe('smarc_pos_broadcast_node') is None, reason="node not built")
def test_smarc_pos_broadcast_node_emits_frame():
    """A SMaRC latlon update is broadcast to the modem as a ``$B`` position frame."""
    from geographic_msgs.msg import GeoPoint

    st = Stack(profile='succorfish').start_driver()
    st.start_probe()
    pub = st.probe.publisher(GeoPoint, '/lolo/smarc/latlon')
    st.start_node(exe('smarc_pos_broadcast_node'))
    time.sleep(1.5)  # let the latlon subscription match

    # The node rate-limits to one broadcast per ~3 s (and drops the first that
    # lands within 3 s of startup), so pump until a $B frame goes out.
    saw = False
    end = time.time() + 14
    while time.time() < end and not saw:
        msg = GeoPoint()
        msg.latitude = 59.1
        msg.longitude = 18.8
        pub.publish(msg)
        time.sleep(0.5)
        saw = any(c.startswith('$B') and '59.10000000' in c for c in st.fake.commands())
    node_out, _ = st.stop()
    assert saw, f"no $B position frame broadcast; got {st.fake.commands()!r}\n{node_out}"


@_skip
@pytest.mark.skipif(exe('smarc_pos_receiver_node') is None, reason="node not built")
def test_smarc_pos_receiver_node_publishes_position():
    """An inbound ``#B`` position frame is decoded onto the robot's latlon topic."""
    from geographic_msgs.msg import GeoPoint

    st = Stack(profile='succorfish').start_driver()
    st.start_probe()
    # Default robot table maps modem id 7 -> 'lolo' -> /relay_lolo/smarc/latlon.
    got = st.probe.collect(GeoPoint, '/relay_lolo/smarc/latlon')
    st.start_node(exe('smarc_pos_receiver_node'))
    time.sleep(1.5)
    data = '59.10000000,18.80000000,012.3,270'
    st.fake.inject(f'#B007{len(data):02d}{data}')
    ok = wait_until(lambda: len(got) > 0, timeout=8)
    node_out, _ = st.stop()
    assert ok, f"position not republished; out:\n{node_out}"
    assert got[0].latitude == pytest.approx(59.1)
    assert got[0].longitude == pytest.approx(18.8)
