"""Pure-logic tests for the tuper_twtt leader/follower wire format.

Exercises ``serial_ping_pkg.tuper_twtt.leader_protocol`` directly
(no ROS, no hardware): the leader's ``$B`` position+distance builder, the
follower's bare ``lat,lon,dist`` parser, and that the leader reuses the shared
travel-time conversion. Mirrors the style of ``test_owtt_protocol.py``.
"""

import time

import pytest

from serial_ping_pkg.common.ping_protocol import travel_time_to_distance
from serial_ping_pkg.tuper_twtt.leader_protocol import (
    build_position_broadcast,
    parse_position_distance,
)

from full_stack_harness import (
    DRIVER_AVAILABLE,
    Stack,
    exe,
    succorfish_modem_responder,
    wait_until,
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


# --------------------------------------------------------------------------- #
# Full-stack pty integration (Succorfish USB modem profile)                  #
#                                                                             #
# fake modem <--pty--> succorfish_driver <--ROS--> twtt node. The leader      #
# pings ($P -> #R...T...) then broadcasts its GPS+range ($B); the follower    #
# decodes the bare lat,lon,dist payload onto ROS.                             #
# --------------------------------------------------------------------------- #

_skip = pytest.mark.skipif(
    not DRIVER_AVAILABLE, reason="succorfish_driver not built/sourced")


@_skip
@pytest.mark.skipif(exe('twtt_leader_node') is None, reason="node not built")
def test_twtt_leader_pings_then_broadcasts_position():
    """The leader pings, converts the T-reply to a range, and broadcasts $B<lat,lon,dist>."""
    from geographic_msgs.msg import GeoPoint

    st = Stack(responder=succorfish_modem_responder, profile='succorfish').start_driver()
    st.start_probe()
    gps_pub = st.probe.publisher(GeoPoint, '/leader/smarc/latlon')
    st.start_node(
        exe('twtt_leader_node'),
        ['-p', 'ping_command:=$P111',
         '-p', 'timer_period:=0.5',
         '-p', 'sound_velocity:=1500.0',
         '-p', 'leader_gps_msg_type:=GeoPoint',
         '-p', 'leader_gps_topic:=/leader/smarc/latlon'])
    time.sleep(1.5)

    # Feed the leader its own GPS so it has a position to attach to the range,
    # and let it ping repeatedly until a position broadcast goes out.
    saw = False
    end = time.time() + 14
    while time.time() < end and not saw:
        msg = GeoPoint()
        msg.latitude = 59.1
        msg.longitude = 18.8
        gps_pub.publish(msg)
        time.sleep(0.5)
        saw = any(c.startswith('$B') and '59.1' in c for c in st.fake.commands())
    node_out, _ = st.stop()
    assert saw, f"leader never broadcast position+range; got {st.fake.commands()!r}\n{node_out}"
    # The broadcast carries the 1500 m range from the 32000-tick T-reply.
    assert any(c.startswith('$B') and '1500.0' in c for c in st.fake.commands())


@_skip
@pytest.mark.skipif(exe('twtt_follower_node') is None, reason="node not built")
def test_twtt_follower_publishes_position_and_range():
    """The follower decodes a bare ``lat,lon,dist`` payload onto its ROS topics."""
    from geographic_msgs.msg import GeoPoint
    from std_msgs.msg import Float32

    st = Stack(profile='succorfish').start_driver()
    st.start_probe()
    pos = st.probe.collect(GeoPoint, '/leader/smarc/latlon')
    dist = st.probe.collect(Float32, '/leader/distance')
    st.start_node(exe('twtt_follower_node'))
    time.sleep(1.5)
    st.fake.inject('59.1,18.8,123.0')
    ok = wait_until(lambda: pos and dist, timeout=8)
    node_out, _ = st.stop()
    assert ok, f"follower did not republish leader pos/range; out:\n{node_out}"
    assert pos[0].latitude == pytest.approx(59.1)
    assert pos[0].longitude == pytest.approx(18.8)
    assert dist[0].data == pytest.approx(123.0)
