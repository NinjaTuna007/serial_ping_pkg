"""Pure-logic tests for the common ping/broadcast protocol helpers.

Exercises ``serial_ping_pkg.common.ping_protocol`` directly (no ROS, no
hardware): the Succorfish travel-time -> range conversion, the response
completeness gate, and the ``#B00`` leader-broadcast parser. Mirrors the style
of ``test_owtt_protocol.py``.
"""

import time

import pytest

from serial_ping_pkg.common.ping_protocol import (
    parse_leader_broadcast,
    parse_ping_distance,
    ping_response_complete,
    travel_time_to_distance,
)

from full_stack_harness import (
    DRIVER_AVAILABLE,
    Stack,
    exe,
    succorfish_modem_responder,
    wait_until,
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


# --------------------------------------------------------------------------- #
# Full-stack pty integration (Succorfish USB modem profile)                  #
#                                                                             #
# fake modem <--pty--> succorfish_driver <--ROS--> common node. Exercises the #
# real $P -> #R...T... ping exchange and the #B00 leader-broadcast receive    #
# path end to end (skipped if the packages are not built/sourced).            #
# --------------------------------------------------------------------------- #

_skip = pytest.mark.skipif(
    not DRIVER_AVAILABLE, reason="succorfish_driver not built/sourced")


@_skip
@pytest.mark.skipif(exe('serial_ping_node') is None, reason="node not built")
def test_serial_ping_node_publishes_distance():
    """serial_ping_node: $P ping over the driver -> T-reply -> leader distance."""
    from std_msgs.msg import Float32

    st = Stack(responder=succorfish_modem_responder, profile='succorfish').start_driver()
    st.start_probe()
    distances = st.probe.collect(Float32, '/leader1/distance')
    st.start_node(exe('serial_ping_node'))
    ok = wait_until(lambda: len(distances) > 0, timeout=20)
    node_out, _ = st.stop()
    assert ok, f"no distance published; out:\n{node_out}"
    # 32000 ticks * 1500 m/s * 3.125e-5 == 1500 m.
    assert distances[0].data == pytest.approx(1500.0, rel=1e-3)
    assert any(c.startswith('$P') for c in st.fake.commands()), \
        f"no ping command went out; got {st.fake.commands()!r}"


@_skip
@pytest.mark.skipif(exe('single_target_ping_node') is None, reason="node not built")
def test_single_target_ping_node_publishes_distance():
    """single_target_ping_node: the configured ping command yields a range."""
    from std_msgs.msg import Float32

    st = Stack(responder=succorfish_modem_responder, profile='succorfish').start_driver()
    st.start_probe()
    ranges = st.probe.collect(Float32, '/tgt/range')
    st.start_node(
        exe('single_target_ping_node'),
        ['-p', 'robot_name:=tgt',
         '-p', 'ping_command:=$P009',
         '-p', 'distance_topic_suffix:=range',
         '-p', 'timer_period:=0.5'])
    ok = wait_until(lambda: len(ranges) > 0, timeout=20)
    node_out, _ = st.stop()
    assert ok, f"no range published; out:\n{node_out}"
    assert ranges[0].data == pytest.approx(1500.0, rel=1e-3)
    assert any(c.strip() == '$P009' for c in st.fake.commands()), \
        f"configured ping command not sent; got {st.fake.commands()!r}"


@_skip
@pytest.mark.skipif(exe('serial_broadcast_receiver') is None, reason="node not built")
def test_serial_broadcast_receiver_republishes_leader():
    """serial_broadcast_receiver: a #B00 leader frame -> /leader<id>/gps + distance."""
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import Float32

    st = Stack(profile='succorfish').start_driver()
    st.start_probe()
    gps = st.probe.collect(NavSatFix, '/leader1/gps')
    dist = st.probe.collect(Float32, '/leader1/distance')
    st.start_node(exe('serial_broadcast_receiver'))
    time.sleep(1.5)  # let the RX subscription match the driver
    data = '59.1,18.8,123.0'                      # lat,lon,dist
    st.fake.inject(f'#B001{len(data):02d}{data}')  # leader id '1'
    ok = wait_until(lambda: gps and dist, timeout=8)
    node_out, _ = st.stop()
    assert ok, f"leader fix/range not republished; out:\n{node_out}"
    assert gps[0].latitude == pytest.approx(59.1)
    assert gps[0].longitude == pytest.approx(18.8)
    assert dist[0].data == pytest.approx(123.0)
