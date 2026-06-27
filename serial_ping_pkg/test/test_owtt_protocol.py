"""Tests for the tuper_owtt Teensy serial protocol.

Two layers:

* Pure-logic checks of the command builders / parsers in ``teensy_interface``
  (fast, no ROS, no hardware).
* pty-based *full-stack* integration tests: the nodes no longer open a serial
  port, so these launch the ``succorfish_driver`` node bound to a pseudo-serial
  port AND the owtt node, and verify the wire-mode-on-exit guarantee and the
  leader's ``#A`` config gating travel correctly over the driver's ROS interface
  (TX/RX/SendCommand). They are skipped automatically if the packages are not
  built/sourced.
"""

import signal
import time

import pytest

from serial_ping_pkg.tuper_owtt import teensy_interface as ti

from full_stack_harness import (
    DRIVER_AVAILABLE,
    Stack,
    exe,
    teensy_responder,
    wait_until,
)


# --------------------------------------------------------------------------- #
# Pure protocol logic                                                         #
# --------------------------------------------------------------------------- #

def test_config_command_wire():
    """Wire mode is ``$Y<own_id>W``."""
    assert ti.build_config_command(ti.TeensyMode.WIRE, '007') == '$Y007W'


def test_config_command_receiver():
    """Receiver mode is ``$Y<own_id>R``."""
    assert ti.build_config_command(ti.TeensyMode.RECEIVER, '101') == '$Y101R'


def test_config_command_transmitter_first():
    """Transmitter that goes first listens for 000."""
    cmd = ti.build_config_command(ti.TeensyMode.TRANSMITTER, '042', '000', 1)
    assert cmd == '$Y042T0001s'


def test_config_command_transmitter_wait():
    """Transmitter that waits for modem 007 with a 4-epoch cadence."""
    cmd = ti.build_config_command(ti.TeensyMode.TRANSMITTER, '042', '007', 4)
    assert cmd == '$Y042T0074s'


def test_config_command_zero_pads_ids():
    """Numeric ids are zero-padded to three digits."""
    cmd = ti.build_config_command(ti.TeensyMode.TRANSMITTER, 7, 7, 4)
    assert cmd == '$Y007T0074s'


def test_gps_command():
    """GPS update is ``$G<lat>,<lon>``."""
    assert ti.build_gps_command('59.1', '18.8') == '$G59.1,18.8'


def test_parse_broadcast_latlon():
    """A well-formed ``#B`` frame yields (modem_id, lat, lon)."""
    data = '59.12345678,18.87654321'
    frame = '#B007%02d%s' % (len(data), data)
    assert ti.parse_broadcast(frame) == ('007', 59.12345678, 18.87654321)


def test_parse_broadcast_bad_length():
    """A length field that does not match the payload is rejected."""
    assert ti.parse_broadcast('#B00799,18') is None


def test_parse_broadcast_non_broadcast():
    """Non-``#B`` lines are not treated as broadcasts."""
    assert ti.parse_broadcast('#I123') is None


def test_parse_owtt_delta():
    """An ``#I`` line yields the delta in microseconds."""
    assert ti.parse_owtt_delta('#I123456') == 123456.0


def test_parse_owtt_delta_wrong_prefix():
    """A non-``#I`` line is not parsed as a delta."""
    assert ti.parse_owtt_delta('#B123') is None


def test_delta_to_range():
    """Range is (delta - offset) * 1e-6 * c."""
    assert ti.delta_to_range_m(123456, 6456, 1500.0) == pytest.approx(175.5)


# --------------------------------------------------------------------------- #
# pty-based integration tests                                                 #
# --------------------------------------------------------------------------- #

_skip_owtt = pytest.mark.skipif(
    not DRIVER_AVAILABLE or exe('owtt_follower_node') is None,
    reason="serial_ping_pkg / succorfish_driver not built/sourced")


@_skip_owtt
@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM])
def test_wire_mode_reset_on_exit(sig):
    """A graceful shutdown leaves the Teensy in wire mode ($Y<id>W) on the wire.

    Full-stack: signal the follower, then tear the whole stack down. The Teensy
    must end up in wire mode by the time everything is gone. Two mechanisms can
    deliver it -- the node's best-effort publish on its way out, and (the
    race-free backstop) the driver replaying the follower's registered shutdown
    command when the driver itself exits. We assert the *end state* (wire mode
    reached the pty), not which path produced it.
    """
    st = Stack(profile='teensy').start_driver()
    st.start_node(exe('owtt_follower_node'))
    assert st.fake.wait_for_command(lambda c: c.endswith('R'), timeout=6.0), \
        f"never saw receiver config; got {st.fake.commands()!r}"
    st.node.send_signal(sig)
    node_out, driver_out = st.stop()  # stops node, then driver (replays $Y<id>W)
    saw_wire = any(c.endswith('W') for c in st.fake.commands())
    assert saw_wire, (
        f"no wire-mode reset after shutdown; got {st.fake.commands()!r}\n"
        f"--- node ---\n{node_out}\n--- driver ---\n{driver_out}")


@_skip_owtt
def test_wire_mode_reset_when_driver_exits():
    """The driver replays the node's registered ``$Y<id>W`` on its OWN exit.

    The follower registers wire-mode as the driver's shutdown command at startup.
    Here we kill the *driver* (the node stays up) and assert the wire-mode command
    still reaches the pty -- the race-free guarantee, since the driver is the last
    holder of the open port.
    """
    st = Stack(profile='teensy').start_driver()
    st.start_node(exe('owtt_follower_node'))
    assert st.fake.wait_for_command(lambda c: c.endswith('R'), timeout=6.0)
    st.driver.send_signal(signal.SIGINT)  # kill the driver, not the node
    saw_wire = st.fake.wait_for_command(lambda c: c.endswith('W'), timeout=6.0)
    st.stop()
    assert saw_wire, f"driver did not write wire mode on exit; got {st.fake.commands()!r}"


@_skip_owtt
def test_owtt_follower_publishes_position_and_range():
    """Receiver mechanics: a Teensy ``#B``/``#I`` pair -> ROS position + range.

    Inject the leader's relayed broadcast and the OWTT delta the Teensy emits;
    the follower must publish the decoded leader position and the converted range
    (delta_us * 1e-6 * c with the default zero offset).
    """
    from geographic_msgs.msg import GeoPoint
    from std_msgs.msg import Float32

    st = Stack(responder=teensy_responder, profile='teensy').start_driver()
    st.start_probe()
    positions = st.probe.collect(GeoPoint, '/leader1/smarc/latlon')
    ranges = st.probe.collect(Float32, '/leader1/distance')
    st.start_node(exe('owtt_follower_node'),
                  ['-p', 'follower.leader1_modem_id:=7', '-p', 'owtt.offset_us:=0.0'])
    assert st.fake.wait_for_command(lambda c: c.endswith('R'), timeout=6.0)
    time.sleep(0.5)
    data = '59.10000000,18.80000000'
    st.fake.inject(f'#B007{len(data):02d}{data}')  # leader1 routed by modem id 007
    st.fake.inject('#I150000')                      # 150000 us * 1e-6 * 1500 = 225 m
    ok = wait_until(lambda: positions and ranges, timeout=8.0)
    node_out, _ = st.stop()
    assert ok, f"no position/range published; out:\n{node_out}"
    assert positions[0].latitude == pytest.approx(59.1)
    assert positions[0].longitude == pytest.approx(18.8)
    assert ranges[0].data == pytest.approx(225.0, rel=1e-3)


@pytest.mark.skipif(not DRIVER_AVAILABLE or exe('owtt_leader_node') is None,
                    reason="serial_ping_pkg / succorfish_driver not built/sourced")
def test_leader_waits_for_ack():
    """The leader withholds $G until it sees #A<own_id>, then proceeds.

    Full-stack: the leader's config goes leader -> driver -> pty, and the ack we
    inject on the pty flows pty -> driver RX -> leader.
    """
    st = Stack(profile='teensy').start_driver()
    st.start_node(
        exe('owtt_leader_node'),
        ['-p', "teensy.own_modem_id:='042'",
         '-p', 'teensy.broadcast_interval_s:=4',
         '-p', 'leader.send_period_s:=0.5'])
    assert st.fake.wait_for_command('$Y042T0004s', timeout=6.0), \
        f"never saw transmitter config; got {st.fake.commands()!r}"
    # Before the ack, the leader must NOT have started sending GPS ($G) frames.
    time.sleep(1.5)
    before_ack = st.fake.commands()
    st.fake.inject('#A042')
    time.sleep(1.5)  # let the leader process the ack and start
    out, _ = st.stop()
    assert not any(c.startswith('$G') for c in before_ack), \
        f"leader sent $G before the ack: {before_ack!r}"
    assert 'Teensy config confirmed: #A042' in out
