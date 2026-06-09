"""Tests for the tuper_owtt Teensy serial protocol.

Two layers:

* Pure-logic checks of the command builders / parsers in ``teensy_interface``
  (fast, no ROS, no hardware).
* pty-based integration tests that launch the real nodes against a pseudo-serial
  port to verify the wire-mode-on-exit guarantee and the leader's ``#A`` config
  gating. These are skipped automatically if the package is not built/sourced.
"""

import os
import pty
import select
import signal
import subprocess
import time

import pytest

from serial_ping_pkg.tuper_owtt import teensy_interface as ti


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

def _exe(name):
    """Return the installed path of a package executable, or None."""
    try:
        from ament_index_python.packages import get_package_prefix
        path = os.path.join(
            get_package_prefix('serial_ping_pkg'), 'lib', 'serial_ping_pkg', name)
    except Exception:
        return None
    return path if os.path.exists(path) else None


def _drain(fd, seconds):
    """Read from ``fd`` for up to ``seconds``, returning whatever arrived."""
    data = b''
    end = time.time() + seconds
    while time.time() < end:
        ready, _, _ = select.select([fd], [], [], 0.1)
        if ready:
            try:
                data += os.read(fd, 4096)
            except OSError:
                break
    return data


def _spawn(exe, extra_args):
    """Launch a node bound to a fresh pty; return (proc, master_fd, slave_fd)."""
    master_fd, slave_fd = pty.openpty()
    slave = os.ttyname(slave_fd)
    proc = subprocess.Popen(
        [exe, '--ros-args',
         '-p', 'serial.port:=' + slave,
         '-p', 'serial.port_fallback:=' + slave] + extra_args,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    return proc, master_fd, slave_fd


def _shutdown(proc, master_fd, slave_fd):
    """Best-effort teardown of a spawned node + pty."""
    try:
        out = proc.communicate(timeout=5)[0]
    except subprocess.TimeoutExpired:
        proc.kill()
        out = proc.communicate()[0]
    os.close(slave_fd)
    os.close(master_fd)
    return out


@pytest.mark.skipif(_exe('owtt_follower_node') is None,
                    reason="serial_ping_pkg not built/sourced")
@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM])
def test_wire_mode_reset_on_exit(sig):
    """On any exit the follower returns the Teensy to wire mode ($Y<id>W)."""
    proc, master_fd, slave_fd = _spawn(_exe('owtt_follower_node'), [])
    startup = _drain(master_fd, 2.5).decode(errors='ignore')
    proc.send_signal(sig)
    shutdown = _drain(master_fd, 2.5).decode(errors='ignore')
    _shutdown(proc, master_fd, slave_fd)
    assert startup.strip().startswith('$Y') and startup.strip().endswith('R')
    assert shutdown.strip().endswith('W')


@pytest.mark.skipif(_exe('owtt_leader_node') is None,
                    reason="serial_ping_pkg not built/sourced")
def test_leader_waits_for_ack():
    """The leader withholds $G until it sees #A<own_id>, then proceeds."""
    proc, master_fd, slave_fd = _spawn(
        _exe('owtt_leader_node'),
        ['-p', "teensy.own_modem_id:='042'",
         '-p', 'teensy.broadcast_interval_s:=4',
         '-p', 'leader.send_period_s:=0.5'])
    startup = _drain(master_fd, 2.0).decode(errors='ignore')
    before_ack = _drain(master_fd, 1.5).decode(errors='ignore')
    os.write(master_fd, b'#A042\r\n')
    _drain(master_fd, 1.0)
    out = _shutdown(proc, master_fd, slave_fd)
    assert startup.strip().startswith('$Y042T0004s')
    assert '$G' not in before_ack
    assert 'Teensy config confirmed: #A042' in out
