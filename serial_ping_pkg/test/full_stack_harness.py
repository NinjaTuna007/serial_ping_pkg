"""Shared full-stack pty harness for the serial_ping_pkg node tests.

The nodes no longer open a serial port -- the ``succorfish_driver`` owns it and
everyone else talks to it over ROS. So a faithful integration test wires up the
*whole* path:

    fake device  <--pty-->  succorfish_driver  <--ROS-->  serial_ping_pkg node
    (this file)             (subprocess)                  (subprocess)

* ``FakeSerialPort`` is a pseudo-terminal that emulates the thing on the far end
  of the driver. The driver opens the pty *slave*; this owns the *master*. A
  background reader records every command the driver writes (so a test can make
  TX assertions) and, given a ``responder``, writes protocol replies back. Tests
  can also ``inject`` unsolicited lines (a broadcast, an ``#I`` delta, ...).
* ``ResponderS`` are tiny pure functions mapping an inbound command line to the
  reply line(s) the fake device should emit -- one models the **Succorfish**
  modem (``$P`` -> ``#R...T...``), the other the **Teensy** OWTT front-end
  (``$Y...`` -> ``#A...``). This is the "vanilla USB modem" vs "Teensy mechanics"
  split the deployment actually has.
* ``RosProbe`` joins the same ROS graph from the test process so a test can
  subscribe to a node's outputs and publish its inputs.

Everything is skipped automatically (``DRIVER_AVAILABLE``) when the packages are
not built/sourced, so the pure-logic tests in the same files still run.
"""

import os
import select
import shutil
import signal
import subprocess
import tempfile
import threading
import time

try:
    from ament_index_python.packages import get_package_prefix
except Exception:  # pragma: no cover - ament missing
    get_package_prefix = None


# --------------------------------------------------------------------------- #
# Executable discovery                                                        #
# --------------------------------------------------------------------------- #

def exe(name, package='serial_ping_pkg'):
    """Return the installed path of a package executable, or ``None``."""
    if get_package_prefix is None:
        return None
    try:
        path = os.path.join(get_package_prefix(package), 'lib', package, name)
    except Exception:
        return None
    return path if os.path.exists(path) else None


DRIVER_EXE = exe('succorfish_driver_node', package='succorfish_driver')
SOCAT = shutil.which('socat')
# The fake serial port is a socat-linked pty pair (see ``FakeSerialPort``), so
# the full-stack tests need both the built driver *and* socat on PATH.
DRIVER_AVAILABLE = DRIVER_EXE is not None and SOCAT is not None


def _baud_for(profile):
    """The two bundled hardware profiles: Teensy is 115200, Succorfish 9600."""
    return 115200 if profile == 'teensy' else 9600


# --------------------------------------------------------------------------- #
# Generic waiting                                                             #
# --------------------------------------------------------------------------- #

def wait_until(predicate, timeout=10.0, period=0.05):
    """Poll ``predicate`` until it is truthy or ``timeout`` elapses."""
    end = time.time() + timeout
    while time.time() < end:
        if predicate():
            return True
        time.sleep(period)
    return bool(predicate())


# --------------------------------------------------------------------------- #
# Protocol responders (the fake device's "brain")                            #
# --------------------------------------------------------------------------- #

def succorfish_modem_responder(line):
    """Emulate a Succorfish modem: answer a ``$P<id>`` ping with ``#R<id>T<ticks>``.

    32000 ticks * c (1500) * 3.125e-5 == 1500 m, a convenient round range.
    """
    line = line.strip()
    if line.startswith('$P') and len(line) >= 5:
        modem_id = line[2:5]
        return [f'#R{modem_id}T32000']
    return None


def teensy_responder(line):
    """Emulate a Teensy: ACK any ``$Y<id><mode>`` config with ``#A<id>``.

    The Teensy sets the modem address and echoes the modem's ``#A<id>`` back to
    the host; the OWTT nodes gate on exactly that line.
    """
    line = line.strip()
    if line.startswith('$Y') and len(line) >= 6:
        own_id = line[2:5]
        return [f'#A{own_id}']
    return None


# --------------------------------------------------------------------------- #
# Fake serial port (socat-linked pty pair)                                    #
# --------------------------------------------------------------------------- #

class FakeSerialPort:
    """A virtual serial line: one end the driver opens, the other this drives.

    Backed by a ``socat`` pair of linked ptys rather than a bare
    ``pty.openpty()``. A bare pty read through pyserial 3.5 intermittently trips
    ``device reports readiness to read but returned no data``, which the driver
    treats as a disconnect and reconnects in a loop -- dropping commands and
    making the tests flap. socat relays bytes between two real ptys and keeps
    both masters open, so the device the driver opens behaves like an actual
    modem tty.
    """

    def __init__(self, responder=None):
        self._tmp = tempfile.mkdtemp(prefix='fakeserial_')
        # The driver opens ``slave_name``; the harness owns the other end.
        self.slave_name = os.path.join(self._tmp, 'driver')
        self._host_name = os.path.join(self._tmp, 'host')
        self._socat = _track(subprocess.Popen(
            [SOCAT,
             f'pty,raw,echo=0,link={self.slave_name}',
             f'pty,raw,echo=0,link={self._host_name}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        end = time.time() + 5.0
        while time.time() < end and not (
                os.path.exists(self.slave_name) and os.path.exists(self._host_name)):
            if self._socat.poll() is not None:
                raise RuntimeError('socat exited before creating the pty pair')
            time.sleep(0.02)
        if not (os.path.exists(self.slave_name) and os.path.exists(self._host_name)):
            raise RuntimeError('socat did not create the pty links in time')
        self._fd = os.open(self._host_name, os.O_RDWR | os.O_NOCTTY)
        self._responder = responder
        self._commands = []        # complete lines the driver wrote to the wire
        self._raw = bytearray()    # everything received, undecoded
        self._buf = b''
        self._lock = threading.Lock()
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self):
        while self._running:
            try:
                ready, _, _ = select.select([self._fd], [], [], 0.1)
            except (OSError, ValueError):
                break
            if not ready:
                continue
            try:
                chunk = os.read(self._fd, 4096)
            except OSError:
                break
            if not chunk:
                continue
            replies = []
            with self._lock:
                self._raw.extend(chunk)
                self._buf += chunk
                while b'\n' in self._buf:
                    raw_line, self._buf = self._buf.split(b'\n', 1)
                    line = raw_line.decode(errors='ignore').rstrip('\r')
                    if line == '':
                        continue
                    self._commands.append(line)
                    if self._responder is not None:
                        try:
                            replies.extend(self._responder(line) or [])
                        except Exception:
                            pass
            for reply in replies:
                self.write_line(reply)

    def write_line(self, line):
        """Push a single line to the driver (the harness adds CRLF)."""
        try:
            os.write(self._fd, (line + '\r\n').encode())
        except OSError:
            pass

    # A test "injects" an unsolicited device line (broadcast, #I delta, ...).
    inject = write_line

    def raw_text(self):
        with self._lock:
            return bytes(self._raw).decode(errors='ignore')

    def commands(self):
        with self._lock:
            return list(self._commands)

    def wait_for_command(self, predicate, timeout=8.0):
        """Wait until any received command satisfies ``predicate`` (str or fn)."""
        if isinstance(predicate, str):
            sub = predicate
            def predicate(cmd, _sub=sub):  # noqa: E306
                return _sub in cmd
        return wait_until(
            lambda: any(predicate(c) for c in self.commands()), timeout=timeout)

    def close(self):
        self._running = False
        try:
            self._reader.join(timeout=1.0)
        except Exception:
            pass
        try:
            os.close(self._fd)
        except OSError:
            pass
        if self._socat is not None:
            try:
                self._socat.terminate()
                self._socat.wait(timeout=2.0)
            except Exception:
                try:
                    self._socat.kill()
                except Exception:
                    pass
        shutil.rmtree(self._tmp, ignore_errors=True)


# --------------------------------------------------------------------------- #
# Process spawning                                                            #
# --------------------------------------------------------------------------- #

# Every spawned subprocess is tracked here so a test that fails *before* it can
# call ``Stack.stop()`` does not leak an orphaned driver/node (those keep running
# and pollute the ROS graph + load the machine). ``reap_spawned`` -- wired to an
# autouse fixture in conftest.py -- guarantees teardown after each test.
_SPAWNED = []


def _track(proc):
    _SPAWNED.append(proc)
    return proc


def reap_spawned():
    """Force-kill any spawned subprocess still alive (safety net for failures)."""
    for proc in _SPAWNED:
        if proc.poll() is None:
            try:
                proc.kill()
            except Exception:
                pass
            try:
                proc.wait(timeout=3.0)
            except Exception:
                pass
    _SPAWNED.clear()


def spawn_driver(slave, profile='succorfish', reconnect_delay_s=0.5, env=None):
    """Launch the driver bound to the pty slave at the given hardware profile."""
    return _track(subprocess.Popen(
        [DRIVER_EXE, '--ros-args',
         '-p', 'serial.port:=' + slave,
         '-p', 'serial.port_fallback:=' + slave,
         '-p', f'serial.baudrate:={_baud_for(profile)}',
         '-p', f'reconnect_delay_s:={reconnect_delay_s}'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env))


def spawn_node(exe_path, extra_args=None, env=None):
    """Launch a serial_ping_pkg node (it discovers the driver over ROS)."""
    return _track(subprocess.Popen(
        [exe_path, '--ros-args'] + (extra_args or []),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env))


def stop(proc, sig=signal.SIGINT, timeout=8):
    """Signal a process and return its merged stdout/stderr."""
    if proc is None:
        return ''
    proc.send_signal(sig)
    try:
        return proc.communicate(timeout=timeout)[0]
    except subprocess.TimeoutExpired:
        proc.kill()
        return proc.communicate()[0]


# --------------------------------------------------------------------------- #
# In-process ROS probe                                                        #
# --------------------------------------------------------------------------- #

_PROBE_SEQ = [0]


class RosProbe:
    """A spun-up rclpy node in the test process to observe/drive node topics."""

    def __init__(self, name=None):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        if name is None:
            _PROBE_SEQ[0] += 1
            name = f'full_stack_probe_{_PROBE_SEQ[0]}'
        self._rclpy = rclpy
        self._owns_init = not rclpy.ok()
        if self._owns_init:
            rclpy.init()
        self.node = Node(name)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        self._spin = threading.Thread(target=self._exec.spin, daemon=True)
        self._spin.start()

    def collect(self, msg_type, topic, depth=10):
        """Subscribe to ``topic`` and return a growing list of received msgs."""
        received = []
        self.node.create_subscription(
            msg_type, topic, lambda m: received.append(m), depth)
        return received

    def publisher(self, msg_type, topic, depth=10):
        return self.node.create_publisher(msg_type, topic, depth)

    def shutdown(self):
        try:
            self._exec.shutdown()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if self._owns_init and self._rclpy.ok():
            try:
                self._rclpy.shutdown()
            except Exception:
                pass


# --------------------------------------------------------------------------- #
# Stack lifecycle                                                             #
# --------------------------------------------------------------------------- #

# Each Stack runs on its own ROS_DOMAIN_ID so concurrent (or back-to-back) tests
# never share the relative ``/succorfish/*`` topics or the ``/smarc_modem_ping``
# action -- without this, a lingering driver from one test can swallow the next
# test's commands. Cycle through a small band of domain ids per process.
_DOMAIN_SEQ = [0]
_DOMAIN_BASE = 71
_DOMAIN_SPAN = 25


def _next_domain():
    _DOMAIN_SEQ[0] += 1
    return _DOMAIN_BASE + (_DOMAIN_SEQ[0] % _DOMAIN_SPAN)


class Stack:
    """Owns the fake port + driver + node (+ optional ROS probe) for one test.

    The whole stack is pinned to a private ``ROS_DOMAIN_ID`` for isolation.
    """

    def __init__(self, responder=None, profile='succorfish'):
        self.profile = profile
        self.fake = FakeSerialPort(responder=responder)
        self.driver = None
        self.node = None
        self.probe = None
        self.domain = _next_domain()
        self._env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain))
        # The in-process probe reads ROS_DOMAIN_ID at rclpy.init time, so the
        # process env must match the subprocesses; saved/restored around the run.
        self._saved_domain = os.environ.get('ROS_DOMAIN_ID')
        os.environ['ROS_DOMAIN_ID'] = str(self.domain)

    def start_driver(self, warmup=1.5):
        self.driver = spawn_driver(
            self.fake.slave_name, profile=self.profile, env=self._env)
        time.sleep(warmup)  # let the driver open the port + become discoverable
        return self

    def start_probe(self):
        self.probe = RosProbe()
        return self

    def start_node(self, exe_path, args=None):
        self.node = spawn_node(exe_path, args, env=self._env)
        return self

    def stop(self):
        """Tear everything down; return ``(node_output, driver_output)``."""
        node_out = stop(self.node)
        driver_out = stop(self.driver)
        if self.probe is not None:
            self.probe.shutdown()
        self.fake.close()
        if self._saved_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = self._saved_domain
        return node_out, driver_out
