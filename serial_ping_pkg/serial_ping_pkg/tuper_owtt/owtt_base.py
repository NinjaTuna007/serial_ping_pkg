"""Shared base for the OWTT nodes.

These nodes no longer own a serial port: the ``succorfish_driver`` node owns the
Teensy/modem link, and every command/response travels over its ROS interface
(see ``serial_ping_pkg.common.driver_client.DriverClient``). This base wires that
client in and *guarantees* the Teensy is returned to WIRE mode (``$YW``) whenever
the process goes away -- clean shutdown, Ctrl-C, an unhandled exception/crash, or
a SIGTERM -- by publishing the wire-mode command to the driver before exit.

The guard is installed in three layers so it survives every exit path we can
intercept:

  * ``run_node`` wraps spin in try/except/finally,
  * an ``atexit`` hook fires on normal interpreter exit (incl. crashes),
  * a SIGTERM handler raises ``KeyboardInterrupt`` so the finally block runs.

Because the driver owns the port persistently, the wire-mode command still
reaches the Teensy as long as the driver stays up; we give DDS a brief moment to
flush it before tearing the node down. Only an uncatchable ``SIGKILL`` / power
loss (or the driver being down) can bypass this.
"""

import atexit
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions

from serial_ping_pkg.common.driver_client import DriverClient
from serial_ping_pkg.tuper_owtt import teensy_interface as ti


class WireSafeSerialNode(Node):
    """Base node talking to the Teensy via the driver, with a wire-mode guard."""

    def __init__(self, name):
        super().__init__(name)
        self.driver = None
        self.command_terminator = '\r\n'
        # This node's modem address; used in the $Y config command. Subclasses
        # overwrite it from their parameters before installing the guard.
        self.own_modem_id = '001'
        self._went_wire = False
        self._guard_installed = False

    def connect_driver(self, on_line=None, on_status=None, callback_group=None,
                       wait_timeout=None):
        """Create the ``DriverClient``. Call once, after params are read.

        ``on_line`` (if given) receives every inbound serial line; write-only
        nodes leave it ``None``.

        These nodes send a critical ``$Y`` config command at startup, which would
        be lost if the driver's subscription is not yet matched. When
        ``wait_timeout`` is given, block (up to that many seconds) for the
        driver's ``SendCommand`` service so topic discovery has time to settle;
        if it never appears we warn and carry on (the node still runs).
        """
        self.driver = DriverClient(
            self, on_line=on_line, on_status=on_status, callback_group=callback_group)
        # Register wire-mode as the driver's on-exit command, so the Teensy is
        # returned to WIRE even if the driver outlives us or we are torn down
        # together (the driver is the guaranteed last holder of the open port).
        try:
            self.driver.register_shutdown_command(
                ti.build_config_command(ti.TeensyMode.WIRE, self.own_modem_id))
        except Exception:
            pass
        if wait_timeout is not None:
            if self.driver.wait_for_driver(timeout_sec=wait_timeout):
                # Service is up; give pub/sub matching a brief moment to settle
                # so the first config command is not dropped.
                time.sleep(0.3)
            else:
                self.get_logger().warn(
                    "succorfish_driver not detected yet; startup commands may be "
                    "dropped until it comes up.")
        return self.driver

    def send_command(self, cmd):
        """Publish a command to the driver (it appends the terminator)."""
        self.get_logger().info(f"-> Teensy: {cmd!r}")
        if self.driver is not None:
            self.driver.write(cmd)

    def install_shutdown_guard(self):
        """Register atexit + SIGTERM hooks. Call once, after connect_driver."""
        if self._guard_installed:
            return
        self._guard_installed = True
        atexit.register(self.to_wire_mode)
        try:
            signal.signal(signal.SIGTERM, self._on_sigterm)
        except (ValueError, OSError):
            # Not running in the main thread; run_node's finally still covers us.
            pass

    def _on_sigterm(self, signum, frame):
        raise KeyboardInterrupt()

    def to_wire_mode(self):
        """Ask the driver to put the Teensy in ``$YW``. Idempotent, exception-safe."""
        if self._went_wire:
            return
        self._went_wire = True
        driver = getattr(self, 'driver', None)
        if driver is None:
            return
        try:
            cmd = ti.build_config_command(ti.TeensyMode.WIRE, self.own_modem_id)
            driver.write(cmd)
            # Give DDS a window to flush the command to the driver before the
            # node (and its publisher) are torn down. This publish is best-effort:
            # once we return main() destroys the node, so anything not yet
            # delivered is lost. The race-free guarantee is the driver replaying
            # its registered shutdown command when *it* exits (it is the last
            # holder of the open port), so we keep this window short.
            if rclpy.ok():
                deadline = time.time() + 0.5
                while time.time() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.05)
            try:
                self.get_logger().info("Requested Teensy WIRE mode on shutdown.")
            except Exception:
                pass
        except Exception:
            pass


def spin_wire_safe(node, executor=None):
    """Spin until SIGINT/SIGTERM, leaving the rclpy context alive on return.

    The default rclpy signal handlers shut the context down the instant a signal
    arrives, which would make the wire-mode command published from
    ``to_wire_mode`` un-deliverable (the publisher and DDS are already torn
    down). Instead we install our own handlers that merely set a stop flag and
    drive the executor manually, so the caller can still publish + flush the
    wire-mode command *before* shutting rclpy down.

    Pass ``executor`` to drive a custom (e.g. ``MultiThreadedExecutor``) spin;
    otherwise the node is spun directly.

    Requires that ``rclpy.init`` was called with
    ``signal_handler_options=SignalHandlerOptions.NO``.
    """
    stop = {'stop': False}

    def _handler(signum, frame):
        stop['stop'] = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _handler)
        except (ValueError, OSError):
            # Not on the main thread; the executor's own handling still applies.
            pass

    try:
        while rclpy.ok() and not stop['stop']:
            if executor is not None:
                executor.spin_once(timeout_sec=0.1)
            else:
                rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


def run_node(node_factory, args=None):
    """Init rclpy, spin the node, and always leave the Teensy in WIRE mode."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = node_factory()
        if getattr(node, 'driver', None) is not None:
            spin_wire_safe(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.to_wire_mode()
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()
