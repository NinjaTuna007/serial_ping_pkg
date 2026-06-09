"""Shared base for the OWTT nodes.

Provides a node base class that *guarantees* the Teensy is returned to WIRE mode
(``$YW``) whenever the process goes away -- clean shutdown, Ctrl-C, an unhandled
exception/crash, or a SIGTERM. That way the Teensy is always left as a tame,
transparent wire, ready for the next run.

The guard is installed in three layers so it survives every exit path we can
intercept:

  * ``run_node`` wraps spin in try/except/finally,
  * an ``atexit`` hook fires on normal interpreter exit (incl. crashes),
  * a SIGTERM handler raises ``KeyboardInterrupt`` so the finally block runs.

Only an uncatchable ``SIGKILL`` / power loss can bypass this -- nothing in user
space can.
"""

import atexit
import signal

import rclpy
from rclpy.node import Node

from serial_ping_pkg.tuper_owtt import teensy_interface as ti


class WireSafeSerialNode(Node):
    """Base node owning the Teensy serial link with a wire-mode shutdown guard."""

    def __init__(self, name):
        super().__init__(name)
        self.ser = None
        self.command_terminator = '\r\n'
        # This node's modem address; used in the $Y config command. Subclasses
        # overwrite it from their parameters before installing the guard.
        self.own_modem_id = '001'
        self._went_wire = False
        self._guard_installed = False

    def send_command(self, cmd):
        """Write a command to the Teensy with the configured terminator."""
        self.get_logger().info(f"-> Teensy: {cmd!r}")
        self.ser.write((cmd + self.command_terminator).encode())

    def install_shutdown_guard(self):
        """Register atexit + SIGTERM hooks. Call once, after the serial is open."""
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
        """Send ``$YW`` and close the port. Idempotent and exception-safe."""
        if self._went_wire:
            return
        self._went_wire = True
        ser = self.ser
        if ser is None:
            return
        terminator = getattr(self, 'command_terminator', '\r\n')
        try:
            if ser.is_open:
                cmd = ti.build_config_command(ti.TeensyMode.WIRE, self.own_modem_id)
                ser.write((cmd + terminator).encode())
                ser.flush()
                try:
                    self.get_logger().info("Reset Teensy to WIRE mode on shutdown.")
                except Exception:
                    pass
        except Exception:
            pass
        finally:
            try:
                ser.close()
            except Exception:
                pass


def run_node(node_factory, args=None):
    """Init rclpy, spin the node, and always leave the Teensy in WIRE mode."""
    rclpy.init(args=args)
    node = None
    try:
        node = node_factory()
        if getattr(node, 'ser', None) is not None:
            rclpy.spin(node)
    except KeyboardInterrupt:
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
