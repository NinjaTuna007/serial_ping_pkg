"""Signal-safe spin helper for the plain (non-wire) nodes.

rclpy's default signal handlers shut the context down the instant a signal
arrives, and ``ros2 launch`` escalates with repeated SIGINTs while a process is
tearing down -- both of which surface as an ugly ``KeyboardInterrupt`` traceback
mid-shutdown. ``run_simple_node`` instead disables rclpy's signal handling and
drives a manual spin loop with our own handler that merely sets a stop flag, so
the first signal cleanly exits the loop and any escalating repeats are no-ops.

Nodes that own the Teensy link use the richer ``owtt_base.run_node`` /
``spin_wire_safe`` (which additionally guarantee wire-mode-on-exit); everything
else uses this.
"""

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions


def run_simple_node(node_factory, args=None):
    """Init rclpy, construct + spin the node, and tear down cleanly on a signal."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    stop = {'stop': False}

    def _handler(signum, frame):
        stop['stop'] = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _handler)
        except (ValueError, OSError):
            # Not on the main thread; rare for a node entry point.
            pass

    try:
        node = node_factory()
        while rclpy.ok() and not stop['stop']:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()
