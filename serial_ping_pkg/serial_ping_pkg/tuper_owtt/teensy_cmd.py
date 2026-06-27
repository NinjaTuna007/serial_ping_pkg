"""One-shot command tool for the Teensy / Succorfish modem (via the driver).

Fire a single command at the modem without running a long-lived node. The
``succorfish_driver`` node owns the serial port, so this tool talks to it over
the ``SendCommand`` service: it writes the command (the driver appends the
configured terminator), optionally collects the reply for a few seconds, prints
it, and exits. It never opens a serial port itself, so it is safe to run while
the driver and other nodes are live.

The driver must be running. To target a specific driver (e.g. the Teensy
profile) launch this tool in the same namespace, or remap the service, e.g.::

    ros2 run serial_ping_pkg teensy_cmd '$P002' --ros-args -r __ns:=/teensy

Examples (run via ``ros2 run serial_ping_pkg teensy_cmd``):

    # Set the Teensy to WIRE mode with modem address 101
    teensy_cmd --wire --id 101

    # Same thing, raw form
    teensy_cmd '$Y101W'

    # Configure as receiver 101 / transmitter 042
    teensy_cmd --receiver --id 101
    teensy_cmd --transmitter --id 042 --listen 007 --epochs 4

    # Send an arbitrary command (ping, broadcast, classic Succorfish, ...)
    teensy_cmd '$P002'
    teensy_cmd '$B05HELLO'
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

from serial_ping_pkg.tuper_owtt import teensy_interface as ti
from serial_ping_pkg.common.driver_client import DriverClient


def _build_command(args):
    """Resolve the command string from the convenience flags or positional arg."""
    modes = [('wire', ti.TeensyMode.WIRE),
             ('receiver', ti.TeensyMode.RECEIVER),
             ('transmitter', ti.TeensyMode.TRANSMITTER)]
    selected = [m for name, m in modes if getattr(args, name)]
    if selected and args.command:
        raise SystemExit("Provide either a raw command or a mode flag, not both.")
    if len(selected) > 1:
        raise SystemExit("Pick only one of --wire / --receiver / --transmitter.")
    if selected:
        return ti.build_config_command(
            selected[0], args.id,
            listen_for_modem_id=args.listen,
            broadcast_interval_s=args.epochs)
    if not args.command:
        raise SystemExit("No command given. Pass a raw command or a mode flag "
                         "(--wire/--receiver/--transmitter). See --help.")
    return args.command


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog='teensy_cmd',
        description="Send a single command to the Teensy/modem via succorfish_driver and exit.")
    parser.add_argument('command', nargs='?',
                        help="Raw command string, e.g. '$Y101W', '$P002', '$B05HELLO'.")
    parser.add_argument('--no-terminator', action='store_true',
                        help="Ask the driver not to append its line terminator.")
    parser.add_argument('--read', type=float, default=1.5, metavar='SECONDS',
                        help="Collect and print reply lines for this many seconds "
                             "(default %(default)s; 0 = don't read).")
    parser.add_argument('--connect-timeout', type=float, default=5.0, metavar='SECONDS',
                        help="How long to wait for the driver service (default %(default)s).")

    grp = parser.add_argument_group('config-command builders (instead of a raw command)')
    grp.add_argument('--wire', action='store_true', help="Build $Y<id>W (wire / passthrough).")
    grp.add_argument('--receiver', action='store_true', help="Build $Y<id>R (receiver).")
    grp.add_argument('--transmitter', action='store_true',
                     help="Build $Y<id>T<listen><epochs>s (transmitter).")
    grp.add_argument('--id', default='101', help="Own modem id for the mode flags (default %(default)s).")
    grp.add_argument('--listen', default='000',
                     help="Transmitter: modem id to wait for, 000=go first (default %(default)s).")
    grp.add_argument('--epochs', type=int, default=1,
                     help="Transmitter: broadcast cadence in epochs 1-4 (default %(default)s).")

    # Split out ROS args (e.g. --ros-args -r __ns:=/teensy) so argparse ignores them.
    argv = sys.argv[1:] if argv is None else argv
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    args = parser.parse_args(argv)

    command = _build_command(args)

    rclpy.init()
    node = Node('teensy_cmd')
    client = DriverClient(node)
    try:
        if not client.wait_for_driver(timeout_sec=args.connect_timeout):
            print("succorfish_driver SendCommand service not available; is the driver running?",
                  file=sys.stderr)
            return 1

        future = client.request(
            command,
            expect_regex='',
            timeout=max(args.read, 0.0),
            append_terminator=not args.no_terminator,
        )
        print(f"-> {command!r}")
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.read + args.connect_timeout)

        if not future.done():
            print("<- (no response from driver)")
            return 1
        resp = future.result()
        if not resp.success and resp.message:
            print(f"!! {resp.message}", file=sys.stderr)
        if args.read > 0:
            if resp.lines:
                for line in resp.lines:
                    print(f"<- {line}")
            else:
                print("<- (no reply)")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
