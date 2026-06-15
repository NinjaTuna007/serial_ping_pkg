"""One-shot serial command tool for the Teensy / Succorfish modem.

Fire a single command at the serial link without running any ROS node: the
tool opens the port, writes the command (with the ``\\r\\n`` terminator),
optionally reads back the reply, then closes the port and exits. Nothing keeps
running and the port is left free.

Examples (run via ``ros2 run serial_ping_pkg teensy_cmd``):

    # Set the Teensy to WIRE mode with modem address 101
    teensy_cmd --wire --id 101 --port /dev/ttyUSB0

    # Same thing, raw form
    teensy_cmd '$Y101W' --port /dev/ttyUSB0

    # Configure as receiver 101 / transmitter 042
    teensy_cmd --receiver --id 101
    teensy_cmd --transmitter --id 042 --listen 007 --epochs 4

    # Send an arbitrary command (ping, broadcast, classic Succorfish, ...)
    teensy_cmd '$P002'
    teensy_cmd '$B05HELLO'
"""

import argparse
import codecs
import sys
import time

from serial_ping_pkg.tuper_owtt import teensy_interface as ti
from serial_ping_pkg.utils import init_serial


class _PrintLogger:
    """Minimal logger shim so we can reuse init_serial without rclpy."""

    def info(self, msg):
        print(msg, file=sys.stderr)

    def error(self, msg):
        print(msg, file=sys.stderr)


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
        description="Send a single command to the Teensy/modem over serial and exit.")
    parser.add_argument('command', nargs='?',
                        help="Raw command string, e.g. '$Y101W', '$P002', '$B05HELLO'.")
    parser.add_argument('--port', default='/dev/ttyUSB0', help="Serial port (default %(default)s).")
    parser.add_argument('--port-fallback', default='/dev/ttyUSB1',
                        help="Fallback port if the primary fails (default %(default)s).")
    parser.add_argument('--baud', type=int, default=9600, help="Baud rate (default %(default)s).")
    parser.add_argument('--terminator', default='\r\n',
                        help="Line terminator appended to the command "
                             "(default CR LF; accepts escapes like '\\r\\n').")
    parser.add_argument('--no-terminator', action='store_true',
                        help="Send the command with no terminator appended.")
    parser.add_argument('--read', type=float, default=1.5, metavar='SECONDS',
                        help="Read and print the reply for this many seconds "
                             "(default %(default)s; 0 = don't read).")

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

    args = parser.parse_args(argv)

    command = _build_command(args)
    terminator = '' if args.no_terminator else codecs.decode(args.terminator, 'unicode_escape')

    ser = init_serial(args.port, args.port_fallback, args.baud, _PrintLogger())
    if ser is None:
        return 1

    try:
        payload = (command + terminator).encode()
        ser.write(payload)
        ser.flush()
        print(f"-> {command!r} (+{terminator!r})")

        if args.read > 0:
            deadline = time.time() + args.read
            chunks = []
            while time.time() < deadline:
                waiting = ser.in_waiting
                if waiting:
                    chunks.append(ser.read(waiting))
                else:
                    time.sleep(0.05)
            reply = b''.join(chunks)
            if reply:
                print(f"<- {reply.decode('utf-8', errors='replace')}", end='')
                if not reply.endswith(b'\n'):
                    print()
            else:
                print("<- (no reply)")
    finally:
        try:
            ser.close()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
