"""Listen for acoustic commands, and transmit them.

Bringup starts this node once, in its own window, beside the driver.
It advertises ``acoustic_abort`` and ``acoustic_command`` in that
namespace. Ranging and beacon launches do not start a second copy.

Targeted ``ABORT`` publishes ``std_msgs/Empty`` on the SMARC abort topic and
answers with a DCCL ``OK``. Targeted ``ENOUGH`` publishes the WARAPS
``signal-unit`` ``$enough`` JSON on ``waraps/tst/command`` (the same string
the MQTT bridge delivers) and answers with ``OK``. It does not publish the
abort topic. The service waits for that ``OK`` and sends the command again,
with a new sequence, until it arrives. ``start_telemetry`` and
``stop_telemetry`` use the same wait. When ``handle_telemetry`` is set, an
addressed telemetry command is published on ``acoustic_telemetry`` for the
beacon and answered with ``OK``.

``ABORT_ALL`` is a panic shout. Vehicles that hear it abort and stay quiet.
The sender is the control center and does not abort itself. The service
repeats the shout and returns once the frames are on the air.

This node does not configure the Teensy and does not return it to WIRE mode.
The follower or beacon already owns that, and a second ``$Y`` would kick it
off the air.
"""

import json
import threading
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String

from serial_ping_interfaces.srv import AcousticAbort, AcousticCommand
from serial_ping_pkg.common.acoustic_command import (
    CODE_ABORT,
    CODE_ABORT_ALL,
    CODE_ENOUGH,
    CODE_NAMES,
    CODE_OK,
    CODE_START_TELEMETRY,
    CODE_STOP_TELEMETRY,
    CONFIRM_ALL,
    SeqMemory,
    abort_applies,
    ack_matches,
    beacon_verb,
    enough_applies,
    enough_tst_body,
    modem_int,
    pack_command,
    parse_command,
    parse_target,
    sender_allowed,
    unpack_command,
)
from serial_ping_pkg.common.driver_client import DriverClient
from serial_ping_pkg.tuper_owtt import teensy_interface as ti


class AcousticCmdNode(Node):
    def __init__(self):
        super().__init__('acoustic_cmd_node')
        id_desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('own_modem_id', '', id_desc)
        self.declare_parameter('abort_topic', 'smarc/abort')
        self.declare_parameter('enough_topic', 'waraps/tst/command')
        self.declare_parameter('allowed_senders', '')
        self.declare_parameter('handle_telemetry', False)
        self.declare_parameter('telemetry_senders', '')
        self.declare_parameter('telemetry_topic', 'acoustic_telemetry')
        self.declare_parameter('ack_timeout_s', 3.0)
        self.declare_parameter('max_attempts', 3)
        self.declare_parameter('panic_shouts', 3)
        self.declare_parameter('panic_gap_s', 2.0)

        self._own = modem_int(self.get_parameter('own_modem_id').value)
        self._abort_topic = self.get_parameter('abort_topic').get_parameter_value().string_value
        self._enough_topic = self.get_parameter('enough_topic').get_parameter_value().string_value
        self._ack_timeout_s = self.get_parameter('ack_timeout_s').get_parameter_value().double_value
        self._max_attempts = max(1, self.get_parameter('max_attempts').get_parameter_value().integer_value)
        self._panic_shouts = max(1, self.get_parameter('panic_shouts').get_parameter_value().integer_value)
        self._panic_gap_s = self.get_parameter('panic_gap_s').get_parameter_value().double_value
        allow_text = self.get_parameter('allowed_senders').get_parameter_value().string_value
        self._allow = self._id_list(allow_text)
        self._handle_telemetry = self.get_parameter('handle_telemetry').get_parameter_value().bool_value
        telem_text = self.get_parameter('telemetry_senders').get_parameter_value().string_value
        self._telemetry_allow = self._id_list(telem_text)
        telem_topic = self.get_parameter('telemetry_topic').get_parameter_value().string_value

        self._abort_pub = self.create_publisher(Empty, self._abort_topic, 10)
        self._enough_pub = self.create_publisher(String, self._enough_topic, 10)
        self._telemetry_pub = self.create_publisher(String, telem_topic, 10)
        self._seqs = SeqMemory()
        self._seq = 0
        self._lock = threading.Lock()
        self._ack_event = threading.Event()
        self._pending = None

        group = ReentrantCallbackGroup()
        self.driver = DriverClient(self, on_frame=self._on_frame, callback_group=group)
        self.create_service(
            AcousticAbort, 'acoustic_abort', self._on_abort, callback_group=group)
        self.create_service(
            AcousticCommand, 'acoustic_command', self._on_command, callback_group=group)

        if self._own is None:
            self.get_logger().error(
                'own_modem_id is not 001-255; acoustic abort will refuse to send or act.')
        else:
            allow = ', '.join(f'{n:03d}' for n in self._allow) or 'anyone'
            self.get_logger().info(
                f'Acoustic commands for modem {self._own:03d}. '
                f'Abort topic {self._abort_topic}. '
                f'Enough topic {self._enough_topic}. Senders: {allow}. '
                f'Targeted abort waits {self._ack_timeout_s:.1f}s, '
                f'{self._max_attempts} attempts. '
                f'Panic shouts {self._panic_shouts} times, {self._panic_gap_s:.1f}s apart, no ack. '
                f'Telemetry commands {"on" if self._handle_telemetry else "off"}.')

    @staticmethod
    def _id_list(text):
        found = []
        cleaned = '' if text is None else str(text).replace('[', '').replace(']', '')
        for piece in cleaned.replace(';', ',').split(','):
            number = modem_int(piece)
            if number is not None:
                found.append(number)
        return found

    def _next_seq(self):
        with self._lock:
            self._seq = 1 if self._seq >= 65535 else self._seq + 1
            return self._seq

    def _send_blob(self, blob):
        self.driver.write_bytes(ti.build_broadcast_command(blob))

    def _send_ok(self, cmd):
        blob = pack_command(
            CODE_OK, dest=cmd['sender'], sender=self._own, seq=cmd['seq'])
        self._send_blob(blob)

    def _on_frame(self, data, stamp):
        del stamp
        parsed = ti.parse_broadcast_payload(data)
        if parsed is None:
            return
        modem_id, payload = parsed
        cmd = unpack_command(payload)
        if cmd is None:
            return
        if modem_int(modem_id) != cmd['sender']:
            return
        if cmd['code'] == CODE_OK:
            self._note_ok(cmd)
            return
        if cmd['code'] in (CODE_ABORT, CODE_ABORT_ALL):
            self._receive_abort(cmd)
            return
        if cmd['code'] == CODE_ENOUGH:
            self._receive_enough(cmd)
            return
        if cmd['code'] in (CODE_START_TELEMETRY, CODE_STOP_TELEMETRY):
            self._receive_telemetry(cmd)

    def _receive_abort(self, cmd):
        if self._own is None:
            return
        if not sender_allowed(cmd['sender'], self._allow):
            self.get_logger().info(
                f"Ignoring {CODE_NAMES.get(cmd['code'], cmd['code'])} "
                f"from {cmd['sender']:03d} (not allowed).",
                throttle_duration_sec=5.0)
            return
        if not abort_applies(cmd, self._own):
            return
        with self._lock:
            fresh = not self._seqs.already_acted(cmd['sender'], cmd['seq'])
            if fresh:
                self._seqs.remember(cmd['sender'], cmd['seq'])
        if fresh:
            self._abort_pub.publish(Empty())
            self.get_logger().warn(
                f"{CODE_NAMES[cmd['code']]} from {cmd['sender']:03d} "
                f"seq {cmd['seq']}; published {self._abort_topic}.")
        if cmd['code'] != CODE_ABORT:
            return
        try:
            self._send_ok(cmd)
        except Exception as exc:
            self.get_logger().error(f'Failed to ack abort: {exc}')

    def _receive_enough(self, cmd):
        if self._own is None or not enough_applies(cmd, self._own):
            return
        if not sender_allowed(cmd['sender'], self._allow):
            self.get_logger().info(
                f"Ignoring ENOUGH from {cmd['sender']:03d} (not allowed).",
                throttle_duration_sec=5.0)
            return
        with self._lock:
            fresh = not self._seqs.already_acted(cmd['sender'], cmd['seq'])
            if fresh:
                self._seqs.remember(cmd['sender'], cmd['seq'])
        if fresh:
            unit = self.get_namespace().strip('/')
            body = enough_tst_body(unit)
            body['com-uuid'] = f"acoustic-enough-{cmd['sender']:03d}-{cmd['seq']}"
            msg = String()
            msg.data = json.dumps(body)
            self._enough_pub.publish(msg)
            self.get_logger().warn(
                f"ENOUGH from {cmd['sender']:03d} seq {cmd['seq']}; "
                f"published {self._enough_topic}.")
        try:
            self._send_ok(cmd)
        except Exception as exc:
            self.get_logger().error(f'Failed to ack enough: {exc}')

    def _receive_telemetry(self, cmd):
        if not self._handle_telemetry or self._own is None:
            return
        if not sender_allowed(cmd['sender'], self._telemetry_allow):
            self.get_logger().info(
                f"Ignoring {CODE_NAMES.get(cmd['code'], cmd['code'])} "
                f"from {cmd['sender']:03d} (not allowed).",
                throttle_duration_sec=5.0)
            return
        verb = beacon_verb(cmd, self._own, cmd['sender'])
        if verb is None:
            return
        msg = String()
        msg.data = verb
        self._telemetry_pub.publish(msg)
        self.get_logger().info(
            f"{verb} from {cmd['sender']:03d} seq {cmd['seq']}; "
            f"published {self._telemetry_pub.topic_name}.")
        try:
            self._send_ok(cmd)
        except Exception as exc:
            self.get_logger().error(f'Failed to ack {verb}: {exc}')

    def _note_ok(self, cmd):
        with self._lock:
            pending = self._pending
            if pending is None or self._own is None:
                return
            if not ack_matches(
                    cmd, cmd['sender'], pending['seq'], self._own, pending['expect']):
                return
            pending['acked_by'] = cmd['sender']
        self._ack_event.set()

    def _arm(self, seq, expect_sender):
        with self._lock:
            self._pending = {'seq': seq, 'expect': expect_sender, 'acked_by': None}
        self._ack_event.clear()

    def _on_abort(self, request, response):
        if self._own is None:
            response.success = False
            response.message = 'own_modem_id is not a modem id 001-255'
            return response
        parsed = parse_target(request.target)
        if parsed is None:
            response.success = False
            response.message = "target must be a modem id 001-255 or 'all'"
            return response
        kind, dest = parsed
        if kind == 'all':
            return self._panic(response)
        return self._addressed(CODE_ABORT, dest, response)

    def _on_command(self, request, response):
        if self._own is None:
            response.success = False
            response.message = 'own_modem_id is not a modem id 001-255'
            return response
        code = parse_command(request.command)
        if code is None:
            response.success = False
            response.message = (
                'command must be abort, abort_all, enough, '
                'start_telemetry, or stop_telemetry')
            return response
        if code == CODE_ABORT_ALL:
            return self._panic(response)
        parsed = parse_target(request.target)
        if parsed is None or parsed[0] != 'one':
            response.success = False
            response.message = 'target must be a modem id 001-255'
            return response
        return self._addressed(code, parsed[1], response)

    def _addressed(self, code, dest, response):
        if (dest == self._own and not self._handle_telemetry
                and code in (CODE_START_TELEMETRY, CODE_STOP_TELEMETRY)):
            response.success = False
            response.message = 'this modem is not handling telemetry commands'
            return response
        for attempt in range(1, self._max_attempts + 1):
            seq = self._next_seq()
            cmd = {
                'code': code, 'dest': dest, 'sender': self._own,
                'seq': seq, 'confirm': 0,
            }
            self._arm(seq, dest)
            try:
                self._send_blob(pack_command(**cmd))
            except Exception as exc:
                response.success = False
                response.message = f'not sent: {exc}'
                return response
            if dest == self._own:
                return self._apply_locally(code, cmd, response)
            if self._ack_event.wait(timeout=self._ack_timeout_s):
                with self._lock:
                    who = self._pending['acked_by'] if self._pending else dest
                response.success = True
                response.message = (
                    f"ack from {who:03d} for {CODE_NAMES[code]} {dest:03d} "
                    f"on attempt {attempt}")
                return response
            self.get_logger().warn(
                f'No ack from {dest:03d} for seq {seq} (attempt {attempt}).')
        response.success = False
        response.message = (
            f'no ack from {dest:03d} after {self._max_attempts} attempts')
        return response

    def _apply_locally(self, code, cmd, response):
        if code == CODE_ABORT:
            self._receive_abort(cmd)
            response.success = True
            response.message = f'aborted this modem {self._own:03d} locally'
            return response
        if code == CODE_ENOUGH:
            self._receive_enough(cmd)
            response.success = True
            response.message = f'enough on this modem {self._own:03d} locally'
            return response
        if not self._handle_telemetry:
            response.success = False
            response.message = 'this modem is not handling telemetry commands'
            return response
        self._receive_telemetry(cmd)
        response.success = True
        response.message = (
            f'{CODE_NAMES[code]} applied on this modem {self._own:03d}')
        return response

    def _panic(self, response):
        sent = 0
        for shout in range(self._panic_shouts):
            seq = self._next_seq()
            try:
                blob = pack_command(
                    CODE_ABORT_ALL, dest=self._own, sender=self._own,
                    seq=seq, confirm=CONFIRM_ALL)
                self._send_blob(blob)
            except Exception as exc:
                response.success = False
                response.message = f'sent {sent} of {self._panic_shouts}: {exc}'
                return response
            sent += 1
            if shout + 1 < self._panic_shouts and self._panic_gap_s > 0.0:
                time.sleep(self._panic_gap_s)
        response.success = True
        response.message = (
            f'shouted ABORT_ALL {sent} times from {self._own:03d}; no ack expected')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AcousticCmdNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
