"""OWTT beacon node (telemetry transmitter).

This is the "beacon" of the owtt_beacon scenario: a vehicle (default ``lolo``,
configurable) that periodically broadcasts a small telemetry payload over the
acoustic link. It talks to a Teensy 4.1 in *transmitter* mode (not the
Succorfish directly).

It subscribes to a configurable subset of telemetry sources and packs them into
the compact frame defined in ``beacon_telemetry`` (default: bt tip only), then
pushes it to the Teensy as ``$K<payload>``. The Teensy adds the ``TEL:`` marker
and broadcasts ``$BnnTEL:<payload>`` on its PPS/OCXO schedule, so the surface units
both receive the telemetry *and* can measure a one-way-travel-time range to the
beacon.

Telemetry sources (all under the beacon's namespace, smarc2 conventions):
    position  /<name>/smarc/latlon          geographic_msgs/GeoPoint
    depth     /<name>/smarc/depth           std_msgs/Float32 (metres, +down)
    svs       /<name>/sensors/svs           svs_interfaces/msg/SVS (sound velocity)
    speed     /<name>/smarc/speed           std_msgs/Float32
    bt        /<name>/waraps/sensor/bt      std_msgs/String (JSON, field 'tip')

Broadcasting svs lets the surface units convert travel time to range with the
beacon's in-situ sound velocity instead of a frozen default.

Broadcasting depth lets the inference node turn the surface units' *slant*
ranges into horizontal ranges (the beacon may be submerged).

Like the OWTT leader, it waits for the Teensy's ``#A<own_id>`` config
confirmation before sending, and always resets the Teensy to WIRE mode on exit.

Command channel: the beacon does NOT broadcast until it hears a ``START``
broadcast (and stops on ``STOP``); unless ``autostart`` is set. Every command is
acknowledged with an ``OK`` broadcast -- on a real transition AND when already in
the requested state (idempotent), so a request/response caller always gets a
definitive reply. ``commander_modem_ids`` optionally restricts who may command
(empty = anyone).
"""

import json

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Float32, String

from serial_ping_pkg.utils import load_yaml_config, init_serial
from serial_ping_pkg.tuper_owtt import teensy_interface as ti
from serial_ping_pkg.tuper_owtt.owtt_base import WireSafeSerialNode, run_node
from serial_ping_pkg.owtt_beacon import beacon_telemetry as bt_codec


class BeaconNode(WireSafeSerialNode):
    def __init__(self):
        super().__init__('owtt_beacon_node')

        config = load_yaml_config('serial_ping_pkg', 'owtt_beacon_config.yaml')
        serial_cfg = config.get('serial', {})
        teensy_cfg = config.get('teensy', {})
        beacon_cfg = config.get('beacon', {})
        owtt_cfg = config.get('owtt', {})

        # --- Serial link to the Teensy ---
        self.declare_parameter('serial.port', serial_cfg.get('port', '/dev/ttyACM0'))
        self.declare_parameter('serial.port_fallback', serial_cfg.get('port_fallback', '/dev/ttyACM1'))
        self.declare_parameter('serial.baudrate', serial_cfg.get('baudrate', 115200))

        # --- Teensy transmitter config ---
        id_desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('teensy.own_modem_id', teensy_cfg.get('own_modem_id', '101'), id_desc)
        self.declare_parameter('teensy.listen_for_modem_id', teensy_cfg.get('listen_for_modem_id', '000'), id_desc)
        self.declare_parameter('teensy.broadcast_interval_s', teensy_cfg.get('broadcast_interval_s', 1))
        self.declare_parameter('teensy.command_terminator', teensy_cfg.get('command_terminator', '\r\n'))
        self.declare_parameter('teensy.mode', teensy_cfg.get('mode', 'transmitter'))

        # --- Beacon telemetry config ---
        self.declare_parameter('beacon.name', beacon_cfg.get('name', 'lolo'))
        # Subset of {position, speed, bt}; default broadcasts ONLY bt info.
        self.declare_parameter('beacon.telemetry_fields', beacon_cfg.get('telemetry_fields', ['bt']))
        self.declare_parameter('beacon.latlon_topic', beacon_cfg.get('latlon_topic', ''))
        self.declare_parameter('beacon.depth_topic', beacon_cfg.get('depth_topic', ''))
        self.declare_parameter('beacon.speed_topic', beacon_cfg.get('speed_topic', ''))
        # Sound velocity source (broadcast so surface units don't use a frozen value).
        self.declare_parameter('owtt.sound_velocity_topic', owtt_cfg.get('sound_velocity_topic', '/lolo/sensors/svs'))
        self.declare_parameter('owtt.sound_velocity_msg_type', owtt_cfg.get('sound_velocity_msg_type', 'svs_interfaces/msg/SVS'))
        self.declare_parameter('owtt.sound_velocity_field', owtt_cfg.get('sound_velocity_field', 'svs'))
        self.declare_parameter('beacon.bt_topic', beacon_cfg.get('bt_topic', ''))
        self.declare_parameter('beacon.bt_json_field', beacon_cfg.get('bt_json_field', 'tip'))
        self.declare_parameter('beacon.position_precision', beacon_cfg.get('position_precision', 6))
        self.declare_parameter('beacon.max_bt_len', beacon_cfg.get('max_bt_len', 32))
        self.declare_parameter('beacon.send_period_s', beacon_cfg.get('send_period_s', 1.0))
        # Include the beacon's own GPS in the first N broadcasts (even if
        # 'position' is not a permanent field) so the inference node can lock the
        # correct branch, then drop it to save acoustic bytes. 0 = disabled.
        self.declare_parameter('beacon.position_seed_count', beacon_cfg.get('position_seed_count', 0))

        # --- Broadcast START/STOP command channel ---
        # The beacon does NOT broadcast until it hears a START broadcast (and
        # stops on STOP). Both are acknowledged once with an OK broadcast.
        self.declare_parameter('beacon.autostart', beacon_cfg.get('autostart', False))
        # Modem ids allowed to command START/STOP. Empty list = accept anyone.
        self.declare_parameter('beacon.commander_modem_ids',
                               beacon_cfg.get('commander_modem_ids', []), id_desc)
        self.declare_parameter('beacon.start_keyword', beacon_cfg.get('start_keyword', 'START'))
        self.declare_parameter('beacon.stop_keyword', beacon_cfg.get('stop_keyword', 'STOP'))
        self.declare_parameter('beacon.ack_message', beacon_cfg.get('ack_message', 'OK'))

        self.port = self.get_parameter('serial.port').get_parameter_value().string_value
        self.port_fallback = self.get_parameter('serial.port_fallback').get_parameter_value().string_value
        self.baudrate = self.get_parameter('serial.baudrate').get_parameter_value().integer_value
        self.own_modem_id = ti.normalize_modem_id(self.get_parameter('teensy.own_modem_id').value)
        self.listen_for_modem_id = ti.normalize_modem_id(self.get_parameter('teensy.listen_for_modem_id').value)
        self.broadcast_interval_s = self.get_parameter('teensy.broadcast_interval_s').get_parameter_value().integer_value
        self.command_terminator = self.get_parameter('teensy.command_terminator').get_parameter_value().string_value
        self.mode = self.get_parameter('teensy.mode').get_parameter_value().string_value.lower()

        self.name = self.get_parameter('beacon.name').get_parameter_value().string_value
        self.telemetry_fields = list(self.get_parameter('beacon.telemetry_fields').get_parameter_value().string_array_value)
        self.latlon_topic = self.get_parameter('beacon.latlon_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('beacon.depth_topic').get_parameter_value().string_value
        self.speed_topic = self.get_parameter('beacon.speed_topic').get_parameter_value().string_value
        self.svs_topic = self.get_parameter('owtt.sound_velocity_topic').get_parameter_value().string_value
        self.svs_msg_type = self.get_parameter('owtt.sound_velocity_msg_type').get_parameter_value().string_value
        self.svs_field = self.get_parameter('owtt.sound_velocity_field').get_parameter_value().string_value
        self.bt_topic = self.get_parameter('beacon.bt_topic').get_parameter_value().string_value
        self.bt_json_field = self.get_parameter('beacon.bt_json_field').get_parameter_value().string_value
        self.position_precision = self.get_parameter('beacon.position_precision').get_parameter_value().integer_value
        self.max_bt_len = self.get_parameter('beacon.max_bt_len').get_parameter_value().integer_value
        self.send_period_s = self.get_parameter('beacon.send_period_s').get_parameter_value().double_value
        self.position_seed_count = self.get_parameter('beacon.position_seed_count').get_parameter_value().integer_value

        self.autostart = self.get_parameter('beacon.autostart').get_parameter_value().bool_value
        commander_ids_raw = self.get_parameter('beacon.commander_modem_ids').value or []
        self.commander_modem_ids = {ti.normalize_modem_id(c) for c in commander_ids_raw}
        self.start_keyword = self.get_parameter('beacon.start_keyword').get_parameter_value().string_value
        self.stop_keyword = self.get_parameter('beacon.stop_keyword').get_parameter_value().string_value
        self.ack_message = self.get_parameter('beacon.ack_message').get_parameter_value().string_value

        if not self.telemetry_fields:
            self.telemetry_fields = ['bt']

        # Telemetry is gated by START/STOP commands (unless autostart is set).
        self.broadcasting = self.autostart

        # Default telemetry topics derived from the beacon name.
        if not self.latlon_topic:
            self.latlon_topic = f"/{self.name}/smarc/latlon"
        if not self.depth_topic:
            self.depth_topic = f"/{self.name}/smarc/depth"
        if not self.speed_topic:
            self.speed_topic = f"/{self.name}/smarc/speed"
        if not self.bt_topic:
            self.bt_topic = f"/{self.name}/waraps/sensor/bt"

        # Latest telemetry values.
        self._telemetry_sent = 0      # count of telemetry broadcasts pushed
        self.latest_position = None   # (lat, lon)
        self.latest_depth = None      # float, metres below surface (+down)
        self.latest_svs = None        # float, sound velocity m/s
        self.latest_speed = None      # float
        self.latest_bt = None         # str

        self.ser = init_serial(self.port, self.port_fallback, self.baudrate, self.get_logger())
        if self.ser is None:
            rclpy.shutdown()
            return

        # Always leave the Teensy as a tame wire when this process exits.
        self.install_shutdown_guard()

        if self.mode == 'wire':
            self.send_command(ti.build_config_command(ti.TeensyMode.WIRE, self.own_modem_id))
            self.get_logger().warn("Started in WIRE mode: Teensy is transparent, beacon is passive.")
            return

        # Configure the Teensy as a transmitter.
        self.send_command(ti.build_config_command(
            ti.TeensyMode.TRANSMITTER,
            self.own_modem_id,
            listen_for_modem_id=self.listen_for_modem_id,
            broadcast_interval_s=self.broadcast_interval_s,
        ))

        # Hold off on $K telemetry until the modem confirms the config (#A<own_id>).
        self.config_confirmed = False
        self._expected_ack = "#A" + str(self.own_modem_id).zfill(3)
        self.buffer = ""

        self._setup_subscriptions()

        # Read serial (config ack + START/STOP commands) faster than we transmit.
        self.read_timer = self.create_timer(0.2, self.read_serial)
        self.send_timer = self.create_timer(self.send_period_s, self.push_telemetry)
        cmd_src = (', '.join(sorted(self.commander_modem_ids))
                   if self.commander_modem_ids else 'anyone')
        self.get_logger().info(
            f"OWTT beacon '{self.name}' initialised (transmitter mode); fields="
            f"{self.telemetry_fields}, every {self.send_period_s}s. "
            f"Telemetry is {'ON (autostart)' if self.broadcasting else 'OFF'}; "
            f"waiting for '{self.start_keyword}'/'{self.stop_keyword}' from {cmd_src}.")

    # ------------------------------------------------------------------ setup

    def _setup_subscriptions(self):
        fields = set(self.telemetry_fields)
        # Subscribe to position if it is a permanent field OR we are seeding it.
        if 'position' in fields or self.position_seed_count > 0:
            self.create_subscription(GeoPoint, self.latlon_topic, self._latlon_cb, 10)
            self.get_logger().info(f"Telemetry[position] <- {self.latlon_topic}")
        if 'depth' in fields:
            self.create_subscription(Float32, self.depth_topic, self._depth_cb, 10)
            self.get_logger().info(f"Telemetry[depth] <- {self.depth_topic}")
        if 'svs' in fields:
            self._setup_svs_subscription()
        if 'speed' in fields:
            self.create_subscription(Float32, self.speed_topic, self._speed_cb, 10)
            self.get_logger().info(f"Telemetry[speed] <- {self.speed_topic}")
        if 'bt' in fields:
            self.create_subscription(String, self.bt_topic, self._bt_cb, 10)
            self.get_logger().info(f"Telemetry[bt] <- {self.bt_topic}")

    def _setup_svs_subscription(self):
        if not self.svs_topic:
            return
        try:
            svs_type = ti.import_message_type(self.svs_msg_type)
        except Exception as e:
            self.get_logger().warn(
                f"SVS msg type '{self.svs_msg_type}' unavailable ({e}); "
                "sound velocity will not be broadcast.")
            return
        self.create_subscription(svs_type, self.svs_topic, self._svs_cb, 10)
        self.get_logger().info(f"Telemetry[svs] <- {self.svs_topic} (field '{self.svs_field}')")

    # ------------------------------------------------------------------ callbacks

    def _latlon_cb(self, msg):
        self.latest_position = (msg.latitude, msg.longitude)

    def _depth_cb(self, msg):
        self.latest_depth = float(msg.data)

    def _svs_cb(self, msg):
        try:
            self.latest_svs = float(getattr(msg, self.svs_field))
        except (AttributeError, TypeError, ValueError) as e:
            self.get_logger().warn(f"Could not read SVS field '{self.svs_field}': {e}",
                                   throttle_duration_sec=5.0)

    def _speed_cb(self, msg):
        self.latest_speed = float(msg.data)

    def _bt_cb(self, msg):
        # bt is published as a JSON string, e.g. {"agent-uuid": ..., "tip": "..."}.
        # Extract the configured field; fall back to the raw string.
        raw = msg.data
        try:
            obj = json.loads(raw)
            self.latest_bt = str(obj.get(self.bt_json_field, raw))
        except (ValueError, TypeError):
            self.latest_bt = raw

    # ------------------------------------------------------------------ runtime

    def read_serial(self):
        """Parse incoming serial: config confirmation + START/STOP commands.

        The Teensy forwards every received modem broadcast to the host even in
        transmitter mode, so the beacon can listen for command broadcasts here.
        """
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                self.buffer = (self.buffer + data).replace('\r\n', '\n')
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self.handle_line(line)
        except Exception as e:
            self.get_logger().warn(f"Error reading serial: {e}", throttle_duration_sec=5.0)

    def handle_line(self, line):
        if not self.config_confirmed and line.startswith(self._expected_ack):
            self.config_confirmed = True
            self.get_logger().info(f"Teensy config confirmed: {line}")
            return

        broadcast = ti.parse_broadcast_payload(line)
        if broadcast is not None:
            modem_id, payload = broadcast
            self._handle_command(modem_id, payload.strip())

    def _commander_allowed(self, modem_id):
        # Empty allow-list == accept commands from anyone.
        return (not self.commander_modem_ids) or (modem_id in self.commander_modem_ids)

    def _handle_command(self, modem_id, payload):
        if payload == self.start_keyword:
            cmd = 'START'
        elif payload == self.stop_keyword:
            cmd = 'STOP'
        else:
            return  # not a command broadcast

        if not self._commander_allowed(modem_id):
            self.get_logger().info(
                f"Ignoring {cmd} from modem {modem_id} (not in commander list).",
                throttle_duration_sec=5.0)
            return

        want_broadcasting = (cmd == 'START')
        transition = (want_broadcasting != self.broadcasting)
        self.broadcasting = want_broadcasting
        # Always acknowledge so the commanding side gets a definitive reply
        # (idempotent): on a real transition or when already in the requested
        # state. Only the state change is logged distinctly.
        if transition:
            self.get_logger().info(
                f"{cmd} from modem {modem_id}: telemetry "
                f"{'STARTED' if self.broadcasting else 'STOPPED'}; acking '{self.ack_message}'.")
        else:
            self.get_logger().info(
                f"{cmd} from modem {modem_id}: already "
                f"{'broadcasting' if self.broadcasting else 'stopped'}; "
                f"acking '{self.ack_message}' (idempotent).")
        self._broadcast_ack()

    def _broadcast_ack(self):
        """Broadcast the ack message immediately (passed straight to the modem)."""
        try:
            self.send_command(ti.build_broadcast_command(self.ack_message))
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast ack: {e}")

    def push_telemetry(self):
        if not self.broadcasting:
            return
        if not self.config_confirmed:
            self.get_logger().info(
                f"Waiting for Teensy config confirmation ({self._expected_ack}) "
                "before sending telemetry...", throttle_duration_sec=5.0)
            return

        # During the seeding window, force-include position so the inference
        # node can lock the correct branch; afterwards revert to configured fields.
        fields = list(self.telemetry_fields)
        seeding = self._telemetry_sent < self.position_seed_count
        if seeding and 'position' not in fields:
            fields = fields + ['position']

        payload = bt_codec.encode_telemetry(
            fields,
            position=self.latest_position,
            depth=self.latest_depth,
            svs=self.latest_svs,
            speed=self.latest_speed,
            bt=self.latest_bt,
            precision=self.position_precision,
            max_bt_len=self.max_bt_len,
        )
        if not payload:
            self.get_logger().info(
                "No telemetry available yet for the enabled fields; nothing to send.",
                throttle_duration_sec=5.0)
            return

        try:
            self.send_command(ti.build_telemetry_command(payload))
            self._telemetry_sent += 1
            if seeding and self._telemetry_sent == self.position_seed_count:
                self.get_logger().info(
                    f"Position-seeding done after {self.position_seed_count} broadcasts; "
                    "reverting to configured telemetry fields.")
        except Exception as e:
            self.get_logger().error(f"Serial communication error: {e}")


def main(args=None):
    run_node(BeaconNode, args=args)


if __name__ == '__main__':
    main()
