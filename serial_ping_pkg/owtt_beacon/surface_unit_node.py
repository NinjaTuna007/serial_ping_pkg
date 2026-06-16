"""OWTT surface-unit node (telemetry receiver + range estimator + MQTT uplink).

A surface unit listens (Teensy in *receiver* mode) for the beacon's telemetry
broadcasts. For each telemetry frame it:

  1. decodes the compact telemetry payload (``beacon_telemetry``),
  2. turns the paired ``#I<delta_us>`` line into a one-way-travel-time range::

         range = (delta_us - offset_us) * 1e-6 * sound_velocity

  3. publishes the range locally on ROS, and
  4. publishes a JSON "range report" (its own GPS position + the measured range
     + the decoded beacon telemetry) straight to an MQTT broker, mirroring the
     smarc2 ``str_json_mqtt_bridge`` JSON-string convention.

The inference node consumes those MQTT reports from both surface units to
triangulate the beacon. As with the other OWTT nodes, the Teensy is always
reset to WIRE mode on exit.
"""

import json
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Float32, String

from serial_ping_pkg.utils import load_yaml_config, init_serial
from serial_ping_pkg.tuper_owtt import teensy_interface as ti
from serial_ping_pkg.tuper_owtt.owtt_base import WireSafeSerialNode, run_node
from serial_ping_pkg.owtt_beacon import beacon_telemetry as bt_codec
from serial_ping_pkg.owtt_beacon.mqtt_helper import MqttClient


class SurfaceUnitNode(WireSafeSerialNode):
    def __init__(self):
        super().__init__('owtt_surface_unit_node')

        config = load_yaml_config('serial_ping_pkg', 'owtt_beacon_config.yaml')
        serial_cfg = config.get('serial', {})
        owtt_cfg = config.get('owtt', {})
        teensy_cfg = config.get('teensy', {})
        unit_cfg = config.get('surface_unit', {})
        beacon_cfg = config.get('beacon', {})
        mqtt_cfg = config.get('mqtt', {})

        id_desc = ParameterDescriptor(dynamic_typing=True)

        # --- Serial link to the Teensy ---
        self.declare_parameter('serial.port', serial_cfg.get('port', '/dev/ttyACM0'))
        self.declare_parameter('serial.port_fallback', serial_cfg.get('port_fallback', '/dev/ttyACM1'))
        self.declare_parameter('serial.baudrate', serial_cfg.get('baudrate', 115200))

        # --- OWTT range conversion ---
        self.declare_parameter('owtt.delta_prefix', owtt_cfg.get('delta_prefix', '#I'))
        self.declare_parameter('owtt.offset_us', owtt_cfg.get('offset_us', 771600.0))
        self.declare_parameter('owtt.default_sound_velocity', owtt_cfg.get('default_sound_velocity', 1500.0))
        self.declare_parameter('owtt.sound_velocity_topic', owtt_cfg.get('sound_velocity_topic', '/lolo/sensors/svs'))
        self.declare_parameter('owtt.sound_velocity_msg_type', owtt_cfg.get('sound_velocity_msg_type', 'svs_interfaces/msg/SVS'))
        self.declare_parameter('owtt.sound_velocity_field', owtt_cfg.get('sound_velocity_field', 'svs'))

        # --- Teensy receiver config ---
        self.declare_parameter('teensy.own_modem_id', teensy_cfg.get('own_modem_id', '067'), id_desc)
        self.declare_parameter('teensy.command_terminator', teensy_cfg.get('command_terminator', '\r\n'))
        self.declare_parameter('teensy.mode', teensy_cfg.get('mode', 'receiver'))

        # --- This surface unit + the beacon it tracks ---
        self.declare_parameter('surface_unit.name', unit_cfg.get('name', 'surface_unit'))
        self.declare_parameter('surface_unit.latlon_topic', unit_cfg.get('latlon_topic', ''))
        # Exactly ONE surface unit should be the commander: only it transmits the
        # acoustic START/STOP (so the modems never transmit on top of each other).
        # Every unit still listens for the beacon's OK and relays it to MQTT.
        self.declare_parameter('surface_unit.commander', unit_cfg.get('commander', False))
        self.declare_parameter('beacon.name', beacon_cfg.get('name', 'lolo'))
        # If set, only telemetry frames from this modem id are accepted. Empty =
        # accept any telemetry frame (single-beacon deployments).
        self.declare_parameter('beacon.modem_id', beacon_cfg.get('modem_id', ''), id_desc)
        # Command keywords (must match the beacon's).
        self.declare_parameter('beacon.start_keyword', beacon_cfg.get('start_keyword', 'START'))
        self.declare_parameter('beacon.stop_keyword', beacon_cfg.get('stop_keyword', 'STOP'))
        self.declare_parameter('beacon.ack_message', beacon_cfg.get('ack_message', 'OK'))

        # --- MQTT uplink ---
        self.declare_parameter('mqtt.enabled', mqtt_cfg.get('enabled', True))
        self.declare_parameter('mqtt.host', mqtt_cfg.get('host', 'localhost'))
        self.declare_parameter('mqtt.port', mqtt_cfg.get('port', 1884))
        self.declare_parameter('mqtt.keepalive', mqtt_cfg.get('keepalive', 30))
        self.declare_parameter('mqtt.username', mqtt_cfg.get('username', ''))
        self.declare_parameter('mqtt.password', mqtt_cfg.get('password', ''))
        self.declare_parameter('mqtt.topic_prefix', mqtt_cfg.get('topic_prefix', 'owtt_beacon'))
        self.declare_parameter('mqtt.qos', mqtt_cfg.get('qos', 0))

        self.port = self.get_parameter('serial.port').get_parameter_value().string_value
        self.port_fallback = self.get_parameter('serial.port_fallback').get_parameter_value().string_value
        self.baudrate = self.get_parameter('serial.baudrate').get_parameter_value().integer_value
        self.delta_prefix = self.get_parameter('owtt.delta_prefix').get_parameter_value().string_value
        self.offset_us = self.get_parameter('owtt.offset_us').get_parameter_value().double_value
        self.default_sound_velocity = self.get_parameter('owtt.default_sound_velocity').get_parameter_value().double_value
        self.sound_velocity_topic = self.get_parameter('owtt.sound_velocity_topic').get_parameter_value().string_value
        self.sound_velocity_msg_type = self.get_parameter('owtt.sound_velocity_msg_type').get_parameter_value().string_value
        self.sound_velocity_field = self.get_parameter('owtt.sound_velocity_field').get_parameter_value().string_value
        self.own_modem_id = ti.normalize_modem_id(self.get_parameter('teensy.own_modem_id').value)
        self.command_terminator = self.get_parameter('teensy.command_terminator').get_parameter_value().string_value
        self.mode = self.get_parameter('teensy.mode').get_parameter_value().string_value.lower()

        self.unit_name = self.get_parameter('surface_unit.name').get_parameter_value().string_value
        self.unit_latlon_topic = self.get_parameter('surface_unit.latlon_topic').get_parameter_value().string_value
        self.is_commander = self.get_parameter('surface_unit.commander').get_parameter_value().bool_value
        self.beacon_name = self.get_parameter('beacon.name').get_parameter_value().string_value
        beacon_modem_raw = self.get_parameter('beacon.modem_id').value
        self.beacon_modem_id = ti.normalize_modem_id(beacon_modem_raw) if beacon_modem_raw not in (None, '') else ''
        self.start_keyword = self.get_parameter('beacon.start_keyword').get_parameter_value().string_value
        self.stop_keyword = self.get_parameter('beacon.stop_keyword').get_parameter_value().string_value
        self.ack_message = self.get_parameter('beacon.ack_message').get_parameter_value().string_value

        self.mqtt_enabled = self.get_parameter('mqtt.enabled').get_parameter_value().bool_value
        self.mqtt_host = self.get_parameter('mqtt.host').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter('mqtt.port').get_parameter_value().integer_value
        self.mqtt_keepalive = self.get_parameter('mqtt.keepalive').get_parameter_value().integer_value
        self.mqtt_username = self.get_parameter('mqtt.username').get_parameter_value().string_value
        self.mqtt_password = self.get_parameter('mqtt.password').get_parameter_value().string_value
        self.mqtt_topic_prefix = self.get_parameter('mqtt.topic_prefix').get_parameter_value().string_value
        self.mqtt_qos = self.get_parameter('mqtt.qos').get_parameter_value().integer_value

        if not self.unit_latlon_topic:
            self.unit_latlon_topic = f"/{self.unit_name}/smarc/latlon"

        self.sound_velocity = self.default_sound_velocity
        self.unit_position = None     # (lat, lon) of this surface unit
        self.pending = None           # telemetry dict awaiting its #I delta
        # Latest START/STOP command relayed from the inference node over MQTT,
        # awaiting the beacon's OK so we can relay the ack back.
        self.pending_cmd = None       # {'command', 'request_id'}
        self._acked_request_ids = set()

        self.ser = init_serial(self.port, self.port_fallback, self.baudrate, self.get_logger())
        if self.ser is None:
            rclpy.shutdown()
            return

        self.install_shutdown_guard()

        if self.mode == 'wire':
            self.send_command(ti.build_config_command(ti.TeensyMode.WIRE, self.own_modem_id))
            self.get_logger().warn("Started in WIRE mode: Teensy is transparent, surface unit is passive.")
            return

        self.send_command(ti.build_config_command(ti.TeensyMode.RECEIVER, self.own_modem_id))

        # --- ROS publishers ---
        self.range_pub = self.create_publisher(
            Float32, f"/{self.unit_name}/owtt_beacon/{self.beacon_name}/range", 10)
        self.telemetry_pub = self.create_publisher(
            String, f"/{self.unit_name}/owtt_beacon/{self.beacon_name}/telemetry", 10)
        # JSON range report, also usable via str_json_mqtt_bridge if preferred.
        self.report_pub = self.create_publisher(
            String, f"/{self.unit_name}/owtt_beacon/range_report", 10)

        # --- subscriptions ---
        self.create_subscription(GeoPoint, self.unit_latlon_topic, self._unit_latlon_cb, 10)
        self.get_logger().info(f"Own position <- {self.unit_latlon_topic}")
        self._setup_sound_velocity_subscription()

        # --- MQTT uplink + command channel ---
        self.mqtt = None
        self.report_topic = f"{self.mqtt_topic_prefix}/{self.beacon_name}/range/{self.unit_name}"
        self.cmd_topic = f"{self.mqtt_topic_prefix}/{self.beacon_name}/cmd"
        self.cmd_ack_topic = f"{self.mqtt_topic_prefix}/{self.beacon_name}/cmd_ack"
        if self.mqtt_enabled:
            self.mqtt = MqttClient(
                self.mqtt_host, self.mqtt_port, keepalive=self.mqtt_keepalive,
                username=(self.mqtt_username or None), password=(self.mqtt_password or None),
                client_id=f"owtt_surface_{self.unit_name}", logger=self.get_logger())
            self.mqtt.set_message_handler(self._on_mqtt)
            self.mqtt.add_subscription(self.cmd_topic, qos=1)
            self.mqtt.start()
            self.get_logger().info(
                f"MQTT uplink -> {self.mqtt_host}:{self.mqtt_port} topic '{self.report_topic}'; "
                f"command channel <- '{self.cmd_topic}'"
                + (" (this unit is the COMMANDER)." if self.is_commander else " (listener)."))

        self.buffer = ""
        self.timer = self.create_timer(0.2, self.read_serial)
        self.get_logger().info(
            f"OWTT surface unit '{self.unit_name}' initialised (receiver mode), "
            f"tracking beacon '{self.beacon_name}'"
            + (f" (modem {self.beacon_modem_id})." if self.beacon_modem_id else " (any modem)."))

    # ------------------------------------------------------------------ setup

    def _setup_sound_velocity_subscription(self):
        if not self.sound_velocity_topic:
            return
        try:
            svs_type = ti.import_message_type(self.sound_velocity_msg_type)
        except Exception as e:
            self.get_logger().warn(
                f"Sound-velocity msg type '{self.sound_velocity_msg_type}' unavailable "
                f"({e}); using default {self.default_sound_velocity} m/s.")
            return
        self.create_subscription(svs_type, self.sound_velocity_topic, self._sound_velocity_cb, 10)
        self.get_logger().info(
            f"Sound velocity <- {self.sound_velocity_topic} (field '{self.sound_velocity_field}').")

    # ------------------------------------------------------------------ callbacks

    def _unit_latlon_cb(self, msg):
        self.unit_position = (msg.latitude, msg.longitude)

    def _sound_velocity_cb(self, msg):
        try:
            self.sound_velocity = float(getattr(msg, self.sound_velocity_field))
        except (AttributeError, TypeError, ValueError) as e:
            self.get_logger().warn(f"Could not read sound velocity field "
                                   f"'{self.sound_velocity_field}': {e}")

    # ---------------------------------------------------- command channel (MQTT)

    def _on_mqtt(self, topic, payload):
        """MQTT thread: a START/STOP request from the inference node.

        The commander transmits the acoustic keyword; every unit remembers the
        pending command so that whoever hears the beacon's OK relays the ack.
        The inference node re-publishes until acked, so we de-dup by request_id.
        """
        try:
            cmd = json.loads(payload)
        except ValueError:
            return
        command = str(cmd.get('command', '')).upper()
        request_id = cmd.get('request_id')
        if command not in ('START', 'STOP') or request_id is None:
            return
        if request_id in self._acked_request_ids:
            return  # already handled this request
        self.pending_cmd = {'command': command, 'request_id': request_id}
        if self.is_commander:
            keyword = self.start_keyword if command == 'START' else self.stop_keyword
            try:
                self.send_command(ti.build_broadcast_command(keyword))
                self.get_logger().info(
                    f"[commander] transmitting acoustic {command} ('{keyword}') "
                    f"for request {request_id}.")
            except Exception as e:
                self.get_logger().error(f"Failed to transmit {command}: {e}")

    def _on_beacon_ack(self, modem_id):
        """Heard the beacon's OK; relay an ack to MQTT for the pending command."""
        if self.pending_cmd is None:
            self.get_logger().debug("Heard OK with no pending command; ignoring.")
            return
        pend = self.pending_cmd
        ack = {
            'command': pend['command'],
            'status': 'OK',
            'unit': self.unit_name,
            'beacon': self.beacon_name,
            'beacon_modem_id': modem_id,
            'request_id': pend['request_id'],
            'stamp': time.time(),
        }
        if self.mqtt is not None:
            self.mqtt.publish(self.cmd_ack_topic, json.dumps(ack), qos=1)
        self._acked_request_ids.add(pend['request_id'])
        if len(self._acked_request_ids) > 256:          # bound the memory
            self._acked_request_ids = set(list(self._acked_request_ids)[-128:])
        self.pending_cmd = None
        self.get_logger().info(
            f"Beacon OK heard -> relayed {ack['command']} ack (request "
            f"{ack['request_id']}) to '{self.cmd_ack_topic}'.")

    # ------------------------------------------------------------------ runtime

    def read_serial(self):
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
            self.get_logger().error(f"Serial communication error: {e}")

    def handle_line(self, line):
        broadcast = ti.parse_broadcast_payload(line)
        if broadcast is not None:
            modem_id, payload_data = broadcast
            # The beacon's OK ack to a START/STOP command -> relay to MQTT.
            if payload_data.strip() == self.ack_message and \
                    (not self.beacon_modem_id or modem_id == self.beacon_modem_id):
                self._on_beacon_ack(modem_id)
                return
            telem_payload = bt_codec.strip_marker(payload_data, ti.TELEMETRY_MARKER)
            if telem_payload is None:
                # Not a telemetry frame (e.g. plain lat,lon); ignore here.
                self.pending = None
                return
            if self.beacon_modem_id and modem_id != self.beacon_modem_id:
                self.get_logger().debug(
                    f"Telemetry from modem {modem_id} != beacon {self.beacon_modem_id}; ignoring.")
                self.pending = None
                return
            telemetry = bt_codec.decode_telemetry(telem_payload)
            self.pending = {'modem_id': modem_id, 'telemetry': telemetry}
            self._publish_telemetry(telemetry)
            return

        delta_us = ti.parse_owtt_delta(line, self.delta_prefix)
        if delta_us is not None:
            self._handle_delta(delta_us)
            return

        if line.startswith('#A'):
            self.get_logger().info(f"Teensy config confirmed: {line}")
            return

        self.get_logger().debug(f"<- Teensy (unhandled): {line}")

    def _publish_telemetry(self, telemetry):
        msg = String()
        msg.data = json.dumps(self._jsonable_telemetry(telemetry))
        self.telemetry_pub.publish(msg)

    @staticmethod
    def _jsonable_telemetry(telemetry):
        out = dict(telemetry)
        if 'position' in out and out['position'] is not None:
            lat, lon = out['position']
            out['position'] = {'lat': lat, 'lon': lon}
        return out

    def _handle_delta(self, delta_us):
        if self.pending is None:
            self.get_logger().warn(
                "Got OWTT delta with no preceding telemetry frame, ignoring.",
                throttle_duration_sec=5.0)
            return
        # Prefer the beacon's broadcast in-situ sound velocity over our own
        # SVS topic / frozen default.
        c, c_src = self.sound_velocity, 'local'
        telem_svs = self.pending['telemetry'].get('svs')
        if telem_svs is not None:
            try:
                c, c_src = float(telem_svs), 'beacon'
            except (TypeError, ValueError):
                pass
        rng = ti.delta_to_range_m(delta_us, self.offset_us, c)

        range_msg = Float32()
        range_msg.data = float(rng)
        self.range_pub.publish(range_msg)

        report = {
            'surface_unit': self.unit_name,
            'beacon': self.beacon_name,
            'beacon_modem_id': self.pending['modem_id'],
            'stamp': time.time(),
            'unit_lat': self.unit_position[0] if self.unit_position else None,
            'unit_lon': self.unit_position[1] if self.unit_position else None,
            'range_m': float(rng),
            'delta_us': float(delta_us),
            'offset_us': float(self.offset_us),
            'sound_velocity': float(c),
            'sound_velocity_src': c_src,
            'telemetry': self._jsonable_telemetry(self.pending['telemetry']),
        }
        report_json = json.dumps(report)

        report_msg = String()
        report_msg.data = report_json
        self.report_pub.publish(report_msg)

        if self.mqtt is not None:
            self.mqtt.publish(self.report_topic, report_json, qos=self.mqtt_qos)

        if self.unit_position is None:
            self.get_logger().warn(
                f"range={rng:.2f} m but own position unknown ({self.unit_latlon_topic} "
                "not received yet); report has null position.", throttle_duration_sec=5.0)
        self.get_logger().info(
            f"[{self.unit_name}] beacon '{self.beacon_name}' delta={delta_us} us, "
            f"c={c:.1f} m/s ({c_src}) -> range={rng:.2f} m")
        self.pending = None

    def to_wire_mode(self):
        # Stop the MQTT loop alongside the Teensy reset.
        if getattr(self, 'mqtt', None) is not None:
            try:
                self.mqtt.stop()
            except Exception:
                pass
        super().to_wire_mode()


def main(args=None):
    run_node(SurfaceUnitNode, args=args)


if __name__ == '__main__':
    main()
