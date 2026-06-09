"""OWTT follower / receiver node.

Talks to a Teensy 4.1 (not the Succorfish directly). The Teensy is put into
"receiver" mode and:
  * passes through the Succorfish broadcast frames (``#B<modem><nn><lat,lon>``)
  * additionally emits a one-way-travel-time delta line (default prefix ``#I``)
    once it has measured the OWTT for the most recent broadcast.

This node decodes the leader position from the broadcast and turns the delta
into a range using::

    range = (delta_us - offset_us) * 1e-6 * sound_velocity

``sound_velocity`` defaults to 1500 m/s but is sourced live from a configurable
topic/msg type (default ``/lolo/sensors/svs`` of type ``svs_interfaces/msg/SVS``,
field ``svs``) when available. It then publishes leader position + range on the
same topics the legacy ``informed_follower_node`` used:
``/<leader>/smarc/latlon`` and ``/<leader>/distance``.
"""

import rclpy
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

from serial_ping_pkg.utils import load_yaml_config, init_serial
from serial_ping_pkg.tuper_owtt import teensy_interface as ti
from serial_ping_pkg.tuper_owtt.owtt_base import WireSafeSerialNode, run_node


class OwttFollowerNode(WireSafeSerialNode):
    def __init__(self):
        super().__init__('owtt_follower_node')

        config = load_yaml_config('serial_ping_pkg', 'tuper_owtt_config.yaml')
        serial_cfg = config.get('serial', {})
        owtt_cfg = config.get('owtt', {})
        follower_cfg = config.get('follower', {})
        teensy_cfg = config.get('teensy', {})

        # --- Serial parameters (override via launch) ---
        self.declare_parameter('serial.port', serial_cfg.get('port', '/dev/ttyACM0'))
        self.declare_parameter('serial.port_fallback', serial_cfg.get('port_fallback', '/dev/ttyACM1'))
        self.declare_parameter('serial.baudrate', serial_cfg.get('baudrate', 115200))

        # --- OWTT parameters ---
        self.declare_parameter('owtt.delta_prefix', owtt_cfg.get('delta_prefix', '#I'))
        self.declare_parameter('owtt.offset_us', owtt_cfg.get('offset_us', 0.0))
        self.declare_parameter('owtt.default_sound_velocity', owtt_cfg.get('default_sound_velocity', 1500.0))
        self.declare_parameter('owtt.sound_velocity_topic', owtt_cfg.get('sound_velocity_topic', '/lolo/sensors/svs'))
        self.declare_parameter('owtt.sound_velocity_msg_type', owtt_cfg.get('sound_velocity_msg_type', 'svs_interfaces/msg/SVS'))
        self.declare_parameter('owtt.sound_velocity_field', owtt_cfg.get('sound_velocity_field', 'svs'))

        # --- Follower parameters ---
        self.declare_parameter('follower.leader_gps_msg_type', follower_cfg.get('leader_gps_msg_type', 'GeoPoint'))
        # Two leaders, routed by acoustic modem id. Names + ids overridable via launch.
        self.declare_parameter('follower.leader1_name', follower_cfg.get('leader1_name', 'leader1'))
        self.declare_parameter('follower.leader1_modem_id', follower_cfg.get('leader1_modem_id', '007'))
        self.declare_parameter('follower.leader2_name', follower_cfg.get('leader2_name', 'leader2'))
        self.declare_parameter('follower.leader2_modem_id', follower_cfg.get('leader2_modem_id', '111'))

        # --- Teensy / mode parameters ---
        self.declare_parameter('teensy.own_modem_id', teensy_cfg.get('own_modem_id', '101'))
        self.declare_parameter('teensy.command_terminator', teensy_cfg.get('command_terminator', '\r\n'))
        self.declare_parameter('teensy.mode', teensy_cfg.get('mode', 'receiver'))

        self.port = self.get_parameter('serial.port').get_parameter_value().string_value
        self.port_fallback = self.get_parameter('serial.port_fallback').get_parameter_value().string_value
        self.baudrate = self.get_parameter('serial.baudrate').get_parameter_value().integer_value
        self.delta_prefix = self.get_parameter('owtt.delta_prefix').get_parameter_value().string_value
        self.offset_us = self.get_parameter('owtt.offset_us').get_parameter_value().double_value
        self.default_sound_velocity = self.get_parameter('owtt.default_sound_velocity').get_parameter_value().double_value
        self.sound_velocity_topic = self.get_parameter('owtt.sound_velocity_topic').get_parameter_value().string_value
        self.sound_velocity_msg_type = self.get_parameter('owtt.sound_velocity_msg_type').get_parameter_value().string_value
        self.sound_velocity_field = self.get_parameter('owtt.sound_velocity_field').get_parameter_value().string_value
        self.leader_gps_msg_type = self.get_parameter('follower.leader_gps_msg_type').get_parameter_value().string_value
        self.leader1_name = self.get_parameter('follower.leader1_name').get_parameter_value().string_value
        self.leader1_modem_id = self.get_parameter('follower.leader1_modem_id').get_parameter_value().string_value
        self.leader2_name = self.get_parameter('follower.leader2_name').get_parameter_value().string_value
        self.leader2_modem_id = self.get_parameter('follower.leader2_modem_id').get_parameter_value().string_value
        self.own_modem_id = self.get_parameter('teensy.own_modem_id').get_parameter_value().string_value
        self.command_terminator = self.get_parameter('teensy.command_terminator').get_parameter_value().string_value
        self.mode = self.get_parameter('teensy.mode').get_parameter_value().string_value.lower()

        # Latest sound velocity (falls back to default until an SVS msg arrives).
        self.sound_velocity = self.default_sound_velocity

        self.ser = init_serial(self.port, self.port_fallback, self.baudrate, self.get_logger())
        if self.ser is None:
            rclpy.shutdown()
            return

        # Always leave the Teensy as a tame wire when this process exits.
        self.install_shutdown_guard()

        # Wire mode: configure the Teensy as a transparent passthrough and stay idle.
        if self.mode == 'wire':
            self.send_command(ti.build_config_command(ti.TeensyMode.WIRE, self.own_modem_id))
            self.get_logger().warn("Started in WIRE mode: Teensy is transparent, follower is passive.")
            return

        # Put the Teensy into receiver mode.
        self.send_command(ti.build_config_command(ti.TeensyMode.RECEIVER, self.own_modem_id))

        # Resolve leader GPS message type.
        self.LeaderMsgType = self._resolve_gps_type(self.leader_gps_msg_type)

        # Build modem_id -> {name, pos_pub, dist_pub} routing for the two leaders.
        self.robots = {}
        for name, modem_id in ((self.leader1_name, self.leader1_modem_id),
                               (self.leader2_name, self.leader2_modem_id)):
            if not name or not modem_id:
                continue
            modem_id = str(modem_id).zfill(3)
            pos_topic = f"/{name}/smarc/latlon"
            dist_topic = f"/{name}/distance"
            self.robots[modem_id] = {
                'name': name,
                'pos_pub': self.create_publisher(self.LeaderMsgType, pos_topic, 10),
                'dist_pub': self.create_publisher(Float32, dist_topic, 10),
            }
            self.get_logger().info(f"Leader {name}: modem {modem_id} -> {pos_topic}, {dist_topic}")

        # Pairing state: the delta line refers to the most recent broadcast.
        self.pending_modem_id = None

        # Optional live sound-velocity subscription.
        self._setup_sound_velocity_subscription()

        self.buffer = ""
        self.timer = self.create_timer(0.5, self.read_serial)
        self.get_logger().info("OWTT follower initialised (receiver mode).")

    # ------------------------------------------------------------------ helpers

    def _resolve_gps_type(self, name):
        if name == 'GeoPoint':
            return GeoPoint
        if name == 'NavSatFix':
            return NavSatFix
        raise ValueError(f"Unsupported leader_gps_msg_type: {name}")

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
        self.create_subscription(svs_type, self.sound_velocity_topic, self.sound_velocity_callback, 10)
        self.get_logger().info(
            f"Subscribed to sound velocity on {self.sound_velocity_topic} "
            f"(field '{self.sound_velocity_field}').")

    # ------------------------------------------------------------------ runtime

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                self.buffer += data
                # Accept either \r\n or \n line endings.
                self.buffer = self.buffer.replace('\r\n', '\n')
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self.handle_line(line)
        except Exception as e:
            self.get_logger().error(f"Serial communication error: {e}")

    def handle_line(self, line):
        # Leader position broadcast.
        broadcast = ti.parse_broadcast(line)
        if broadcast is not None:
            modem_id, lat, lon = broadcast
            self.pending_modem_id = modem_id
            self.publish_position(modem_id, lat, lon)
            return

        # OWTT delta -> range, paired with the most recent broadcast.
        delta_us = ti.parse_owtt_delta(line, self.delta_prefix)
        if delta_us is not None:
            self.publish_range(delta_us)
            return

        # Teensy/modem config confirmation (forwarded to the host).
        if line.startswith('#A'):
            self.get_logger().info(f"Teensy config confirmed: {line}")
            return

        # Anything else (e.g. classic Succorfish responses) is just logged.
        self.get_logger().debug(f"<- Teensy (unhandled): {line}")

    def publish_position(self, modem_id, lat, lon):
        robot = self.robots.get(modem_id)
        if robot is None:
            self.get_logger().warn(f"Broadcast from unknown modem_id {modem_id}, ignoring.")
            return
        msg = self.LeaderMsgType()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 0.0
        robot['pos_pub'].publish(msg)
        self.get_logger().info(f"[{robot['name']}] position lat={lat}, lon={lon}")

    def publish_range(self, delta_us):
        if self.pending_modem_id is None:
            self.get_logger().warn("Got OWTT delta with no preceding broadcast, ignoring.")
            return
        robot = self.robots.get(self.pending_modem_id)
        if robot is None:
            self.get_logger().warn(
                f"OWTT delta for unknown modem_id {self.pending_modem_id}, ignoring.")
            return
        rng = ti.delta_to_range_m(delta_us, self.offset_us, self.sound_velocity)
        msg = Float32()
        msg.data = float(rng)
        robot['dist_pub'].publish(msg)
        self.get_logger().info(
            f"[{robot['name']}] delta={delta_us} us, c={self.sound_velocity} m/s -> range={rng:.3f} m")

    def sound_velocity_callback(self, msg):
        try:
            self.sound_velocity = float(getattr(msg, self.sound_velocity_field))
        except (AttributeError, TypeError, ValueError) as e:
            self.get_logger().warn(f"Could not read sound velocity field "
                                   f"'{self.sound_velocity_field}': {e}")


def main(args=None):
    run_node(OwttFollowerNode, args=args)


if __name__ == '__main__':
    main()
