"""OWTT leader / transmitter node.

Talks to a Teensy 4.1 (not the Succorfish directly). The Teensy is put into
"transmitter" mode and broadcasts the leader's position on a PPS/OCXO-disciplined
schedule. This node's job on the ROS side is:

  1. Subscribe to the vehicle's latlon topic and cache the latest fix.
  2. On a timer, transform that position from the base frame to the modem frame
     (the modem's geographic position, accounting for the lever arm and the
     vehicle's current orientation via tf2) and push it to the Teensy as
     ``$G<lat>,<lon>``.

The base/modem tf frames are auto-discovered from the tf tree (the single
``*base_link`` and ``*modem_link`` frames), so the node does not need to be told
the vehicle's name for the transform. Leaders are vehicle-specific and are never
named ``lolo`` (that is the follower).

The Teensy stores the latest GPS and emits the actual ``$B`` broadcast to the
Succorfish at the configured interval, optionally waiting to hear another modem
id first for round-robin scheduling among multiple leaders.

Lever-arm handling: rather than rotating the lever arm manually, we look up the
modem and base origins in the world frame and take their difference. That
difference is the lever arm already rotated into the world ENU frame, which we
then convert into a small lat/lon delta (flat-earth approximation).
"""

import math

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from geographic_msgs.msg import GeoPoint

import tf2_ros
from rclpy.time import Time
from rclpy.duration import Duration

from serial_ping_pkg.utils import load_yaml_config, init_serial
from serial_ping_pkg.tuper_owtt import teensy_interface as ti
from serial_ping_pkg.tuper_owtt.owtt_base import WireSafeSerialNode, run_node

# WGS84 semi-major axis (m); good enough for short lever-arm offsets.
_EARTH_RADIUS_M = 6378137.0


class OwttLeaderNode(WireSafeSerialNode):
    def __init__(self):
        super().__init__('owtt_leader_node')

        config = load_yaml_config('serial_ping_pkg', 'tuper_owtt/tuper_owtt_config.yaml')
        serial_cfg = config.get('serial', {})
        teensy_cfg = config.get('teensy', {})
        leader_cfg = config.get('leader', {})

        # --- Serial parameters (override via launch) ---
        self.declare_parameter('serial.port', serial_cfg.get('port', '/dev/ttyACM0'))
        self.declare_parameter('serial.port_fallback', serial_cfg.get('port_fallback', '/dev/ttyACM1'))
        self.declare_parameter('serial.baudrate', serial_cfg.get('baudrate', 115200))

        # --- Teensy transmitter parameters ---
        # Modem ids tolerate ros2 launch coercion (str '069' / int 69 / float 69.0);
        # declared dynamically typed and normalised on read.
        id_desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('teensy.own_modem_id', teensy_cfg.get('own_modem_id', '007'), id_desc)
        self.declare_parameter('teensy.listen_for_modem_id', teensy_cfg.get('listen_for_modem_id', '000'), id_desc)
        self.declare_parameter('teensy.broadcast_interval_s', teensy_cfg.get('broadcast_interval_s', 4))
        self.declare_parameter('teensy.command_terminator', teensy_cfg.get('command_terminator', '\r\n'))
        self.declare_parameter('teensy.mode', teensy_cfg.get('mode', 'transmitter'))

        # --- Leader parameters ---
        # Leaders are vehicle-specific and are NOT named 'lolo' (that's the follower).
        # robot_name / latlon_topic identify the position topic to subscribe to.
        # base_frame / modem_frame are auto-discovered from the tf tree when left
        # empty (there is only one base_link and one modem_link in the tree).
        self.declare_parameter('leader.robot_name', leader_cfg.get('robot_name', ''))
        self.declare_parameter('leader.latlon_topic', leader_cfg.get('latlon_topic', ''))
        self.declare_parameter('leader.base_frame', leader_cfg.get('base_frame', ''))
        self.declare_parameter('leader.modem_frame', leader_cfg.get('modem_frame', ''))
        self.declare_parameter('leader.base_link_suffix', leader_cfg.get('base_link_suffix', 'base_link'))
        self.declare_parameter('leader.modem_link_suffix', leader_cfg.get('modem_link_suffix', 'modem_link'))
        self.declare_parameter('leader.world_frame', leader_cfg.get('world_frame', 'map'))
        self.declare_parameter('leader.send_period_s', leader_cfg.get('send_period_s', 1.0))

        self.port = self.get_parameter('serial.port').get_parameter_value().string_value
        self.port_fallback = self.get_parameter('serial.port_fallback').get_parameter_value().string_value
        self.baudrate = self.get_parameter('serial.baudrate').get_parameter_value().integer_value
        self.own_modem_id = ti.normalize_modem_id(self.get_parameter('teensy.own_modem_id').value)
        self.listen_for_modem_id = ti.normalize_modem_id(self.get_parameter('teensy.listen_for_modem_id').value)
        self.broadcast_interval_s = self.get_parameter('teensy.broadcast_interval_s').get_parameter_value().integer_value
        self.command_terminator = self.get_parameter('teensy.command_terminator').get_parameter_value().string_value
        self.mode = self.get_parameter('teensy.mode').get_parameter_value().string_value.lower()
        self.robot_name = self.get_parameter('leader.robot_name').get_parameter_value().string_value
        self.latlon_topic = self.get_parameter('leader.latlon_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('leader.base_frame').get_parameter_value().string_value
        self.modem_frame = self.get_parameter('leader.modem_frame').get_parameter_value().string_value
        self.base_link_suffix = self.get_parameter('leader.base_link_suffix').get_parameter_value().string_value
        self.modem_link_suffix = self.get_parameter('leader.modem_link_suffix').get_parameter_value().string_value
        self.world_frame = self.get_parameter('leader.world_frame').get_parameter_value().string_value
        self.send_period_s = self.get_parameter('leader.send_period_s').get_parameter_value().double_value

        if not self.latlon_topic and self.robot_name:
            self.latlon_topic = f"/{self.robot_name}/smarc/latlon"

        self.ser = init_serial(self.port, self.port_fallback, self.baudrate, self.get_logger())
        if self.ser is None:
            rclpy.shutdown()
            return

        # Always leave the Teensy as a tame wire when this process exits.
        self.install_shutdown_guard()

        # Wire mode: configure the Teensy as a transparent passthrough and stay idle.
        if self.mode == 'wire':
            self.send_command(ti.build_config_command(ti.TeensyMode.WIRE, self.own_modem_id))
            self.get_logger().warn("Started in WIRE mode: Teensy is transparent, leader is passive.")
            return

        # Put the Teensy into transmitter mode.
        self.send_command(ti.build_config_command(
            ti.TeensyMode.TRANSMITTER,
            self.own_modem_id,
            listen_for_modem_id=self.listen_for_modem_id,
            broadcast_interval_s=self.broadcast_interval_s,
        ))

        # The Teensy applies the config only after the modem confirms with
        # #A<own_id> (forwarded to the host). Hold off on $G until we see it.
        self.config_confirmed = False
        self._expected_ack = "#A" + str(self.own_modem_id).zfill(3)
        self._ack_buffer = ""

        self.latest_position = None
        self.subscription = None

        # tf2 for the base_link -> modem_link lever arm (rotated into world).
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Frames / subscription may not be resolvable yet (tf tree not populated);
        # they are retried each cycle in the timer.
        self._resolve_frames()
        self._ensure_subscription()

        self.timer = self.create_timer(self.send_period_s, self.push_gps_to_teensy)
        self.get_logger().info(
            f"OWTT leader initialised (transmitter mode), pushing every {self.send_period_s}s.")

    # ------------------------------------------------------------------ helpers

    def latlon_callback(self, msg):
        self.latest_position = msg

    def _tf_frame_ids(self):
        """Return the set of frame ids currently known to the tf tree."""
        import yaml
        try:
            data = yaml.safe_load(self.tf_buffer.all_frames_as_yaml()) or {}
        except Exception:
            return set()
        ids = set(data.keys())
        for info in data.values():
            if isinstance(info, dict) and info.get('parent'):
                ids.add(info['parent'])
        return ids

    def _match_suffix(self, frame_ids, suffix):
        """Find the single frame whose name is/ends with ``suffix`` (e.g. base_link)."""
        matches = sorted(f for f in frame_ids if f == suffix or f.endswith('/' + suffix))
        if not matches:
            return ''
        if len(matches) > 1:
            self.get_logger().warn(
                f"Multiple frames match '{suffix}': {matches}; using {matches[0]}.")
        return matches[0]

    def _resolve_frames(self):
        """Auto-discover base/modem frames from the tf tree if not explicitly set."""
        if self.base_frame and self.modem_frame:
            return True
        frame_ids = self._tf_frame_ids()
        if not frame_ids:
            return False
        if not self.base_frame:
            self.base_frame = self._match_suffix(frame_ids, self.base_link_suffix)
            if self.base_frame:
                self.get_logger().info(f"Auto-discovered base frame: {self.base_frame}")
        if not self.modem_frame:
            self.modem_frame = self._match_suffix(frame_ids, self.modem_link_suffix)
            if self.modem_frame:
                self.get_logger().info(f"Auto-discovered modem frame: {self.modem_frame}")
        return bool(self.base_frame and self.modem_frame)

    def _ensure_subscription(self):
        """Create the latlon subscription once we know which topic to use."""
        if self.subscription is not None:
            return
        topic = self.latlon_topic
        if not topic and self.base_frame and '/' in self.base_frame:
            # Derive the vehicle namespace from the discovered base frame prefix.
            prefix = self.base_frame.rsplit('/', 1)[0]
            topic = f"/{prefix}/smarc/latlon"
        if not topic:
            self.get_logger().warn(
                "No latlon topic resolved yet (set leader.robot_name or "
                "leader.latlon_topic, or wait for tf); not subscribed.",
                throttle_duration_sec=5.0)
            return
        self.latlon_topic = topic
        self.subscription = self.create_subscription(
            GeoPoint, topic, self.latlon_callback, 10)
        self.get_logger().info(f"Subscribing to {topic}")

    def _lever_arm_offset_enu(self):
        """Return (dE, dN) metres of the modem relative to base, in the world frame.

        Computed as (world->modem origin) - (world->base origin), which yields
        the lever arm already rotated by the vehicle's orientation. Returns
        ``None`` if the frames or transforms are unavailable.
        """
        if not self._resolve_frames():
            self.get_logger().warn(
                "tf frames not resolved yet; sending untransformed lat/lon.",
                throttle_duration_sec=5.0)
            return None
        try:
            t_wb = self.tf_buffer.lookup_transform(
                self.world_frame, self.base_frame, Time(), timeout=Duration(seconds=0.2))
            t_wm = self.tf_buffer.lookup_transform(
                self.world_frame, self.modem_frame, Time(), timeout=Duration(seconds=0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"tf lookup failed ({e}); sending untransformed lat/lon.",
                                   throttle_duration_sec=5.0)
            return None
        dE = t_wm.transform.translation.x - t_wb.transform.translation.x
        dN = t_wm.transform.translation.y - t_wb.transform.translation.y
        return dE, dN

    def _apply_offset(self, lat, lon):
        """Apply the modem lever-arm offset to a base-frame lat/lon."""
        offset = self._lever_arm_offset_enu()
        if offset is None:
            return lat, lon
        dE, dN = offset
        dlat = math.degrees(dN / _EARTH_RADIUS_M)
        dlon = math.degrees(dE / (_EARTH_RADIUS_M * math.cos(math.radians(lat))))
        return lat + dlat, lon + dlon

    # ------------------------------------------------------------------ runtime

    def _drain_serial(self):
        """Drain incoming serial; flag config as confirmed when #A<own_id> seen."""
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                self._ack_buffer = (self._ack_buffer + data).replace('\r\n', '\n')
                while '\n' in self._ack_buffer:
                    line, self._ack_buffer = self._ack_buffer.split('\n', 1)
                    line = line.strip()
                    if not self.config_confirmed and line.startswith(self._expected_ack):
                        self.config_confirmed = True
                        self.get_logger().info(f"Teensy config confirmed: {line}")
        except Exception as e:
            self.get_logger().warn(f"Error reading serial: {e}", throttle_duration_sec=5.0)

    def push_gps_to_teensy(self):
        # Drain serial to catch the modem's #A<own_id> config confirmation.
        self._drain_serial()
        # Retry frame discovery / subscription until the tf tree + topic are ready.
        self._resolve_frames()
        self._ensure_subscription()
        if not self.config_confirmed:
            self.get_logger().info(
                f"Waiting for Teensy config confirmation ({self._expected_ack}) "
                "before sending GPS...", throttle_duration_sec=5.0)
            return
        if self.latest_position is None:
            self.get_logger().debug("No position fix yet; nothing to push.")
            return
        lat = self.latest_position.latitude
        lon = self.latest_position.longitude
        modem_lat, modem_lon = self._apply_offset(lat, lon)
        cmd = ti.build_gps_command(f"{modem_lat:.8f}", f"{modem_lon:.8f}")
        try:
            self.send_command(cmd)
        except Exception as e:
            self.get_logger().error(f"Serial communication error: {e}")


def main(args=None):
    run_node(OwttLeaderNode, args=args)


if __name__ == '__main__':
    main()
