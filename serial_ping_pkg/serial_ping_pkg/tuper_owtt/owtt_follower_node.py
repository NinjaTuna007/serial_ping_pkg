"""OWTT follower / receiver node.

Talks to a Teensy 4.1 (not the Succorfish directly). The Teensy is put into
"receiver" mode and:
  * passes through the Succorfish broadcast frames (``#B<modem><nn><lat,lon>``)
  * additionally emits a one-way-travel-time delta line (default prefix ``#I``)
    once it has measured the OWTT for the most recent broadcast.

This node decodes the leader position from the broadcast and turns the delta
into a range using::

    range = (delta_us - offset_us) * 1e-6 * sound_velocity

``offset_us`` can be **auto-calibrated per leader modem** (same idea as the
beacon surface unit): while this stick has a GPS fix and the leader's broadcast
carries a position, each sample solves
``offset = delta_us - true_range / c * 1e6``, folds it into a running median,
and locks after ``calib_min_samples``. Until then (or if auto-cal is off) the
configured ``owtt.offset_us`` is used.

True range is **modem-to-modem**: the leader broadcast already carries the
leader modem geographic position (``owtt_leader_node`` applies
``base_link``→``modem_link`` before ``$G``). This node applies the same lever
arm to its own antenna fix via tf2 before the geodesic, so auto-cal matches
the acoustic path rather than antenna-to-modem.

``sound_velocity`` defaults to 1500 m/s but is sourced live from a configurable
topic/msg type (default ``/lolo/sensors/svs`` of type ``svs_interfaces/msg/SVS``,
field ``svs``) when available. It then publishes leader position + range under
this node's namespace (so two followers on one computer do not collide)::

    owtt/<leader>/smarc/latlon
    owtt/<leader>/distance

e.g. with ``namespace:=stick_3`` and leader ``stick_1`` that becomes
``/stick_3/owtt/stick_1/distance``.
"""

import math

from rcl_interfaces.msg import ParameterDescriptor
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

import tf2_ros
from rclpy.time import Time
from rclpy.duration import Duration

from serial_ping_pkg.utils import load_yaml_config
from serial_ping_pkg.tuper_owtt import teensy_interface as ti
from serial_ping_pkg.tuper_owtt.owtt_base import WireSafeSerialNode, run_node

# WGS84 semi-major axis (m); matches owtt_leader_node lever-arm conversion.
_EARTH_RADIUS_M = 6378137.0


class OwttFollowerNode(WireSafeSerialNode):
    # A leader whose decoded GPS repeats byte-for-byte this many times in a row
    # is treated as frozen/stale (e.g. its bringup died and the Teensy keeps
    # re-broadcasting the last stored fix). We then drop BOTH its position and
    # range until it moves again, because a frozen position fused with a live,
    # changing range diverges downstream filters.
    FROZEN_REPEAT_LIMIT = 3

    def __init__(self):
        super().__init__('owtt_follower_node')

        config = load_yaml_config('serial_ping_pkg', 'tuper_owtt/tuper_owtt_config.yaml')
        owtt_cfg = config.get('owtt', {})
        follower_cfg = config.get('follower', {})
        teensy_cfg = config.get('teensy', {})

        # --- OWTT parameters ---
        self.declare_parameter('owtt.delta_prefix', owtt_cfg.get('delta_prefix', '#I'))
        self.declare_parameter('owtt.offset_us', owtt_cfg.get('offset_us', 0.0))
        self.declare_parameter('owtt.default_sound_velocity', owtt_cfg.get('default_sound_velocity', 1500.0))
        self.declare_parameter('owtt.sound_velocity_topic', owtt_cfg.get('sound_velocity_topic', '/lolo/sensors/svs'))
        self.declare_parameter('owtt.sound_velocity_msg_type', owtt_cfg.get('sound_velocity_msg_type', 'svs_interfaces/msg/SVS'))
        self.declare_parameter('owtt.sound_velocity_field', owtt_cfg.get('sound_velocity_field', 'svs'))
        # Physical validity gate: acoustic ranges are strictly positive and
        # bounded by modem reach. Anything outside is a timing transient or
        # an RxS/decode mispair on the Teensy — drop it. Default matches the
        # firmware's 3 s TOF ceiling at nominal 1500 m/s sound speed.
        self.declare_parameter('owtt.max_range_m', owtt_cfg.get('max_range_m', 4500.0))
        # Auto-calibrate offset_us per leader modem from modem-modem GPS truth:
        # offset = delta_us - range / c * 1e6. Locks after min_samples.
        self.declare_parameter('owtt.auto_calibrate', owtt_cfg.get('auto_calibrate', True))
        self.declare_parameter('owtt.calib_min_samples', owtt_cfg.get('calib_min_samples', 5))
        self.declare_parameter('owtt.calib_window', owtt_cfg.get('calib_window', 20))

        # Modem ids must tolerate however ros2 launch coerced the override
        # (str '069', int 69, or float 69.0). Declare them dynamically typed and
        # normalise on read; see teensy_interface.normalize_modem_id.
        id_desc = ParameterDescriptor(dynamic_typing=True)

        # --- Follower parameters ---
        self.declare_parameter('follower.leader_gps_msg_type', follower_cfg.get('leader_gps_msg_type', 'GeoPoint'))
        # Two leaders, routed by acoustic modem id. Names + ids overridable via launch.
        self.declare_parameter('follower.leader1_name', follower_cfg.get('leader1_name', 'leader1'))
        self.declare_parameter('follower.leader1_modem_id', follower_cfg.get('leader1_modem_id', '007'), id_desc)
        self.declare_parameter('follower.leader2_name', follower_cfg.get('leader2_name', 'leader2'))
        self.declare_parameter('follower.leader2_modem_id', follower_cfg.get('leader2_modem_id', '111'), id_desc)
        # Own GPS for auto-cal (relative -> /<namespace>/smarc/latlon).
        self.declare_parameter(
            'follower.latlon_topic',
            follower_cfg.get('latlon_topic', 'smarc/latlon'))
        # Antenna (base) → modem lever, same discovery pattern as owtt_leader_node.
        self.declare_parameter('follower.base_frame', follower_cfg.get('base_frame', ''))
        self.declare_parameter('follower.modem_frame', follower_cfg.get('modem_frame', ''))
        self.declare_parameter(
            'follower.base_link_suffix', follower_cfg.get('base_link_suffix', 'base_link'))
        self.declare_parameter(
            'follower.modem_link_suffix', follower_cfg.get('modem_link_suffix', 'modem_link'))
        self.declare_parameter('follower.world_frame', follower_cfg.get('world_frame', 'map'))

        # --- Teensy / mode parameters ---
        self.declare_parameter('teensy.own_modem_id', teensy_cfg.get('own_modem_id', '101'), id_desc)
        self.declare_parameter('teensy.command_terminator', teensy_cfg.get('command_terminator', '\r\n'))
        self.declare_parameter('teensy.mode', teensy_cfg.get('mode', 'receiver'))
        # Holdover experiment: seconds after arming ($ZIGNOREPPSAFTER=N) before
        # the Teensy stops accepting PPS and free-runs on the OCXO.
        # 0 = never (normal sticks). On start we send =0 (ack) then =N (ack)
        # so a driver restart restarts the countdown from *now*.
        self.declare_parameter('teensy.ignore_pps_after_s', teensy_cfg.get('ignore_pps_after_s', 0))
        self.IGNORE_PPS_ACK_RETRY_S = 2.0
        self._ignore_pps_phase = 'idle'  # idle | clear | arm | done
        self._ignore_pps_retry_timer = None

        self.delta_prefix = self.get_parameter('owtt.delta_prefix').get_parameter_value().string_value
        self.offset_us = self.get_parameter('owtt.offset_us').get_parameter_value().double_value
        self.max_range_m = self.get_parameter('owtt.max_range_m').get_parameter_value().double_value
        self.auto_calibrate = self.get_parameter('owtt.auto_calibrate').get_parameter_value().bool_value
        self.calib_min_samples = self.get_parameter('owtt.calib_min_samples').get_parameter_value().integer_value
        self.calib_window = self.get_parameter('owtt.calib_window').get_parameter_value().integer_value
        self._configured_offset_us = self.offset_us
        self._calib = {}  # modem_id -> {samples, offset, locked}
        self.default_sound_velocity = self.get_parameter('owtt.default_sound_velocity').get_parameter_value().double_value
        self.sound_velocity_topic = self.get_parameter('owtt.sound_velocity_topic').get_parameter_value().string_value
        self.sound_velocity_msg_type = self.get_parameter('owtt.sound_velocity_msg_type').get_parameter_value().string_value
        self.sound_velocity_field = self.get_parameter('owtt.sound_velocity_field').get_parameter_value().string_value
        self.leader_gps_msg_type = self.get_parameter('follower.leader_gps_msg_type').get_parameter_value().string_value
        self.leader1_name = self.get_parameter('follower.leader1_name').get_parameter_value().string_value
        self.leader1_modem_id = ti.normalize_modem_id(self.get_parameter('follower.leader1_modem_id').value)
        self.leader2_name = self.get_parameter('follower.leader2_name').get_parameter_value().string_value
        self.leader2_modem_id = ti.normalize_modem_id(self.get_parameter('follower.leader2_modem_id').value)
        self.latlon_topic = self.get_parameter('follower.latlon_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('follower.base_frame').get_parameter_value().string_value
        self.modem_frame = self.get_parameter('follower.modem_frame').get_parameter_value().string_value
        self.base_link_suffix = self.get_parameter(
            'follower.base_link_suffix').get_parameter_value().string_value
        self.modem_link_suffix = self.get_parameter(
            'follower.modem_link_suffix').get_parameter_value().string_value
        self.world_frame = self.get_parameter('follower.world_frame').get_parameter_value().string_value
        self.own_modem_id = ti.normalize_modem_id(self.get_parameter('teensy.own_modem_id').value)
        self.command_terminator = self.get_parameter('teensy.command_terminator').get_parameter_value().string_value
        self.mode = self.get_parameter('teensy.mode').get_parameter_value().string_value.lower()
        self.ignore_pps_after_s = self.get_parameter('teensy.ignore_pps_after_s').get_parameter_value().integer_value

        # Latest sound velocity (falls back to default until an SVS msg arrives).
        self.sound_velocity = self.default_sound_velocity
        self.own_position = None  # antenna (lat, lon) from smarc/latlon
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Lines other than config ACKs are ignored until fully initialised
        # (guards against an inbound broadcast racing robots/state setup).
        self._ready = False

        # Connect to the modem/Teensy via succorfish_driver (no direct serial).
        self.connect_driver(on_line=self._on_serial_line, wait_timeout=5.0)

        # Always leave the Teensy as a tame wire when this process exits.
        self.install_shutdown_guard()

        # Wire mode: configure the Teensy as a transparent passthrough and stay idle.
        if self.mode == 'wire':
            self.send_command(ti.build_config_command(ti.TeensyMode.WIRE, self.own_modem_id))
            self.get_logger().warn("Started in WIRE mode: Teensy is transparent, follower is passive.")
            return

        # Put the Teensy into receiver mode; retry until #Y,OK confirms apply.
        self.arm_config_retry(
            ti.build_config_command(ti.TeensyMode.RECEIVER, self.own_modem_id))

        # Holdover experiment (stick 4): clear any prior deadline, then arm N s
        # from *now*. Requires firmware that treats N as time-since-command
        # (not Teensy boot). $Z is accepted while $Y is still pending.
        self._arm_ignore_pps_experiment()

        # Resolve leader GPS message type.
        self.LeaderMsgType = self._resolve_gps_type(self.leader_gps_msg_type)

        # Build modem_id -> {name, pos_pub, dist_pub} routing for the two leaders.
        self.robots = {}
        for name, modem_id in ((self.leader1_name, self.leader1_modem_id),
                               (self.leader2_name, self.leader2_modem_id)):
            if not name or not modem_id:
                continue
            modem_id = str(modem_id).zfill(3)
            # Relative topics inherit the launch namespace (robot_name), so
            # stick_3 and stick_4 on one host each own their OWTT outputs.
            pos_topic = f"owtt/{name}/smarc/latlon"
            dist_topic = f"owtt/{name}/distance"
            self.robots[modem_id] = {
                'name': name,
                'pos_pub': self.create_publisher(self.LeaderMsgType, pos_topic, 10),
                'dist_pub': self.create_publisher(Float32, dist_topic, 10),
                # Frozen-position detection state.
                'last_pos': None,
                'identical_count': 0,
                'stale': False,
                'stale_warned': False,
            }
            self.get_logger().info(
                f"Leader {name}: modem {modem_id} -> "
                f"{self.get_namespace().rstrip('/')}/{pos_topic}, "
                f"{self.get_namespace().rstrip('/')}/{dist_topic}")

        # Pairing state: the delta line refers to the most recent broadcast.
        self.pending_modem_id = None

        # Optional live sound-velocity subscription.
        self._setup_sound_velocity_subscription()
        self._setup_own_position_subscription()

        self._ready = True
        cal_msg = (
            f"auto-cal ON (lock after {self.calib_min_samples} samples/leader)"
            if self.auto_calibrate else "auto-cal OFF (fixed offset_us)"
        )
        self.get_logger().info(
            f"OWTT follower initialised (receiver mode); {cal_msg}, "
            f"configured offset_us={self._configured_offset_us:.0f}.")

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

    def _setup_own_position_subscription(self):
        if not self.auto_calibrate or not self.latlon_topic:
            return
        self.create_subscription(GeoPoint, self.latlon_topic, self._own_latlon_callback, 10)
        self.get_logger().info(
            f"Auto-cal: subscribed to own position on {self.latlon_topic}")

    def _own_latlon_callback(self, msg):
        self.own_position = (float(msg.latitude), float(msg.longitude))

    def _tf_frame_ids(self):
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
        """Find the frame whose name is/ends with ``suffix`` (e.g. base_link).

        Prefer frames under this node's namespace when several sticks share a
        tf tree (stick_3/base_link vs stick_4/base_link).
        """
        matches = sorted(f for f in frame_ids if f == suffix or f.endswith('/' + suffix))
        if not matches:
            return ''
        ns = self.get_namespace().strip('/')
        if ns:
            preferred = [f for f in matches if f == f'{ns}/{suffix}' or f.startswith(ns + '/')]
            if preferred:
                return preferred[0]
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

    def _lever_arm_offset_enu(self):
        """ENU (dE, dN) from base origin to modem origin in the world frame.

        Same construction as owtt_leader_node: world→modem minus world→base.
        Returns None if frames/transforms are unavailable.
        """
        if not self._resolve_frames():
            return None
        try:
            t_wb = self.tf_buffer.lookup_transform(
                self.world_frame, self.base_frame, Time(), timeout=Duration(seconds=0.2))
            t_wm = self.tf_buffer.lookup_transform(
                self.world_frame, self.modem_frame, Time(), timeout=Duration(seconds=0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        dE = t_wm.transform.translation.x - t_wb.transform.translation.x
        dN = t_wm.transform.translation.y - t_wb.transform.translation.y
        return dE, dN

    def _own_modem_latlon(self):
        """Antenna fix shifted by the base→modem lever (rotated into world ENU)."""
        if self.own_position is None:
            return None
        lat, lon = self.own_position
        offset = self._lever_arm_offset_enu()
        if offset is None:
            # Fall back to antenna lat/lon rather than dropping auto-cal entirely.
            self.get_logger().warn(
                "tf lever unavailable; auto-cal using antenna lat/lon "
                "(not modem). Check base_link/modem_link in the tf tree.",
                throttle_duration_sec=10.0)
            return lat, lon
        dE, dN = offset
        dlat = math.degrees(dN / _EARTH_RADIUS_M)
        dlon = math.degrees(dE / (_EARTH_RADIUS_M * math.cos(math.radians(lat))))
        return lat + dlat, lon + dlon

    def _resolve_offset(self, modem_id, delta_us, leader_pos):
        """Return (offset_us, source) for this leader modem pair.

        When auto-cal is on and both ends have GPS, derive a per-sample offset
        from the true modem-modem horizontal range and fold into a running
        median. Lock after ``calib_min_samples``; otherwise fall back to
        configured offset.
        """
        entry = self._calib.get(modem_id)
        c = self.sound_velocity
        own_modem = self._own_modem_latlon()
        if (self.auto_calibrate and leader_pos is not None
                and own_modem is not None and c > 0.0):
            slant = self._geodesic_m(own_modem, leader_pos)
            sample = float(delta_us) - (slant / c) * 1e6
            if entry is None:
                entry = {'samples': [], 'offset': None, 'locked': False}
                self._calib[modem_id] = entry
            if not entry['locked']:
                entry['samples'].append(sample)
                if len(entry['samples']) > self.calib_window:
                    entry['samples'] = entry['samples'][-self.calib_window:]
                entry['offset'] = self._median(entry['samples'])
                name = self.robots.get(modem_id, {}).get('name', modem_id)
                if len(entry['samples']) >= self.calib_min_samples:
                    entry['locked'] = True
                    self.get_logger().info(
                        f"Auto-calibrated offset for pair (self {self.own_modem_id} "
                        f"<-> {name}/{modem_id}): {entry['offset']:.0f} us from "
                        f"{len(entry['samples'])} modem-modem GPS-truth samples "
                        f"(configured was {self._configured_offset_us:.0f} us). Locked.")
                else:
                    self.get_logger().info(
                        f"Calibrating offset for {name}/{modem_id}: "
                        f"{len(entry['samples'])}/{self.calib_min_samples} samples, "
                        f"provisional {entry['offset']:.0f} us "
                        f"(true modem range {slant:.1f} m).", throttle_duration_sec=2.0)
        if entry is not None and entry['offset'] is not None:
            return entry['offset'], ('auto-locked' if entry['locked'] else 'auto-provisional')
        return self._configured_offset_us, 'config'

    @staticmethod
    def _median(xs):
        s = sorted(xs)
        n = len(s)
        if n == 0:
            return 0.0
        mid = n // 2
        return s[mid] if n % 2 else 0.5 * (s[mid - 1] + s[mid])

    @staticmethod
    def _geodesic_m(a, b):
        """Distance (m) between (lat, lon) pairs; equirectangular, fine for <km."""
        lat1, lon1 = a
        lat2, lon2 = b
        mean_lat = math.radians((lat1 + lat2) * 0.5)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1) * math.cos(mean_lat)
        return 6371000.0 * math.hypot(dlat, dlon)

    # ------------------------------------------------------------------ runtime

    def _on_serial_line(self, line):
        line = line.strip()
        if not line:
            return
        # Config ACKs must be handled even before _ready (fast #Y,OK during init).
        if self.handle_config_line(line):
            return
        if self._handle_ignore_pps_ack(line):
            return
        if not getattr(self, '_ready', False):
            return
        self.handle_line(line)

    def _arm_ignore_pps_experiment(self):
        """Send $ZIGNOREPPSAFTER=0, wait for ack, then =N and wait for ack."""
        if self.ignore_pps_after_s <= 0:
            self._ignore_pps_phase = 'idle'
            return
        self._ignore_pps_phase = 'clear'
        self.send_command('$ZIGNOREPPSAFTER=0')
        self.get_logger().warn(
            f"Holdover experiment: clearing PPS-ignore, then arming "
            f"{self.ignore_pps_after_s} s from now (OCXO free-run after that).")
        if self._ignore_pps_retry_timer is not None:
            self._ignore_pps_retry_timer.cancel()
            self.destroy_timer(self._ignore_pps_retry_timer)
        self._ignore_pps_retry_timer = self.create_timer(
            self.IGNORE_PPS_ACK_RETRY_S, self._retry_ignore_pps_if_needed)

    def _retry_ignore_pps_if_needed(self):
        if self._ignore_pps_phase == 'clear':
            self.get_logger().warn("No #IGNOREPPSAFTER,0 yet; retrying clear.")
            self.send_command('$ZIGNOREPPSAFTER=0')
        elif self._ignore_pps_phase == 'arm':
            cmd = f'$ZIGNOREPPSAFTER={self.ignore_pps_after_s}'
            self.get_logger().warn(
                f"No #IGNOREPPSAFTER,{self.ignore_pps_after_s} yet; retrying arm.")
            self.send_command(cmd)

    def _handle_ignore_pps_ack(self, line):
        """Advance clear→arm→done on ``#IGNOREPPSAFTER,<n>``. Returns True if handled."""
        if not line.startswith('#IGNOREPPSAFTER,'):
            return False
        payload = line.split(',', 1)[1].strip()
        if payload == 'ERROR' or self.ignore_pps_after_s <= 0:
            self.get_logger().warn(f"Ignore-PPS ACK unexpected: {line}")
            return True
        try:
            value = int(payload)
        except ValueError:
            self.get_logger().warn(f"Ignore-PPS ACK parse failed: {line}")
            return True

        if self._ignore_pps_phase == 'clear' and value == 0:
            self._ignore_pps_phase = 'arm'
            self.get_logger().info("PPS-ignore cleared (#IGNOREPPSAFTER,0); arming countdown.")
            self.send_command(f'$ZIGNOREPPSAFTER={self.ignore_pps_after_s}')
            return True

        if self._ignore_pps_phase == 'arm' and value == self.ignore_pps_after_s:
            self._ignore_pps_phase = 'done'
            if self._ignore_pps_retry_timer is not None:
                self._ignore_pps_retry_timer.cancel()
                self.destroy_timer(self._ignore_pps_retry_timer)
                self._ignore_pps_retry_timer = None
            self.get_logger().warn(
                f"Holdover armed: Teensy will ignore PPS in "
                f"{self.ignore_pps_after_s} s from now.")
            return True

        self.get_logger().info(f"Ignore-PPS status: {line}")
        return True

    def handle_line(self, line):
        # Leader position broadcast.
        broadcast = ti.parse_broadcast(line)
        if broadcast is not None:
            modem_id, lat, lon = broadcast
            self.pending_modem_id = modem_id
            self.publish_position(modem_id, lat, lon)
            return

        # Absolute #I TOF -> range, paired with the most recent broadcast.
        delta_us = ti.parse_owtt_delta(line, self.delta_prefix)
        if delta_us is not None:
            self.publish_range(delta_us)
            return

        # Anything else (e.g. classic Succorfish responses) is just logged.
        self.get_logger().debug(f"<- Teensy (unhandled): {line}")

    def publish_position(self, modem_id, lat, lon):
        robot = self.robots.get(modem_id)
        if robot is None:
            self.get_logger().warn(f"Broadcast from unknown modem_id {modem_id}, ignoring.")
            return

        # Frozen-position guard: count byte-for-byte repeats of this leader's GPS.
        pos = (lat, lon)
        if pos == robot['last_pos']:
            robot['identical_count'] += 1
        else:
            if robot['stale']:
                self.get_logger().info(
                    f"[{robot['name']}] position updating again; resuming position + range.")
            robot['identical_count'] = 1
            robot['stale'] = False
            robot['stale_warned'] = False
        robot['last_pos'] = pos

        if robot['identical_count'] >= self.FROZEN_REPEAT_LIMIT:
            robot['stale'] = True

        if robot['stale']:
            if not robot['stale_warned']:
                self.get_logger().warn(
                    f"[{robot['name']}] GPS position frozen (identical for "
                    f"{robot['identical_count']} broadcasts: lat={lat}, lon={lon}); "
                    f"suppressing position AND range until it changes "
                    f"(leader bringup / GPS feed likely died).")
                robot['stale_warned'] = True
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
        modem_id = self.pending_modem_id
        robot = self.robots.get(modem_id)
        if robot is None:
            self.get_logger().warn(
                f"OWTT delta for unknown modem_id {modem_id}, ignoring.")
            return
        if robot['stale']:
            self.get_logger().debug(
                f"[{robot['name']}] range suppressed: leader position is frozen/stale.")
            return
        offset_us, offset_src = self._resolve_offset(
            modem_id, delta_us, robot.get('last_pos'))
        rng = ti.delta_to_range_m(delta_us, offset_us, self.sound_velocity)
        if not (0.0 < rng <= self.max_range_m):
            self.get_logger().warn(
                f"[{robot['name']}] dropping unphysical range {rng:.3f} m "
                f"(delta={delta_us} us, offset={offset_us:.0f} us [{offset_src}]) "
                f"— timing transient on TX or RX stick.")
            return
        msg = Float32()
        msg.data = float(rng)
        robot['dist_pub'].publish(msg)
        self.get_logger().info(
            f"[{robot['name']}] delta={delta_us} us, offset={offset_us:.0f} us "
            f"({offset_src}), c={self.sound_velocity} m/s -> range={rng:.3f} m")

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
