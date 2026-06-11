#!/usr/bin/python3

import math
import re
from enum import Enum

import rclpy
from rclpy.time import Time
from rclpy.duration import Duration

from geographic_msgs.msg import GeoPoint, GeoPointStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray

from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

from smarc_action_base.gentler_action_server import GentlerActionServer
from smarc_utilities.georef_utils import convert_latlon_to_utm

from serial_ping_pkg.utils import load_yaml_config, init_serial
from serial_ping_pkg.tuper_owtt.owtt_base import WireSafeSerialNode, run_node
from serial_ping_pkg.tuper_owtt import teensy_interface as ti


class PingState(Enum):
    IDLE = 0
    RUNNING = 1
    DONE = 2


class ModemPingEstimatorNode(WireSafeSerialNode):
    """
    Maintains a list of acoustic modem ids, pings them on request, and republishes
    good-enough position estimates as GeoPointStamped.

    Expected estimator input:
      sensor_msgs/NavSatFix
      msg.header.frame_id = "modem_007" or "007"
      msg.position_covariance gives uncertainty.
    """

    def __init__(self):
        super().__init__('modem_ping_estimator_node')

        config = load_yaml_config('serial_ping_pkg', 'ping_estimator_config.yaml')
        serial_cfg = config.get('serial', {})
        teensy_cfg = config.get('teensy', {})
        ping_cfg = config.get('ping', {})


        # ---------------- parameters ----------------
        self.declare_parameter('serial.port', serial_cfg.get('port', '/dev/ttyACM0'))
        self.declare_parameter('serial.port_fallback', serial_cfg.get('port_fallback', '/dev/ttyACM1'))
        self.declare_parameter('serial.baudrate', serial_cfg.get('baudrate', 115200))

        self.declare_parameter('teensy.own_modem_id', teensy_cfg.get('own_modem_id', '101'))
        self.declare_parameter('teensy.command_terminator', teensy_cfg.get('command_terminator', '\r\n'))

        self.declare_parameter('estimate_topic', '/modem_estimator/fix')
        self.declare_parameter('estimate_max_uncertainty_m', 4.0)
        self.declare_parameter('require_successful_ping_before_publish', True)

        self.declare_parameter('ping_response_timeout_s', 5.0)
        self.declare_parameter('default_retry_count', 3)
        self.declare_parameter('default_task_timeout_s', 60.0)

        self.declare_parameter('map_frame', 'M350/map')
        self.declare_parameter('marker_topic', '/modem_estimates/rviz')
        self.declare_parameter('geopoint_topic', '/modem_estimates/geopoint')

        self.port = self.get_parameter('serial.port').get_parameter_value().string_value
        self.port_fallback = self.get_parameter('serial.port_fallback').get_parameter_value().string_value
        self.baudrate = self.get_parameter('serial.baudrate').get_parameter_value().integer_value

        self.own_modem_id = self.get_parameter('teensy.own_modem_id').get_parameter_value().string_value
        self.command_terminator = self.get_parameter('teensy.command_terminator').get_parameter_value().string_value

        self.estimate_topic = self.get_parameter('estimate_topic').get_parameter_value().string_value
        self.max_uncertainty_m = self.get_parameter('estimate_max_uncertainty_m').get_parameter_value().double_value
        self.require_ping_success = self.get_parameter(
            'require_successful_ping_before_publish'
        ).get_parameter_value().bool_value

        self.ping_response_timeout_s = self.get_parameter('ping_response_timeout_s').get_parameter_value().double_value
        self.default_retry_count = self.get_parameter('default_retry_count').get_parameter_value().integer_value
        self.default_task_timeout_s = self.get_parameter('default_task_timeout_s').get_parameter_value().double_value

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.geopoint_topic = self.get_parameter('geopoint_topic').get_parameter_value().string_value

        self.declare_parameter('ping.max_retries', ping_cfg.get('max_retries', 3))
        self.declare_parameter('ping.response_timeout_s', ping_cfg.get('response_timeout_s', 5.0))
        self.declare_parameter('ping.task_timeout_s', ping_cfg.get('task_timeout_s', 60.0))

        self.default_retry_count = self.get_parameter('ping.max_retries').value
        self.ping_response_timeout_s = self.get_parameter('ping.response_timeout_s').value
        self.default_task_timeout_s = self.get_parameter('ping.task_timeout_s').value

        # ---------------- serial ----------------
        self.ser = init_serial(self.port, self.port_fallback, self.baudrate, self.get_logger())
        if self.ser is None:
            rclpy.shutdown()
            return

        self.install_shutdown_guard()

        # This node uses classic Succorfish commands, so put Teensy in wire mode.
        self.send_command(ti.build_config_command(ti.TeensyMode.WIRE, self.own_modem_id))

        # ---------------- state ----------------
        self.modem_ids: list[str] = []

        self.state = PingState.IDLE
        self.action_done = True
        self.status_msg = "idle"

        self.ping_queue: list[str] = []
        self.attempts_left: dict[str, int] = {}
        self.ping_success: set[str] = set()
        self.ping_failed: set[str] = set()

        self.waiting_for_id: str | None = None
        self.ping_deadline = None
        self.task_deadline = None

        self.serial_buffer = ""

        # Latest good estimates, keyed by modem id.
        self.good_estimates: dict[str, GeoPointStamped] = {}
        self.estimate_uncertainty: dict[str, float] = {}

        # tf / visualization
        self.utm_frame: str | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- ROS I/O ----------------
        self.estimate_sub = self.create_subscription(
            NavSatFix,
            self.estimate_topic,
            self._estimate_cb,
            10,
        )

        self.geopoint_pub = self.create_publisher(GeoPointStamped, self.geopoint_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        # One action server with three modes: add/remove/ping.
        self.start_as = GentlerActionServer(
            self,
            "smarc_modem_ping",
            self._on_goal_received,
            self._on_cancel_received,
            self._tick,
            self._action_is_done,
            self._feedback,
            loop_frequency=10,
        )

        # Stop action: drops all modem ids.
        self.stop_as = GentlerActionServer(
            self,
            "smarc_stop_modem_ping",
            self._on_goal_received_stop,
            lambda: True,
            lambda: None,
            lambda: True,
            lambda: "modem ping stopped",
            loop_frequency=5,
        )

        self.get_logger().info(
            f"modem_ping_estimator_node ready. estimate_topic={self.estimate_topic}, "
            f"output={self.geopoint_topic}, markers={self.marker_topic}"
        )

    # ------------------------------------------------------------------ actions

    def _on_goal_received(self, goal_request: dict) -> bool:
        mode = str(goal_request.get('mode', '')).lower().strip()

        if mode == 'add':
            modem_id = self._parse_goal_modem_id(goal_request)
            if modem_id is None:
                return False

            if modem_id not in self.modem_ids:
                self.modem_ids.append(modem_id)
                self.modem_ids.sort()

            self.status_msg = f"added modem {modem_id}; list={self.modem_ids}"
            self.action_done = True
            self.get_logger().info(self.status_msg)
            return True

        if mode == 'remove':
            modem_id = self._parse_goal_modem_id(goal_request)
            if modem_id is None:
                return False

            if modem_id in self.modem_ids:
                self.modem_ids.remove(modem_id)

            self.good_estimates.pop(modem_id, None)
            self.estimate_uncertainty.pop(modem_id, None)
            self.ping_success.discard(modem_id)
            self.ping_failed.discard(modem_id)
            self.attempts_left.pop(modem_id, None)

            if self.waiting_for_id == modem_id:
                self.waiting_for_id = None
                self.ping_deadline = None

            self.ping_queue = [mid for mid in self.ping_queue if mid != modem_id]

            self._delete_marker_for_modem(modem_id)

            self.status_msg = f"removed modem {modem_id}; list={self.modem_ids}"
            self.action_done = True
            self.get_logger().info(self.status_msg)
            return True

        if mode in ('clear', 'clear_all', 'remove_all'):
            self._clear_all_modems("cleared all modem ids")
            return True

        if mode == 'ping':
            if not self.modem_ids:
                self.status_msg = "no modem ids to ping"
                self.action_done = True
                self.get_logger().warn(self.status_msg)
                return True

            retry_count = int(goal_request.get('retry_count', self.default_retry_count))
            task_timeout_s = float(goal_request.get('task_timeout_s', self.default_task_timeout_s))

            self._start_ping_task(
                retry_count=retry_count,
                task_timeout_s=task_timeout_s
            )
            return True

        self.get_logger().error(f"Unknown modem ping action mode: {mode!r}")
        return False
    
    def _clear_all_modems(self, reason: str = "cleared all modem ids"):
        self._cancel_ping_task(reason)

        self.modem_ids = []
        self.good_estimates.clear()
        self.estimate_uncertainty.clear()

        self.ping_queue = []
        self.attempts_left = {}
        self.ping_success.clear()
        self.ping_failed.clear()
        self.waiting_for_id = None
        self.ping_deadline = None
        self.task_deadline = None

        self._delete_all_markers()

        self.state = PingState.IDLE
        self.action_done = True
        self.status_msg = reason

        self.get_logger().info(reason)
        
    def _on_cancel_received(self):
        self._cancel_ping_task("cancelled")
        return True

    def _on_goal_received_stop(self, goal_request: dict) -> bool:
        self._clear_all_modems("stop action: cleared all modem ids")
        return True

    def _action_is_done(self) -> bool:
        return self.action_done

    def _feedback(self) -> str:
        return self.status_msg

    # ------------------------------------------------------------------ ping task

    def _start_ping_task(self, retry_count: int, task_timeout_s: float):
        # retry_count means retries after the first attempt.
        # attempts_left = retry_count + 1.
        attempts = max(1, retry_count + 1)

        self.ping_queue = list(self.modem_ids)
        self.attempts_left = {mid: attempts for mid in self.modem_ids}
        self.ping_success = set()
        self.ping_failed = set()

        self.waiting_for_id = None
        self.ping_deadline = None
        self.task_deadline = self.get_clock().now() + Duration(seconds=task_timeout_s)

        self.state = PingState.RUNNING
        self.action_done = False
        self.status_msg = f"pinging {self.modem_ids}, retries={retry_count}, timeout={task_timeout_s}s"
        self.get_logger().info(self.status_msg)

    def _cancel_ping_task(self, reason: str):
        self.state = PingState.IDLE
        self.action_done = True
        self.waiting_for_id = None
        self.ping_queue = []
        self.attempts_left = {}
        self.status_msg = reason

    def _tick(self):
        self._read_serial()

        if self.state != PingState.RUNNING:
            return

        now = self.get_clock().now()

        if self.task_deadline is not None and now > self.task_deadline:
            self.state = PingState.DONE
            self.action_done = True
            self.status_msg = (
                f"ping task timeout. success={sorted(self.ping_success)}, "
                f"failed={sorted(self.ping_failed)}, remaining={self.ping_queue}"
            )
            self.get_logger().warn(self.status_msg)
            return

        if self.waiting_for_id is not None:
            if self.ping_deadline is not None and now > self.ping_deadline:
                self._handle_ping_timeout(self.waiting_for_id)
            return

        # Done when everyone either succeeded or exhausted retries.
        completed = self.ping_success | self.ping_failed
        if set(self.modem_ids).issubset(completed):
            self.state = PingState.DONE
            self.action_done = True
            self.status_msg = (
                f"ping complete. success={sorted(self.ping_success)}, "
                f"failed={sorted(self.ping_failed)}"
            )
            if self.ping_failed:
                self.get_logger().warn(self.status_msg)
            else:
                self.get_logger().info(self.status_msg)
            return

        if self.ping_queue:
            next_id = self.ping_queue.pop(0)
            if next_id in completed:
                return
            self._send_ping(next_id)

    def _send_ping(self, modem_id: str):
        cmd = f"$P{modem_id}"
        self.send_command(cmd)

        self.waiting_for_id = modem_id
        self.ping_deadline = self.get_clock().now() + Duration(seconds=self.ping_response_timeout_s)
        remaining = self.attempts_left.get(modem_id, 0)
        self.status_msg = f"pinging {modem_id}, attempts_left={remaining}"
        self.get_logger().info(self.status_msg)

    def _handle_ping_success(self, modem_id: str):
        self.ping_success.add(modem_id)
        self.waiting_for_id = None
        self.ping_deadline = None
        self.status_msg = f"ping success {modem_id}"
        self.get_logger().info(self.status_msg)

    def _handle_ping_timeout(self, modem_id: str):
        self.attempts_left[modem_id] = self.attempts_left.get(modem_id, 0) - 1

        if self.attempts_left[modem_id] > 0:
            self.get_logger().warn(
                f"ping timeout {modem_id}; retrying, attempts_left={self.attempts_left[modem_id]}"
            )
            self.ping_queue.append(modem_id)
        else:
            self.get_logger().error(f"ping failed {modem_id}; retries exhausted")
            self.ping_failed.add(modem_id)

        self.waiting_for_id = None
        self.ping_deadline = None

    # ------------------------------------------------------------------ serial

    def _read_serial(self):
        try:
            if self.ser.in_waiting <= 0:
                return

            data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
            self.serial_buffer += data
            self.serial_buffer = self.serial_buffer.replace('\r\n', '\n')

            while '\n' in self.serial_buffer:
                line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                line = line.strip()
                if line:
                    self._handle_serial_line(line)

        except Exception as e:
            self.get_logger().error(f"serial read error: {e}")

    def _handle_serial_line(self, line: str):
        self.get_logger().debug(f"<- serial: {line}")

        # Local echo/ack from ping command; ignore.
        # Example: $P007
        if line.startswith('$P'):
            return

        # Success:
        #   #R007T12345
        if line.startswith('#R') and len(line) >= 5:
            modem_id = line[2:5]
            if self.waiting_for_id == modem_id:
                self._handle_ping_success(modem_id)
            else:
                self.get_logger().warn(
                    f"got range response for {modem_id}, but waiting_for={self.waiting_for_id}"
                )
            return

        # Timeout from modem:
        #   #TO
        # It does not include id, so assign it to current waiting id.
        if line == '#TO':
            if self.waiting_for_id is not None:
                self._handle_ping_timeout(self.waiting_for_id)
            else:
                self.get_logger().warn("got #TO but not waiting for any modem")
            return

        # Modem address/config confirms from Teensy or classic modem lines.
        if line.startswith('#A'):
            self.get_logger().info(f"config/status line: {line}")
            return

        self.get_logger().debug(f"unhandled serial line: {line}")

    # ------------------------------------------------------------------ estimates

    def _estimate_cb(self, msg: NavSatFix):
        modem_id = self._modem_id_from_frame(msg.header.frame_id)
        if modem_id is None:
            self.get_logger().warn(
                f"estimate message has no modem id in frame_id={msg.header.frame_id!r}",
                throttle_duration_sec=5.0,
            )
            return

        if modem_id not in self.modem_ids:
            return

        if self.require_ping_success and modem_id not in self.ping_success:
            return

        sigma = self._estimate_uncertainty_m(msg)
        if sigma is None:
            self.get_logger().warn(
                f"estimate for modem {modem_id} has unknown covariance; ignoring",
                throttle_duration_sec=5.0,
            )
            return

        if sigma > self.max_uncertainty_m:
            self.get_logger().debug(
                f"estimate for modem {modem_id} too uncertain: {sigma:.2f} m"
            )
            return

        out = GeoPointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = f"modem_{modem_id}"
        out.position.latitude = float(msg.latitude)
        out.position.longitude = float(msg.longitude)
        out.position.altitude = float(msg.altitude)

        self.good_estimates[modem_id] = out
        self.estimate_uncertainty[modem_id] = sigma

        self.geopoint_pub.publish(out)
        self._publish_marker_for_estimate(modem_id, out, sigma)

        self.get_logger().info(
            f"published good estimate for modem {modem_id}: "
            f"lat={out.position.latitude:.8f}, lon={out.position.longitude:.8f}, sigma={sigma:.2f} m"
        )

    def _estimate_uncertainty_m(self, msg: NavSatFix) -> float | None:
        # NavSatFix covariance layout:
        # [xx, xy, xz,
        #  yx, yy, yz,
        #  zx, zy, zz]
        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return None

        var_x = float(msg.position_covariance[0])
        var_y = float(msg.position_covariance[4])

        if var_x < 0.0 or var_y < 0.0:
            return None

        # Conservative horizontal 1-sigma.
        return math.sqrt(max(var_x, var_y))

    def _modem_id_from_frame(self, frame_id: str) -> str | None:
        # Accept "007", "modem_007", "modem/007", etc.
        m = re.search(r'(\d{3})', frame_id or '')
        if not m:
            return None
        modem_id = m.group(1)
        value = int(modem_id)
        if value < 0 or value > 255:
            return None
        return modem_id

    def _parse_goal_modem_id(self, goal_request: dict) -> str | None:
        raw = goal_request.get('modem_id', None)
        if raw is None:
            self.get_logger().error("goal missing modem_id")
            return None

        try:
            value = int(raw)
        except (TypeError, ValueError):
            self.get_logger().error(f"bad modem_id: {raw!r}")
            return None

        if value < 0 or value > 255:
            self.get_logger().error(f"modem_id out of range: {value}")
            return None

        return f"{value:03d}"

    # ------------------------------------------------------------------ markers

    def _publish_marker_for_estimate(self, modem_id: str, estimate: GeoPointStamped, sigma: float):
        map_point = self._geopoint_to_map_point(estimate.position)
        if map_point is None:
            return

        sphere = Marker()
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.header.frame_id = self.map_frame
        sphere.ns = "modem_estimates"
        sphere.id = int(modem_id) * 2
        sphere.type = Marker.SPHERE
        sphere.action = Marker.MODIFY
        sphere.pose.position = map_point
        sphere.pose.orientation.w = 1.0

        # Show uncertainty footprint roughly as diameter.
        diameter = max(1.0, 2.0 * sigma)
        sphere.scale.x = diameter
        sphere.scale.y = diameter
        sphere.scale.z = 1.0

        sphere.color.r = 0.0
        sphere.color.g = 0.4
        sphere.color.b = 1.0
        sphere.color.a = 0.7

        text = Marker()
        text.header.stamp = sphere.header.stamp
        text.header.frame_id = self.map_frame
        text.ns = "modem_estimates_text"
        text.id = int(modem_id) * 2 + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.MODIFY
        text.pose.position.x = map_point.x
        text.pose.position.y = map_point.y
        text.pose.position.z = map_point.z + 2.0
        text.pose.orientation.w = 1.0
        text.scale.z = 1.5
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = f"modem {modem_id}\nσ={sigma:.1f}m"

        self.marker_pub.publish(MarkerArray(markers=[sphere, text]))

    def _delete_marker_for_modem(self, modem_id: str):
        markers = []
        for marker_id in (int(modem_id) * 2, int(modem_id) * 2 + 1):
            m = Marker()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.map_frame
            m.id = marker_id
            m.action = Marker.DELETE
            markers.append(m)
        self.marker_pub.publish(MarkerArray(markers=markers))

    def _delete_all_markers(self):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.map_frame
        m.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[m]))

    def _geopoint_to_map_point(self, gp: GeoPoint) -> Point | None:
        try:
            ps = convert_latlon_to_utm(gp)
            if self.utm_frame is None:
                self.utm_frame = ps.header.frame_id
                self.get_logger().info(f"UTM frame set to {self.utm_frame}")

            tf = self.tf_buffer.lookup_transform(self.map_frame, ps.header.frame_id, Time())
            transformed = do_transform_point(ps, tf)
            return transformed.point

        except Exception as e:
            self.get_logger().warn(
                f"could not transform modem estimate to {self.map_frame}: {e}",
                throttle_duration_sec=5.0,
            )
            return None


def main(args=None):
    run_node(ModemPingEstimatorNode, args=args)


if __name__ == '__main__':
    main()