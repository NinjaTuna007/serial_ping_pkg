#!/usr/bin/python3

import math
import re
from enum import Enum

import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration

from geographic_msgs.msg import GeoPoint, GeoPointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from geodesy import utm as geo_utm
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


class LeastSquaresRangeEstimator:
    """
    Sliding-window range-only nonlinear least-squares estimator.

    Measurement model:
        range_i = || p_remote - p_own_i ||

    If z_remote is known, only x,y are estimated and z is fixed.
    If z_remote is None, full x,y,z are estimated.
    """

    def __init__(
        self,
        modem_id: str,
        z_remote: float | None,
        max_measurements: int,
        range_sigma_m: float,
        remote_depth_sigma_m: float,
        damping: float,
        max_iterations: int,
        outlier_gate_m: float,
    ):
        self.modem_id = modem_id
        self.z_remote = z_remote
        self.max_measurements = max_measurements
        self.range_sigma_m = range_sigma_m
        self.remote_depth_sigma_m = remote_depth_sigma_m
        self.damping = damping
        self.max_iterations = max_iterations
        self.outlier_gate_m = outlier_gate_m

        self.measurements: list[tuple[np.ndarray, float]] = []
        self.last_xyz: np.ndarray | None = None
        self.last_cov: np.ndarray | None = None

    def add_measurement(self, own_xyz: np.ndarray, range_m: float):
        self.measurements.append((np.asarray(own_xyz, dtype=float), float(range_m)))
        self.measurements = self.measurements[-self.max_measurements:]

    def estimate(self, min_measurements: int):
        if len(self.measurements) < min_measurements:
            return None, None

        anchors = np.asarray([m[0] for m in self.measurements], dtype=float)
        ranges = np.asarray([m[1] for m in self.measurements], dtype=float)

        result = self._solve(anchors, ranges)
        if result is None:
            return None, None

        xyz, cov, residuals = result

        # One-pass outlier rejection. Keep it conservative; this is for obvious
        # bad ranges, not heavy robust optimization.
        if self.outlier_gate_m > 0.0 and len(residuals) >= min_measurements + 2:
            keep = np.abs(residuals) <= self.outlier_gate_m
            if np.count_nonzero(keep) >= min_measurements and not np.all(keep):
                result2 = self._solve(anchors[keep], ranges[keep])
                if result2 is not None:
                    xyz, cov, residuals = result2

        self.last_xyz = xyz
        self.last_cov = cov
        return xyz, cov

    def _solve(self, anchors: np.ndarray, ranges: np.ndarray):
        known_z = self.z_remote is not None

        if self.last_xyz is not None:
            x = self.last_xyz[:2].copy() if known_z else self.last_xyz.copy()
        else:
            x = np.mean(anchors[:, :2], axis=0) if known_z else np.mean(anchors, axis=0)

        J = None
        residuals = None

        for _ in range(self.max_iterations):
            residuals_list = []
            jac_rows = []

            for a, r_meas in zip(anchors, ranges):
                if known_z:
                    dx = x[0] - a[0]
                    dy = x[1] - a[1]
                    dz = float(self.z_remote) - a[2]
                    pred = math.sqrt(dx * dx + dy * dy + dz * dz)
                    pred = max(pred, 1e-6)

                    residuals_list.append(pred - r_meas)
                    jac_rows.append([dx / pred, dy / pred])
                else:
                    d = x - a
                    pred = max(float(np.linalg.norm(d)), 1e-6)

                    residuals_list.append(pred - r_meas)
                    jac_rows.append((d / pred).tolist())

            residuals = np.asarray(residuals_list, dtype=float)
            J = np.asarray(jac_rows, dtype=float)

            H = J.T @ J + self.damping * np.eye(J.shape[1])
            g = J.T @ residuals

            try:
                step = -np.linalg.solve(H, g)
            except np.linalg.LinAlgError:
                return None

            x = x + step

            if np.linalg.norm(step) < 1e-3:
                break

        if J is None or residuals is None:
            return None

        try:
            cov_state = (self.range_sigma_m ** 2) * np.linalg.inv(J.T @ J)
        except np.linalg.LinAlgError:
            return None

        if known_z:
            xyz = np.array([x[0], x[1], float(self.z_remote)], dtype=float)
            cov = np.zeros((3, 3), dtype=float)
            cov[:2, :2] = cov_state
            cov[2, 2] = self.remote_depth_sigma_m ** 2
        else:
            xyz = np.asarray(x, dtype=float)
            cov = cov_state

        return xyz, cov, residuals


class EkfRangeEstimator:
    """
    Static-beacon EKF for range-only measurements.

    State:
        x = [remote_x, remote_y, remote_z]

    If z_remote is known, z is clamped after every update.
    EKF is initialized from a bootstrap least-squares estimator once enough
    measurements exist. This avoids the usual garbage EKF initialization problem.
    """

    def __init__(
        self,
        modem_id: str,
        z_remote: float | None,
        range_sigma_m: float,
        remote_depth_sigma_m: float,
        process_noise_std_m: float,
        initial_sigma_xy_m: float,
        initial_sigma_z_m: float,
        bootstrap_max_measurements: int,
        bootstrap_damping: float,
        bootstrap_iterations: int,
    ):
        self.modem_id = modem_id
        self.z_remote = z_remote
        self.range_sigma_m = range_sigma_m
        self.remote_depth_sigma_m = remote_depth_sigma_m
        self.process_noise_std_m = process_noise_std_m
        self.initial_sigma_xy_m = initial_sigma_xy_m
        self.initial_sigma_z_m = initial_sigma_z_m

        self.x: np.ndarray | None = None
        self.P: np.ndarray | None = None
        self.measurement_count = 0

        self.bootstrap = LeastSquaresRangeEstimator(
            modem_id=modem_id,
            z_remote=z_remote,
            max_measurements=bootstrap_max_measurements,
            range_sigma_m=range_sigma_m,
            remote_depth_sigma_m=remote_depth_sigma_m,
            damping=bootstrap_damping,
            max_iterations=bootstrap_iterations,
            outlier_gate_m=0.0,
        )

    def add_measurement(self, own_xyz: np.ndarray, range_m: float):
        own_xyz = np.asarray(own_xyz, dtype=float)
        range_m = float(range_m)

        self.bootstrap.add_measurement(own_xyz, range_m)
        self.measurement_count += 1

        if self.x is None:
            xyz, cov = self.bootstrap.estimate(min_measurements=4)
            if xyz is None:
                return
            self._initialize(xyz)

        assert self.x is not None
        assert self.P is not None

        # Static random-walk prediction.
        q = self.process_noise_std_m ** 2
        self.P = self.P + q * np.eye(3)

        if self.z_remote is not None:
            self.x[2] = float(self.z_remote)

        d = self.x - own_xyz
        pred = max(float(np.linalg.norm(d)), 1e-6)

        H = (d / pred).reshape(1, 3)
        R = np.array([[self.range_sigma_m ** 2]], dtype=float)

        y = np.array([[range_m - pred]], dtype=float)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + (K @ y).reshape(3)
        self.P = (np.eye(3) - K @ H) @ self.P

        if self.z_remote is not None:
            self.x[2] = float(self.z_remote)
            self.P[2, :] = 0.0
            self.P[:, 2] = 0.0
            self.P[2, 2] = self.remote_depth_sigma_m ** 2

    def _initialize(self, xyz: np.ndarray):
        self.x = np.asarray(xyz, dtype=float)
        self.P = np.diag([
            self.initial_sigma_xy_m ** 2,
            self.initial_sigma_xy_m ** 2,
            self.initial_sigma_z_m ** 2,
        ]).astype(float)

        if self.z_remote is not None:
            self.x[2] = float(self.z_remote)
            self.P[2, 2] = self.remote_depth_sigma_m ** 2

    def estimate(self, min_measurements: int):
        if self.x is None or self.P is None:
            return None, None
        if self.measurement_count < min_measurements:
            return None, None
        return self.x.copy(), self.P.copy()


class ModemPingEstimatorNode(WireSafeSerialNode):
    """
    Maintains a list of acoustic modem ids, pings them, converts two-way travel
    time into range, and estimates underwater modem positions.

    Action server modes:
      {"mode": "add", "modem_id": "007", "depth_m": 12.0}
      {"mode": "remove", "modem_id": "007"}
      {"mode": "clear"}
      {"mode": "ping", "retry_count": 3, "task_timeout_s": 180.0}

    Output:
      geographic_msgs/GeoPointStamped
      header.frame_id = "modem_007"
      position.latitude / longitude / altitude
    """

    def __init__(self):
        super().__init__('modem_ping_estimator_node')

        config = load_yaml_config('serial_ping_pkg', 'ping_estimator_config.yaml')
        serial_cfg = config.get('serial', {})
        teensy_cfg = config.get('teensy', {})
        ping_cfg = config.get('ping', {})
        estimator_cfg = config.get('estimator', {})
        ls_cfg = estimator_cfg.get('least_squares', {})
        ekf_cfg = estimator_cfg.get('ekf', {})
        topics_cfg = config.get('topics', {})

        # ---------------- parameters ----------------
        self.declare_parameter('serial.port', serial_cfg.get('port', '/dev/ttyACM0'))
        self.declare_parameter('serial.port_fallback', serial_cfg.get('port_fallback', '/dev/ttyACM1'))
        self.declare_parameter('serial.baudrate', serial_cfg.get('baudrate', 115200))

        self.declare_parameter('teensy.enable_wire_on_startup', teensy_cfg.get('enable_wire_on_startup', True))
        self.declare_parameter('teensy.own_modem_id', teensy_cfg.get('own_modem_id', '101'))
        self.declare_parameter('teensy.command_terminator', teensy_cfg.get('command_terminator', '\r\n'))

        self.declare_parameter('ping.max_retries', ping_cfg.get('max_retries', 3))
        self.declare_parameter('ping.response_timeout_s', ping_cfg.get('response_timeout_s', 8.0))
        self.declare_parameter('ping.task_timeout_s', ping_cfg.get('task_timeout_s', 180.0))
        self.declare_parameter('ping.cycle_until_estimated', ping_cfg.get('cycle_until_estimated', True))

        self.declare_parameter('estimator.method', estimator_cfg.get('method', 'least_squares'))
        self.declare_parameter('estimator.default_sound_velocity', estimator_cfg.get('default_sound_velocity', 1500.0))
        self.declare_parameter('estimator.sound_velocity_topic', estimator_cfg.get('sound_velocity_topic', '/lolo/sensors/svs'))
        self.declare_parameter('estimator.sound_velocity_msg_type', estimator_cfg.get('sound_velocity_msg_type', 'svs_interfaces/msg/SVS'))
        self.declare_parameter('estimator.sound_velocity_field', estimator_cfg.get('sound_velocity_field', 'svs'))

        self.declare_parameter('estimator.range_sigma_m', estimator_cfg.get('range_sigma_m', 1.0))
        self.declare_parameter('estimator.own_position_sigma_m', estimator_cfg.get('own_position_sigma_m', 1.0))
        self.declare_parameter('estimator.own_depth_sigma_m', estimator_cfg.get('own_depth_sigma_m', 0.3))
        self.declare_parameter('estimator.remote_depth_sigma_m', estimator_cfg.get('remote_depth_sigma_m', 0.5))

        self.declare_parameter('estimator.min_measurements', estimator_cfg.get('min_measurements', 4))
        self.declare_parameter('estimator.max_measurements', estimator_cfg.get('max_measurements', 40))
        self.declare_parameter('estimator.max_position_uncertainty_m', estimator_cfg.get('max_position_uncertainty_m', 5.0))
        self.declare_parameter('estimator.use_known_remote_depth', estimator_cfg.get('use_known_remote_depth', True))

        self.declare_parameter('estimator.least_squares.max_iterations', ls_cfg.get('max_iterations', 20))
        self.declare_parameter('estimator.least_squares.damping', ls_cfg.get('damping', 0.001))
        self.declare_parameter('estimator.least_squares.outlier_gate_m', ls_cfg.get('outlier_gate_m', 10.0))

        self.declare_parameter('estimator.ekf.process_noise_std_m', ekf_cfg.get('process_noise_std_m', 0.05))
        self.declare_parameter('estimator.ekf.initial_sigma_xy_m', ekf_cfg.get('initial_sigma_xy_m', 50.0))
        self.declare_parameter('estimator.ekf.initial_sigma_z_m', ekf_cfg.get('initial_sigma_z_m', 2.0))

        self.declare_parameter('topics.own_latlon_topic', topics_cfg.get('own_latlon_topic', '/lolo/smarc/latlon'))
        self.declare_parameter('topics.own_depth_topic', topics_cfg.get('own_depth_topic', '/lolo/smarc/depth'))
        self.declare_parameter('topics.own_position_max_age_s', topics_cfg.get('own_position_max_age_s', 3.0))
        self.declare_parameter('topics.geopoint_topic', topics_cfg.get('geopoint_topic', '/modem_estimates/geopoint'))
        self.declare_parameter('topics.marker_topic', topics_cfg.get('marker_topic', '/modem_estimates/rviz'))
        self.declare_parameter('topics.map_frame', topics_cfg.get('map_frame', 'M350/map'))

        # ---------------- parameter values ----------------
        self.port = self.get_parameter('serial.port').value
        self.port_fallback = self.get_parameter('serial.port_fallback').value
        self.baudrate = int(self.get_parameter('serial.baudrate').value)

        self.enable_wire_on_startup = bool(self.get_parameter('teensy.enable_wire_on_startup').value)
        self.own_modem_id = str(self.get_parameter('teensy.own_modem_id').value).zfill(3)
        self.command_terminator = self.get_parameter('teensy.command_terminator').value

        self.default_retry_count = int(self.get_parameter('ping.max_retries').value)
        self.ping_response_timeout_s = float(self.get_parameter('ping.response_timeout_s').value)
        self.default_task_timeout_s = float(self.get_parameter('ping.task_timeout_s').value)
        self.cycle_until_estimated = bool(self.get_parameter('ping.cycle_until_estimated').value)

        self.estimator_method = str(self.get_parameter('estimator.method').value).lower()
        self.default_sound_velocity = float(self.get_parameter('estimator.default_sound_velocity').value)
        self.sound_velocity = self.default_sound_velocity
        self.sound_velocity_topic = self.get_parameter('estimator.sound_velocity_topic').value
        self.sound_velocity_msg_type = self.get_parameter('estimator.sound_velocity_msg_type').value
        self.sound_velocity_field = self.get_parameter('estimator.sound_velocity_field').value

        self.range_sigma_m = float(self.get_parameter('estimator.range_sigma_m').value)
        self.own_position_sigma_m = float(self.get_parameter('estimator.own_position_sigma_m').value)
        self.own_depth_sigma_m = float(self.get_parameter('estimator.own_depth_sigma_m').value)
        self.remote_depth_sigma_m = float(self.get_parameter('estimator.remote_depth_sigma_m').value)

        # Effective range sigma: simple conservative lumping of range and own-position uncertainty.
        self.effective_range_sigma_m = math.sqrt(
            self.range_sigma_m ** 2 + self.own_position_sigma_m ** 2
        )

        self.min_measurements = int(self.get_parameter('estimator.min_measurements').value)
        self.max_measurements = int(self.get_parameter('estimator.max_measurements').value)
        self.max_position_uncertainty_m = float(self.get_parameter('estimator.max_position_uncertainty_m').value)
        self.use_known_remote_depth = bool(self.get_parameter('estimator.use_known_remote_depth').value)

        self.ls_max_iterations = int(self.get_parameter('estimator.least_squares.max_iterations').value)
        self.ls_damping = float(self.get_parameter('estimator.least_squares.damping').value)
        self.ls_outlier_gate_m = float(self.get_parameter('estimator.least_squares.outlier_gate_m').value)

        self.ekf_process_noise_std_m = float(self.get_parameter('estimator.ekf.process_noise_std_m').value)
        self.ekf_initial_sigma_xy_m = float(self.get_parameter('estimator.ekf.initial_sigma_xy_m').value)
        self.ekf_initial_sigma_z_m = float(self.get_parameter('estimator.ekf.initial_sigma_z_m').value)

        self.own_latlon_topic = self.get_parameter('topics.own_latlon_topic').value
        self.own_depth_topic = self.get_parameter('topics.own_depth_topic').value
        self.own_position_max_age_s = float(self.get_parameter('topics.own_position_max_age_s').value)
        self.geopoint_topic = self.get_parameter('topics.geopoint_topic').value
        self.marker_topic = self.get_parameter('topics.marker_topic').value
        self.map_frame = self.get_parameter('topics.map_frame').value

        # ---------------- serial ----------------
        self.ser = init_serial(self.port, self.port_fallback, self.baudrate, self.get_logger())
        if self.ser is None:
            rclpy.shutdown()
            return

        self.install_shutdown_guard()

        # Optional: put Teensy bridge into transparent wire mode.
        # If the serial target is a raw modem, this may just return E; harmless.
        if self.enable_wire_on_startup:
            self.send_command(ti.build_config_command(ti.TeensyMode.WIRE, self.own_modem_id))

        # ---------------- state ----------------
        self.modem_ids: list[str] = []
        self.modem_depths: dict[str, float] = {}       # depth positive down [m]
        self.estimators = {}                           # modem_id -> estimator
        self.estimate_done: set[str] = set()

        self.state = PingState.IDLE
        self.action_done = True
        self.status_msg = "idle"

        self.ping_queue: list[str] = []
        self.attempts_left: dict[str, int] = {}
        self.attempt_budget = self.default_retry_count + 1
        self.ping_success: set[str] = set()
        self.ping_failed: set[str] = set()

        self.waiting_for_id: str | None = None
        self.ping_deadline = None
        self.task_deadline = None

        self.serial_buffer = ""

        self.good_estimates: dict[str, GeoPointStamped] = {}
        self.estimate_uncertainty: dict[str, float] = {}

        self.latest_own_latlon: GeoPoint | None = None
        self.latest_own_latlon_time = None
        self.latest_own_depth_m: float | None = None
        self.latest_own_depth_time = None

        self.utm_zone = None
        self.utm_band = None
        self.utm_frame: str | None = None

        # tf / visualization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- ROS I/O ----------------
        self.create_subscription(GeoPoint, self.own_latlon_topic, self._own_latlon_cb, 10)

        if self.own_depth_topic:
            self.create_subscription(Float32, self.own_depth_topic, self._own_depth_cb, 10)

        self._setup_sound_velocity_subscription()

        self.geopoint_pub = self.create_publisher(GeoPointStamped, self.geopoint_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

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
            f"modem_ping_estimator_node ready. method={self.estimator_method}, "
            f"own_latlon={self.own_latlon_topic}, own_depth={self.own_depth_topic}, "
            f"output={self.geopoint_topic}, markers={self.marker_topic}"
        )

    # ------------------------------------------------------------------ setup

    def _setup_sound_velocity_subscription(self):
        if not self.sound_velocity_topic:
            self.get_logger().warn(
                f"No sound velocity topic configured; using default {self.default_sound_velocity:.1f} m/s."
            )
            return

        try:
            MsgType = ti.import_message_type(self.sound_velocity_msg_type)
        except Exception as e:
            self.get_logger().warn(
                f"Sound velocity msg type '{self.sound_velocity_msg_type}' unavailable ({e}); "
                f"using default {self.default_sound_velocity:.1f} m/s."
            )
            return

        self.create_subscription(MsgType, self.sound_velocity_topic, self._sound_velocity_cb, 10)
        self.get_logger().info(
            f"Subscribed to sound velocity: {self.sound_velocity_topic}, "
            f"type={self.sound_velocity_msg_type}, field={self.sound_velocity_field}"
        )

    # ------------------------------------------------------------------ actions

    def _on_goal_received(self, goal_request: dict) -> bool:
        mode = str(goal_request.get('mode', '')).lower().strip()

        if mode == 'add':
            modem_id = self._parse_goal_modem_id(goal_request)
            if modem_id is None:
                return False

            if 'depth_m' in goal_request and goal_request['depth_m'] is not None:
                self.modem_depths[modem_id] = float(goal_request['depth_m'])

            if modem_id not in self.modem_ids:
                self.modem_ids.append(modem_id)
                self.modem_ids.sort()

            # Rebuild estimator if depth changed or estimator did not exist.
            self.estimators[modem_id] = self._make_estimator(modem_id)
            self.estimate_done.discard(modem_id)
            self.ping_failed.discard(modem_id)

            self.status_msg = (
                f"added modem {modem_id}, depth_m={self.modem_depths.get(modem_id)}, "
                f"list={self.modem_ids}"
            )
            self.action_done = True
            self.get_logger().info(self.status_msg)
            return True

        if mode == 'remove':
            modem_id = self._parse_goal_modem_id(goal_request)
            if modem_id is None:
                return False

            self._remove_modem(modem_id)
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

            self._start_ping_task(retry_count=retry_count, task_timeout_s=task_timeout_s)
            return True

        self.get_logger().error(f"Unknown modem ping action mode: {mode!r}")
        return False

    def _remove_modem(self, modem_id: str):
        if modem_id in self.modem_ids:
            self.modem_ids.remove(modem_id)

        self.modem_depths.pop(modem_id, None)
        self.estimators.pop(modem_id, None)
        self.good_estimates.pop(modem_id, None)
        self.estimate_uncertainty.pop(modem_id, None)

        self.estimate_done.discard(modem_id)
        self.ping_success.discard(modem_id)
        self.ping_failed.discard(modem_id)
        self.attempts_left.pop(modem_id, None)

        if self.waiting_for_id == modem_id:
            self.waiting_for_id = None
            self.ping_deadline = None

        self.ping_queue = [mid for mid in self.ping_queue if mid != modem_id]
        self._delete_marker_for_modem(modem_id)

    def _clear_all_modems(self, reason: str = "cleared all modem ids"):
        self._cancel_ping_task(reason)

        self.modem_ids = []
        self.modem_depths.clear()
        self.estimators.clear()

        self.good_estimates.clear()
        self.estimate_uncertainty.clear()
        self.estimate_done.clear()

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
        # retry_count = retries after the first attempt.
        # attempt_budget = first attempt + retries.
        self.attempt_budget = max(1, retry_count + 1)

        self.ping_queue = list(self.modem_ids)
        self.attempts_left = {mid: self.attempt_budget for mid in self.modem_ids}
        self.ping_success = set()
        self.ping_failed = set()

        # Keep old estimates if already good, but normally a new ping task should
        # try to improve/reconfirm everything.
        self.estimate_done = set()

        self.waiting_for_id = None
        self.ping_deadline = None
        self.task_deadline = self.get_clock().now() + Duration(seconds=task_timeout_s)

        self.state = PingState.RUNNING
        self.action_done = False
        self.status_msg = (
            f"ping/localize {self.modem_ids}, retries={retry_count}, "
            f"timeout={task_timeout_s}s"
        )
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
            remaining = sorted(set(self.modem_ids) - self.estimate_done - self.ping_failed)
            self.status_msg = (
                f"ping task timeout. estimated={sorted(self.estimate_done)}, "
                f"failed={sorted(self.ping_failed)}, remaining={remaining}"
            )
            self.get_logger().warn(self.status_msg)
            return

        if self.waiting_for_id is not None:
            if self.ping_deadline is not None and now > self.ping_deadline:
                self._handle_ping_timeout(self.waiting_for_id)
            return

        completed = self.estimate_done | self.ping_failed

        if set(self.modem_ids).issubset(completed):
            self.state = PingState.DONE
            self.action_done = True
            self.status_msg = (
                f"ping/localization complete. estimated={sorted(self.estimate_done)}, "
                f"failed={sorted(self.ping_failed)}"
            )
            if self.ping_failed:
                self.get_logger().warn(self.status_msg)
            else:
                self.get_logger().info(self.status_msg)
            return

        if not self.ping_queue:
            self.ping_queue = [mid for mid in self.modem_ids if mid not in completed]

        if self.ping_queue:
            next_id = self.ping_queue.pop(0)
            if next_id not in completed:
                self._send_ping(next_id)

    def _send_ping(self, modem_id: str):
        cmd = f"$P{modem_id}"
        self.send_command(cmd)

        self.waiting_for_id = modem_id
        self.ping_deadline = self.get_clock().now() + Duration(seconds=self.ping_response_timeout_s)

        remaining = self.attempts_left.get(modem_id, self.attempt_budget)
        self.status_msg = f"pinging {modem_id}, attempts_left={remaining}"
        self.get_logger().info(self.status_msg)

    def _handle_ping_success(self, modem_id: str, range_m: float):
        self.ping_success.add(modem_id)

        # Reset timeout budget on successful communication. This makes retries
        # behave like consecutive timeout tolerance, not lifetime timeout count.
        self.attempts_left[modem_id] = self.attempt_budget

        own_xyz = self._current_own_modem_xyz()
        if own_xyz is None:
            self.get_logger().warn(
                f"ping success {modem_id}, range={range_m:.2f}m, "
                "but own xyz unavailable; measurement discarded"
            )
        else:
            self._add_range_measurement(modem_id, own_xyz, range_m)

        self.waiting_for_id = None
        self.ping_deadline = None

        self.status_msg = f"ping success {modem_id}, range={range_m:.2f}m"
        self.get_logger().info(self.status_msg)

    def _handle_ping_timeout(self, modem_id: str):
        self.attempts_left[modem_id] = self.attempts_left.get(modem_id, self.attempt_budget) - 1

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

        # Local ack/echo from ping command.
        # Example: $P007
        if line.startswith('$P'):
            return

        # Success:
        #   #R007T12345
        # Succorfish range formula:
        #   range_m = ticks * c * 3.125e-5
        m = re.match(r'^#R(\d{3})T(\d+)$', line)
        if m:
            modem_id = m.group(1)
            ticks = int(m.group(2))
            range_m = ticks * self.sound_velocity * 3.125e-5

            if self.waiting_for_id == modem_id:
                self._handle_ping_success(modem_id, range_m)
            else:
                self.get_logger().warn(
                    f"got range for {modem_id}, but waiting_for={self.waiting_for_id}"
                )
            return

        # Timeout from modem. It does not include id, so assign to current wait.
        if line == '#TO':
            if self.waiting_for_id is not None:
                self._handle_ping_timeout(self.waiting_for_id)
            else:
                self.get_logger().warn("got #TO but not waiting for any modem")
            return

        # Config/status lines from Teensy or raw modem.
        if line.startswith('#A') or line == 'E':
            self.get_logger().info(f"config/status line: {line}")
            return

        self.get_logger().debug(f"unhandled serial line: {line}")

    # ------------------------------------------------------------------ own state

    def _own_latlon_cb(self, msg: GeoPoint):
        self.latest_own_latlon = msg
        self.latest_own_latlon_time = self.get_clock().now()

        # Initialize UTM zone/band from own GPS.
        try:
            up = geo_utm.fromMsg(msg)
            self.utm_zone = up.zone
            self.utm_band = up.band
        except Exception as e:
            self.get_logger().warn(f"could not initialize UTM zone/band: {e}")

    def _own_depth_cb(self, msg: Float32):
        self.latest_own_depth_m = float(msg.data)
        self.latest_own_depth_time = self.get_clock().now()

    def _sound_velocity_cb(self, msg):
        try:
            self.sound_velocity = float(getattr(msg, self.sound_velocity_field))
        except (AttributeError, TypeError, ValueError) as e:
            self.get_logger().warn(
                f"could not read sound velocity field '{self.sound_velocity_field}': {e}"
            )

    def _current_own_modem_xyz(self):
        if self.latest_own_latlon is None or self.latest_own_latlon_time is None:
            return None

        now = self.get_clock().now()
        max_age = Duration(seconds=self.own_position_max_age_s)

        if self.latest_own_latlon_time + max_age < now:
            self.get_logger().warn(
                f"own latlon older than {self.own_position_max_age_s:.1f}s; not using range",
                throttle_duration_sec=5.0,
            )
            return None

        gp = GeoPoint()
        gp.latitude = float(self.latest_own_latlon.latitude)
        gp.longitude = float(self.latest_own_latlon.longitude)

        if self.latest_own_depth_m is not None:
            gp.altitude = -float(self.latest_own_depth_m)
        else:
            gp.altitude = float(self.latest_own_latlon.altitude)

        try:
            up = geo_utm.fromMsg(gp)
        except Exception as e:
            self.get_logger().warn(f"could not convert own latlon to UTM: {e}")
            return None

        self.utm_zone = up.zone
        self.utm_band = up.band

        return np.array([
            float(up.easting),
            float(up.northing),
            float(gp.altitude),
        ], dtype=float)

    # ------------------------------------------------------------------ estimator

    def _make_estimator(self, modem_id: str):
        z_remote = None

        if self.use_known_remote_depth:
            depth_m = self.modem_depths.get(modem_id)
            if depth_m is None:
                self.get_logger().warn(
                    f"modem {modem_id} has no depth_m; falling back to full 3D estimate. "
                    "This is weaker unless the own vehicle has good 3D geometry."
                )
            else:
                # Depth is positive down, ENU altitude z is positive up.
                z_remote = -float(depth_m)

        if self.estimator_method == 'ekf':
            return EkfRangeEstimator(
                modem_id=modem_id,
                z_remote=z_remote,
                range_sigma_m=self.effective_range_sigma_m,
                remote_depth_sigma_m=self.remote_depth_sigma_m,
                process_noise_std_m=self.ekf_process_noise_std_m,
                initial_sigma_xy_m=self.ekf_initial_sigma_xy_m,
                initial_sigma_z_m=self.ekf_initial_sigma_z_m,
                bootstrap_max_measurements=self.max_measurements,
                bootstrap_damping=self.ls_damping,
                bootstrap_iterations=self.ls_max_iterations,
            )

        return LeastSquaresRangeEstimator(
            modem_id=modem_id,
            z_remote=z_remote,
            max_measurements=self.max_measurements,
            range_sigma_m=self.effective_range_sigma_m,
            remote_depth_sigma_m=self.remote_depth_sigma_m,
            damping=self.ls_damping,
            max_iterations=self.ls_max_iterations,
            outlier_gate_m=self.ls_outlier_gate_m,
        )

    def _add_range_measurement(self, modem_id: str, own_xyz, range_m: float):
        if modem_id not in self.modem_ids:
            return

        estimator = self.estimators.get(modem_id)
        if estimator is None:
            estimator = self._make_estimator(modem_id)
            self.estimators[modem_id] = estimator

        estimator.add_measurement(own_xyz, range_m)
        xyz, cov = estimator.estimate(self.min_measurements)

        if xyz is None or cov is None:
            n = len(getattr(estimator, 'measurements', []))
            if hasattr(estimator, 'bootstrap'):
                n = len(estimator.bootstrap.measurements)
            self.get_logger().info(
                f"modem {modem_id}: range={range_m:.2f}m stored, "
                f"waiting for estimate ({n}/{self.min_measurements} measurements)"
            )
            return

        sigma = self._position_uncertainty_from_cov(cov)

        if sigma <= self.max_position_uncertainty_m:
            self.estimate_done.add(modem_id)
            self._publish_estimated_xyz(modem_id, xyz, sigma)
        else:
            self.get_logger().info(
                f"modem {modem_id}: estimate not good enough yet, "
                f"sigma={sigma:.2f}m > {self.max_position_uncertainty_m:.2f}m"
            )

    def _position_uncertainty_from_cov(self, cov):
        cov = np.asarray(cov, dtype=float)

        if cov.shape[0] < 2 or cov.shape[1] < 2:
            return float('inf')

        cov_xy = cov[:2, :2]

        try:
            eigvals = np.linalg.eigvalsh(cov_xy)
        except np.linalg.LinAlgError:
            return float('inf')

        return math.sqrt(max(float(np.max(eigvals)), 0.0))

    # ------------------------------------------------------------------ publishing

    def _publish_estimated_xyz(self, modem_id: str, xyz, sigma: float):
        if self.utm_zone is None or self.utm_band is None:
            self.get_logger().warn("UTM zone/band unknown; cannot publish GeoPoint estimate yet")
            return

        xyz = np.asarray(xyz, dtype=float)

        up = geo_utm.UTMPoint(
            easting=float(xyz[0]),
            northing=float(xyz[1]),
            altitude=float(xyz[2]),
            zone=self.utm_zone,
            band=self.utm_band,
        )

        geo = up.toMsg()

        out = GeoPointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = f"modem_{modem_id}"
        out.position.latitude = float(geo.latitude)
        out.position.longitude = float(geo.longitude)
        out.position.altitude = float(xyz[2])

        self.good_estimates[modem_id] = out
        self.estimate_uncertainty[modem_id] = sigma

        self.geopoint_pub.publish(out)
        self._publish_marker_for_estimate(modem_id, out, sigma)

        self.get_logger().info(
            f"modem {modem_id} estimate: "
            f"lat={out.position.latitude:.8f}, lon={out.position.longitude:.8f}, "
            f"z={out.position.altitude:.2f}m, sigma={sigma:.2f}m"
        )

    # ------------------------------------------------------------------ helpers

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

        diameter = max(1.0, 2.0 * sigma)
        sphere.scale.x = diameter
        sphere.scale.y = diameter
        sphere.scale.z = diameter

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

        depth_m = -float(estimate.position.altitude)
        text.text = f"modem {modem_id}\nσ={sigma:.1f}m\nz={estimate.position.altitude:.1f}m depth={depth_m:.1f}m"

        self.marker_pub.publish(MarkerArray(markers=[sphere, text]))

    def _delete_marker_for_modem(self, modem_id: str):
        markers = []

        sphere = Marker()
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.header.frame_id = self.map_frame
        sphere.ns = "modem_estimates"
        sphere.id = int(modem_id) * 2
        sphere.action = Marker.DELETE
        markers.append(sphere)

        text = Marker()
        text.header.stamp = self.get_clock().now().to_msg()
        text.header.frame_id = self.map_frame
        text.ns = "modem_estimates_text"
        text.id = int(modem_id) * 2 + 1
        text.action = Marker.DELETE
        markers.append(text)

        self.marker_pub.publish(MarkerArray(markers=markers))

    def _delete_all_markers(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[marker]))

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