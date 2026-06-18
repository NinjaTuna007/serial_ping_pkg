"""OWTT beacon inference node -- GTSAM iSAM2 fixed-lag smoother backend (prototype).

This is a drop-in alternative to ``inference_node`` that keeps the *same model*
(constant-velocity beacon, slant->horizontal range correction, reported-GPS
seeding, START/STOP orchestration) and publishes the *same topics*
(``.../estimate``, ``.../estimate_alt``, ``.../reported``, ``.../telemetry``,
``.../receiver/<unit>``) -- but replaces the Markov EKF / two-hypothesis tracker
with an **incremental fixed-lag factor-graph smoother** (GTSAM iSAM2).

Why bother? A Markov filter can only consume measurements in (roughly) time
order; a range that shows up on MQTT seconds-to-tens-of-seconds late can only be
*dropped* (it describes where the beacon *was*, not where it *is*). A smoother
keeps a sliding window of recent beacon states and the factors between them, so a
late/out-of-order range is simply attached to the graph node at *its own*
measurement time and the window re-optimises -- retroactively correcting the
trajectory and, through the motion chain, the current estimate too.

Graph layout (local ENU about a fixed origin):
  * Variables ``X(k)`` = beacon position (m, ENU) and ``V(k)`` = velocity (m/s)
    on a fixed time grid of period ``node_period_s``.
  * A constant-velocity factor links consecutive nodes
    (``x_{k+1} = x_k + v_k*dt``, ``v_{k+1} = v_k``), noise from
    ``process_accel_std_mps2``.
  * Each surface-unit range adds a unary range factor on the node nearest its
    measurement time, anchored at *that report's* receiver position (so moving
    buoys are handled), using the depth-corrected horizontal range.
  * The beacon's reported GPS (telemetry ``position``), when present, adds a
    position prior -- this both seeds the frame and locks the correct branch of
    the 2-receiver ambiguity.

Late injection: every node within ``smoother_lag_s`` of the newest measurement is
still alive in the smoother, so a range that arrives late lands on its correct
(past) node and is fused properly. Older than the lag -> the node has been
marginalised out, so the measurement is genuinely too old and is dropped (a
Markov filter would have had to drop it immediately regardless).

Requires ``gtsam`` (``pip install gtsam``). If it is missing the node logs a
clear error and exits; the EKF ``owtt_inference_node`` remains the default.
"""

import json
import math
import re
import threading
import time
import uuid

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger

from serial_ping_pkg.utils import load_yaml_config
from serial_ping_pkg.owtt_beacon.mqtt_helper import MqttClient
from serial_ping_pkg.owtt_beacon.inference_node import (
    geodetic_to_enu, enu_to_geodetic, circle_intersections)

try:
    import gtsam
    from gtsam.symbol_shorthand import X, V
    _GTSAM_IMPORT_ERROR = None
except Exception as exc:                                  # pragma: no cover
    gtsam = None
    X = V = None
    _GTSAM_IMPORT_ERROR = exc


def make_range_factor(key, anchor_xy, rng, noise):
    """Unary GTSAM factor: ``||X(key) - anchor|| - rng`` (2-D, planar ENU)."""
    a = np.asarray(anchor_xy, dtype=float)

    def error(this, values, jacobians):
        x = values.atVector(key)
        d = x - a
        dist = float(math.hypot(d[0], d[1]))
        dist = dist if dist > 1e-6 else 1e-6
        if jacobians is not None:
            jacobians[0] = np.array([[d[0] / dist, d[1] / dist]])
        return np.array([dist - rng])

    return gtsam.CustomFactor(noise, [key], error)


def make_cv_factor(xk, vk, xk1, vk1, dt, noise):
    """Constant-velocity factor: ``x1 = x0 + v0*dt`` and ``v1 = v0`` (4-D error)."""
    def error(this, values, jacobians):
        x0 = values.atVector(xk)
        v0 = values.atVector(vk)
        x1 = values.atVector(xk1)
        v1 = values.atVector(vk1)
        e = np.concatenate([x1 - (x0 + v0 * dt), v1 - v0])
        if jacobians is not None:
            jacobians[0] = np.vstack([-np.eye(2), np.zeros((2, 2))])      # d/dx0
            jacobians[1] = np.vstack([-dt * np.eye(2), -np.eye(2)])       # d/dv0
            jacobians[2] = np.vstack([np.eye(2), np.zeros((2, 2))])       # d/dx1
            jacobians[3] = np.vstack([np.zeros((2, 2)), np.eye(2)])       # d/dv1
        return e

    return gtsam.CustomFactor(noise, [xk, vk, xk1, vk1], error)
class SmootherInferenceNode(Node):
    def __init__(self):
        super().__init__('owtt_inference_smoother_node')

        config = load_yaml_config('serial_ping_pkg', 'owtt_beacon/owtt_beacon_config.yaml')
        mqtt_cfg = config.get('mqtt', {})
        inf_cfg = config.get('inference', {})
        sm_cfg = config.get('smoother', {})
        beacon_cfg = config.get('beacon', {})

        self.declare_parameter('beacon.name', beacon_cfg.get('name', 'lolo'))

        self.declare_parameter('mqtt.host', mqtt_cfg.get('host', 'localhost'))
        self.declare_parameter('mqtt.port', mqtt_cfg.get('port', 1884))
        self.declare_parameter('mqtt.keepalive', mqtt_cfg.get('keepalive', 30))
        self.declare_parameter('mqtt.username', mqtt_cfg.get('username', ''))
        self.declare_parameter('mqtt.password', mqtt_cfg.get('password', ''))
        self.declare_parameter('mqtt.topic_prefix', mqtt_cfg.get('topic_prefix', 'owtt_beacon'))

        # Shared model knobs (kept identical to inference_node for parity).
        self.declare_parameter('inference.update_period_s', inf_cfg.get('update_period_s', 1.0))
        self.declare_parameter('inference.publish_period_s', inf_cfg.get('publish_period_s', 0.1))
        self.declare_parameter('inference.freshness_window_s', inf_cfg.get('freshness_window_s', 10.0))
        self.declare_parameter('inference.min_units', inf_cfg.get('min_units', 2))
        self.declare_parameter('inference.frame_id', inf_cfg.get('frame_id', 'map'))
        self.declare_parameter('inference.first_fix_prefer', inf_cfg.get('first_fix_prefer', 'north'))
        self.declare_parameter('inference.publish_both', inf_cfg.get('publish_both', True))
        self.declare_parameter('inference.assumed_depth_m', inf_cfg.get('assumed_depth_m', 0.0))
        self.declare_parameter('inference.max_speed_mps', inf_cfg.get('max_speed_mps', 2.0))
        self.declare_parameter('inference.use_reported_position', inf_cfg.get('use_reported_position', True))
        self.declare_parameter('inference.initial_lat', inf_cfg.get('initial_lat', 0.0))
        self.declare_parameter('inference.initial_lon', inf_cfg.get('initial_lon', 0.0))
        self.declare_parameter('inference.range_std_m', inf_cfg.get('range_std_m', 2.0))
        self.declare_parameter('inference.single_init_pos_std_m', inf_cfg.get('single_init_pos_std_m', 50.0))
        self.declare_parameter('inference.single_init_vel_std_mps', inf_cfg.get('single_init_vel_std_mps', 1.0))
        self.declare_parameter('inference.process_accel_std_mps2', inf_cfg.get('process_accel_std_mps2', 0.3))
        self.declare_parameter('inference.use_report_stamp', inf_cfg.get('use_report_stamp', True))
        self.declare_parameter('inference.max_clock_skew_s', inf_cfg.get('max_clock_skew_s', 30.0))
        self.declare_parameter('inference.command_timeout_s', inf_cfg.get('command_timeout_s', 10.0))
        self.declare_parameter('inference.command_repeat_s', inf_cfg.get('command_repeat_s', 1.5))

        # Smoother-specific knobs.
        # Sliding window length: nodes older than this (relative to the newest
        # measurement) are marginalised out -> a later range than this is dropped.
        self.declare_parameter('smoother.lag_s', sm_cfg.get('lag_s', 60.0))
        # Time grid spacing for the state nodes (range factors snap to the nearest).
        self.declare_parameter('smoother.node_period_s', sm_cfg.get('node_period_s', 1.0))
        # Noise (std-dev) on the reported-GPS position prior, when telemetry has it.
        self.declare_parameter('smoother.reported_pos_std_m', sm_cfg.get('reported_pos_std_m', 5.0))

        self.beacon_name = self.get_parameter('beacon.name').get_parameter_value().string_value
        self.mqtt_host = self.get_parameter('mqtt.host').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter('mqtt.port').get_parameter_value().integer_value
        self.mqtt_keepalive = self.get_parameter('mqtt.keepalive').get_parameter_value().integer_value
        self.mqtt_username = self.get_parameter('mqtt.username').get_parameter_value().string_value
        self.mqtt_password = self.get_parameter('mqtt.password').get_parameter_value().string_value
        self.mqtt_topic_prefix = self.get_parameter('mqtt.topic_prefix').get_parameter_value().string_value
        self.update_period_s = self.get_parameter('inference.update_period_s').get_parameter_value().double_value
        self.publish_period_s = self.get_parameter('inference.publish_period_s').get_parameter_value().double_value
        self.freshness_window_s = self.get_parameter('inference.freshness_window_s').get_parameter_value().double_value
        self.min_units = self.get_parameter('inference.min_units').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('inference.frame_id').get_parameter_value().string_value
        self.first_fix_prefer = self.get_parameter('inference.first_fix_prefer').get_parameter_value().string_value.lower()
        self.publish_both = self.get_parameter('inference.publish_both').get_parameter_value().bool_value
        self.assumed_depth_m = self.get_parameter('inference.assumed_depth_m').get_parameter_value().double_value
        self.max_speed_mps = self.get_parameter('inference.max_speed_mps').get_parameter_value().double_value
        self.use_reported_position = self.get_parameter('inference.use_reported_position').get_parameter_value().bool_value
        self.initial_lat = self.get_parameter('inference.initial_lat').get_parameter_value().double_value
        self.initial_lon = self.get_parameter('inference.initial_lon').get_parameter_value().double_value
        self.range_std_m = self.get_parameter('inference.range_std_m').get_parameter_value().double_value
        self.init_pos_std_m = self.get_parameter('inference.single_init_pos_std_m').get_parameter_value().double_value
        self.init_vel_std_mps = self.get_parameter('inference.single_init_vel_std_mps').get_parameter_value().double_value
        self.process_accel_std_mps2 = self.get_parameter('inference.process_accel_std_mps2').get_parameter_value().double_value
        self.use_report_stamp = self.get_parameter('inference.use_report_stamp').get_parameter_value().bool_value
        self.max_clock_skew_s = self.get_parameter('inference.max_clock_skew_s').get_parameter_value().double_value
        self.command_timeout_s = self.get_parameter('inference.command_timeout_s').get_parameter_value().double_value
        self.command_repeat_s = self.get_parameter('inference.command_repeat_s').get_parameter_value().double_value
        self.lag_s = self.get_parameter('smoother.lag_s').get_parameter_value().double_value
        self.node_period_s = max(1e-2, self.get_parameter('smoother.node_period_s').get_parameter_value().double_value)
        self.reported_pos_std_m = self.get_parameter('smoother.reported_pos_std_m').get_parameter_value().double_value

        if gtsam is None:
            self.get_logger().fatal(
                f"gtsam is not importable ({_GTSAM_IMPORT_ERROR}). Install it "
                "(`pip install gtsam`) or use the EKF node `owtt_inference_node`.")
            raise SystemExit(1)

        # ------------------------------------------------------------ state
        self._lock = threading.Lock()
        self._latest = {}             # unit -> latest report (receiver/telemetry view)
        self._pending = []            # unprocessed range reports (each a factor)
        self._prev_estimate = None    # (lat, lon) of last published primary fix
        self._receiver_pubs = {}      # unit -> NavSatFix publisher
        self._smooth = None           # (lat, lon, vE, vN, t) for the fast publisher

        # Graph / smoother state (all touched only on the ROS timer thread).
        self._sm = None
        self._origin = None           # (lat0, lon0) of the ENU frame
        self._t0 = None               # wall time of node index 0
        self._nodes = set()           # existing grid indices
        self._frontier = -1           # highest created index
        self._latest_meas_t = None    # newest measurement time fused so far

        # NavSatFix / telemetry publishers.
        self.fix_pub = self.create_publisher(
            NavSatFix, f"/owtt_beacon/{self.beacon_name}/estimate", 10)
        self.fix_alt_pub = self.create_publisher(
            NavSatFix, f"/owtt_beacon/{self.beacon_name}/estimate_alt", 10)
        self.reported_pub = self.create_publisher(
            NavSatFix, f"/owtt_beacon/{self.beacon_name}/reported", 10)
        self.telemetry_pub = self.create_publisher(
            String, f"/owtt_beacon/{self.beacon_name}/telemetry", 10)

        # START/STOP orchestration (identical contract to the EKF node).
        beacon_sel = self.beacon_name if self.beacon_name else '+'
        self.cmd_topic = f"{self.mqtt_topic_prefix}/{self.beacon_name}/cmd"
        self.cmd_ack_topic = f"{self.mqtt_topic_prefix}/{beacon_sel}/cmd_ack"
        self._cmd_lock = threading.Lock()
        self._pending_cmd = None

        self.sub_topic = f"{self.mqtt_topic_prefix}/{beacon_sel}/range/+"
        self.mqtt = MqttClient(
            self.mqtt_host, self.mqtt_port, keepalive=self.mqtt_keepalive,
            username=(self.mqtt_username or None), password=(self.mqtt_password or None),
            client_id="owtt_inference_smoother", logger=self.get_logger())
        self.mqtt.set_message_handler(self._on_mqtt)
        self.mqtt.add_subscription(self.sub_topic)
        self.mqtt.add_subscription(self.cmd_ack_topic, qos=1)
        self.mqtt.start()

        self._srv_group = ReentrantCallbackGroup()
        self.start_srv = self.create_service(
            Trigger, f"/owtt_beacon/{self.beacon_name}/start",
            self._on_start_service, callback_group=self._srv_group)
        self.stop_srv = self.create_service(
            Trigger, f"/owtt_beacon/{self.beacon_name}/stop",
            self._on_stop_service, callback_group=self._srv_group)

        self.timer = self.create_timer(self.update_period_s, self.update)
        self.smooth_timer = self.create_timer(
            self.publish_period_s, self._publish_smooth, callback_group=self._srv_group)
        self.get_logger().info(
            f"OWTT smoother inference node up (GTSAM iSAM2, lag {self.lag_s:.0f}s, "
            f"grid {self.node_period_s:.2f}s): MQTT {self.mqtt_host}:{self.mqtt_port} "
            f"sub '{self.sub_topic}', beacon '{self.beacon_name}'.")
    # --------------------------------------------------------------- MQTT in
    def _on_mqtt(self, topic, payload):
        if topic.endswith('/cmd_ack'):
            self._on_cmd_ack(payload)
            return
        try:
            report = json.loads(payload)
        except ValueError:
            self.get_logger().warn(f"Bad MQTT payload on {topic}", throttle_duration_sec=5.0)
            return
        unit = report.get('surface_unit') or topic.rsplit('/', 1)[-1]
        report['surface_unit'] = unit
        report['recv_time'] = time.time()
        report['meas_time'] = self._measurement_time(report, unit)
        with self._lock:
            self._latest[unit] = report
            if report.get('range_m') is not None and report.get('unit_lat') is not None \
                    and report.get('unit_lon') is not None:
                self._pending.append(report)

    def _measurement_time(self, report, unit):
        """Epoch (s, this host's clock) at which the range was measured.

        Prefers the surface unit's local 'stamp' so MQTT/network delay does not
        mis-time the measurement; falls back to arrival time if the stamp is
        missing or the sender clock looks unsynced (see inference_node)."""
        recv = report['recv_time']
        if not self.use_report_stamp:
            return recv
        stamp = report.get('stamp')
        if not isinstance(stamp, (int, float)):
            return recv
        skew = recv - float(stamp)
        if -self.max_clock_skew_s <= skew <= self.max_clock_skew_s:
            return float(stamp)
        self.get_logger().warn(
            f"Report from '{unit}' stamp {skew:.1f}s from arrival (> "
            f"max_clock_skew_s {self.max_clock_skew_s:.0f}s); using arrival time.",
            throttle_duration_sec=10.0)
        return recv

    @staticmethod
    def _meas_time(report):
        return report.get('meas_time', report.get('recv_time', 0.0))

    # ------------------------------------------------------------- ROS timer
    def update(self):
        now = time.time()
        with self._lock:
            located = [r for r in self._latest.values()
                       if (now - self._meas_time(r)) <= self.freshness_window_s
                       and r.get('unit_lat') is not None and r.get('unit_lon') is not None]
            pending = self._pending
            self._pending = []

        # Receivers + telemetry passthrough -- independent of range validity.
        self._publish_receivers(located)
        self._publish_latest_telemetry(located)

        # Fuse every new range measurement into the graph (each at its own time).
        fresh_ranges = [r for r in pending if (now - self._meas_time(r)) <= self.lag_s]
        if fresh_ranges:
            self._fuse(sorted(fresh_ranges, key=self._meas_time))

        # Publish the current smoothed estimate (+ mirror) from the latest node.
        self._publish_estimate(located)

    def _fuse(self, reports):
        """Insert a batch of range reports into the fixed-lag smoother.

        Each report lands on the grid node nearest its OWN measurement time, so a
        late/out-of-order range is fused at the correct past node (as long as that
        node is still inside the lag window) and the whole window re-optimises.
        """
        if not self._init_graph(reports):
            return
        self._latest_meas_t = max(self._latest_meas_t, max(self._meas_time(r) for r in reports))
        k_min = self._k_index(self._latest_meas_t - self.lag_s)

        graph = gtsam.NonlinearFactorGraph()
        added = 0
        for r in reports:
            t = self._meas_time(r)
            k = self._k_index(t)
            if k < max(0, k_min):
                self.get_logger().warn(
                    f"Range from '{r['surface_unit']}' is "
                    f"{self._latest_meas_t - t:.1f}s late (> lag {self.lag_s:.0f}s); "
                    "its node is marginalised, dropping.", throttle_duration_sec=5.0)
                continue
            if not self._ensure_node(k, t):
                continue
            anchor = geodetic_to_enu(float(r['unit_lat']), float(r['unit_lon']), *self._origin)
            rng = self._horizontal_range(float(r['range_m']), self._report_depth(r))
            graph.add(make_range_factor(X(k), anchor, rng, self._range_noise))
            rep = self._report_position(r)
            if self.use_reported_position and rep is not None:
                pe, pn = geodetic_to_enu(rep[0], rep[1], *self._origin)
                graph.add(gtsam.PriorFactorVector(X(k), np.array([pe, pn]), self._reported_noise))
            added += 1
        if added:
            self._safe_update(graph, gtsam.Values(), {})
    # --------------------------------------------------------------- graph
    def _k_index(self, t):
        return max(0, int(round((t - self._t0) / self.node_period_s)))

    def _build_noise(self):
        n = gtsam.noiseModel
        self._range_noise = n.Isotropic.Sigma(1, max(1e-3, self.range_std_m))
        self._reported_noise = n.Isotropic.Sigma(2, max(1e-3, self.reported_pos_std_m))
        self._pos_prior_noise = n.Isotropic.Sigma(2, max(1e-3, self.init_pos_std_m))
        self._vel_prior_noise = n.Isotropic.Sigma(2, max(1e-3, self.init_vel_std_mps))

    def _cv_noise(self, dt):
        q = max(1e-3, self.process_accel_std_mps2)
        sp = max(0.05, 0.5 * q * dt * dt)
        sv = max(0.02, q * dt)
        return gtsam.noiseModel.Diagonal.Sigmas(np.array([sp, sp, sv, sv]))

    def _init_graph(self, reports):
        """Lazily create the smoother + ENU frame from the first usable batch."""
        if self._sm is not None:
            return True
        seed = self._init_seed(reports)
        if seed is None:
            with self._lock:                          # retry these next tick
                self._pending = reports + self._pending
            self.get_logger().info(
                "Waiting for a seed (>=2 receivers, a reported GPS, or "
                "inference.initial_lat/lon) before starting the smoother.",
                throttle_duration_sec=5.0)
            return False
        params = gtsam.ISAM2Params()
        self._sm = gtsam.IncrementalFixedLagSmoother(self.lag_s, params)
        self._origin = seed
        self._t0 = min(self._meas_time(r) for r in reports)
        self._latest_meas_t = self._t0
        self._nodes = set()
        self._frontier = -1
        self._build_noise()
        self.get_logger().info(
            f"Smoother frame seeded at lat={seed[0]:.7f}, lon={seed[1]:.7f}.")
        return True

    def _init_seed(self, reports):
        if self.initial_lat != 0.0 or self.initial_lon != 0.0:
            return (self.initial_lat, self.initial_lon)
        rep = self._reported_position(reports)
        if rep is not None:
            return rep
        latest = {}
        for r in reports:
            latest[r['surface_unit']] = r
        units = list(latest.values())
        if len(units) >= 2:
            return self._triangulate_seed(units)
        return None

    def _triangulate_seed(self, units):
        lat0 = sum(float(r['unit_lat']) for r in units) / len(units)
        lon0 = sum(float(r['unit_lon']) for r in units) / len(units)
        anchors = [geodetic_to_enu(float(r['unit_lat']), float(r['unit_lon']), lat0, lon0)
                   for r in units]
        ranges = [self._horizontal_range(float(r['range_m']), self._report_depth(r))
                  for r in units]
        best, best_d = None, -1.0
        for i in range(len(anchors)):
            for j in range(i + 1, len(anchors)):
                d = math.hypot(anchors[i][0] - anchors[j][0], anchors[i][1] - anchors[j][1])
                if d > best_d:
                    best_d, best = d, (i, j)
        if best is None:
            return None
        i, j = best
        sols = circle_intersections(anchors[i], ranges[i], anchors[j], ranges[j])
        if not sols:
            return None
        geos = [enu_to_geodetic(e, n, lat0, lon0) for (e, n) in sols]
        return self._pick_solution(geos)

    def _ensure_node(self, k, t):
        """Create grid node ``k`` (position+velocity) if absent, chained by a CV
        factor to its nearest existing neighbour. Returns False on failure."""
        if k in self._nodes:
            return True
        prev = max((i for i in self._nodes if i < k), default=None)
        nxt = min((i for i in self._nodes if i > k), default=None)
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        ts = {X(k): t, V(k): t}
        if prev is not None:
            pe, pn, pve, pvn = self._node_state(prev)
            dt = max(self.node_period_s, (k - prev) * self.node_period_s)
            seed = np.array([pe + pve * dt, pn + pvn * dt])
            vel = np.array([pve, pvn])
            graph.add(make_cv_factor(X(prev), V(prev), X(k), V(k), dt, self._cv_noise(dt)))
        elif nxt is not None:
            ne, nn, nve, nvn = self._node_state(nxt)
            dt = max(self.node_period_s, (nxt - k) * self.node_period_s)
            seed = np.array([ne - nve * dt, nn - nvn * dt])
            vel = np.array([nve, nvn])
            graph.add(make_cv_factor(X(k), V(k), X(nxt), V(nxt), dt, self._cv_noise(dt)))
        else:
            # Very first node: anchor the frame with priors at the seed (origin).
            seed = np.array([0.0, 0.0])
            vel = np.array([0.0, 0.0])
            graph.add(gtsam.PriorFactorVector(X(k), seed, self._pos_prior_noise))
            graph.add(gtsam.PriorFactorVector(V(k), vel, self._vel_prior_noise))
        values.insert(X(k), seed)
        values.insert(V(k), vel)
        if not self._safe_update(graph, values, ts):
            return False
        self._nodes.add(k)
        self._frontier = max(self._frontier, k)
        return True

    def _node_state(self, k):
        """(E, N, vE, vN) estimate at node k, or zeros if unavailable."""
        try:
            est = self._sm.calculateEstimate()
            x = est.atVector(X(k))
            v = est.atVector(V(k))
            return float(x[0]), float(x[1]), float(v[0]), float(v[1])
        except Exception:
            return 0.0, 0.0, 0.0, 0.0

    def _safe_update(self, graph, values, ts):
        try:
            self._sm.update(graph, values, ts)
            return True
        except Exception as exc:
            self.get_logger().warn(f"Smoother update failed: {exc}",
                                   throttle_duration_sec=5.0)
            return False

    # ------------------------------------------------------- estimate output
    def _publish_estimate(self, located):
        if self._sm is None or not self._nodes:
            return
        kmax = self._frontier
        e, n, ve, vn = self._node_state(kmax)
        lat, lon = enu_to_geodetic(e, n, *self._origin)
        self._prev_estimate = (lat, lon)
        epoch = self._t0 + kmax * self.node_period_s
        self._set_smooth_target(lat, lon, ve, vn, epoch)

        units = [r for r in located if r.get('range_m') is not None]
        seen = {}
        for r in units:
            seen[r['surface_unit']] = r
        if self.publish_both and len(seen) == 2:
            mirror = self._two_circle_mirror(list(seen.values()), (lat, lon))
            if mirror is not None:
                self._publish_fix(self.fix_alt_pub, mirror[0], mirror[1])

    def _two_circle_mirror(self, units, primary):
        """The reflected 2-receiver solution farther from the primary, or None."""
        lat0 = sum(float(r['unit_lat']) for r in units) / len(units)
        lon0 = sum(float(r['unit_lon']) for r in units) / len(units)
        anchors = [geodetic_to_enu(float(r['unit_lat']), float(r['unit_lon']), lat0, lon0)
                   for r in units]
        ranges = [self._horizontal_range(float(r['range_m']), self._report_depth(r))
                  for r in units]
        sols = circle_intersections(anchors[0], ranges[0], anchors[1], ranges[1])
        if not sols:
            return None
        geos = [enu_to_geodetic(e, n, lat0, lon0) for (e, n) in sols]
        return max(geos, key=lambda g: self._dist_m(g, primary))

    # --------------------------------------------------------- smooth output
    def _set_smooth_target(self, lat, lon, vE, vN, t):
        self._smooth = (lat, lon, vE, vN, t)

    def _publish_smooth(self):
        sm = self._smooth
        if sm is None:
            return
        lat0, lon0, vE, vN, t = sm
        dt = time.time() - t
        if dt < 0.0 or dt > self.freshness_window_s:
            if dt > self.freshness_window_s:
                return
            dt = 0.0
        sp = math.hypot(vE, vN)
        if sp > self.max_speed_mps and sp > 0.0:
            scale = self.max_speed_mps / sp
            vE, vN = vE * scale, vN * scale
        lat, lon = enu_to_geodetic(vE * dt, vN * dt, lat0, lon0)
        self._publish_fix(self.fix_pub, lat, lon)
    # ------------------------------------------------------------- helpers
    def _report_depth(self, report):
        t = report.get('telemetry')
        if isinstance(t, dict) and t.get('depth') is not None:
            try:
                return max(0.0, float(t['depth']))
            except (TypeError, ValueError):
                pass
        return max(0.0, self.assumed_depth_m)

    @staticmethod
    def _horizontal_range(slant_m, depth_m):
        if depth_m <= 0.0:
            return slant_m
        if slant_m <= depth_m:
            return 0.5
        return math.sqrt(slant_m * slant_m - depth_m * depth_m)

    @staticmethod
    def _report_position(report):
        t = report.get('telemetry')
        if isinstance(t, dict) and isinstance(t.get('position'), dict):
            p = t['position']
            try:
                if p.get('lat') is not None and p.get('lon') is not None:
                    return (float(p['lat']), float(p['lon']))
            except (TypeError, ValueError):
                pass
        return None

    def _reported_position(self, reports):
        with_pos = [r for r in reports if self._report_position(r) is not None]
        if not with_pos:
            return None
        return self._report_position(max(with_pos, key=self._meas_time))

    def _pick_solution(self, geos):
        if len(geos) == 1 or geos[0] == geos[1]:
            return geos[0]
        if self._prev_estimate is not None:
            plat, plon = self._prev_estimate
            return min(geos, key=lambda g: (g[0] - plat) ** 2 + (g[1] - plon) ** 2)
        pref = self.first_fix_prefer
        if pref == 'south':
            return min(geos, key=lambda g: g[0])
        if pref == 'east':
            return max(geos, key=lambda g: g[1])
        if pref == 'west':
            return min(geos, key=lambda g: g[1])
        return max(geos, key=lambda g: g[0])

    @staticmethod
    def _dist_m(a, b):
        e, n = geodetic_to_enu(b[0], b[1], a[0], a[1])
        return math.hypot(e, n)

    def _publish_receivers(self, located):
        for r in located:
            unit = r.get('surface_unit')
            if not unit:
                continue
            pub = self._receiver_pubs.get(unit)
            if pub is None:
                topic = f"/owtt_beacon/{self.beacon_name}/receiver/{self._safe_token(unit)}"
                pub = self.create_publisher(NavSatFix, topic, 10)
                self._receiver_pubs[unit] = pub
                self.get_logger().info(f"Publishing receiver '{unit}' position on {topic}")
            self._publish_fix(pub, float(r['unit_lat']), float(r['unit_lon']))

    @staticmethod
    def _safe_token(name):
        token = re.sub(r'[^0-9A-Za-z_]', '_', str(name))
        if not token or not (token[0].isalpha() or token[0] == '_'):
            token = '_' + token
        return token

    def _publish_latest_telemetry(self, located):
        if not located:
            return
        latest = max(located, key=self._meas_time)
        telem = latest.get('telemetry') or {}
        msg = String()
        msg.data = json.dumps(telem)
        self.telemetry_pub.publish(msg)
        pos = self._report_position(latest)
        if pos is not None:
            self._publish_fix(self.reported_pub, pos[0], pos[1])

    def _publish_fix(self, pub, lat, lon):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = 0.0
        pub.publish(msg)

    # ------------------------------------------------------ START/STOP cmds
    def _on_cmd_ack(self, payload):
        try:
            ack = json.loads(payload)
        except ValueError:
            return
        pend = self._pending_cmd
        if pend is None:
            return
        if ack.get('request_id') == pend['request_id'] \
                and str(ack.get('command', '')).upper() == pend['command'] \
                and ack.get('status') == 'OK':
            pend['unit'] = ack.get('unit')
            pend['event'].set()
            self.get_logger().info(
                f"{pend['command']} acked by '{ack.get('unit')}' (request {pend['request_id']}).")

    def _on_start_service(self, request, response):
        return self._run_command('START', response)

    def _on_stop_service(self, request, response):
        return self._run_command('STOP', response)

    def _run_command(self, command, response):
        if not self._cmd_lock.acquire(blocking=False):
            response.success = False
            response.message = "another START/STOP command is already in progress"
            return response
        try:
            request_id = uuid.uuid4().hex[:8]
            event = threading.Event()
            self._pending_cmd = {'command': command, 'request_id': request_id,
                                 'event': event, 'unit': None}
            self.get_logger().info(
                f"Service {command}: request {request_id}, timeout {self.command_timeout_s:.0f}s.")
            deadline = time.time() + self.command_timeout_s
            while time.time() < deadline:
                self._publish_cmd(command, request_id)
                wait = min(self.command_repeat_s, max(0.0, deadline - time.time()))
                if event.wait(timeout=wait):
                    break
            pend = self._pending_cmd
            self._pending_cmd = None
            if pend is not None and pend['event'].is_set():
                response.success = True
                response.message = f"{command} acknowledged by '{pend['unit']}' (beacon OK)."
                if command == 'START':
                    self._reset_filter()
                    response.message += " Smoother re-initialised."
                elif command == 'STOP':
                    self._reset_filter()
                    response.message += " Estimation halted."
            else:
                response.success = False
                response.message = (f"{command} timed out after {self.command_timeout_s:.0f}s "
                                    "with no beacon OK relayed.")
            self.get_logger().info(response.message)
            return response
        finally:
            self._cmd_lock.release()

    def _reset_filter(self):
        """Tear down the factor graph and buffers so the next batch re-seeds."""
        with self._lock:
            self._latest.clear()
            self._pending = []
            self._prev_estimate = None
            self._smooth = None
            self._sm = None
            self._origin = None
            self._t0 = None
            self._nodes = set()
            self._frontier = -1
            self._latest_meas_t = None
        self.get_logger().info("Smoother re-initialised (graph, frame, buffers cleared).")

    def _publish_cmd(self, command, request_id):
        payload = json.dumps({
            'command': command, 'request_id': request_id,
            'beacon': self.beacon_name, 'ts': time.time(),
        })
        if self.mqtt is not None:
            self.mqtt.publish(self.cmd_topic, payload, qos=1)
        self.get_logger().info(f"-> {command} (request {request_id}) on '{self.cmd_topic}'",
                               throttle_duration_sec=1.0)

    def destroy_node(self):
        try:
            self.mqtt.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SmootherInferenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
