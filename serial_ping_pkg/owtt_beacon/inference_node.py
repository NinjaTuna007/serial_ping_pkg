"""OWTT beacon inference node (MQTT -> triangulation -> map).

Runs anywhere with MQTT reachability (e.g. a laptop). It subscribes to the
range reports the surface units publish to MQTT, and from two units' known
positions + measured ranges it triangulates the beacon's position (two-circle
intersection) and publishes it as a ``sensor_msgs/NavSatFix`` for visualisation
(e.g. Foxglove map panel).

Each MQTT report (JSON, see ``surface_unit_node``) carries the publishing
unit's lat/lon, its measured range to the beacon, and the decoded beacon
telemetry. We keep the latest report per surface unit and, on a timer, fuse the
two freshest into a fix.

Two circles generally intersect at two points; we disambiguate by choosing the
solution closest to the motion-model prediction (and a configurable side on the
very first fix). With only two receivers the mirror solution is also published.

Depth: the measured acoustic range is a *slant* range to a possibly-submerged
beacon. If the beacon broadcasts its depth in telemetry (or an assumed depth is
configured), each slant range ``s`` is converted to a horizontal range
``sqrt(s^2 - d^2)`` before the (2-D) triangulation.

Resolving the 2-receiver mirror over time: the two-circle ambiguity reflects the
beacon across the *baseline* joining the two units. As the units move, that
baseline rotates, so the mirror "ghost" jumps around while the true position
stays put (or moves smoothly within the beacon's speed budget). We therefore run
a small **two-hypothesis tracker**: both branches are propagated with a
constant-velocity model (capped at ``max_speed_mps``, ~2 m/s for the XUUV) and
scored by accumulated motion-consistency. The ghost racks up large innovations /
impossible jumps and is demoted; the surviving hypothesis is the ``estimate``,
the loser the ``estimate_alt`` mirror. The CV clamp also rejects single bad
fixes (e.g. a frozen/outlier range).

Position seeding: if the beacon includes its own GPS in telemetry (e.g. for the
first few broadcasts before it dives, to save acoustic bytes), the tracker uses
that reported position to lock onto the correct branch immediately; once the
beacon goes silent on position, the converged tracker carries on.
"""

import json
import math
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

_EARTH_RADIUS_M = 6378137.0


def geodetic_to_enu(lat, lon, lat0, lon0):
    """Flat-earth local ENU (metres) of (lat, lon) about reference (lat0, lon0)."""
    dN = math.radians(lat - lat0) * _EARTH_RADIUS_M
    dE = math.radians(lon - lon0) * _EARTH_RADIUS_M * math.cos(math.radians(lat0))
    return dE, dN


def enu_to_geodetic(dE, dN, lat0, lon0):
    lat = lat0 + math.degrees(dN / _EARTH_RADIUS_M)
    lon = lon0 + math.degrees(dE / (_EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


def circle_intersections(p1, r1, p2, r2):
    """Return up to two intersection points of two circles in the plane.

    ``p1``/``p2`` are (x, y); ``r1``/``r2`` are radii. If the circles do not
    intersect (too far apart, or one inside the other), returns the best single
    point on the baseline (the radical foot), duplicated, so the caller still
    gets an estimate. Returns an empty list only for concentric centres.
    """
    x1, y1 = p1
    x2, y2 = p2
    dx, dy = x2 - x1, y2 - y1
    d = math.hypot(dx, dy)
    if d == 0:
        return []
    ux, uy = dx / d, dy / d
    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    mx, my = x1 + a * ux, y1 + a * uy
    h2 = r1 * r1 - a * a
    if h2 < 0:
        # Circles don't meet; fall back to the point on the line between them.
        return [(mx, my), (mx, my)]
    h = math.sqrt(h2)
    px, py = -uy, ux
    return [(mx + h * px, my + h * py), (mx - h * px, my - h * py)]


def multilaterate(anchors, ranges, init, iters=12, tol=1e-3):
    """Least-squares multilateration of a point from N>=2 range circles.

    ``anchors`` is a list of (x, y) anchor positions and ``ranges`` the measured
    range to each, all in a local planar (ENU) frame. Minimises
    ``sum_i (||x - anchor_i|| - range_i)^2`` with Gauss-Newton, seeded from
    ``init`` (which also resolves the two-point ambiguity for N == 2). Naturally
    uses ALL anchors, so accuracy improves as more receivers report.
    """
    P = np.asarray(anchors, dtype=float)
    r = np.asarray(ranges, dtype=float)
    x = np.asarray(init, dtype=float)
    for _ in range(iters):
        d = x - P                                   # (N, 2)
        dist = np.hypot(d[:, 0], d[:, 1])
        dist = np.where(dist < 1e-6, 1e-6, dist)
        res = dist - r                              # (N,)
        J = d / dist[:, None]                       # (N, 2)
        H = J.T @ J
        g = J.T @ res
        try:
            step = np.linalg.solve(H, -g)
        except np.linalg.LinAlgError:
            break
        x = x + step
        if math.hypot(step[0], step[1]) < tol:
            break
    return float(x[0]), float(x[1])


class InferenceNode(Node):
    def __init__(self):
        super().__init__('owtt_inference_node')

        config = load_yaml_config('serial_ping_pkg', 'owtt_beacon_config.yaml')
        mqtt_cfg = config.get('mqtt', {})
        inf_cfg = config.get('inference', {})
        beacon_cfg = config.get('beacon', {})

        self.declare_parameter('beacon.name', beacon_cfg.get('name', 'lolo'))

        self.declare_parameter('mqtt.host', mqtt_cfg.get('host', 'localhost'))
        self.declare_parameter('mqtt.port', mqtt_cfg.get('port', 1884))
        self.declare_parameter('mqtt.keepalive', mqtt_cfg.get('keepalive', 30))
        self.declare_parameter('mqtt.username', mqtt_cfg.get('username', ''))
        self.declare_parameter('mqtt.password', mqtt_cfg.get('password', ''))
        self.declare_parameter('mqtt.topic_prefix', mqtt_cfg.get('topic_prefix', 'owtt_beacon'))

        self.declare_parameter('inference.update_period_s', inf_cfg.get('update_period_s', 1.0))
        self.declare_parameter('inference.freshness_window_s', inf_cfg.get('freshness_window_s', 10.0))
        self.declare_parameter('inference.min_units', inf_cfg.get('min_units', 2))
        self.declare_parameter('inference.frame_id', inf_cfg.get('frame_id', 'map'))
        # first-fix disambiguation: which of the two solutions to prefer before
        # we have a previous estimate: north | south | east | west
        self.declare_parameter('inference.first_fix_prefer', inf_cfg.get('first_fix_prefer', 'north'))
        # With only 2 receivers the fix is genuinely two-fold ambiguous; publish
        # the mirror solution too (on .../estimate_alt) so both show on the map.
        self.declare_parameter('inference.publish_both', inf_cfg.get('publish_both', True))
        # Beacon depth (m, +down) to use when telemetry carries none: converts the
        # measured slant range to a horizontal range. 0 = treat ranges as horizontal.
        self.declare_parameter('inference.assumed_depth_m', inf_cfg.get('assumed_depth_m', 0.0))
        # Constant-velocity motion model: disambiguates over time + clamps jumps.
        self.declare_parameter('inference.motion_model', inf_cfg.get('motion_model', True))
        self.declare_parameter('inference.max_speed_mps', inf_cfg.get('max_speed_mps', 2.0))
        # Use the beacon's own reported GPS (telemetry 'position') to lock the
        # correct branch when it is available (e.g. seed broadcasts before diving).
        self.declare_parameter('inference.use_reported_position', inf_cfg.get('use_reported_position', True))
        # START/STOP command orchestration (services -> MQTT -> surface units).
        self.declare_parameter('inference.command_timeout_s', inf_cfg.get('command_timeout_s', 10.0))
        self.declare_parameter('inference.command_repeat_s', inf_cfg.get('command_repeat_s', 1.5))

        self.beacon_name = self.get_parameter('beacon.name').get_parameter_value().string_value
        self.mqtt_host = self.get_parameter('mqtt.host').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter('mqtt.port').get_parameter_value().integer_value
        self.mqtt_keepalive = self.get_parameter('mqtt.keepalive').get_parameter_value().integer_value
        self.mqtt_username = self.get_parameter('mqtt.username').get_parameter_value().string_value
        self.mqtt_password = self.get_parameter('mqtt.password').get_parameter_value().string_value
        self.mqtt_topic_prefix = self.get_parameter('mqtt.topic_prefix').get_parameter_value().string_value
        self.update_period_s = self.get_parameter('inference.update_period_s').get_parameter_value().double_value
        self.freshness_window_s = self.get_parameter('inference.freshness_window_s').get_parameter_value().double_value
        self.min_units = self.get_parameter('inference.min_units').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('inference.frame_id').get_parameter_value().string_value
        self.first_fix_prefer = self.get_parameter('inference.first_fix_prefer').get_parameter_value().string_value.lower()
        self.publish_both = self.get_parameter('inference.publish_both').get_parameter_value().bool_value
        self.assumed_depth_m = self.get_parameter('inference.assumed_depth_m').get_parameter_value().double_value
        self.motion_model = self.get_parameter('inference.motion_model').get_parameter_value().bool_value
        self.max_speed_mps = self.get_parameter('inference.max_speed_mps').get_parameter_value().double_value
        self.use_reported_position = self.get_parameter('inference.use_reported_position').get_parameter_value().bool_value
        self.command_timeout_s = self.get_parameter('inference.command_timeout_s').get_parameter_value().double_value
        self.command_repeat_s = self.get_parameter('inference.command_repeat_s').get_parameter_value().double_value

        # Latest report per surface unit, guarded for the MQTT thread.
        self._lock = threading.Lock()
        self._reports = {}            # unit_name -> report dict (+ 'recv_time')
        self._prev_estimate = None    # (lat, lon) of the published primary fix

        # Two-hypothesis branch tracker state. Each hypothesis is a dict with
        # lat/lon, velocity (vE, vN) m/s, last update time, and an accumulated
        # cost (lower = more motion-consistent over its history).
        self._hyps = []
        self._hyp_decay = 0.6         # weight retained on past cost each epoch
        self._speed_penalty_w = 4.0   # cost weight on exceeding max_speed
        self._seed_penalty = 1.0e4    # cost bump to branches far from a reported GPS
        self._merge_thresh_m = 3.0    # collapse hypotheses closer than this

        self.fix_pub = self.create_publisher(
            NavSatFix, f"/owtt_beacon/{self.beacon_name}/estimate", 10)
        self.fix_alt_pub = self.create_publisher(
            NavSatFix, f"/owtt_beacon/{self.beacon_name}/estimate_alt", 10)
        self.reported_pub = self.create_publisher(
            NavSatFix, f"/owtt_beacon/{self.beacon_name}/reported", 10)
        self.telemetry_pub = self.create_publisher(
            String, f"/owtt_beacon/{self.beacon_name}/telemetry", 10)

        # START/STOP command orchestration state.
        beacon_sel = self.beacon_name if self.beacon_name else '+'
        self.cmd_topic = f"{self.mqtt_topic_prefix}/{self.beacon_name}/cmd"
        self.cmd_ack_topic = f"{self.mqtt_topic_prefix}/{beacon_sel}/cmd_ack"
        self._cmd_lock = threading.Lock()     # one command in flight at a time
        self._pending_cmd = None              # {'command','request_id','event','unit'}

        self.sub_topic = f"{self.mqtt_topic_prefix}/{beacon_sel}/range/+"
        self.mqtt = MqttClient(
            self.mqtt_host, self.mqtt_port, keepalive=self.mqtt_keepalive,
            username=(self.mqtt_username or None), password=(self.mqtt_password or None),
            client_id="owtt_inference", logger=self.get_logger())
        self.mqtt.set_message_handler(self._on_mqtt)
        self.mqtt.add_subscription(self.sub_topic)
        self.mqtt.add_subscription(self.cmd_ack_topic, qos=1)
        self.mqtt.start()

        # Services callable from Foxglove etc. Run in a reentrant group so the
        # blocking wait does not stall the estimation timer (MultiThreadedExecutor).
        self._srv_group = ReentrantCallbackGroup()
        self.start_srv = self.create_service(
            Trigger, f"/owtt_beacon/{self.beacon_name}/start",
            self._on_start_service, callback_group=self._srv_group)
        self.stop_srv = self.create_service(
            Trigger, f"/owtt_beacon/{self.beacon_name}/stop",
            self._on_stop_service, callback_group=self._srv_group)

        self.timer = self.create_timer(self.update_period_s, self.update)
        self.get_logger().info(
            f"OWTT inference node up: MQTT {self.mqtt_host}:{self.mqtt_port} "
            f"sub '{self.sub_topic}', publishing beacon '{self.beacon_name}' estimate. "
            f"Services: /owtt_beacon/{self.beacon_name}/start|stop.")

    # ------------------------------------------------------------------ MQTT thread

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
        report['recv_time'] = time.time()
        with self._lock:
            self._reports[unit] = report

    def _on_cmd_ack(self, payload):
        """A surface unit relayed the beacon's OK for the in-flight command."""
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

    # ------------------------------------------------------------------ services

    def _on_start_service(self, request, response):
        return self._run_command('START', response)

    def _on_stop_service(self, request, response):
        return self._run_command('STOP', response)

    def _run_command(self, command, response):
        """Publish the command to MQTT, re-sending until a surface unit relays
        the beacon's OK, or the timeout elapses. Returns a std_srvs/Trigger."""
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
                # A fresh START means a new run: drop all stale tracker state and
                # buffered reports so the filter re-initialises from scratch.
                if command == 'START':
                    self._reset_filter()
                    response.message += " Filter re-initialised."
            else:
                response.success = False
                response.message = (f"{command} timed out after {self.command_timeout_s:.0f}s "
                                    "with no beacon OK relayed.")
            self.get_logger().info(response.message)
            return response
        finally:
            self._cmd_lock.release()

    def _reset_filter(self):
        """Clear the tracker hypotheses, last estimate, and buffered range
        reports so the next update() starts a fresh estimation run."""
        with self._lock:
            self._hyps = []
            self._prev_estimate = None
            self._reports.clear()
        self.get_logger().info("Filter re-initialised (hypotheses, last fix, and reports cleared).")

    def _publish_cmd(self, command, request_id):
        payload = json.dumps({
            'command': command, 'request_id': request_id,
            'beacon': self.beacon_name, 'ts': time.time(),
        })
        if self.mqtt is not None:
            self.mqtt.publish(self.cmd_topic, payload, qos=1)
        self.get_logger().info(f"-> {command} (request {request_id}) on '{self.cmd_topic}'",
                               throttle_duration_sec=1.0)

    # ------------------------------------------------------------------ ROS timer

    def update(self):
        now = time.time()
        with self._lock:
            fresh = [r for r in self._reports.values()
                     if (now - r.get('recv_time', 0.0)) <= self.freshness_window_s
                     and r.get('unit_lat') is not None
                     and r.get('unit_lon') is not None
                     and r.get('range_m') is not None]

        # Republish the most recent beacon telemetry / self-reported position.
        self._publish_latest_telemetry(fresh)

        if len(fresh) < max(2, self.min_units):
            self.get_logger().info(
                f"Need >={max(2, self.min_units)} fresh surface-unit reports to "
                f"triangulate; have {len(fresh)}.", throttle_duration_sec=5.0)
            return

        sols = self._triangulate(fresh)
        if not sols:
            return

        reported = self._reported_position(fresh) if self.use_reported_position else None
        primary, mirror = self._estimate(sols, now, reported)
        lat, lon = primary
        self._prev_estimate = primary
        self._publish_fix(self.fix_pub, lat, lon)

        units = ', '.join(f"{r['surface_unit']}={r['range_m']:.1f}m" for r in fresh)
        if not (self.publish_both and len(sols) == 2):
            mirror = None
        if mirror is not None:
            mlat, mlon = mirror
            self._publish_fix(self.fix_alt_pub, mlat, mlon)
            self.get_logger().info(
                f"Beacon '{self.beacon_name}' estimate: lat={lat:.7f}, lon={lon:.7f} "
                f"| mirror: lat={mlat:.7f}, lon={mlon:.7f} (2-receiver ambiguity) "
                f"({len(fresh)} units: {units})")
        else:
            self.get_logger().info(
                f"Beacon '{self.beacon_name}' estimate: lat={lat:.7f}, lon={lon:.7f} "
                f"({len(fresh)} units: {units})")

    def _triangulate(self, fresh):
        """Multilaterate from all fresh reports; returns a list of solutions.

        Slant ranges are first corrected to horizontal ranges using the beacon
        depth. Returns the refined candidate solutions as geodetic tuples: two
        (distinct) for the genuinely two-fold 2-receiver case, one for the
        unique >=3-receiver least-squares fix, or ``[]`` if degenerate.
        """
        lat0 = sum(r['unit_lat'] for r in fresh) / len(fresh)
        lon0 = sum(r['unit_lon'] for r in fresh) / len(fresh)
        anchors = [geodetic_to_enu(r['unit_lat'], r['unit_lon'], lat0, lon0) for r in fresh]
        ranges = [self._horizontal_range(float(r['range_m']), fresh) for r in fresh]

        geos = self._pair_solutions(anchors, ranges, lat0, lon0)
        if not geos:
            self.get_logger().warn("Surface units report identical positions; cannot triangulate.",
                                   throttle_duration_sec=5.0)
            return []

        # Refine each candidate seed over ALL anchors. For >=3 receivers both
        # seeds converge to the same least-squares point, so de-duplicate.
        sols = []
        for seed in geos:
            refined = self._refine(anchors, ranges, seed, lat0, lon0)
            if not any(abs(refined[0] - s[0]) < 1e-7 and abs(refined[1] - s[1]) < 1e-7
                       for s in sols):
                sols.append(refined)
        return sols

    def _horizontal_range(self, slant_m, fresh):
        """Convert a slant range to a horizontal range using the beacon depth.

        Surface units sit at the surface (depth ~0); the beacon may be at depth
        ``d`` (from telemetry, else ``assumed_depth_m``). If ``slant <= d`` the
        unit is essentially overhead, so clamp to a small positive floor.
        """
        d = self._beacon_depth(fresh)
        if d <= 0.0:
            return slant_m
        if slant_m <= d:
            self.get_logger().warn(
                f"Slant range {slant_m:.1f}m <= beacon depth {d:.1f}m; "
                "clamping horizontal range to 0.5m.", throttle_duration_sec=10.0)
            return 0.5
        return math.sqrt(slant_m * slant_m - d * d)

    def _beacon_depth(self, fresh):
        """Beacon depth (m, +down): newest telemetry depth, else assumed default."""
        with_depth = [r for r in fresh
                      if isinstance(r.get('telemetry'), dict)
                      and r['telemetry'].get('depth') is not None]
        if with_depth:
            newest = max(with_depth, key=lambda r: r['recv_time'])
            try:
                return max(0.0, float(newest['telemetry']['depth']))
            except (TypeError, ValueError):
                pass
        return max(0.0, self.assumed_depth_m)

    # ------------------------------------------------------------ estimation

    def _estimate(self, sols, now, reported):
        """Return (primary_latlon, mirror_latlon|None) for this epoch.

        With the motion model off it is memoryless: prefer the branch matching
        the reported GPS, else the side-based first-fix pick. With the motion
        model on it runs the two-hypothesis branch tracker.
        """
        if not self.motion_model:
            if reported is not None and len(sols) > 1:
                primary = min(sols, key=lambda s: self._dist_m(s, reported))
            else:
                primary = self._pick_solution(sols)
            mirror = next((s for s in sols if s is not primary), None) if len(sols) > 1 else None
            return primary, mirror
        return self._track(sols, now, reported)

    def _track(self, sols, now, reported):
        """Two-hypothesis constant-velocity tracker over the candidate branches.

        Both branches are propagated and scored by accumulated motion
        consistency; as the receiver baseline changes, the mirror branch accrues
        large innovations / over-speed penalties and loses. A reported GPS, when
        present, strongly favours the branch nearest it.
        """
        if not self._hyps:
            self._hyps = [self._new_hyp(s, now) for s in sols]
        else:
            self._associate_and_update(sols, now)
            # If we only had one hypothesis but now see two branches, spawn the
            # second so both are tracked from here on.
            if len(self._hyps) == 1 and len(sols) == 2:
                far = max(sols, key=lambda s: self._dist_m(s, (self._hyps[0]['lat'], self._hyps[0]['lon'])))
                seed = self._new_hyp(far, now)
                seed['cost'] = self._hyps[0]['cost'] + 1.0
                self._hyps.append(seed)

        if reported is not None:
            best = min(self._hyps, key=lambda h: self._dist_m((h['lat'], h['lon']), reported))
            for h in self._hyps:
                if h is not best:
                    h['cost'] += self._seed_penalty

        # Collapse near-duplicate hypotheses (e.g. once >=3 receivers make the
        # fix unique, or the branches converge).
        if len(self._hyps) == 2 and \
                self._dist_m((self._hyps[0]['lat'], self._hyps[0]['lon']),
                             (self._hyps[1]['lat'], self._hyps[1]['lon'])) < self._merge_thresh_m:
            self._hyps.sort(key=lambda h: h['cost'])
            self._hyps = self._hyps[:1]

        self._hyps.sort(key=lambda h: h['cost'])
        primary = (self._hyps[0]['lat'], self._hyps[0]['lon'])
        mirror = None
        if len(self._hyps) > 1 and len(sols) == 2:
            mirror = (self._hyps[1]['lat'], self._hyps[1]['lon'])
        return primary, mirror

    def _new_hyp(self, latlon, now):
        return {'lat': latlon[0], 'lon': latlon[1], 'vE': 0.0, 'vN': 0.0,
                't': now, 'cost': 0.0}

    def _associate_and_update(self, sols, now):
        preds = [self._predict(h, now) for h in self._hyps]
        if len(self._hyps) == 2 and len(sols) == 2:
            ident = self._dist_m(preds[0], sols[0]) + self._dist_m(preds[1], sols[1])
            swap = self._dist_m(preds[0], sols[1]) + self._dist_m(preds[1], sols[0])
            pairs = [(0, 0), (1, 1)] if ident <= swap else [(0, 1), (1, 0)]
        else:
            pairs = [(i, min(range(len(sols)), key=lambda j: self._dist_m(preds[i], sols[j])))
                     for i in range(len(self._hyps))]
        for hi, sj in pairs:
            self._update_hyp(self._hyps[hi], preds[hi], sols[sj], now)

    def _update_hyp(self, h, pred, cand, now):
        dt = max(1e-3, now - h['t'])
        innov = self._dist_m(pred, cand)
        step = self._dist_m((h['lat'], h['lon']), cand)
        over = max(0.0, step - self.max_speed_mps * dt)
        h['cost'] = self._hyp_decay * h['cost'] + innov * innov + self._speed_penalty_w * over * over
        # Move toward the candidate, clamped to the max-speed step.
        ce, cn = geodetic_to_enu(cand[0], cand[1], h['lat'], h['lon'])
        dist = math.hypot(ce, cn)
        max_dist = self.max_speed_mps * dt
        if dist > max_dist and dist > 0.0:
            scale = max_dist / dist
            ce, cn = ce * scale, cn * scale
        new_lat, new_lon = enu_to_geodetic(ce, cn, h['lat'], h['lon'])
        h['vE'], h['vN'] = ce / dt, cn / dt
        h['lat'], h['lon'], h['t'] = new_lat, new_lon, now

    def _predict(self, h, now):
        dt = max(0.0, now - h['t'])
        return enu_to_geodetic(h['vE'] * dt, h['vN'] * dt, h['lat'], h['lon'])

    @staticmethod
    def _dist_m(a, b):
        e, n = geodetic_to_enu(b[0], b[1], a[0], a[1])
        return math.hypot(e, n)

    def _reported_position(self, fresh):
        """Newest beacon-reported GPS (lat, lon) from telemetry, or None."""
        with_pos = [r for r in fresh
                    if isinstance(r.get('telemetry'), dict)
                    and isinstance(r['telemetry'].get('position'), dict)
                    and r['telemetry']['position'].get('lat') is not None
                    and r['telemetry']['position'].get('lon') is not None]
        if not with_pos:
            return None
        newest = max(with_pos, key=lambda r: r['recv_time'])
        p = newest['telemetry']['position']
        try:
            return (float(p['lat']), float(p['lon']))
        except (TypeError, ValueError):
            return None

    def _refine(self, anchors, ranges, seed_latlon, lat0, lon0):
        x, y = multilaterate(anchors, ranges, geodetic_to_enu(*seed_latlon, lat0, lon0))
        return enu_to_geodetic(x, y, lat0, lon0)

    def _pair_solutions(self, anchors, ranges, lat0, lon0):
        """Both circle solutions of the best-separated receiver pair (geodetic).

        The widest-separated pair has the strongest geometry; its two circle
        intersections seed the solver and (for 2 receivers) are the two
        candidate fixes. Returns 1 or 2 geodetic tuples, or [] if degenerate.
        """
        best, best_d = None, -1.0
        for i in range(len(anchors)):
            for j in range(i + 1, len(anchors)):
                d = math.hypot(anchors[i][0] - anchors[j][0], anchors[i][1] - anchors[j][1])
                if d > best_d:
                    best_d, best = d, (i, j)
        if best is None:
            return []
        i, j = best
        sols = circle_intersections(anchors[i], ranges[i], anchors[j], ranges[j])
        if not sols:
            return []
        return [enu_to_geodetic(e, n, lat0, lon0) for (e, n) in sols]

    def _pick_solution(self, geos):
        if len(geos) == 1 or geos[0] == geos[1]:
            return geos[0]
        if self._prev_estimate is not None:
            plat, plon = self._prev_estimate
            return min(geos, key=lambda g: (g[0] - plat) ** 2 + (g[1] - plon) ** 2)
        # First fix: pick by configured side.
        pref = self.first_fix_prefer
        if pref == 'south':
            return min(geos, key=lambda g: g[0])
        if pref == 'east':
            return max(geos, key=lambda g: g[1])
        if pref == 'west':
            return min(geos, key=lambda g: g[1])
        return max(geos, key=lambda g: g[0])  # 'north' (default)

    def _publish_latest_telemetry(self, fresh):
        if not fresh:
            return
        latest = max(fresh, key=lambda r: r['recv_time'])
        telem = latest.get('telemetry') or {}
        msg = String()
        msg.data = json.dumps(telem)
        self.telemetry_pub.publish(msg)
        pos = telem.get('position')
        if isinstance(pos, dict) and pos.get('lat') is not None and pos.get('lon') is not None:
            self._publish_fix(self.reported_pub, float(pos['lat']), float(pos['lon']))

    def _publish_fix(self, pub, lat, lon):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = 0.0
        pub.publish(msg)

    def destroy_node(self):
        try:
            self.mqtt.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    # Multi-threaded so a blocking START/STOP service call (waiting on the MQTT
    # ack) doesn't stall the estimation timer.
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
