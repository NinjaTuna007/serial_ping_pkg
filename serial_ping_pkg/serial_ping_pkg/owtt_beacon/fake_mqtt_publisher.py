#!/usr/bin/env python3
"""Fake MQTT range-report publisher for exercising the owtt_beacon stack.

Simulates a beacon flying a closed-loop trajectory and N fixed "buoy" surface
units bobbing about their moorings, and publishes JSON range reports in exactly
the schema ``surface_unit_node`` uses, to:

    <topic_prefix>/<beacon>/range/<unit>

so the ``owtt_inference_node`` (and whatever map you point at its ``NavSatFix``
output) can be tested end-to-end with no hardware. It loops until Ctrl+C. The
beacon trajectory is periodic, so every lap starts and ends at the same point;
the buoys only jitter around fixed positions.

The beacon loops a **simple circle** (radius ``--traj-radius``) centred on a
configurable offset from the scenario centre (default 90 m west and 150 m south),
so the beacon -- and its estimate -- are clearly moving round and round.

The simulated telemetry mirrors the beacon's real on-air sentence
(``P<lat,lon>;D<depth>;C<svs>;S<speed>;B<bt>``): it carries depth, sound velocity,
speed and the bt tip every report, and the beacon's own **position only while
seeding** -- the first ``--seed-seconds`` (default 30 s) after each START -- then
drops position (just like the real beacon saving acoustic bytes once seeded).

The buoys are spread on a wide **East-West baseline ~4x the circle radius**
(``--buoy-baseline``), centred just north of lolo's start, so the triangulation
geometry is well-conditioned. Each still **patrols back and forth** along an axis
rotated 90 deg from the previous one (perpendicular tracks), with a quarter-lap
phase stagger.

It models the beacon's **broadcast lifecycle**, so it works with the inference
node's ``/start`` and ``/stop`` services: by default it stays silent until a
START (just like a real beacon with ``autostart:=false``), streams range reports
while started, and goes quiet on STOP. It listens on ``<prefix>/<beacon>/cmd``
and, acting as the beacon, relays an idempotent ``OK`` to
``<prefix>/<beacon>/cmd_ack`` so the service call returns success. Pass
``--autostart`` to stream immediately (old behaviour, for pure inference tests),
or ``--no-command-ack`` when a *real* commander + units close the loop and you
only want the fake range stream to follow the same lifecycle.

It also fills the optional bits the pipeline consumes:
  * telemetry ``svs`` (sound velocity, jittered each report) -> what a real
    surface unit would use for its range conversion instead of a frozen value,
  * telemetry ``depth`` (when ``--depth`` > 0) -> tests slant->horizontal range,
  * telemetry ``position`` for the first ``--seed-count`` reports -> tests the
    branch-locking / reported-position path.

Examples:
    # defaults: smarc broker, lolo beacon, 2 buoys, ~1.5 m/s circle.
    # Silent until you call the /start service; stops on /stop.
    ros2 run serial_ping_pkg owtt_fake_mqtt

    # stream immediately without waiting for a START (pure inference test)
    ros2 run serial_ping_pkg owtt_fake_mqtt --autostart

    # local mosquitto, 3 receivers, GPS-seed first 5 reports, beacon at 12 m
    ros2 run serial_ping_pkg owtt_fake_mqtt --mqtt-host localhost --mqtt-port 1889 \
        --num-units 3 --seed-count 5 --depth 12
"""

import argparse
import json
import math
import random
import time

from serial_ping_pkg.owtt_beacon.mqtt_helper import MqttClient

_EARTH_RADIUS_M = 6378137.0


def enu_to_geodetic(dE, dN, lat0, lon0):
    """Flat-earth (lat, lon) of a local ENU offset (metres) about a reference."""
    lat = lat0 + math.degrees(dN / _EARTH_RADIUS_M)
    lon = lon0 + math.degrees(dE / (_EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


def beacon_enu(phase, args):
    """Beacon ENU position (m) at trajectory ``phase`` in [0, 2*pi).

    A simple circle of radius ``traj_radius`` centred on the configured offset,
    looping round and round so the beacon (and its estimate) are clearly moving.
    """
    be = args.traj_offset_east + args.traj_radius * math.cos(phase)
    bn = args.traj_offset_north + args.traj_radius * math.sin(phase)
    return be, bn


def estimate_speed(args, samples=720):
    """Numerically estimate (mean, max) ground speed (m/s) over one lap."""
    pts = [beacon_enu(2.0 * math.pi * i / samples, args) for i in range(samples + 1)]
    seglens = [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(samples)]
    dt = args.loop_period / samples
    perimeter = sum(seglens)
    v_mean = perimeter / max(1e-6, args.loop_period)
    v_max = max(seglens) / max(1e-6, dt)
    return v_mean, v_max


class _StdoutLogger:
    """Minimal logger shim so MqttClient can report connection state."""

    def _emit(self, level, msg):
        print(f"[{time.strftime('%H:%M:%S')}] {level}: {msg}", flush=True)

    def info(self, msg):
        self._emit('INFO', msg)

    def warn(self, msg):
        self._emit('WARN', msg)

    def error(self, msg):
        self._emit('ERROR', msg)


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Publish fake OWTT range reports to MQTT for testing.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # MQTT
    p.add_argument('--mqtt-host', default='20.240.40.232', help='MQTT broker host')
    p.add_argument('--mqtt-port', type=int, default=1884, help='MQTT broker port')
    p.add_argument('--username', default='', help='MQTT username (optional)')
    p.add_argument('--password', default='', help='MQTT password (optional)')
    p.add_argument('--topic-prefix', default='tuper/owtt_beacon',
                   help='Report topic prefix: <prefix>/<beacon>/range/<unit>')

    # Scenario identity
    p.add_argument('--beacon-name', default='lolo', help='Beacon name in the topic/report')
    p.add_argument('--beacon-modem-id', default='101', help='Beacon modem id in the report')

    # Geometry (centre defaults to the Asko 2026 test spot)
    p.add_argument('--center-lat', type=float, default=58.82322874, help='Scenario centre latitude')
    p.add_argument('--center-lon', type=float, default=17.63599778, help='Scenario centre longitude')
    p.add_argument('--num-units', type=int, default=2, help='Number of surface-unit buoys')
    p.add_argument('--buoy-baseline', type=float, default=0.0,
                   help='Separation (m) between buoys, spread E-W (0 = auto = 4x the circle radius)')
    p.add_argument('--buoy-north-gap', type=float, default=50.0,
                   help='Buoy baseline sits this far (m) north of lolo\'s start')
    p.add_argument('--buoy-east-shift', type=float, default=-30.0,
                   help='Shift the whole buoy baseline this far (m) east of lolo\'s start (negative = west)')
    p.add_argument('--buoy-jitter', type=float, default=1.0,
                   help='Std-dev (m) of each buoy bobbing about its patrol point')
    p.add_argument('--buoy-patrol', type=float, default=25.0,
                   help='Amplitude (m) each buoy patrols back and forth (0 = stay put)')
    p.add_argument('--buoy-patrol-period', type=float, default=90.0,
                   help='Seconds for one full back-and-forth buoy patrol')
    p.add_argument('--traj-radius', type=float, default=30.0,
                   help='Beacon circle radius (m)')
    p.add_argument('--traj-offset-east', type=float, default=-90.0,
                   help='East offset (m) of the circle centre from the scenario centre (default 90 m west)')
    p.add_argument('--traj-offset-north', type=float, default=-150.0,
                   help='North offset (m) of the circle centre from the scenario centre (default 150 m south)')
    p.add_argument('--loop-period', type=float, default=100.0,
                   help='Seconds for one full circle lap (smaller = faster; keeps speed under ~2 m/s)')
    p.add_argument('--depth', type=float, default=0.0,
                   help='Beacon depth (m); reports slant range + telemetry depth')

    # Measurement / publish behaviour
    p.add_argument('--period', type=float, default=1.0, help='Publish period per unit (s)')
    p.add_argument('--range-noise', type=float, default=0.5, help='Std-dev (m) of range noise')
    p.add_argument('--sound-velocity', type=float, default=1481.6,
                   help='Beacon sound velocity (m/s); broadcast in telemetry as svs and used for range')
    p.add_argument('--svs-drift', type=float, default=0.5,
                   help='Std-dev (m/s) of per-report svs jitter (proves it is not frozen)')
    p.add_argument('--offset-us', type=float, default=0.0, help='offset_us field in the report (cosmetic)')
    p.add_argument('--seed-seconds', type=float, default=30.0,
                   help='Include the beacon true GPS in telemetry for the first N seconds after each START (0=disabled)')
    p.add_argument('--seed-count', type=int, default=0,
                   help='Also seed for at least the first N reports after each START (OR-ed with --seed-seconds; 0=off)')
    p.add_argument('--bt-text', default='move_to', help='Fake bt tip string in telemetry (mimics lolo, already basenamed)')
    p.add_argument('--seed', type=int, default=None, help='RNG seed for reproducible jitter/noise')
    p.add_argument('--autostart', action='store_true',
                   help='Begin broadcasting immediately instead of waiting for a START command')
    p.add_argument('--ignore-commands', action='store_true',
                   help='Ignore the START/STOP command channel entirely (implies --autostart)')
    p.add_argument('--no-command-ack', action='store_true',
                   help='Follow the START/STOP lifecycle but do NOT relay the OK ack '
                        '(use when a real commander + units close the loop)')
    p.add_argument('--command-ack-delay', type=float, default=0.4,
                   help='Simulated acoustic round-trip before the OK ack (s)')

    return p.parse_known_args(argv)[0]


def main(argv=None):
    args = parse_args(argv)
    if args.seed is not None:
        random.seed(args.seed)

    log = _StdoutLogger()

    lat0, lon0 = args.center_lat, args.center_lon
    n_units = max(1, args.num_units)

    # The buoys are spread along an East-West baseline ~4x the circle radius (so
    # the triangulation geometry is well-conditioned), centred just north of
    # lolo's start (shiftable E/W via buoy_east_shift). buoy_1 is the WEST buoy
    # patrolling E-W; buoy_2 is the EAST buoy patrolling N-S (axis rotates 90 deg
    # per index), with a quarter-lap phase stagger.
    baseline = args.buoy_baseline if args.buoy_baseline > 0.0 else 4.0 * args.traj_radius
    start_e, start_n = beacon_enu(0.0, args)
    cluster_e, cluster_n = start_e + args.buoy_east_shift, start_n + args.buoy_north_gap
    buoys = []
    for i in range(n_units):
        offset_e = (-0.5 * baseline + baseline * i / (n_units - 1)) if n_units > 1 else 0.0
        axis = 0.5 * math.pi * i        # 0, 90, 180, 270 deg -> alternating E-W / N-S
        buoys.append({
            'name': f'buoy_{i + 1}',
            'e': cluster_e + offset_e,
            'n': cluster_n,
            'axis_e': math.cos(axis),
            'axis_n': math.sin(axis),
            'phase': 0.5 * math.pi * i,
        })

    v_mean, v_max = estimate_speed(args)
    log.info(f"Beacon circle r={args.traj_radius:.0f} m, centre offset "
             f"({args.traj_offset_east:+.0f} E, {args.traj_offset_north:+.0f} N) m, "
             f"lap={args.loop_period:.0f} s -> speed {v_mean:.2f} m/s, depth {args.depth:.1f} m.")
    if v_max > 2.0:
        log.warn(f"Beacon speed {v_max:.2f} m/s exceeds the XUUV's ~2 m/s cap; "
                 "the inference motion clamp may lag. Raise --loop-period or lower --traj-radius.")
    baseline = args.buoy_baseline if args.buoy_baseline > 0.0 else 4.0 * args.traj_radius
    log.info(f"{n_units} buoys on a {baseline:.0f} m E-W baseline (~{baseline / args.traj_radius:.0f}x the "
             f"circle radius), {args.buoy_north_gap:.0f} m north of lolo's start, patrolling "
             f"+/-{args.buoy_patrol:.0f} m on perpendicular tracks (period {args.buoy_patrol_period:.0f} s), "
             f"publishing every {args.period:.1f} s to "
             f"{args.topic_prefix}/{args.beacon_name}/range/<unit>. Ctrl+C to stop.")

    mqtt = MqttClient(
        args.mqtt_host, args.mqtt_port,
        username=(args.username or None), password=(args.password or None),
        client_id='owtt_fake_mqtt', logger=log)

    # Model the beacon's broadcast lifecycle so the inference /start /stop
    # services drive the fake stack. The command channel is the inference
    # node's .../cmd; acting as the beacon we relay an idempotent OK on
    # .../cmd_ack (unless --no-command-ack, when a real unit closes the loop).
    cmd_topic = f"{args.topic_prefix}/{args.beacon_name}/cmd"
    cmd_ack_topic = f"{args.topic_prefix}/{args.beacon_name}/cmd_ack"
    ack_unit = buoys[0]['name']
    # Dict so the paho-thread handler can mutate it (atomic in CPython). Seeding is
    # (re)armed on each START: position is reported until both seed_until (wall
    # clock) and seed_remaining (report count) have elapsed.
    started = args.autostart or args.ignore_commands
    now0 = time.time()
    state = {'broadcasting': started,
             'seed_until': (now0 + args.seed_seconds) if started else 0.0,
             'seed_remaining': args.seed_count if started else 0}

    def on_cmd(topic, payload):
        if not topic.endswith('/cmd'):
            return
        try:
            d = json.loads(payload)
        except ValueError:
            return
        command = str(d.get('command', '')).upper()
        rid = d.get('request_id')
        if command not in ('START', 'STOP') or rid is None:
            return
        # Toggle the broadcast lifecycle, like a real beacon.
        want = (command == 'START')
        if want != state['broadcasting']:
            log.info(f"{command} (req {rid}) -> {'begin' if want else 'stop'} broadcasting.")
        state['broadcasting'] = want
        # Each START re-arms position seeding (beacon re-announces its GPS, then
        # drops position once seeded) -- exactly what the real beacon does.
        if want:
            state['seed_until'] = time.time() + args.seed_seconds
            state['seed_remaining'] = args.seed_count
            if args.seed_seconds > 0 or args.seed_count > 0:
                log.info(f"seeding position for the next {args.seed_seconds:.0f} s "
                         f"(and >= {args.seed_count} reports).")
        # Relay the beacon's OK (idempotent: ack every valid command).
        if not args.no_command_ack:
            time.sleep(max(0.0, args.command_ack_delay))   # fake acoustic round-trip
            ack = {'command': command, 'status': 'OK', 'unit': ack_unit,
                   'beacon': args.beacon_name, 'request_id': rid, 'stamp': time.time()}
            mqtt.publish(cmd_ack_topic, json.dumps(ack), qos=1)
            log.info(f"beacon OK for {command} (req {rid}) -> {cmd_ack_topic}")

    if not args.ignore_commands:
        mqtt.set_message_handler(on_cmd)
        mqtt.add_subscription(cmd_topic, qos=1)
        ack_note = (f"relaying OK on {cmd_ack_topic} (ack unit '{ack_unit}')"
                    if not args.no_command_ack else "NOT relaying OK (real unit will)")
        log.info(f"command channel <- {cmd_topic}; {ack_note}.")
        if not state['broadcasting']:
            log.info("Silent until START (call the inference /start service).")
    else:
        log.info("Ignoring the command channel; broadcasting immediately.")

    mqtt.start()

    t0 = time.time()
    sent = 0
    prev_pt = None        # (be, bn, t) for finite-difference ground speed
    try:
        while True:
            if not state['broadcasting']:
                # Beacon is "off" — emit nothing, just wait for a START.
                prev_pt = None
                time.sleep(0.1)
                continue
            now = time.time()
            phase = 2.0 * math.pi * ((now - t0) % args.loop_period) / args.loop_period
            be, bn = beacon_enu(phase, args)
            beacon_lat, beacon_lon = enu_to_geodetic(be, bn, lat0, lon0)

            # Beacon's in-situ sound velocity this cycle (jittered, not frozen).
            svs = args.sound_velocity + random.gauss(0.0, args.svs_drift)

            # Ground speed from successive positions (mimics the beacon's S field).
            if prev_pt is not None and now > prev_pt[2]:
                speed = math.hypot(be - prev_pt[0], bn - prev_pt[1]) / (now - prev_pt[2])
            else:
                speed = 0.0
            prev_pt = (be, bn, now)

            seeding = (now < state['seed_until']) or (state['seed_remaining'] > 0)
            patrol_w = 2.0 * math.pi / max(1e-6, args.buoy_patrol_period)
            ranges_dbg = []
            for b in buoys:
                # Patrol back and forth along the buoy's (perpendicular) axis, plus
                # a little bob about the moving point.
                s = args.buoy_patrol * math.sin(patrol_w * now + b['phase'])
                ue = b['e'] + s * b['axis_e'] + random.gauss(0.0, args.buoy_jitter)
                un = b['n'] + s * b['axis_n'] + random.gauss(0.0, args.buoy_jitter)
                unit_lat, unit_lon = enu_to_geodetic(ue, un, lat0, lon0)

                horizontal = math.hypot(be - ue, bn - un)
                slant = math.hypot(horizontal, args.depth)
                rng = max(0.0, slant + random.gauss(0.0, args.range_noise))

                # Mirror lolo's on-air sentence: depth (D), svs (C), speed (S),
                # bt (B) every report; position (P) only while seeding.
                telemetry = {
                    'depth': round(args.depth, 1),
                    'svs': round(svs, 1),
                    'speed': round(speed, 2),
                    'bt': args.bt_text,
                }
                if seeding:
                    telemetry['position'] = {'lat': beacon_lat, 'lon': beacon_lon}

                report = {
                    'surface_unit': b['name'],
                    'beacon': args.beacon_name,
                    'beacon_modem_id': args.beacon_modem_id,
                    'stamp': now,
                    'unit_lat': unit_lat,
                    'unit_lon': unit_lon,
                    'range_m': round(rng, 2),
                    'delta_us': round(rng / svs * 1e6 + args.offset_us, 1),
                    'offset_us': args.offset_us,
                    # Fake ranges come straight from geometry, i.e. as if the unit
                    # had already auto-calibrated its per-pair offset.
                    'offset_src': 'auto-locked',
                    'sound_velocity': round(svs, 1),
                    'sound_velocity_src': 'beacon',
                    'telemetry': telemetry,
                }
                topic = f"{args.topic_prefix}/{args.beacon_name}/range/{b['name']}"
                mqtt.publish(topic, json.dumps(report))
                ranges_dbg.append(f"{b['name']}={rng:.1f}m")

            sent += 1
            if state['seed_remaining'] > 0:
                state['seed_remaining'] -= 1
            # Log the transition when seeding ends (both gates elapsed).
            if seeding and not ((now < state['seed_until']) or (state['seed_remaining'] > 0)):
                log.info("position seeding done; dropping position from telemetry.")
            if sent % 5 == 1:
                if seeding:
                    secs_left = max(0.0, state['seed_until'] - now)
                    seed_note = f" [seeding {secs_left:.0f}s left]"
                else:
                    seed_note = ""
                log.info(f"beacon @ ({beacon_lat:.6f}, {beacon_lon:.6f})  svs={svs:.1f} m/s  "
                         f"spd={speed:.2f} m/s  {', '.join(ranges_dbg)}{seed_note}")
            time.sleep(args.period)
    except KeyboardInterrupt:
        log.info("Ctrl+C received; stopping.")
    finally:
        mqtt.stop()


if __name__ == '__main__':
    main()
