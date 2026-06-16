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
    p.add_argument('--buoy-radius', type=float, default=60.0,
                   help='Buoys sit this far (m) from centre, spaced evenly on a circle')
    p.add_argument('--buoy-jitter', type=float, default=1.0,
                   help='Std-dev (m) of each buoy bobbing about its mooring')
    p.add_argument('--traj-radius', type=float, default=30.0,
                   help='Beacon circular trajectory radius (m)')
    p.add_argument('--loop-period', type=float, default=120.0,
                   help='Seconds for one full beacon lap (sets speed = 2*pi*r/T)')
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
    p.add_argument('--seed-count', type=int, default=0,
                   help='Include the beacon true GPS in telemetry for the first N reports (0=never)')
    p.add_argument('--bt-text', default='A_Sim (Status.RUNNING)', help='Fake bt tip string in telemetry')
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

    # Fixed buoy moorings, evenly spaced on a circle of radius buoy_radius.
    buoys = []
    for i in range(n_units):
        ang = 2.0 * math.pi * i / n_units
        buoys.append({
            'name': f'buoy_{i + 1}',
            'e': args.buoy_radius * math.cos(ang),
            'n': args.buoy_radius * math.sin(ang),
        })

    speed = 2.0 * math.pi * args.traj_radius / max(1e-6, args.loop_period)
    log.info(f"Beacon circle r={args.traj_radius:.0f} m, lap={args.loop_period:.0f} s "
             f"-> speed {speed:.2f} m/s, depth {args.depth:.1f} m.")
    if speed > 2.0:
        log.warn(f"Beacon speed {speed:.2f} m/s exceeds the XUUV's ~2 m/s cap; "
                 "the inference motion clamp may lag. Raise --loop-period or lower --traj-radius.")
    log.info(f"{n_units} buoys at r={args.buoy_radius:.0f} m, publishing every "
             f"{args.period:.1f} s to {args.topic_prefix}/{args.beacon_name}/range/<unit>. Ctrl+C to stop.")

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
    # One-element dict so the paho-thread handler can mutate it (atomic in CPython).
    state = {'broadcasting': args.autostart or args.ignore_commands}

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
    try:
        while True:
            if not state['broadcasting']:
                # Beacon is "off" — emit nothing, just wait for a START.
                time.sleep(0.1)
                continue
            now = time.time()
            phase = 2.0 * math.pi * ((now - t0) % args.loop_period) / args.loop_period
            be = args.traj_radius * math.cos(phase)
            bn = args.traj_radius * math.sin(phase)
            beacon_lat, beacon_lon = enu_to_geodetic(be, bn, lat0, lon0)

            # Beacon's in-situ sound velocity this cycle (jittered, not frozen).
            svs = args.sound_velocity + random.gauss(0.0, args.svs_drift)

            seeding = sent < args.seed_count
            ranges_dbg = []
            for b in buoys:
                ue = b['e'] + random.gauss(0.0, args.buoy_jitter)
                un = b['n'] + random.gauss(0.0, args.buoy_jitter)
                unit_lat, unit_lon = enu_to_geodetic(ue, un, lat0, lon0)

                horizontal = math.hypot(be - ue, bn - un)
                slant = math.hypot(horizontal, args.depth)
                rng = max(0.0, slant + random.gauss(0.0, args.range_noise))

                telemetry = {'bt': args.bt_text, 'svs': round(svs, 1)}
                if args.depth > 0.0:
                    telemetry['depth'] = round(args.depth, 1)
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
                    'sound_velocity': round(svs, 1),
                    'sound_velocity_src': 'beacon',
                    'telemetry': telemetry,
                }
                topic = f"{args.topic_prefix}/{args.beacon_name}/range/{b['name']}"
                mqtt.publish(topic, json.dumps(report))
                ranges_dbg.append(f"{b['name']}={rng:.1f}m")

            sent += 1
            if sent % 5 == 1:
                seed_note = f" [seeding {args.seed_count - sent + 1} left]" if seeding else ""
                log.info(f"beacon @ ({beacon_lat:.6f}, {beacon_lon:.6f})  svs={svs:.1f} m/s  "
                         f"{', '.join(ranges_dbg)}{seed_note}")
            time.sleep(args.period)
    except KeyboardInterrupt:
        log.info("Ctrl+C received; stopping.")
    finally:
        mqtt.stop()


if __name__ == '__main__':
    main()
