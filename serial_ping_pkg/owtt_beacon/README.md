# owtt_beacon

A one-way-travel-time (OWTT) **beacon localisation** stack built on the same
Teensy 4.1 + Succorfish front-end as `tuper_owtt`.

A single **beacon** broadcasts a tiny telemetry frame over the acoustic link.
Two **surface units** decode that telemetry, measure their OWTT range to the
beacon, and push a JSON range report (own GPS + range + telemetry) to an MQTT
broker. An **inference node** subscribes to those reports and triangulates the
beacon, publishing a `sensor_msgs/NavSatFix` for visualisation (e.g. Foxglove).

```
                    acoustic
  beacon (lolo)  ───────────────►  surface_unit_1  ─┐  MQTT (JSON)
  Teensy=TX                        Teensy=RX         ├──────────────►  inference_node
   $K<telemetry>                   surface_unit_2  ─┘                   triangulate
                                   Teensy=RX                            -> NavSatFix
```

## Data flow

1. **Beacon** subscribes to its telemetry sources, encodes the enabled fields
  (default: bt tip only) into a compact payload, and sends `$K<payload>` to its
   Teensy (transmitter mode). The Teensy prepends the `TEL:` telemetry marker and
   broadcasts `$BnnTEL:<payload>` on its PPS/OCXO schedule.
2. **Surface units** (receiver mode) get the relayed broadcast
  `#B<id><nn>TEL:<payload>` followed by the timing line `#I<delta_us>`. They decode
   the telemetry, compute `range = (delta_us - offset_us) * 1e-6 * c`, publish
   the range on ROS, and publish a JSON range report to MQTT.
3. **Inference node** keeps the latest report per surface unit and triangulates
  the beacon from two units' positions + ranges (two-circle intersection),
   publishing the estimate as `NavSatFix`.

## Telemetry frame

The payload after the Teensy's `TEL:` marker is a terse, `;`-separated, tagged
list (see `beacon_telemetry.py`). Only enabled fields with a value are sent:


| tag | field    | encoding                                        |
| --- | -------- | ----------------------------------------------- |
| `P` | position | `P<lat>,<lon>`                                  |
| `D` | depth    | `D<m>` (metres, +down)                          |
| `C` | svs      | `C<m/s>` (sound velocity from the beacon's SVS) |
| `S` | speed    | `S<m/s>`                                        |
| `B` | bt tip   | `B<text>` (sanitised, length-capped, name-only) |


Example (`position,depth,svs,speed,bt`), as broadcast: `TEL:P58.823229,17.635998;D12.3;C1481.6;S1.20;BChilling`

Enable `svs` so the **surface units convert travel-time to range with the
beacon's in-situ sound velocity** instead of a frozen default — the beacon reads
it from its own SVS sensor (`owtt.sound_velocity_*`). When a report's telemetry
carries `svs`, the surface unit uses it (and tags the report
`sound_velocity_src: beacon`); otherwise it falls back to its own SVS topic /
`owtt.default_sound_velocity` (`sound_velocity_src: local`).

Enable `depth` when the beacon dives: the surface units measure a **slant**
range, and the inference node uses the broadcast depth to recover the horizontal
range it triangulates with (else it falls back to `inference.assumed_depth_m`).

Keep it small — the Succorfish NM3 caps a packet at **64 bytes** (`TEL:` + the
payload). The beacon enforces this via `max_onair_bytes`: if the encoded payload
would overflow, it trims the free-text `bt` first (truncate → drop), then drops
`speed`/`depth`/`svs`, always preserving `position`; a throttled warning tells
you when it trims. To save bytes the `bt` tip is reduced to just the action-client
name (`bt_name_only`, dropping `(Status.RUNNING)`), reduced to the last path
segment of the topic-like action name (`bt_basename`, `/lolo/move_to` →
`move_to`), with the `A_` prefix stripped (`bt_strip_prefix`), e.g.
`A_Chilling (Status.RUNNING)` → `Chilling`. The **default field set is `bt`
only**.

> Firmware note: `$K` is the host→Teensy command prefix (it would collide with
> the modem's own `$T`), while `TEL:` is the on-air payload marker the receivers
> look for. The Teensy emits `#I` timing for any `TEL:`-marked broadcast, so
> telemetry frames are range-able.

## Nodes

### `owtt_beacon_node` (beacon / transmitter)

Subscribes (under `beacon.name`'s namespace):

- `…/smarc/latlon` (`geographic_msgs/GeoPoint`) — if `position` enabled
- `…/smarc/depth` (`std_msgs/Float32`, m +down) — if `depth` enabled
- `…/sensors/svs` (`svs_interfaces/msg/SVS`) — if `svs` enabled (sound velocity)
- `…/smarc/speed` (`std_msgs/Float32`) — if `speed` enabled
- `…/waraps/sensor/bt` (`std_msgs/String` JSON, field `tip`) — if `bt` enabled

Sends `$Y…T…` (transmitter config) then periodic `$K<payload>` once the modem
confirms with `#A<own_id>`. Resets the Teensy to WIRE mode on any exit.

`position_seed_count` (default 0): force-include the beacon's own GPS in the
first N broadcasts even if `position` isn't a permanent field, then drop it —
just enough to seed the inference node's branch lock before going silent on
position (handy if the beacon surfaces for a GPS fix and then dives).

**START/STOP command channel.** The beacon does **not** broadcast telemetry
until commanded (unless `autostart:=true`). It listens — even in transmitter
mode the Teensy forwards received broadcasts to the host — for control
broadcasts:


| heard broadcast           | effect                       | ack            |
| ------------------------- | ---------------------------- | -------------- |
| `START` (`start_keyword`) | begin broadcasting telemetry | `OK` broadcast |
| `STOP` (`stop_keyword`)   | stop broadcasting telemetry  | `OK` broadcast |


The ack is **idempotent**: the beacon replies `OK` to *every* valid `START`/`STOP`,
even when its state doesn't change (a `START` while already broadcasting still
acks). This makes the command channel safe to retry — the surface units /
inference service re-send until they hear an `OK`. By default any sender is
accepted; restrict with `commander_modem_ids:="['067','069']"`. The ack is a
plain `$B<nn>OK` broadcast sent immediately (not on the PPS schedule). After a
`STOP`, the Teensy's stored telemetry ages out within a few epochs, so a few
residual broadcasts may still go out before it falls silent.

### `owtt_surface_unit_node` (surface unit / receiver)

Subscribes: `/<unit_name>/smarc/latlon` (own GPS), optional local SVS sound
velocity. For the travel-time → range conversion it prefers the **beacon's
broadcast `svs`** (from telemetry) and only falls back to its local SVS topic /
`owtt.default_sound_velocity` when the beacon doesn't send one.
Publishes:

- `/<unit_name>/owtt_beacon/<beacon>/range` (`std_msgs/Float32`)
- `/<unit_name>/owtt_beacon/<beacon>/telemetry` (`std_msgs/String` JSON)
- `/<unit_name>/owtt_beacon/range_report` (`std_msgs/String` JSON)
- MQTT `→ <topic_prefix>/<beacon>/range/<unit_name>` (same JSON report)

It also takes part in the **command channel** (see *Command orchestration*):
it subscribes MQTT `<topic_prefix>/<beacon>/cmd`, and — if `commander:=true` —
transmits the acoustic `START`/`STOP` keyword. Whichever unit hears the beacon's
`OK` relays an ack to MQTT `<topic_prefix>/<beacon>/cmd_ack`.

### `owtt_inference_node` (laptop)

Subscribes MQTT `<topic_prefix>/<beacon>/range/+` (range reports) and
`<topic_prefix>/<beacon>/cmd_ack` (command acks). Publishes:

- `/owtt_beacon/<beacon>/estimate` (`sensor_msgs/NavSatFix`) — triangulated fix
- `/owtt_beacon/<beacon>/estimate_alt` (`NavSatFix`) — mirror fix (2-receiver case, if `publish_both`)
- `/owtt_beacon/<beacon>/reported` (`NavSatFix`) — beacon's self-reported pos (if telemetry carries position)
- `/owtt_beacon/<beacon>/telemetry` (`std_msgs/String` JSON)
- MQTT `→ <topic_prefix>/<beacon>/cmd` (START/STOP requests)

Services (call from Foxglove / CLI):

- `/owtt_beacon/<beacon>/start` (`std_srvs/Trigger`)
- `/owtt_beacon/<beacon>/stop` (`std_srvs/Trigger`)

## MQTT

Reports are JSON strings, mirroring the smarc2 `str_json_mqtt_bridge`
convention. Default broker is the smarc/WARA-PS broker `**20.240.40.232:1884`**
(use `localhost:1889` for a local mosquitto). Topics live under the `**tuper`**
context in an `**owtt_beacon`** subcontext:

```
tuper/owtt_beacon/<beacon>/range/<surface_unit>      (publish: each surface unit)
tuper/owtt_beacon/<beacon>/range/+                   (subscribe: inference node)
tuper/owtt_beacon/<beacon>/cmd                        (publish: inference node — START/STOP)
tuper/owtt_beacon/<beacon>/cmd_ack                    (publish: whichever unit heard the beacon OK)
```

The subscribe `+` wildcard means the inference node automatically picks up
**every** surface unit that publishes — add more units (each with a distinct
`unit_name`) and they show up with no config change. Range report payload:

```json
{
  "surface_unit": "floatsam_usv_1", "beacon": "lolo", "beacon_modem_id": "101",
  "stamp": 1781020261.2, "unit_lat": 58.82, "unit_lon": 17.63,
  "range_m": 62.3, "delta_us": 813649.0, "offset_us": 771600.0,
  "sound_velocity": 1481.6, "sound_velocity_src": "beacon",
  "telemetry": {"bt": "A_Chilling (Status.RUNNING)", "svs": 1481.6}
}
```

## Launch

```bash
# Beacon (on the beacon vehicle, e.g. lolo) — broadcast EVERYTHING, accept
# commands from anyone, seed its GPS for the first 10 broadcasts, every 2 s.
ros2 launch serial_ping_pkg owtt_beacon_node.launch \
    serial_port:=/dev/ttyACM0 beacon_name:=lolo own_modem_id:=007 \
    telemetry_fields:="['position','depth','svs','speed','bt']" \
    position_seed_count:=10 \
    send_period_s:=2.0 \
    commander_modem_ids:="[]"

# Surface unit 1 — the designated COMMANDER (only one unit gets commander:=true)
ros2 launch serial_ping_pkg owtt_surface_unit_node.launch \
    serial_port:=/dev/succorfish unit_name:=floatsam_usv_0 own_modem_id:=067 \
    beacon_name:=lolo beacon_modem_id:=007 commander:=false \
    mqtt_host:=20.240.40.232 mqtt_port:=1884

# Surface unit 2 — a listener (commander defaults to false)
ros2 launch serial_ping_pkg owtt_surface_unit_node.launch \
    serial_port:=/dev/succorfish unit_name:=floatsam_usv_1 own_modem_id:=069 \
    beacon_name:=lolo beacon_modem_id:=007 commander:=true \
    mqtt_host:=20.240.40.232 mqtt_port:=1884

# Inference (laptop) — hosts the /start /stop services
ros2 launch serial_ping_pkg owtt_inference_node.launch \
    beacon_name:=lolo mqtt_host:=20.240.40.232 mqtt_port:=1884 command_repeat_s:=4.0

# Start / stop the beacon broadcasting (e.g. from a Foxglove service-call panel)
ros2 service call /owtt_beacon/lolo/start std_srvs/srv/Trigger
ros2 service call /owtt_beacon/lolo/stop  std_srvs/srv/Trigger
```

All config defaults live in `config/owtt_beacon_config.yaml`; every value is
overridable as a launch argument.

### `owtt_beacon_node.launch` arguments


| arg                                              | default                                                | meaning                                                     |
| ------------------------------------------------ | ------------------------------------------------------ | ----------------------------------------------------------- |
| `serial_port` / `serial_port_fallback`           | `/dev/ttyACM0` / `/dev/ttyACM1`                        | Teensy serial port (+ fallback)                             |
| `serial_baudrate`                                | `115200`                                               | Teensy link baud                                            |
| `own_modem_id`                                   | `101`                                                  | beacon's own modem address (`$Y`)                           |
| `listen_for_modem_id`                            | `000`                                                  | modem id to wait for before TX; `000` = go first            |
| `broadcast_interval_s`                           | `1`                                                    | Teensy on-air cadence as a PPS multiple (1..4)              |
| `mode`                                           | `transmitter`                                          | `transmitter`                                               |
| `beacon_name`                                    | `lolo`                                                 | telemetry-source namespace                                  |
| `telemetry_fields`                               | `['bt']`                                               | subset of `position`/`depth`/`svs`/`speed`/`bt`             |
| `latlon_topic` / `depth_topic` / `speed_topic`   | derived                                                | override telemetry source topics                            |
| `svs_topic` / `svs_msg_type` / `svs_field`       | `/lolo/sensors/svs` / `svs_interfaces/msg/SVS` / `svs` | sound-velocity source                                       |
| `bt_topic` / `bt_json_field`                     | derived / `tip`                                        | bt source + JSON field                                      |
| `bt_name_only`                                   | `true`                                                 | keep only the action-client name, drop `(Status.RUNNING)`   |
| `bt_basename`                                    | `true`                                                 | keep only the last path segment (`/lolo/move_to`→`move_to`) |
| `bt_strip_prefix`                                | `A_`                                                   | strip this leading prefix from the bt name (`A_Chilling`→`Chilling`) |
| `position_precision`                             | `6`                                                    | lat/lon decimals in payload                                 |
| `max_bt_len`                                     | `32`                                                   | bt text cap (acoustic bandwidth)                            |
| `max_onair_bytes`                                | `64`                                                   | modem packet cap; `TEL:`+payload auto-trimmed to fit        |
| `send_period_s`                                  | `1.0`                                                  | **telemetry broadcast interval** (how often `$K` is pushed) |
| `position_seed_count`                            | `0`                                                    | include GPS in first N broadcasts then drop it              |
| `autostart`                                      | `false`                                                | begin broadcasting without waiting for `START`              |
| `commander_modem_ids`                            | `[]`                                                   | modem ids allowed to command; `**[]` = anyone**             |
| `start_keyword` / `stop_keyword` / `ack_message` | `START` / `STOP` / `OK`                                | command channel words                                       |


> The on-air cadence is `broadcast_interval_s` (PPS-locked, set on the Teensy);
> `send_period_s` is how often the host refreshes the payload it hands the
> Teensy. Keep `send_period_s` ≤ the on-air interval so each broadcast carries
> fresh data.

### `owtt_surface_unit_node.launch` arguments


| arg                                              | default                                                | meaning                                                                       |
| ------------------------------------------------ | ------------------------------------------------------ | ----------------------------------------------------------------------------- |
| `serial_port` / `serial_port_fallback`           | `/dev/ttyACM0` / `/dev/ttyACM1`                        | Teensy serial port (+ fallback)                                               |
| `serial_baudrate`                                | `115200`                                               | Teensy link baud                                                              |
| `own_modem_id`                                   | `067`                                                  | this unit's own modem address (`$Y`)                                          |
| `mode`                                           | `receiver`                                             | `receiver`                                                                    |
| `owtt_delta_prefix`                              | `#I`                                                   | Teensy OWTT delta line prefix                                                 |
| `owtt_offset_us`                                 | `771600.0`                                             | constant offset subtracted from delta (µs); fallback when not auto-calibrated |
| `min_range_m` / `max_range_m`                    | `0.0` / `0.0`                                          | drop ranges outside this band (neg = bad offset/sync; `max 0` = no cap)       |
| `auto_calibrate`                                 | `true`                                                 | learn `offset_us` per modem pair from the beacon's seeded GPS                 |
| `calib_min_samples` / `calib_window`             | `5` / `20`                                             | seeded samples to lock the offset / rolling-median window                     |
| `default_sound_velocity`                         | `1500.0`                                               | fallback c if no beacon/local SVS                                             |
| `sound_velocity_topic` / `_msg_type` / `_field`  | `/lolo/sensors/svs` / `svs_interfaces/msg/SVS` / `svs` | local SVS fallback source                                                     |
| `unit_name`                                      | `surface_unit`                                         | this unit's name (own-GPS namespace + report id)                              |
| `unit_latlon_topic`                              | derived                                                | override own-position topic                                                   |
| `commander`                                      | `false`                                                | **exactly one** unit `true`: only it transmits acoustic START/STOP            |
| `beacon_name`                                    | `lolo`                                                 | beacon being tracked                                                          |
| `beacon_modem_id`                                | `101`                                                  | only accept this beacon's frames; empty = any                                 |
| `start_keyword` / `stop_keyword` / `ack_message` | `START` / `STOP` / `OK`                                | command words (must match the beacon)                                         |
| `mqtt_enabled`                                   | `true`                                                 | publish range reports to MQTT                                                 |
| `mqtt_host` / `mqtt_port`                        | `20.240.40.232` / `1884`                               | broker                                                                        |
| `mqtt_username` / `mqtt_password`                | `/`                                                    | broker auth (optional)                                                        |
| `mqtt_topic_prefix`                              | `tuper/owtt_beacon`                                    | report topic context/subcontext                                               |
| `mqtt_qos`                                       | `0`                                                    | MQTT publish QoS (command channel always uses QoS 1)                          |


> Sound velocity priority at the surface unit: **beacon's broadcast `svs`** →
> local SVS topic → `default_sound_velocity`. Enable `svs` on the beacon to use
> the in-situ value.

> **Auto-calibrated offset (per modem pair).** Rather than trusting a fixed
> (and easily wrong) `owtt_offset_us`, each surface unit self-calibrates while
> the beacon **seeds its GPS** in telemetry (`position_seed_count`). Knowing the
> true range (own GPS ↔ beacon GPS, slant-corrected with depth) and the measured
> `delta_us`, it solves `offset_us = delta_us − true_slant / c · 1e6`, folds it
> into a running median, **locks** after `calib_min_samples`, and uses that value
> for the rest of the run (when the beacon stops sending position). The chosen
> offset and its source appear in the report as `offset_us` / `offset_src`
> (`config` → `auto-provisional` → `auto-locked`). Set `position_seed_count ≥ calib_min_samples` on the beacon so every pair has enough seeds to lock.
> Auto-calibration only removes a *constant* bias — it cannot fix `delta_us` that
> drifts because the two modems aren't sharing a PPS-disciplined epoch.

### `owtt_inference_node.launch` command arguments


| arg                 | default | meaning                                                |
| ------------------- | ------- | ------------------------------------------------------ |
| `command_timeout_s` | `10.0`  | `/start` `/stop` give up if no beacon `OK` within this |
| `command_repeat_s`  | `1.5`   | re-publish the MQTT command this often until acked     |


## Command orchestration (START / STOP)

You drive the beacon from one place — the inference node — via two ROS services,
so a Foxglove **Call Service** panel (or `ros2 service call`) is all you need.
The acoustic side is handled automatically: exactly one surface unit
(`commander:=true`) transmits, and *any* unit that hears the beacon's `OK` closes
the loop. The service blocks until the beacon confirms, or times out.

```
Foxglove ──service──▶ inference_node
                          │  publishes {command, request_id} to .../cmd  (QoS 1, repeats every command_repeat_s)
                          ▼
                  all surface units (sub .../cmd)
                          │  COMMANDER transmits acoustic START/STOP → beacon
                          ▼
                       beacon  ──acoustic OK──▶  any surface unit in range
                          │                          │ publishes {status:OK, request_id} to .../cmd_ack
                          ▼                          ▼
                  inference_node  ◀──── matches request_id ──── service returns success=True
```

Key properties:

- **One transmitter.** Only `commander:=true` sends acoustics, so two units never
talk over each other. Pick whichever unit has the best link to the beacon.
- **Any acker.** The beacon's `OK` is an omnidirectional broadcast; whichever unit
hears it relays the ack, so the commander doesn't have to be the one that hears
back. Reliability scales with the number of units.
- **Idempotent + retried.** The inference node re-publishes the command every
`command_repeat_s` until it sees a matching `cmd_ack`; the beacon always re-acks.
Lost commands/acks simply get re-sent within `command_timeout_s`.
- **De-duped.** Each command carries a `request_id`; units ignore a request they've
already acked, and the service matches the ack by `request_id`.
- **Serialized.** A second `/start` while one is in flight returns `success=False`
("already in progress") rather than racing.
- **Fresh start.** A successful `/start` **re-initialises the filter**: the
two-hypothesis tracker, last published fix, and all buffered range reports are
cleared, so each run begins estimation from scratch (no stale pre-START data).
`/stop` leaves the filter state untouched.

Returns `std_srvs/Trigger`: `success=True` with the acking unit on confirmation,
or `success=False` with a timeout message after `command_timeout_s`.

## Testing without hardware

`owtt_fake_mqtt` simulates the whole acoustic side: a beacon on a closed-loop
(circular) trajectory and N fixed "buoy" surface units bobbing about their
moorings. It publishes JSON range reports in the exact `surface_unit_node`
schema to `<topic_prefix>/<beacon>/range/<unit>`, so you can exercise the
inference node and your map view with no Teensy/modem. It loops until Ctrl+C, and
the trajectory is periodic (each lap starts/ends at the same point).

```bash
# Terminal 1 — fake data (defaults: smarc broker, lolo, 2 buoys, ~1.5 m/s).
# --autostart streams immediately; drop it to drive via the /start /stop services.
ros2 run serial_ping_pkg owtt_fake_mqtt --autostart
# Terminal 2 — the real inference node, same broker/beacon
ros2 launch serial_ping_pkg owtt_inference_node.launch beacon_name:=lolo
# then watch /owtt_beacon/lolo/estimate (+ estimate_alt, reported) in Foxglove
```

It also **models the command lifecycle**, so the `/start` `/stop` services drive
it just like a real beacon. By default it is **silent until a START**: it
subscribes `<topic_prefix>/<beacon>/cmd`, begins streaming range reports on
START and stops on STOP, and — acting as the beacon — relays an idempotent `OK`
to `…/cmd_ack` (after `--command-ack-delay`, default 0.4 s) so the service call
returns success. So the self-contained test is:

```bash
# Terminal 1 — fake beacon + buoys (silent until /start)
ros2 run serial_ping_pkg owtt_fake_mqtt
# Terminal 2 — inference node (hosts the services)
ros2 launch serial_ping_pkg owtt_inference_node.launch beacon_name:=lolo
# Terminal 3 — drive it (or use a Foxglove Call Service panel)
ros2 service call /owtt_beacon/lolo/start std_srvs/srv/Trigger   # ranges start flowing
ros2 service call /owtt_beacon/lolo/stop  std_srvs/srv/Trigger   # ranges cease
```

Command-channel flags:

- `--autostart` — stream immediately without waiting for a START (old behaviour,
for pure inference/visualization tests).
- `--no-command-ack` — still follow the START/STOP lifecycle, but let a **real**
commander + units relay the `OK` (use when testing real surface units against a
fake range stream).
- `--ignore-commands` — ignore the command channel entirely (implies `--autostart`).

Other useful flags: `--num-units 3` (test multilateration / unique fix),
`--depth 12` (reports slant range + telemetry depth → exercises the
slant→horizontal path), `--seed-count 5` (beacon broadcasts its true GPS for the
first 5 reports → exercises branch-locking), `--mqtt-host localhost --mqtt-port 1889` (local mosquitto), `--loop-period`/`--traj-radius` (set the beacon speed).
Run `ros2 run serial_ping_pkg owtt_fake_mqtt --help` for the full list.

## Visualization

The inference node publishes plain `sensor_msgs/NavSatFix`, so the default path
is **Foxglove**: open the *Map* panel and add the `estimate`, `estimate_alt`,
`reported`, and the surface units' `…/smarc/latlon` topics. Foxglove's Map panel
renders over **OpenStreetMap** tiles by default and also supports custom/Mapbox
tile sources (Google's tile API needs a key + has usage terms, so OSM is the
zero-setup choice). RViz `NavSatFix` (rviz_satellite) is an alternative.

No bespoke GUI ships with this package. If you want a standalone web map (e.g.
Leaflet over OSM/Google tiles, subscribing to the same MQTT range reports), it
can be added as a separate, ROS-free viewer.

## Triangulation, scaling & ambiguity

The inference node scales to **any number of receivers** (`min_units` ≥ 2,
default 2):

- **2 receivers:** the two range circles generally meet at two points — a
genuine two-fold ambiguity. The `estimate` topic carries the primary fix
(disambiguated by the motion-model prediction, or `inference.first_fix_prefer`
on the first fix), and with `inference.publish_both` (default on) the mirror
is also published on `estimate_alt` so **both candidates show on the map**.
- **3+ receivers:** it solves a least-squares **multilateration** over *all*
fresh reports (Gauss-Newton, seeded by the disambiguated fix of the
best-separated pair). The fix is unique — no mirror is published.

### Slant vs. horizontal range (depth)

The acoustic OWTT range is a **slant** range to a possibly-submerged beacon, but
the triangulation is 2-D. Each slant range `s` is converted to a horizontal
range `sqrt(s² - d²)` using the beacon depth `d` — taken from telemetry (enable
the `depth` field) or from `inference.assumed_depth_m`. With `d = 0` ranges are
treated as horizontal (the previous behaviour). At short range / large depth the
correction is significant (e.g. `s=50 m, d=30 m → 40 m`).

### Resolving the mirror over time (two-hypothesis tracker)

The mirror reflects the beacon across the **baseline** joining the two units. As
the units move, that baseline rotates, so the **ghost jumps around while the
true position stays put** (or moves smoothly within the ≤2 m/s budget). With
`inference.motion_model` on (default), the node keeps **both branches as
hypotheses**, propagates each with a constant-velocity model capped at
`inference.max_speed_mps`, and scores them by accumulated motion-consistency
(prediction error + an over-speed penalty). The ghost accrues cost and is
demoted to `estimate_alt`; the survivor becomes `estimate`. This also recovers
from a wrong initial lock once the geometry changes (validated: recovers within
~3 epochs after ~24° of baseline rotation), and the CV clamp rejects single bad
fixes (frozen/outlier ranges). Set `motion_model:=false` for a memoryless fix.

### Position seeding (lock the branch instantly)

If the beacon broadcasts its own GPS (telemetry `position`), the tracker locks
onto the branch nearest it (`inference.use_reported_position`, default on). This
pairs with the beacon's `position_seed_count`: include the real GPS in the
**first N broadcasts** (e.g. while surfaced, before diving) to seed the fix, then
drop position to save acoustic bytes and let the converged tracker continue.

Reports are keyed by `surface_unit`, and only those within
`inference.freshness_window_s` are used, so units can come and go. A good
`owtt.offset_us` calibration matters: a wrong offset biases every range and
shifts the fix.