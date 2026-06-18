# ping_estimator_action

Range-only acoustic localization of remote modems. A single node pings a set of
modem ids, converts each two-way travel time into a range, accumulates the range
constraints from the moving own-vehicle, and estimates each remote modem's
position (lat/lon/altitude) with either a sliding-window non-linear
least-squares solver or a static-beacon EKF. Estimates are published as
`geographic_msgs/GeoPointStamped` plus RViz markers once their uncertainty drops
below a configurable threshold.

Unlike `tuper_owtt` (one-way travel time via a Teensy), this stack does
classic **two-way** ranging and talks either to a raw Succorfish modem or to a
Teensy bridge in transparent wire mode.

---

## Nodes / executables

| Executable | Source | Role |
|------------|--------|------|
| `ping_estimator_action_node` | `ping_estimator_action_node.py` | The estimator + action server (`smarc_modem_ping`, `smarc_stop_modem_ping`). |
| `ping_estimator_action_cmd` | `ping_estimator_action_cmd.py` | CLI to send goals (add/remove/clear/ping/broadcast/unicast/stop) to the node. |

Pure math lives in `range_estimators.py` (`LeastSquaresRangeEstimator`,
`EkfRangeEstimator`) and is unit-tested in `test/test_ping_estimator_action.py`.

### Action-server modes

The node runs an action server (`smarc_modem_ping`) whose goal is JSON:

```jsonc
{"mode": "add",       "modem_id": "7", "depth_m": 12.0}  // register a modem (optional known depth, positive down)
{"mode": "remove",    "modem_id": "7"}                   // drop one modem
{"mode": "clear"}                                         // drop all + delete markers
{"mode": "ping",      "retry_count": 3, "task_timeout_s": 180.0}  // ping/localize all registered modems
{"mode": "broadcast", "message": "hello all"}            // $B broadcast payload
{"mode": "unicast",   "modem_id": "8", "message": "hi"}  // $U unicast payload
```

A separate `smarc_stop_modem_ping` action hard-stops a running ping task.

Serial protocol handled by the node:

```
ping out:   $P<id>                       (e.g. $P007)
range in:   #R<id>T<ticks>               range_m = ticks * sound_velocity * 3.125e-5
timeout in: #TO
broadcast:  $B<NN><payload>
unicast:    $U<id><NN><payload>          NN = zero-padded byte count
```

---

## Running it

### Launch (recommended) — every parameter is overridable

```bash
ros2 launch serial_ping_pkg ping_estimator_action.launch \
  robot_name:=lolo \
  use_sim_time:=false \
  serial_port:=/dev/ttyUSB0 \
  serial_port_fallback:=/dev/ttyACM1 \
  serial_baudrate:=9600 \
  enable_wire_on_startup:=true \
  own_modem_id:=007 \
  payload_max_bytes:=64 \
  ping_max_retries:=3 \
  ping_response_timeout_s:=3.0 \
  ping_task_timeout_s:=180.0 \
  ping_cycle_until_estimated:=true \
  estimator_method:=least_squares \
  default_sound_velocity:=1500.0 \
  sound_velocity_topic:=/lolo/sensors/svs \
  sound_velocity_msg_type:=svs_interfaces/msg/SVS \
  sound_velocity_field:=svs \
  range_sigma_m:=1.0 \
  own_position_sigma_m:=1.0 \
  own_depth_sigma_m:=0.3 \
  remote_depth_sigma_m:=0.5 \
  use_known_remote_depth:=true \
  min_measurements:=4 \
  max_measurements:=40 \
  max_position_uncertainty_m:=5.0 \
  ls_max_iterations:=20 \
  ls_damping:=0.001 \
  ls_outlier_gate_m:=10.0 \
  ekf_process_noise_std_m:=0.05 \
  ekf_initial_sigma_xy_m:=50.0 \
  ekf_initial_sigma_z_m:=2.0 \
  own_latlon_topic:=smarc/latlon \
  own_depth_topic:=smarc/depth \
  own_position_max_age_s:=3.0 \
  geopoint_topic:=modem_estimates/geopoint \
  marker_topic:=modem_estimates/rviz \
  map_frame:=M350/map
```

All args have defaults (matching `config/ping_estimator_action/ping_estimator_action_config.yaml`),
so the bare `ros2 launch serial_ping_pkg ping_estimator_action.launch` works too.

> **Namespacing:** the node runs under `namespace:=$(var robot_name)`. The topic
> args (`own_latlon_topic`, `own_depth_topic`, `geopoint_topic`, `marker_topic`)
> default to **relative** names, so they resolve as `/{robot_name}/...`. Pass an
> absolute name (leading `/`) to opt out of namespacing. `command_terminator`
> defaults to CRLF (`\r\n`) and is exposed as a launch arg as well.

### Bare node

```bash
ros2 run serial_ping_pkg ping_estimator_action_node --ros-args \
  -r __ns:=/lolo \
  -p serial.port:=/dev/ttyUSB0 \
  -p estimator.method:=ekf
```

Without launch args, defaults come from `config/ping_estimator_action/ping_estimator_action_config.yaml`.

### Sending goals

```bash
ros2 run serial_ping_pkg ping_estimator_action_cmd add 7 --depth 12.0 --robot lolo
ros2 run serial_ping_pkg ping_estimator_action_cmd ping --retries 3 --timeout 180 --robot lolo
ros2 run serial_ping_pkg ping_estimator_action_cmd broadcast "hello all" --robot lolo
ros2 run serial_ping_pkg ping_estimator_action_cmd unicast 8 "hi modem 8" --robot lolo
ros2 run serial_ping_pkg ping_estimator_action_cmd clear --robot lolo
ros2 run serial_ping_pkg ping_estimator_action_cmd stop --robot lolo
```

---

## Parameters

Defaults live in [`config/ping_estimator_action/ping_estimator_action_config.yaml`](../../../config/ping_estimator_action/ping_estimator_action_config.yaml);
every value is overridable through the launch args above.

| Group | Parameter | Default | Notes |
|-------|-----------|---------|-------|
| serial | `serial.port` / `serial.port_fallback` / `serial.baudrate` | `/dev/ttyUSB0` / `/dev/ttyACM1` / `9600` | 9600 raw modem, 115200 Teensy bridge |
| teensy | `teensy.enable_wire_on_startup` | `true` | wire-mode `$Y<id>W` on startup (harmless on a raw modem) |
| teensy | `teensy.own_modem_id` | `007` | own modem address |
| teensy | `teensy.command_terminator` | `\r\n` | terminator appended to serial commands |
| teensy | `teensy.payload_max_bytes` | `64` | broadcast/unicast payload limit |
| ping | `ping.max_retries` | `3` | retries after first attempt |
| ping | `ping.response_timeout_s` | `3.0` | per-ping timeout (s) |
| ping | `ping.task_timeout_s` | `180.0` | overall task timeout (s) |
| ping | `ping.cycle_until_estimated` | `true` | keep cycling until localized |
| estimator | `estimator.method` | `least_squares` | `least_squares` or `ekf` |
| estimator | `estimator.default_sound_velocity` | `1500.0` | m/s fallback (≈330 in air) |
| estimator | `estimator.sound_velocity_topic` / `_msg_type` / `_field` | `/lolo/sensors/svs` / `svs_interfaces/msg/SVS` / `svs` | live SVS source (empty topic disables) |
| estimator | `estimator.range_sigma_m` / `own_position_sigma_m` / `own_depth_sigma_m` / `remote_depth_sigma_m` | `1.0` / `1.0` / `0.3` / `0.5` | noise model (m) |
| estimator | `estimator.use_known_remote_depth` | `true` | fix remote z from per-modem depth when known |
| estimator | `estimator.min_measurements` / `max_measurements` | `4` / `40` | window bounds |
| estimator | `estimator.max_position_uncertainty_m` | `5.0` | publish once sigma below this |
| least_squares | `estimator.least_squares.max_iterations` / `damping` / `outlier_gate_m` | `20` / `0.001` / `10.0` | Gauss-Newton + LM damping; `0` gate disables rejection |
| ekf | `estimator.ekf.process_noise_std_m` / `initial_sigma_xy_m` / `initial_sigma_z_m` | `0.05` / `50.0` / `2.0` | static random-walk model |
| topics | `topics.own_latlon_topic` / `own_depth_topic` | `smarc/latlon` / `smarc/depth` | own state inputs (relative → namespaced) |
| topics | `topics.own_position_max_age_s` | `3.0` | discard stale own position |
| topics | `topics.geopoint_topic` / `marker_topic` / `map_frame` | `modem_estimates/geopoint` / `modem_estimates/rviz` / `M350/map` | outputs + RViz frame |

---

## Topics

Published:
- `<geopoint_topic>` (`geographic_msgs/GeoPointStamped`) — per-modem estimate, `header.frame_id = modem_<id>`.
- `<marker_topic>` (`visualization_msgs/MarkerArray`) — RViz sphere (1σ) + text label.

Subscribed:
- `<own_latlon_topic>` (`geographic_msgs/GeoPoint`) — own position (required).
- `<own_depth_topic>` (`std_msgs/Float32`) — own depth, positive down (optional).
- `<sound_velocity_topic>` (configurable type) — live sound velocity (optional).
