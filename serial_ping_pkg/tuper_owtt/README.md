# tuper_owtt

One-way-travel-time (OWTT) acoustic ranging stack. Unlike
`informed_leader_follower` (two-way ping → reply → broadcast), here **leaders
just broadcast their position** and each receiver derives range from the
one-way acoustic travel time measured by a **Teensy 4.1** front-end.

The ROS nodes never talk to the Succorfish modem directly — they talk to the
Teensy over serial (115200 baud). The Teensy does the precise timing (PPS +
OCXO disciplined), relays to/from the modem, and is configured into one of three
modes by a `$Y` command.

> The legacy `informed_leader_follower` stack is untouched and still works on the
> same upgraded hardware; this is the parallel "new system" software.

---

## Data flow

```
            ROS (leader)                              ROS (follower / receiver)
 ┌───────────────────────────────┐            ┌────────────────────────────────────┐
 │ /<robot>/smarc/latlon         │            │  /<leader>/smarc/latlon  (position)│
 │            │ (tf base→modem)  │            │  /<leader>/distance      (range)   │
 │            ▼                  │            │            ▲          ▲            │
 │     owtt_leader_node          │            │      owtt_follower_node            │
 └────────────┬──────────────────┘            └────────────┬───────────────────────┘
   $YT… (cfg) │ $G<lat>,<lon>                 $YR (cfg)    │ reads #B + #I
              │ 115200                                     │ 115200
        ┌─────▼──────┐                                  ┌────▼───────┐
        │  Teensy 4.1│  ⇄  Succorfish  (acoustic)  ⇄   │ Teensy 4.1 │
        │ (PPS/OCXO) │                                  │  (timing)  │
        └────────────┘                                  └────────────┘
```

- **Leader:** subscribes to its GPS, applies the `base_link → modem_link` lever
  arm via tf2, and feeds `$G<lat>,<lon>` to the Teensy. The Teensy broadcasts
  `$B…` on the PPS schedule.
- **Follower:** reads the relayed broadcast `#B<modem><nn>lat,lon` and the
  Teensy's one-way time delta `#I<delta_us>`, converts the delta to a range, and
  republishes position + range on the legacy follower topics.

Range conversion:

```
range_m = (delta_us - offset_us) * 1e-6 * sound_velocity
```

---

## Teensy serial protocol

The config command is `$Y<own_id(3)><mode><args>`, where `own_id` is the modem
address to set on this node.

| Direction | Message | Meaning |
|-----------|---------|---------|
| host → Teensy | `$Y<own_id>R` | configure as **receiver** (forced non-broadcasting follower); e.g. `$Y101R` |
| host → Teensy | `$Y<own_id>T<listen_id(3)><epochs>s` | configure as **transmitter**; e.g. `$Y042T0001s` (listen `000` = go first, every 1 epoch), `$Y042T0074s` (wait to hear modem `007`, every 4 epochs) |
| host → Teensy | `$Y<own_id>W` | configure as **wire** (transparent passthrough; Teensy "switched off"); e.g. `$Y007W` |
| host → Teensy | `$G<lat>,<lon>` | update the transmitter's stored GPS position (transmitter mode only; not forwarded) |
| host → Teensy | *(any classic Succorfish cmd)* | passed straight through to the modem |
| Teensy → modem | `$Bnn<lat>,<lon>` | automatic scheduled broadcast (Teensy generates this, not the host) |
| Teensy → host | `#B<modem(3)><nn(2)><lat,lon>` | relayed broadcast frame from the modem, e.g. `#B00722<lat>,<lon>` |
| Teensy → host | `#I<delta_us>` | one-way travel-time delta, sent right after a GPS-bearing modem line |

Commands are terminated with `\r\n` by default (`teensy.command_terminator`).

On a valid `$Y`, the Teensy negotiates the new modem address with the modem
(`$A<own_id>` → `#A<own_id>`) and rejects host commands with `BUSY` until it is
confirmed; the new config is only applied on confirmation.

### Wire-safe shutdown

Both nodes **always reset the Teensy to wire mode (`$YW`) on exit** — clean
shutdown, Ctrl-C, an unhandled crash, or SIGTERM. This is enforced in
`owtt_base.py` via a try/finally around the spin loop, an `atexit` hook, and a
SIGTERM handler. So the Teensy is left as a tame wire, ready for the next run.
(Only `SIGKILL` / power loss can bypass this.)

---

## Nodes & topics

### `owtt_leader_node` (transmitter)

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Subscribe | `leader.latlon_topic` (default `/<robot_name>/smarc/latlon`) | `geographic_msgs/GeoPoint` | leader's own GPS |
| Serial out | `$G<lat>,<lon>` | — | modem position after lever-arm transform |

On startup it sends `$Y<own_id>T…` and then **waits for the modem's `#A<own_id>`
confirmation** (forwarded to the host) before sending any `$G` — the Teensy only
applies the new config once confirmed.

Uses tf2: the modem position is `(world→modem) − (world→base)` translation
difference (lever arm already rotated by vehicle orientation), converted to a
small lat/lon delta. Falls back to the raw fix (with a throttled warning) if tf
is unavailable.

### `owtt_follower_node` (receiver)

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Subscribe | `owtt.sound_velocity_topic` (default `/lolo/sensors/svs`) | `svs_interfaces/msg/SVS` | optional live sound speed (field `svs`); gated, defaults to 1500 m/s |
| Publish | `/<leader>/smarc/latlon` | `GeoPoint` / `NavSatFix` | reconstructed leader position (per modem-id map) |
| Publish | `/<leader>/distance` | `std_msgs/Float32` | OWTT-derived range in metres |

Each `#I` delta is paired with the immediately preceding `#B` broadcast to know
which leader it belongs to.

---

## Parameters

Defaults come from `config/tuper_owtt_config.yaml`; everything is overridable as
a launch argument.

### Common (serial / Teensy)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial.port` | `/dev/ttyACM0` | Teensy serial device (enumerates as ACM) |
| `serial.port_fallback` | `/dev/ttyACM1` | tried if the primary fails |
| `serial.baudrate` | `115200` | Teensy link baudrate |
| `teensy.own_modem_id` | `101` (follower) / `007` (leader) | this node's own modem address, set via `$Y<own_id><mode>` |
| `teensy.command_terminator` | `\r\n` | terminator appended to Teensy commands |
| `teensy.mode` | `receiver` / `transmitter` | per-node; set to `wire` for a passive transparent run |

### `owtt_leader_node`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `teensy.listen_for_modem_id` | `000` | modem id to wait for before broadcasting (`000` = go first) |
| `teensy.broadcast_interval_s` | `4` | broadcast cadence in PPS/holdover epochs (1–4) |
| `leader.robot_name` | `` (empty) | vehicle name; builds `/<robot_name>/smarc/latlon`. Empty → derived from the auto-discovered base frame namespace. Leaders are never `lolo`. |
| `leader.latlon_topic` | `` (→ `/<robot_name>/smarc/latlon`) | subscribed GPS topic |
| `leader.base_frame` | `` (auto) | source tf frame; empty → auto-discover the single `*base_link` frame |
| `leader.modem_frame` | `` (auto) | modem tf frame; empty → auto-discover the single `*modem_link` frame |
| `leader.base_link_suffix` | `base_link` | frame-name suffix searched for the base frame |
| `leader.modem_link_suffix` | `modem_link` | frame-name suffix searched for the modem frame |
| `leader.world_frame` | `map` | world/odom frame for orientation lookup (ENU: x=East, y=North) |
| `leader.send_period_s` | `1.0` | how often to push `$G` to the Teensy |

> The leader auto-discovers its tf frames from the tree (there is only one
> `base_link` and one `modem_link`), so you don't have to hardcode the vehicle
> name. Discovery is retried each cycle until the tf tree is populated.

### `owtt_follower_node`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `owtt.delta_prefix` | `#I` | Teensy OWTT delta line prefix |
| `owtt.offset_us` | `0.0` | constant offset subtracted from the delta (µs) |
| `owtt.default_sound_velocity` | `1500.0` | fallback sound speed (m/s) |
| `owtt.sound_velocity_topic` | `/lolo/sensors/svs` | live sound-velocity topic (empty to disable) |
| `owtt.sound_velocity_msg_type` | `svs_interfaces/msg/SVS` | sound-velocity msg type |
| `owtt.sound_velocity_field` | `svs` | field on the msg holding m/s |
| `follower.leader_gps_msg_type` | `GeoPoint` | `GeoPoint` or `NavSatFix` |
| `follower.leader1_name` | `leader1` | name of the first leader (output topics `/leader1/...`) |
| `follower.leader1_modem_id` | `007` | acoustic modem id of the first leader |
| `follower.leader2_name` | `leader2` | name of the second leader (output topics `/leader2/...`) |
| `follower.leader2_modem_id` | `111` | acoustic modem id of the second leader |

> The follower runs on `lolo`; the leaders are **not** named `lolo`. The
> modem-id → leader-name mapping is two `leaderN_name` / `leaderN_modem_id`
> pairs, all overridable from launch arguments.

---

## Dependencies

- ROS 2 (rclpy), `geographic_msgs`, `sensor_msgs`, `std_msgs`, `tf2_ros`
- `svs_interfaces` (optional at runtime; the follower gates the SVS subscription
  and defaults to 1500 m/s when absent)
- Python: `pyserial`, `pyyaml`
- A Teensy 4.1 front-end on the serial port, bridging to a Succorfish modem.

---

## One-shot serial commands (`teensy_cmd`)

Sometimes you just want to fire a single command at the Teensy/modem — set wire
mode, provision a modem id, send a ping or a broadcast — **without running a
node**. `teensy_cmd` does exactly that: it opens the port, writes the command
(with the `\r\n` terminator), optionally reads back the reply, then closes the
port and exits. Nothing stays running and the port is left free.

It accepts **any raw command string**; the `--wire / --receiver / --transmitter`
flags are optional convenience builders that just assemble the `$Y…` string for
you.

```bash
# Provision wire mode + modem id 101 (raw string, or the convenience flag)
ros2 run serial_ping_pkg teensy_cmd '$Y101W' --port /dev/ttyUSB0
ros2 run serial_ping_pkg teensy_cmd --wire --id 101 --port /dev/ttyUSB0

# Build other $Y configs without remembering the format
ros2 run serial_ping_pkg teensy_cmd --receiver --id 101
ros2 run serial_ping_pkg teensy_cmd --transmitter --id 042 --listen 007 --epochs 4

# Any arbitrary command — ping, broadcast, classic Succorfish, etc.
ros2 run serial_ping_pkg teensy_cmd '$P002'
ros2 run serial_ping_pkg teensy_cmd '$B05HELLO'

# Send raw (no terminator), wait longer for a reply, or skip reading entirely
ros2 run serial_ping_pkg teensy_cmd '$P002' --no-terminator
ros2 run serial_ping_pkg teensy_cmd '$P002' --read 3
ros2 run serial_ping_pkg teensy_cmd '$Y101W' --read 0
```

| Option | Default | Description |
|--------|---------|-------------|
| `command` (positional) | — | raw command string, e.g. `$Y101W`, `$P002`, `$B05HELLO` |
| `--port` | `/dev/ttyACM0` | serial device |
| `--port-fallback` | `/dev/ttyACM1` | tried if the primary fails to open |
| `--baud` | `115200` | baud rate |
| `--terminator` | `\r\n` | terminator appended to the command (accepts escapes like `'\r\n'`) |
| `--no-terminator` | off | send the command with nothing appended |
| `--read` | `1.5` | seconds to read and print the reply (`0` = don't read) |
| `--wire` / `--receiver` / `--transmitter` | — | build a `$Y<id><mode>` command instead of a raw string |
| `--id` | `101` | own modem id for the builder flags |
| `--listen` | `000` | transmitter: modem id to wait for (`000` = go first) |
| `--epochs` | `1` | transmitter: broadcast cadence in epochs (1–4) |

> Because it opens and closes the port itself, don't run `teensy_cmd` while a
> leader/follower node is up — they'd both fight for the same serial device.

---

## Launch

> Launch **argument** names differ from the internal parameter names (e.g. the
> launch arg `robot_name` maps to the param `leader.robot_name`, `own_modem_id`
> → `teensy.own_modem_id`). Use the launch-argument names below on the command
> line. Anything you set on the launch file **overrides** the YAML config, so
> these args are the effective source of truth for a launched run.

### Leader (transmitter) — every argument, shown at its default

```bash
ros2 launch serial_ping_pkg owtt_leader_node.launch \
    serial_port:=/dev/ttyACM0 \
    serial_port_fallback:=/dev/ttyACM1 \
    serial_baudrate:=115200 \
    own_modem_id:=007 \
    listen_for_modem_id:=000 \
    broadcast_interval_s:=1 \
    robot_name:='' \
    latlon_topic:='' \
    base_frame:='' \
    modem_frame:='' \
    base_link_suffix:=base_link \
    modem_link_suffix:=modem_link \
    world_frame:=map \
    send_period_s:=1.0 \
    mode:=transmitter \
    command_terminator:=$'\r\n'
```

| Arg | Default | Notes |
|-----|---------|-------|
| `serial_port` | `/dev/ttyACM0` | Teensy device (FTDI cables enumerate as `/dev/ttyUSB*`) |
| `serial_port_fallback` | `/dev/ttyACM1` | tried if primary fails to open |
| `serial_baudrate` | `115200` | Teensy link baud |
| `own_modem_id` | `007` | this leader's modem address → `$Y007T…` |
| `listen_for_modem_id` | `000` | `000` = go first; else wait to hear this modem id |
| `broadcast_interval_s` | `1` | broadcast cadence in PPS/holdover epochs (1–4) |
| `robot_name` | `''` | empty → derive topic from the auto-discovered base frame; leaders are never `lolo` |
| `latlon_topic` | `''` | empty → `/<robot_name>/smarc/latlon` |
| `base_frame` | `''` | empty → auto-discover the single `*base_link` frame |
| `modem_frame` | `''` | empty → auto-discover the single `*modem_link` frame |
| `base_link_suffix` | `base_link` | suffix used for base-frame auto-discovery |
| `modem_link_suffix` | `modem_link` | suffix used for modem-frame auto-discovery |
| `world_frame` | `map` | world/odom frame for orientation lookup |
| `send_period_s` | `1.0` | how often the node pushes `$G` to the Teensy |
| `mode` | `transmitter` | `transmitter` or `wire` (passive passthrough) |
| `command_terminator` | `\r\n` | appended to every Teensy command; rarely changed |

### Follower (receiver) — every argument, shown at its default

```bash
ros2 launch serial_ping_pkg owtt_follower_node.launch \
    serial_port:=/dev/ttyACM0 \
    serial_port_fallback:=/dev/ttyACM1 \
    serial_baudrate:=115200 \
    owtt_delta_prefix:=#I \
    owtt_offset_us:=0.0 \
    default_sound_velocity:=1500.0 \
    sound_velocity_topic:=/lolo/sensors/svs \
    sound_velocity_msg_type:=svs_interfaces/msg/SVS \
    sound_velocity_field:=svs \
    leader_gps_msg_type:=GeoPoint \
    leader1_name:=leader1 \
    leader1_modem_id:=007 \
    leader2_name:=leader2 \
    leader2_modem_id:=111 \
    own_modem_id:=101 \
    mode:=receiver \
    command_terminator:=$'\r\n'
```

| Arg | Default | Notes |
|-----|---------|-------|
| `serial_port` | `/dev/ttyACM0` | Teensy device |
| `serial_port_fallback` | `/dev/ttyACM1` | tried if primary fails to open |
| `serial_baudrate` | `115200` | Teensy link baud |
| `owtt_delta_prefix` | `#I` | Teensy OWTT delta line prefix |
| `owtt_offset_us` | `0.0` | constant offset subtracted from the delta (µs) |
| `default_sound_velocity` | `1500.0` | fallback sound speed (m/s) |
| `sound_velocity_topic` | `/lolo/sensors/svs` | live SVS topic; empty `''` disables the subscription |
| `sound_velocity_msg_type` | `svs_interfaces/msg/SVS` | SVS message type |
| `sound_velocity_field` | `svs` | field on the SVS msg holding m/s |
| `leader_gps_msg_type` | `GeoPoint` | `GeoPoint` or `NavSatFix` for published leader position |
| `leader1_name` | `leader1` | first leader's name → topics `/leader1/...` |
| `leader1_modem_id` | `007` | first leader's modem id (string; quote if leading zeros) |
| `leader2_name` | `leader2` | second leader's name → topics `/leader2/...` |
| `leader2_modem_id` | `111` | second leader's modem id |
| `own_modem_id` | `101` | this receiver's modem address → `$Y101R` |
| `mode` | `receiver` | `receiver` or `wire` (passive passthrough) |
| `command_terminator` | `\r\n` | appended to every Teensy command; rarely changed |

### Common recipes

```bash
# Bare defaults (Teensy on /dev/ttyACM0 @ 115200)
ros2 launch serial_ping_pkg owtt_follower_node.launch
ros2 launch serial_ping_pkg owtt_leader_node.launch robot_name:=leader1

# Real bring-up over an FTDI USB-RS232 cable (enumerates as ttyUSB), leader1
ros2 launch serial_ping_pkg owtt_leader_node.launch \
    serial_port:=/dev/ttyUSB0 serial_port_fallback:=/dev/ttyUSB0 \
    robot_name:=leader1

# Leader, broadcast every 4 epochs, faster GPS push to the Teensy
ros2 launch serial_ping_pkg owtt_leader_node.launch \
    robot_name:=leader1 broadcast_interval_s:=4 send_period_s:=0.5

# Second leader: only starts broadcasting after hearing modem 007
ros2 launch serial_ping_pkg owtt_leader_node.launch \
    robot_name:=sam own_modem_id:=011 listen_for_modem_id:=007

# Follower with a calibrated OWTT offset and a custom leader-id map
ros2 launch serial_ping_pkg owtt_follower_node.launch \
    owtt_offset_us:=12345.0 \
    leader1_name:=sam leader1_modem_id:=011 \
    leader2_name:=leader1 leader2_modem_id:=007

# Follower without a live SVS feed (force the 1500 m/s default)
ros2 launch serial_ping_pkg owtt_follower_node.launch sound_velocity_topic:=''

# "Switch off" the Teensy and talk to the modem transparently (either node)
ros2 launch serial_ping_pkg owtt_follower_node.launch mode:=wire
ros2 launch serial_ping_pkg owtt_leader_node.launch mode:=wire
```

> To stop either node, press **Ctrl-C** — this always resets the Teensy back to
> wire mode (`$YW`) before exiting.
