# common

General-purpose acoustic nodes that are not tied to any one scenario (OWTT,
TWTT, relay, etc.). These are the simple, reusable building blocks: basic
two-way ranging and a generic broadcast receiver. They talk to a raw Succorfish
modem over serial.

Shared, hardware-free wire-format helpers live in `ping_protocol.py`
(`travel_time_to_distance`, `ping_response_complete`, `parse_ping_distance`,
`parse_leader_broadcast`) and are unit-tested in `test/test_common_protocol.py`.

> Note: the shared library `serial_ping_pkg/utils.py` (`load_yaml_config`,
> `init_serial`) intentionally lives at the package root, not here, since every
> sub-package imports it.

---

## Nodes / executables

| Executable | Source | Role |
|------------|--------|------|
| `serial_ping_node` | `serial_ping_node.py` | Alternates pinging two configured modem addresses and publishes each one's range. |
| `single_target_ping_node` | `single_target_ping_node.py` | Tracks one robot's position and periodically pings a single target, publishing the range. |
| `serial_broadcast_receiver` | `serial_broadcast_receiver.py` | Decodes leader broadcast frames (`leader_id,lat,lon,dist`) and republishes GPS + range per leader. |

### Serial wire format

```
ping out:   $P<id>                        (e.g. $P001)
range in:   ...T<ticks>                    range_m = ticks * sound_velocity * 3.125e-5
leader bcast in: handled by parse_leader_broadcast() -> (leader_id, lat, lon, dist)
```

---

## Running it

### `serial_ping_node` (alternating two-leader ping)

```bash
ros2 launch serial_ping_pkg serial_ping_node.launch \
  serial_port:=/dev/ttyUSB0 \
  serial_port_fallback:=/dev/ttyUSB4 \
  serial_baudrate:=9600 \
  leader1_command:=\$P001 \
  leader2_command:=\$P002 \
  sound_velocity:=1500.0 \
  timeout_threshold:=5.0
```

Publishes `leader1/distance` and `leader2/distance` (`std_msgs/Float32`). Bare
run: `ros2 run serial_ping_pkg serial_ping_node`.

### `single_target_ping_node` (one target, on a timer)

```bash
ros2 launch serial_ping_pkg single_target_ping_node.launch \
  robot_name:=stick \
  ping_command:=\$P002 \
  timer_period:=1.0 \
  distance_topic_suffix:=distance_to_usv \
  serial_port:=/dev/ttyUSB0 \
  serial_port_fallback:=/dev/ttyUSB4 \
  serial_baudrate:=9600 \
  sound_velocity:=1500.0 \
  timeout_threshold:=5.0
```

Subscribes to `/<robot_name>/smarc/latlon` (cached only) and publishes
`/<robot_name>/<distance_topic_suffix>` (`std_msgs/Float32`). Bare run:
`ros2 run serial_ping_pkg single_target_ping_node`.

### `serial_broadcast_receiver` (generic leader-broadcast receiver)

```bash
ros2 launch serial_ping_pkg serial_broadcast_receiver.launch \
  port:=/dev/ttyUSB0 \
  baudrate:=9600
```

Publishes `/leader<id>/gps` (`sensor_msgs/NavSatFix`) and `/leader<id>/distance`
(`std_msgs/Float32`) lazily, per leader id seen on the wire. Bare run:
`ros2 run serial_ping_pkg serial_broadcast_receiver`.

---

## Parameters

| Node | Parameter | Launch arg | Default | Notes |
|------|-----------|-----------|---------|-------|
| `serial_ping_node` | `serial.port` / `serial.port_fallback` / `serial.baudrate` | `serial_port` / `serial_port_fallback` / `serial_baudrate` | `/dev/ttyUSB0` / `/dev/ttyUSB4` / `9600` | modem link |
| `serial_ping_node` | `serial.leader1` / `serial.leader2` | `leader1_command` / `leader2_command` | `$P001` / `$P002` | the two ping commands |
| `serial_ping_node` | `serial.sound_velocity` | `sound_velocity` | `1500.0` | m/s for travel-time -> range |
| `serial_ping_node` | `serial.timeout_threshold` | `timeout_threshold` | `0.5` (YAML) / `5.0` (launch) | per-ping reply timeout (s) |
| `single_target_ping_node` | `robot_name` | `robot_name` | `stick` | builds topic names |
| `single_target_ping_node` | `ping_command` | `ping_command` | `$P002` | ping command string |
| `single_target_ping_node` | `timer_period` | `timer_period` | `1.0` | seconds between pings |
| `single_target_ping_node` | `sound_velocity` | `sound_velocity` | `1500.0` | m/s for travel-time -> range |
| `single_target_ping_node` | `distance_topic_suffix` | `distance_topic_suffix` | `distance_to_usv` | distance topic suffix |
| `single_target_ping_node` | `serial.*` / `serial.timeout_threshold` | `serial_*` / `timeout_threshold` | `/dev/ttyUSB0` ... / `5.0` | modem link + timeout |
| `serial_broadcast_receiver` | `port` | `port` | `/dev/ttyUSB0` | serial device |
| `serial_broadcast_receiver` | `baudrate` | `baudrate` | `9600` | modem baudrate |

Defaults for `serial_ping_node` / `single_target_ping_node` live in
[`config/common/serial_config.yaml`](../../../config/common/serial_config.yaml)
and
[`config/common/single_target_ping_config.yaml`](../../../config/common/single_target_ping_config.yaml);
`serial_broadcast_receiver` declares its defaults in code.
