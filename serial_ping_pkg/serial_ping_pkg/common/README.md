# common

General-purpose acoustic nodes that aren't tied to any one scenario (OWTT,
TWTT, relay, etc.): basic two-way ranging and a generic broadcast receiver.
They talk to the modem through `succorfish_driver` over ROS.

> Driver requirement: a `succorfish_driver` node must be running. These nodes
> reach it through the relative names `succorfish/tx` (`std_msgs/String`, raw
> outbound command — the driver appends the line terminator), `succorfish/rx`
> (`succorfish_msgs/SerialLine`, inbound text lines), `succorfish/tx_bytes` and
> `succorfish/rx_bytes` (`succorfish_msgs/SerialFrame`, raw UART bytes for binary
> `$B`/`$U`), `succorfish/connected`
> (`std_msgs/Bool`, latched link state) and the `succorfish/send_command`
> service (`succorfish_msgs/SendCommand`, synchronous write-and-wait-for-reply).
> Because the names are relative, a node lines up with its driver when they
> share a namespace. The driver also picks the serial profile (9600-baud
> `/dev/ttyUSB*` Succorfish or 115200-baud `/dev/ttyACM*` Teensy) via the config
> you launch. See [`succorfish_driver`](../../../succorfish_driver/README.md).

A thin helper, `serial_ping_pkg/common/driver_client.py` (`DriverClient`),
wraps these topics/service for the nodes here.

Shared, hardware-free wire-format helpers:

| File | Role |
|------|------|
| `ping_protocol.py` | ping travel-time / leader-broadcast parse |
| [`ascii_on_air.py`](ascii_on_air.py) | ASCII decimal catalog (`$G`, relay/TWTT `$B`, `codec:=ascii`) |
| [`dccl_codec.py`](dccl_codec.py) | libdccl `pack`/`unpack` + CRC-8-ATM |
| [`dccl_acoustic_pb2.py`](dccl_acoustic_pb2.py) | generated stubs — do not edit |
| [`../../proto/dccl_acoustic.proto`](../../proto/dccl_acoustic.proto) | DCCL schema (`beacon.codec:=dccl`) |

See [`proto/README.md`](../../proto/README.md) for the codec map (ASCII digits
vs packed DCCL bits). The driver `tx` vs `tx_bytes` table is the UART pipe,
not that codec.

---

## Nodes / executables

| Executable | Source | Role |
|------------|--------|------|
| `serial_ping_node` | `serial_ping_node.py` | Alternates pinging two configured modem addresses and publishes each one's range. |
| `single_target_ping_node` | `single_target_ping_node.py` | Tracks one robot's position and periodically pings a single target, publishing the range. |
| `serial_broadcast_receiver` | `serial_broadcast_receiver.py` | Decodes leader broadcast frames (`leader_id,lat,lon,dist`) and republishes GPS + range per leader. |

### Wire format

Outbound commands go to the driver on `succorfish/tx` (or via the
`succorfish/send_command` service); inbound lines arrive on `succorfish/rx`.

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
  leader1_command:=\$P001 \
  leader2_command:=\$P002 \
  sound_velocity:=1500.0 \
  timeout_threshold:=5.0
```

Publishes `leader1/distance` and `leader2/distance` (`std_msgs/Float32`). Bare
run: `ros2 run serial_ping_pkg serial_ping_node`. Each ping is issued through
the driver's `succorfish/send_command` service (write the command, wait for the
matching reply within the timeout), with an in-flight guard so pings don't
overlap. Make sure a `succorfish_driver` is running in the same namespace.

### `single_target_ping_node` (one target, on a timer)

```bash
ros2 launch serial_ping_pkg single_target_ping_node.launch \
  robot_name:=stick \
  ping_command:=\$P002 \
  timer_period:=1.0 \
  distance_topic_suffix:=distance_to_usv \
  sound_velocity:=1500.0 \
  timeout_threshold:=5.0
```

Subscribes to `/<robot_name>/smarc/latlon` (cached only) and publishes
`/<robot_name>/<distance_topic_suffix>` (`std_msgs/Float32`). Like
`serial_ping_node`, it pings via the driver's `succorfish/send_command` service
with an in-flight guard. Bare run:
`ros2 run serial_ping_pkg single_target_ping_node`. USB Delphis getting-started
(driver + this node): `ros2 launch serial_ping_pkg delphis_ping.launch`.

### `serial_broadcast_receiver` (generic leader-broadcast receiver)

```bash
ros2 launch serial_ping_pkg serial_broadcast_receiver.launch
```

Publishes `/leader<id>/gps` (`sensor_msgs/NavSatFix`) and `/leader<id>/distance`
(`std_msgs/Float32`) lazily, per leader id seen on the wire. It is a passive
listener: each inbound line arrives via the `succorfish/rx` subscription
callback (no read timer/buffer of its own). Bare run:
`ros2 run serial_ping_pkg serial_broadcast_receiver`.

---

## Parameters

| Node | Parameter | Launch arg | Default | Notes |
|------|-----------|-----------|---------|-------|
| `serial_ping_node` | `serial.leader1` / `serial.leader2` | `leader1_command` / `leader2_command` | `$P001` / `$P002` | the two ping commands |
| `serial_ping_node` | `serial.sound_velocity` | `sound_velocity` | `1500.0` | m/s for travel-time -> range |
| `serial_ping_node` | `serial.timeout_threshold` | `timeout_threshold` | `0.5` (YAML) / `5.0` (launch) | per-ping reply timeout (s) |
| `single_target_ping_node` | `robot_name` | `robot_name` | `stick` | builds topic names |
| `single_target_ping_node` | `ping_command` | `ping_command` | `$P002` | ping command string |
| `single_target_ping_node` | `timer_period` | `timer_period` | `1.0` | seconds between pings |
| `single_target_ping_node` | `sound_velocity` | `sound_velocity` | `1500.0` | m/s for travel-time -> range |
| `single_target_ping_node` | `distance_topic_suffix` | `distance_topic_suffix` | `distance_to_usv` | distance topic suffix |
| `single_target_ping_node` | `serial.timeout_threshold` | `timeout_threshold` | `5.0` | per-ping reply timeout (s) |

Port and baudrate live on the `succorfish_driver` you launch (which also
selects the Succorfish vs Teensy profile). These nodes only need a driver
reachable in their namespace.

Defaults for `serial_ping_node` / `single_target_ping_node` live in
[`config/common/serial_config.yaml`](../../../config/common/serial_config.yaml)
and
[`config/common/single_target_ping_config.yaml`](../../../config/common/single_target_ping_config.yaml);
`serial_broadcast_receiver` declares its defaults in code.
