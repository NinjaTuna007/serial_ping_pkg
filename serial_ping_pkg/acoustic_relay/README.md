# acoustic_relay

Relay a robot's navigation state to other robots over the acoustic modem. One
vehicle **broadcasts** its position (lat/lon/depth/heading) as a compact `$B`
frame; any number of **receivers** decode those frames and republish each
sender's position (and depth/heading) onto per-robot ROS topics.

This is a thin, transparent relay over a raw Succorfish modem (9600 baud) — it
does no ranging itself; it just moves SMARC nav data across the acoustic link.

---

## Nodes / executables

| Executable | Source | Role |
|------------|--------|------|
| `smarc_pos_broadcast_node` | `smarc_pos_broadcast_node.py` | Subscribes to one robot's SMARC nav topics and broadcasts a `$B` position frame on each fresh fix. |
| `smarc_pos_receiver_node` | `smarc_pos_receiver_node.py` | Reads `#B` frames, decodes them, and republishes each sender's position per modem id. |

Pure wire-format logic lives in `pos_protocol.py` (`build_pos_broadcast`,
`parse_pos_broadcast`) and is unit-tested in `test/test_acoustic_relay.py`.

### Serial wire format

```
broadcast out: $B<nn><lat>,<lon>,<depth>,<heading>     (nn = zero-padded payload length)
receive in:    #B<modem-id(3)><nn><lat>,<lon>,<depth>,<heading>\r\n
```

---

## Running it

### Broadcaster — every parameter is overridable

```bash
ros2 launch serial_ping_pkg smarc_pos_broadcast_node.launch \
  serial_port:=/dev/ttyUSB0 \
  serial_port_fallback:=/dev/ttyUSB4 \
  serial_baudrate:=9600 \
  robot_name:=lolo
```

Subscribes to `/<robot_name>/smarc/latlon`, `/<robot_name>/smarc/depth`, and
`/<robot_name>/smarc/heading`; broadcasts over the modem (rate-limited to ~once
every 3 s). Bare run: `ros2 run serial_ping_pkg smarc_pos_broadcast_node`.

### Receiver — robot table as index-aligned lists

```bash
ros2 launch serial_ping_pkg smarc_pos_receiver_node.launch \
  serial_port:=/dev/ttyUSB0 \
  serial_port_fallback:=/dev/ttyUSB1 \
  serial_baudrate:=9600 \
  use_sim_time:=false \
  robot_names:=lolo,sam \
  robot_modem_ids:=7,111 \
  robot_topics:=/relay_lolo/smarc/latlon,/relay_sam/smarc/latlon
```

`robot_names`, `robot_modem_ids`, and `robot_topics` are comma-separated and
index-aligned (`names[i]` <-> `modem_ids[i]` <-> `topics[i]`). Modem ids are
integers, zero-padded to three digits on use; a blank topic entry falls back to
`/relay_<name>/smarc/latlon`. Bare run:
`ros2 run serial_ping_pkg smarc_pos_receiver_node`.

---

## Parameters

Defaults live in
[`config/acoustic_relay/acoustic_relay_config.yaml`](../../../config/acoustic_relay/acoustic_relay_config.yaml);
every value is overridable through the launch args above (YAML is only the
bare-`ros2 run` fallback).

### `smarc_pos_broadcast_node`

| Parameter | Launch arg | Default | Notes |
|-----------|-----------|---------|-------|
| `serial.port` / `serial.port_fallback` / `serial.baudrate` | `serial_port` / `serial_port_fallback` / `serial_baudrate` | `/dev/ttyUSB0` / `/dev/ttyUSB4` / `9600` | modem link |
| `robot_name` | `robot_name` | `lolo` | builds the subscribed topic names |

### `smarc_pos_receiver_node`

| Parameter | Launch arg | Default | Notes |
|-----------|-----------|---------|-------|
| `serial.port` / `serial.port_fallback` / `serial.baudrate` | `serial_port` / `serial_port_fallback` / `serial_baudrate` | `/dev/ttyUSB0` / `/dev/ttyUSB1` / `9600` | modem link |
| `robots.names` | `robot_names` | `lolo,sam` | comma-separated robot names |
| `robots.modem_ids` | `robot_modem_ids` | `7,111` | comma-separated modem ids (int) |
| `robots.topics` | `robot_topics` | `/relay_lolo/smarc/latlon,/relay_sam/smarc/latlon` | comma-separated output topics (blank -> default) |
| `use_sim_time` | `use_sim_time` | `false` | standard ROS sim-time flag |

---

## Topics

`smarc_pos_broadcast_node`
- Subscribes: `/<robot_name>/smarc/latlon` (`geographic_msgs/GeoPoint`),
  `/<robot_name>/smarc/depth` and `/<robot_name>/smarc/heading` (`std_msgs/Float32`).
- Publishes: none (output goes out over the serial modem).

`smarc_pos_receiver_node`
- Subscribes: none (input arrives over the serial modem).
- Publishes (per configured robot): `<topics[i]>` (`geographic_msgs/GeoPoint`),
  and `/relay_<name>/smarc/depth` / `/relay_<name>/smarc/heading`
  (`std_msgs/Float32`) when the frame carries them.
