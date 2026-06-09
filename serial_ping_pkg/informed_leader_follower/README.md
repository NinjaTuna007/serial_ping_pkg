# Informed Leader / Follower

Two ROS 2 nodes that exchange GPS position **and** acoustic range over a
Succorfish Delphis / NM3 serial modem. Unlike the plain ping nodes, the leader
sends its *own* GPS fix together with the measured distance, so the follower
can reconstruct where the leader is and how far away it is ("informed" ranging).

- `informed_leader_node` — measures range to the follower and transmits
  `lat,lon,distance` back over the modem.
- `informed_follower_node` — listens on the modem, decodes `lat,lon,distance`,
  and republishes it into the ROS graph.

> The two nodes normally run on **different** robots/computers, each with its
> own modem. They talk to each other acoustically, *not* over ROS topics.

---

## Data flow at a glance

```
        ROS (leader side)                 acoustic / serial link                 ROS (follower side)
 ┌───────────────────────────┐                                          ┌──────────────────────────────┐
 │ leader_gps_topic (GeoPoint│                                          │                              │
 │  or NavSatFix)            │                                          │                              │
 │            │              │      $P… ping ───────────────▶          │                              │
 │            ▼              │      ◀──────── T<time> reply            │                              │
 │   informed_leader_node ───┼──▶ "$B.. lat,lon,dist\r\n" ───────────▶│── informed_follower_node     │
 │                           │                                          │        │            │        │
 └───────────────────────────┘                                          │        ▼            ▼        │
                                                                        │ leader_gps_topic   /<leader_ │
                                                                        │ (GeoPoint/NavSatFix) name>/  │
                                                                        │                    distance  │
                                                                        └──────────────────────────────┘
```

The leader pings, the modem replies with a one-way-travel-time token `T<...>`,
the leader converts that to a distance, then writes
`$B<n>lat,lon,distance\r\n` to the modem. The follower reads that frame off its
own modem and publishes it.

---

## Nodes

### `informed_leader_node`

Pings the follower on a fixed period (or, in *slave* mode, only after hearing a
`#B` broadcast), measures range, and transmits its latest GPS position +
distance back over serial.

**ROS topics**

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Subscribe | `leader_gps_topic` (param; default `/<robot_name>/smarc/latlon`, YAML default `/follower/leader2/core/gps`) | `geographic_msgs/GeoPoint` or `sensor_msgs/NavSatFix` | The leader's own live GPS fix; chosen via `leader_gps_msg_type`. |

This node has **no ROS publishers** — its output goes out over the serial modem.

**Serial I/O**

- Writes `ping_command` (e.g. `$P111`) to the modem.
- Reads the `T<travel_time>` reply and computes `distance = 0.00003125 * sound_velocity * travel_time`.
- Writes `$B<n>{lat},{lon},{dist}\r\n` back to the modem.
- In `is_slave` mode it instead waits for an incoming `#B` broadcast line before pinging.

### `informed_follower_node`

Reads the modem, decodes the `lat,lon,distance` frames the leader sent, and
republishes them.

**ROS topics**

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Publish | `leader_gps_topic` (param; default `/<leader_name>/smarc/latlon`) | `geographic_msgs/GeoPoint` or `sensor_msgs/NavSatFix` | Reconstructed leader position. |
| Publish | `/<leader_name>/distance` | `std_msgs/Float32` | Range between leader and follower, in metres. |

This node has **no ROS subscriptions** — its input arrives over the serial modem.

---

## Parameters

### `informed_leader_node`

| Parameter | Type | Default (YAML / code) | Description |
|-----------|------|-----------------------|-------------|
| `serial.port` | string | `/dev/ttyUSB0` | Primary serial device. |
| `serial.port_fallback` | string | `/dev/ttyUSB4` | Tried if the primary fails to open. |
| `serial.baudrate` | int | `9600` | Modem baudrate. Override for the new hardware stack. |
| `robot_name` | string | `leader` | Used to build the default GPS topic. |
| `leader_gps_topic` | string | `/<robot_name>/smarc/latlon` (YAML: `/follower/leader2/core/gps`) | Topic carrying the leader's own GPS. |
| `leader_gps_msg_type` | string | `GeoPoint` (YAML: `NavSatFix`) | `GeoPoint` or `NavSatFix`. |
| `ping_command` | string | `$P111` | Command string sent to ping the follower. |
| `sound_velocity` | double | `1500.0` | Speed of sound (m/s) used to turn travel time into range. |
| `timeout_threshold` | double | `0.5` (YAML) / `2.0` (declared) | Seconds to wait for a ping reply before giving up. |
| `timer_period` | double | `5.0` (YAML) / `3.0` (declared) | Seconds between pings (non-slave mode). |
| `is_slave` | bool | `false` | If true, ping only after receiving a `#B` broadcast instead of on a timer. |

### `informed_follower_node`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial.port` | string | `/dev/ttyUSB0` | Primary serial device. |
| `serial.port_fallback` | string | `/dev/ttyUSB1` | Tried if the primary fails to open. |
| `serial.baudrate` | int | `9600` | Modem baudrate. Override for the new hardware stack. |
| `leader_name` | string | `leader` | Used to build the published topic names. |
| `leader_gps_topic` | string | `/<leader_name>/smarc/latlon` | Topic to publish the reconstructed leader position on. |
| `leader_gps_msg_type` | string | `GeoPoint` | `GeoPoint` or `NavSatFix`. |

> Defaults come from `config/serial_config.yaml` (loaded at startup) and are
> overridden by the launch files below. Where the YAML and the in-code
> `declare_parameter` default differ, the YAML value wins because it is loaded
> first and passed as the default.

---

## Dependencies

- ROS 2 (rclpy)
- Message packages: `geographic_msgs`, `sensor_msgs`, `std_msgs`
- Python: `pyserial`, `pyyaml` (see the package `requirements.txt` / `setup.py`)
- A serial acoustic modem reachable at the configured port (Succorfish Delphis / NM3).

---

## Launch

Launch files live in `serial_ping_pkg/launch/`:

- `informed_leader_node.launch`
- `informed_follower_node.launch`

```bash
# Leader (old hardware stack, defaults)
ros2 launch serial_ping_pkg informed_leader_node.launch

# Follower on a specific port
ros2 launch serial_ping_pkg informed_follower_node.launch \
    serial_port:=/dev/ttyUSB2

# New hardware stack: just override the baudrate (and port as needed)
ros2 launch serial_ping_pkg informed_leader_node.launch \
    serial_port:=/dev/ttyUSB0 serial_baudrate:=115200
```

### Launch arguments

Both launch files expose `serial_port`, `serial_port_fallback`, and
`serial_baudrate`. The leader additionally exposes `robot_name`,
`ping_command`, `sound_velocity`, `timeout_threshold`, and `is_slave`; the
follower exposes `leader_name`.

| Launch arg | Maps to param | Leader default | Follower default |
|------------|---------------|----------------|------------------|
| `serial_port` | `serial.port` | `/dev/ttyUSB0` | `/dev/ttyUSB0` |
| `serial_port_fallback` | `serial.port_fallback` | `/dev/ttyUSB0` | `/dev/ttyUSB1` |
| `serial_baudrate` | `serial.baudrate` | `9600` | `9600` |

> **Cross-compatibility with the new hardware stack:** the baudrate is a launch
> argument everywhere serial is used, so the old stack keeps the `9600` YAML
> default while the new stack is selected at launch time with
> `serial_baudrate:=<new value>`. No code or config edits required.

---

## Serial wire format

- **Ping command** (leader → modem): the `ping_command` string, e.g. `$P111`.
- **Range reply** (modem → leader): contains `T<travel_time>`; distance =
  `0.00003125 * sound_velocity * travel_time`.
- **Position frame** (leader → modem → follower):
  `"$B<n>{lat},{lon},{dist}\r\n"`, where `<n>` is the payload length.
- **Broadcast trigger** (slave leader only): an incoming line beginning with `#B`.
