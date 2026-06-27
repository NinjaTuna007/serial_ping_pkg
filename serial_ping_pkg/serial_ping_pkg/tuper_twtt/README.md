# tuper_twtt (Informed Leader / Follower, TWTT)

Two ROS 2 nodes that exchange GPS position **and** acoustic range over a
Succorfish Delphis / NM3 serial modem. Unlike the plain ping nodes, the leader
sends its *own* GPS fix together with the measured distance, so the follower
can reconstruct where the leader is and how far away it is ("informed" ranging).

- `twtt_leader_node` — measures range to the follower and transmits
  `lat,lon,distance` back over the modem.
- `twtt_follower_node` — listens on the modem, decodes `lat,lon,distance`,
  and republishes it into the ROS graph.

> The two nodes normally run on **different** robots/computers, each with its
> own modem. They talk to each other acoustically, *not* over ROS topics.

---

## Serial access via the driver

These nodes no longer open a serial port themselves. The sibling
[`succorfish_driver`](../../../succorfish_driver/README.md) package exclusively
owns the modem and **must be running**; the nodes talk to it over ROS through
the `DriverClient` helper using these relative names (lined up with the driver
by namespace):

- `succorfish/tx` (`std_msgs/String`) — outbound command, client → driver (the driver appends the line terminator).
- `succorfish/rx` (`succorfish_msgs/SerialLine`) — inbound lines, driver → clients.
- `succorfish/connected` (`std_msgs/Bool`, latched) — link up/down.
- `succorfish/send_command` (`succorfish_msgs/SendCommand` service) — synchronous write-then-wait-for-matching-reply (regex + timeout).

The leader pings the follower via the `succorfish/send_command` service; the
follower listens for inbound lines on the `succorfish/rx` topic. `tuper_twtt`
uses the Succorfish modem directly (9600-baud `/dev/ttyUSB*` profile), selected
by which driver config is launched.

---

## Data flow at a glance

```
        ROS (leader side)                 acoustic / serial link                 ROS (follower side)
 ┌───────────────────────────┐                                          ┌──────────────────────────────┐
 │ leader_gps_topic (GeoPoint│                                          │                              │
 │  or NavSatFix)            │                                          │                              │
 │            │              │      $P… ping ───────────────▶          │                              │
 │            ▼              │      ◀──────── T<time> reply            │                              │
 │   twtt_leader_node ───┼──▶ "$B.. lat,lon,dist\r\n" ───────────▶│── twtt_follower_node     │
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

### `twtt_leader_node`

Pings the follower on a fixed period (or, in *slave* mode, only after hearing a
`#B` broadcast), measures range, and transmits its latest GPS position +
distance back over the modem (via the driver).

**ROS topics**

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Subscribe | `leader_gps_topic` (param; default `/<robot_name>/smarc/latlon`, YAML default `/follower/leader2/core/gps`) | `geographic_msgs/GeoPoint` or `sensor_msgs/NavSatFix` | The leader's own live GPS fix; chosen via `leader_gps_msg_type`. |

This node has **no ROS publishers** for its measurements — its output goes out
over the modem (through the driver).

**Driver I/O**

- Pings by calling the driver's `succorfish/send_command` service
  (`DriverClient.request`) with `ping_command` (e.g. `$P111`), an expected-reply
  regex, and a timeout; an in-flight guard prevents overlapping pings.
- The matched reply contains `T<travel_time>`; distance =
  `0.00003125 * sound_velocity * travel_time`.
- Sends `$B<n>{lat},{lon},{dist}` to the driver over `succorfish/tx`
  (the driver appends the line terminator).
- In `is_slave` mode it instead waits for an incoming `#B` broadcast line,
  received via the `succorfish/rx` callback, before pinging.

### `twtt_follower_node`

Reads the modem, decodes the `lat,lon,distance` frames the leader sent, and
republishes them.

**ROS topics**

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Publish | `leader_gps_topic` (param; default `/<leader_name>/smarc/latlon`) | `geographic_msgs/GeoPoint` or `sensor_msgs/NavSatFix` | Reconstructed leader position. |
| Publish | `/<leader_name>/distance` | `std_msgs/Float32` | Range between leader and follower, in metres. |

This node has **no ROS subscriptions** for its input — each inbound line arrives
over the modem via the driver's `succorfish/rx` callback (a readiness guard
ensures lines aren't processed before init completes).

---

## Parameters

### `twtt_leader_node`

| Parameter | Type | Default (YAML / code) | Description |
|-----------|------|-----------------------|-------------|
| `robot_name` | string | `leader` | Used to build the default GPS topic. |
| `leader_gps_topic` | string | `/<robot_name>/smarc/latlon` (YAML: `/follower/leader2/core/gps`) | Topic carrying the leader's own GPS. |
| `leader_gps_msg_type` | string | `GeoPoint` (YAML: `NavSatFix`) | `GeoPoint` or `NavSatFix`. |
| `ping_command` | string | `$P111` | Command string sent to ping the follower. |
| `sound_velocity` | double | `1500.0` | Speed of sound (m/s) used to turn travel time into range. |
| `timeout_threshold` | double | `0.5` (YAML) / `2.0` (declared) | Seconds to wait for a ping reply before giving up. |
| `timer_period` | double | `5.0` (YAML) / `3.0` (declared) | Seconds between pings (non-slave mode). |
| `is_slave` | bool | `false` | If true, ping only after receiving a `#B` broadcast instead of on a timer. |

### `twtt_follower_node`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `leader_name` | string | `leader` | Used to build the published topic names. |
| `leader_gps_topic` | string | `/<leader_name>/smarc/latlon` | Topic to publish the reconstructed leader position on. |
| `leader_gps_msg_type` | string | `GeoPoint` | `GeoPoint` or `NavSatFix`. |

> Defaults come from `config/tuper_twtt/tuper_twtt_config.yaml` (loaded at startup) and are
> overridden by the launch files below. Where the YAML and the in-code
> `declare_parameter` default differ, the YAML value wins because it is loaded
> first and passed as the default.

---

## Dependencies

- ROS 2 (rclpy)
- Message packages: `geographic_msgs`, `sensor_msgs`, `std_msgs`, `succorfish_msgs`
- The `succorfish_driver` package (owns the serial port; must be running)
- Python: `pyyaml` (declared in the package `package.xml` / `setup.py`)
- A serial acoustic modem reachable by the driver (Succorfish Delphis / NM3).

---

## Launch

Launch files live in `serial_ping_pkg/launch/`:

- `twtt_leader_node.launch`
- `twtt_follower_node.launch`

> Make sure the `succorfish_driver` is running in the same namespace first,
> since these nodes reach the modem through it.

```bash
# Leader (defaults)
ros2 launch serial_ping_pkg twtt_leader_node.launch

# Follower
ros2 launch serial_ping_pkg twtt_follower_node.launch

# Leader in slave mode
ros2 launch serial_ping_pkg twtt_leader_node.launch is_slave:=true
```

### Launch arguments

The serial port/baudrate are no longer launch arguments — the
`succorfish_driver` owns the port. The leader exposes `robot_name`,
`ping_command`, `sound_velocity`, `timeout_threshold`, and `is_slave`; the
follower exposes `leader_name`.

---

## Wire format

These are the payloads exchanged with the driver (the driver appends the line
terminator on outbound writes):

- **Ping command** (leader → driver, via `send_command`): the `ping_command` string, e.g. `$P111`.
- **Range reply** (driver → leader): contains `T<travel_time>`; distance =
  `0.00003125 * sound_velocity * travel_time`.
- **Position frame** (leader → driver → follower, via `tx`/`rx`):
  `$B<n>{lat},{lon},{dist}`, where `<n>` is the payload length.
- **Broadcast trigger** (slave leader only): an incoming `rx` line beginning with `#B`.
