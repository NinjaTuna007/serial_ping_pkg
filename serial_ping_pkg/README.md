# serial_ping_pkg (ROS 2)

A ROS 2 package for underwater acoustic communication and ranging with
Succorfish Delphis / NM3 modems (and a Teensy 4.1 front-end for the
one-way-travel-time scenarios). It provides several self-contained sub-packages
covering acoustic pinging, range-only localization, position relaying, and
autonomous leader/follower behaviours.

It is one `ament_python` ROS 2 package; each sub-package owns its own code,
config, launch files, and README so scenarios stay independent while sharing one
build.

---

## Sub-packages

| Sub-package | What it does | README |
|-------------|--------------|--------|
| `common` | General-purpose nodes: two-leader alternating ping, single-target ping, generic broadcast receiver. | [serial_ping_pkg/common/README.md](serial_ping_pkg/common/README.md) |
| `acoustic_relay` | Relay a robot's position over the modem: one node broadcasts, others receive and republish per robot. | [serial_ping_pkg/acoustic_relay/README.md](serial_ping_pkg/acoustic_relay/README.md) |
| `ping_estimator_action` | Range-only localization of remote modems via an action server (two-way ranging + LS/EKF). | [serial_ping_pkg/ping_estimator_action/README.md](serial_ping_pkg/ping_estimator_action/README.md) |
| `tuper_twtt` | Two-way-travel-time "informed" leader/follower over the Succorfish modem. | [serial_ping_pkg/tuper_twtt/README.md](serial_ping_pkg/tuper_twtt/README.md) |
| `tuper_owtt` | One-way-travel-time leader/follower using a PPS/OCXO-disciplined Teensy front-end. | [serial_ping_pkg/tuper_owtt/README.md](serial_ping_pkg/tuper_owtt/README.md) |
| `owtt_beacon` | OWTT beacon telemetry + surface-unit reception + MQTT-fed triangulation/inference. | [serial_ping_pkg/owtt_beacon/README.md](serial_ping_pkg/owtt_beacon/README.md) |

The shared library `serial_ping_pkg/utils.py` (`load_yaml_config`) sits at the
package root and is imported by every sub-package. Serial transport is no longer
handled here: every node talks to the `succorfish_driver` over ROS via the
`serial_ping_pkg/common/driver_client.py` (`DriverClient`) helper.

---

## Talking to the modem (the driver)

**No node in this package opens a serial port.** The physical link (Succorfish
modem or Teensy front-end) is owned exclusively by the `succorfish_driver` node,
which lives in the sibling [`succorfish_driver/`](../succorfish_driver/README.md)
submodule. Every node here communicates with it over a small ROS interface
(names from `succorfish_msgs/Topics`, **relative** so they line up by namespace):

| Name | Type | Direction | Use |
|------|------|-----------|-----|
| `succorfish/tx` | `std_msgs/String` | node → driver | raw outbound command (driver appends the terminator) |
| `succorfish/rx` | `succorfish_msgs/SerialLine` | driver → nodes | every inbound line as it arrives |
| `succorfish/connected` | `std_msgs/Bool` (latched) | driver → nodes | link up/down |
| `succorfish/send_command` | `succorfish_msgs/SendCommand` (service) | node ↔ driver | synchronous request/response (write + wait for a reply matching a regex within a timeout) |

So **start the driver first**, picking the profile that matches your hardware —
`profile:=succorfish` (9600-baud `/dev/ttyUSB*`) or `profile:=teensy`
(115200-baud `/dev/ttyACM*`) — and keep node + driver in the same namespace.
See the [driver README](../succorfish_driver/README.md) for its parameters and
the two config profiles.

For a quick single-node run, every node launch file accepts `start_driver:=true`,
which brings up a co-located `succorfish_driver` (right profile/namespace for that
node) so you don't have to launch it yourself. Leave it `false` (the default) in
multi-node setups and run **one** shared driver separately — otherwise each launch
would spawn its own driver all contending for the one serial port. The
`driver_profile` and `driver_namespace` args let you override the picked profile
or namespace.

---

## Layout

This package lives inside the `serial_ping` meta-repo (see the
[meta README](../README.md)), as a sibling of the `succorfish_driver` submodule
and the shared `vendor/` and `microcontroller/` submodules.

```
serial_ping_pkg/                  # meta repo root (no package.xml)
├── serial_ping_pkg/              # THIS package
│   ├── serial_ping_pkg/          # python source
│   │   ├── utils.py              # shared library (config loader)
│   │   ├── common/               # general-purpose nodes (+ ping_protocol.py, driver_client.py)
│   │   ├── acoustic_relay/       # position broadcast/receive (+ pos_protocol.py)
│   │   ├── ping_estimator_action/# range-only localization action server (+ range_estimators.py)
│   │   ├── tuper_twtt/           # TWTT informed leader/follower (+ leader_protocol.py)
│   │   ├── tuper_owtt/           # OWTT leader/follower (Teensy front-end)
│   │   └── owtt_beacon/          # OWTT beacon telemetry + inference
│   ├── config/<subpkg>/...       # YAML defaults, grouped per sub-package
│   ├── launch/<subpkg>/...       # launch files, grouped per sub-package
│   ├── test/                     # pure-logic + lint tests
│   ├── data/                     # sample/data files installed to share/
│   └── package.xml  setup.py  setup.cfg
├── succorfish_driver/            # SUBMODULE: serial bridge node + interfaces
├── vendor/succorfish/            # SUBMODULE: NM3 firmware + Delphis manuals (NinjaTuna007/fishsuccor)
└── microcontroller/succor-sketches/  # SUBMODULE: Teensy/Arduino sketches (NinjaTuna007/succor-sketches)
```

See [../vendor/README.md](../vendor/README.md) and
[../microcontroller/README.md](../microcontroller/README.md) for the vendor
submodules, and [../succorfish_driver/README.md](../succorfish_driver/README.md)
for the serial bridge that every node here now talks to.

---

## Install & build

### Clone with submodules

The vendor firmware/manuals and the microcontroller sketches are git submodules,
so clone recursively:

```bash
git clone --recurse-submodules <repo-url>
# or, in an existing checkout:
git submodule update --init --recursive
```

### Dependencies

All Python and ROS 2 dependencies are declared in `package.xml`. Install them
with rosdep from the workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Build

```bash
cd ~/your_ros2_workspace
# Build this package plus the serial bridge it depends on
colcon build --packages-select succorfish_msgs succorfish_driver serial_ping_pkg
source install/setup.bash
```

---

## Quickstart

Start the `succorfish_driver` first (it owns the serial port), then bring up the
node(s) you need in the same namespace. Every launch file exposes all node
parameters as `arg:=value` overrides (the YAML files are just the bare-`ros2 run`
fallback). See each sub-package README for the full, fully-parametrized command
lines, and the [driver README](../succorfish_driver/README.md) for its profiles.
A few entry points:

```bash
# 0) The serial bridge (pick the profile for your hardware) -- keep it running
ros2 launch succorfish_driver succorfish_driver.launch                 # 9600 / ttyUSB (Succorfish)
ros2 launch succorfish_driver succorfish_driver.launch profile:=teensy # 115200 / ttyACM (OWTT stack)

# Two-leader alternating ping (common)
ros2 launch serial_ping_pkg serial_ping_node.launch

# Relay one robot's position over the modem (acoustic_relay)
ros2 launch serial_ping_pkg smarc_pos_broadcast_node.launch robot_name:=lolo
ros2 launch serial_ping_pkg smarc_pos_receiver_node.launch robot_names:=lolo,sam robot_modem_ids:=7,111

# Range-only localization action server (ping_estimator_action)
ros2 launch serial_ping_pkg ping_estimator_action.launch robot_name:=lolo
ros2 run    serial_ping_pkg ping_estimator_action_cmd add 7 --depth 12.0 --robot lolo
ros2 run    serial_ping_pkg ping_estimator_action_cmd ping --robot lolo

# TWTT informed leader / follower (tuper_twtt)
ros2 launch serial_ping_pkg twtt_leader_node.launch
ros2 launch serial_ping_pkg twtt_follower_node.launch

# OWTT leader / follower (tuper_owtt, Teensy front-end)
ros2 launch serial_ping_pkg owtt_leader_node.launch
ros2 launch serial_ping_pkg owtt_follower_node.launch
```

### No hardware? Run against a fake or simulated modem

The driver's `backend` decides where the bytes come from, so the nodes run
unchanged with no modem attached. Every node launch can also bring up the driver
(`start_driver:=true`, in the node's `robot_name` namespace) and forward the
backend choice:

```bash
# In-memory pretend modem (no hardware, deterministic replies)
ros2 launch serial_ping_pkg owtt_leader_node.launch start_driver:=true driver_backend:=test

# Bridge the smarcUnity acoustic transceiver (one-way-travel-time ranging in sim)
ros2 launch serial_ping_pkg owtt_beacon_node.launch \
    robot_name:=sam start_driver:=true driver_backend:=unity \
    own_modem_id:=101 unity_own_modem_id:=101 autostart:=true \
    telemetry_fields:="['depth']" depth_topic:=/sam/smarc/depth
```

See the [driver README](../succorfish_driver/README.md#backends-where-the-bytes-come-from)
for the full `test`/`unity` configuration and limitations.

---

## Testing

Pure-logic and lint tests live in `test/` (wire-format encode/decode, range
estimators, telemetry, plus `flake8`/`pep257`/copyright). The OWTT protocol
tests also run *full-stack*: they spin up the real `succorfish_driver` bound to a
pseudo-terminal and the application node together, and assert the traffic over
the driver's ROS interface (so the driver must be built/sourced for those):

```bash
# Logic + full-stack tests directly
python3 -m pytest test/ -q

# Or via colcon
colcon test --packages-select serial_ping_pkg
```

---

## Hardware notes

The Succorfish Delphis modems connect over USB serial (default 9600 baud; the
Teensy bridge runs at 115200). The serial port itself is configured on the
`succorfish_driver` (port/baudrate live in its config profiles, not here).
Useful checks:

```bash
ls /dev/ttyUSB*                 # find the modem port
sudo screen /dev/ttyUSB0 9600   # talk to the modem directly
```

In a `screen` session you can drive the modem by hand, e.g. broadcast a
13-character message to everyone:

```
$B13Hello, World!
```

If the port is permission-denied, add yourself to the `dialout` group
(`sudo usermod -a -G dialout $USER`, then re-login). For air testing, align the
modems to face each other; underwater, mind sound-velocity settings and avoid
bubbles. Firmware, flashing tools, and the full Delphis manuals are in the
`vendor/succorfish` submodule.

---

## Maintainer & license

**Shekhar Devm Upadhyay** (sdup@kth.se) — MIT License.

Thanks to the Succorfish team for hardware support and documentation, and the
SMARC project for integration standards.
