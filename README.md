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

The shared library `serial_ping_pkg/utils.py` (`load_yaml_config`,
`init_serial`) sits at the package root and is imported by every sub-package.

---

## Layout

```
serial_ping_pkg/                  # repo root (one ROS 2 package)
├── serial_ping_pkg/              # python source
│   ├── utils.py                  # shared library (config loader + serial init)
│   ├── common/                   # general-purpose nodes (+ ping_protocol.py)
│   ├── acoustic_relay/           # position broadcast/receive (+ pos_protocol.py)
│   ├── ping_estimator_action/    # range-only localization action server (+ range_estimators.py)
│   ├── tuper_twtt/               # TWTT informed leader/follower (+ leader_protocol.py)
│   ├── tuper_owtt/               # OWTT leader/follower (Teensy front-end)
│   └── owtt_beacon/              # OWTT beacon telemetry + inference
├── config/<subpkg>/...           # YAML defaults, grouped per sub-package
├── launch/<subpkg>/...           # launch files, grouped per sub-package
├── test/                         # pure-logic + lint tests
├── data/                         # sample/data files installed to share/
├── vendor/succorfish/            # SUBMODULE: NM3 firmware + Delphis manuals (NinjaTuna007/fishsuccor)
├── microcontroller/succor-sketches/  # SUBMODULE: Teensy/Arduino sketches (NinjaTuna007/succor-sketches)
├── package.xml  setup.py  setup.cfg
```

See [vendor/README.md](vendor/README.md) and
[microcontroller/README.md](microcontroller/README.md) for the submodules.

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
colcon build --packages-select serial_ping_pkg
source install/setup.bash
```

---

## Quickstart

Every launch file exposes all node parameters as `arg:=value` overrides (the
YAML files are just the bare-`ros2 run` fallback). See each sub-package README
for the full, fully-parametrized command lines. A few entry points:

```bash
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

---

## Testing

Pure-logic and lint tests live in `test/` (wire-format encode/decode, range
estimators, telemetry, plus `flake8`/`pep257`/copyright):

```bash
# Logic tests directly
python3 -m pytest test/ -q

# Or via colcon
colcon test --packages-select serial_ping_pkg
```

---

## Hardware notes

The Succorfish Delphis modems connect over USB serial (default 9600 baud; the
Teensy bridge runs at 115200). Useful checks:

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
