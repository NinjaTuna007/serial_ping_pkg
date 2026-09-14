# serial_ping_pkg

Underwater acoustic communication and ranging with Succorfish Delphis / NM3
modems (and a Teensy 4.1 front-end for one-way travel time): pinging,
range-only localization, position relay, and leader/follower.

Each scenario has its own code, config, launch files, and README, sharing one
`ament_python` package.

---

## USB Delphis (no Teensy)

If you have a Succorfish Delphis / NM3 on USB and just want to ping another
modem:

1. Plug the dongle in. It usually shows up as `/dev/ttyUSB0` (9600 baud). Your
   user needs to be in the `dialout` group (`sudo usermod -aG dialout $USER`,
   then log out and back in).
2. Build and source the workspace (see [Install & build](#install--build)).
3. Ping a remote 3-digit modem id:

```bash
ros2 launch serial_ping_pkg delphis_ping.launch ping_command:=\$P007
# if the dongle is not ttyUSB0:
ros2 launch serial_ping_pkg delphis_ping.launch serial_port:=/dev/ttyUSB1 ping_command:=\$P007
```

That starts `succorfish_driver` (Succorfish profile) and
`single_target_ping_node` together. Range is published as
`/<robot_name>/distance` (`std_msgs/Float32`; default robot name `delphis`).
Echo it with `ros2 topic echo /delphis/distance`.

Desk test with no hardware: add `driver_backend:=test` (in-memory pretend
modem). Same launch, no Teensy, no PPS.

To relay GPS over the acoustic link, use
[`acoustic_relay`](serial_ping_pkg/acoustic_relay/README.md). For two-way
informed ranging, [`tuper_twtt`](serial_ping_pkg/tuper_twtt/README.md).
One-way travel time with a Teensy front-end is a separate stack:
[`tuper_owtt`](serial_ping_pkg/tuper_owtt/README.md).

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

Shared config loading lives in `serial_ping_pkg/utils.py`. Nodes talk to the
modem through `succorfish_driver` via `serial_ping_pkg/common/driver_client.py`
(`DriverClient`) — they don't open the serial port themselves.

---

## Talking to the modem (the driver)

The physical link (Succorfish modem or Teensy front-end) is owned by
[`succorfish_driver`](../succorfish_driver/README.md). Nodes here talk to it
over a small ROS interface (names from `succorfish_msgs/Topics`, relative so
they line up by namespace):

| Name | Type | Direction | Use |
|------|------|-----------|-----|
| `succorfish/tx` | `std_msgs/String` | node → driver | raw outbound command (driver appends the terminator) |
| `succorfish/rx` | `succorfish_msgs/SerialLine` | driver → nodes | inbound **text** lines (printable `#B`/`#U` included) |
| `succorfish/tx_bytes` | `succorfish_msgs/SerialFrame` | node → driver | raw UART bytes, no terminator (binary `$B`/`$U`) |
| `succorfish/rx_bytes` | `succorfish_msgs/SerialFrame` | driver → nodes | every inbound frame as bytes |
| `succorfish/connected` | `std_msgs/Bool` (latched) | driver → nodes | link up/down |
| `succorfish/send_command` | `succorfish_msgs/SendCommand` (service) | node ↔ driver | synchronous request/response (write + wait for a reply matching a regex within a timeout) |

Those `tx` / `tx_bytes` rows are the **UART pipe** (text line vs raw bytes,
including a payload that contains `0x0A`). How a float is written *inside* the
acoustic blob — glyphs `14.2334` vs packed DCCL bits — is
[`proto/README.md`](proto/README.md).

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

```
serial_ping_pkg/
├── proto/                     # on-air codecs (ASCII digits vs DCCL bits)
├── serial_ping_pkg/           # python source
│   ├── utils.py               # shared config loader
│   ├── common/                # general-purpose nodes (+ ping_protocol.py, driver_client.py)
│   ├── acoustic_relay/        # position broadcast/receive (+ pos_protocol.py)
│   ├── ping_estimator_action/ # range-only localization action server (+ range_estimators.py)
│   ├── tuper_twtt/            # TWTT informed leader/follower (+ leader_protocol.py)
│   ├── tuper_owtt/            # OWTT leader/follower (Teensy front-end)
│   └── owtt_beacon/           # OWTT beacon telemetry + inference
├── config/<subpkg>/
├── launch/<subpkg>/
├── test/
├── data/
└── package.xml  setup.py  setup.cfg
```

The serial driver, vendor manuals, and Teensy sketches sit next to this package
in the [repo root](../README.md).

---

## Install & build

### Clone with submodules

```bash
git clone --recurse-submodules https://github.com/NinjaTuna007/serial_ping_pkg.git
# already cloned?
git submodule update --init --recursive
```

### Dependencies

ROS 2 Python deps are in `package.xml`. From the workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

The default beacon codec (`beacon.codec:=dccl`) also needs **libdccl**, which
is **not** in Ubuntu/rosdep. Add the [GobySoft release repo](https://packages.gobysoft.org/ubuntu/release/)
and install `python3-dccl4` + `libdccl4` ([upstream steps](https://libdccl.org/4.0/)):

```bash
export COMPONENT=release
export GOBYSOFT_SIGNING_KEY=19478082E2F8D3FE
sudo install -d -m 0755 /etc/apt/keyrings
gpg --keyserver keyserver.ubuntu.com --recv-keys ${GOBYSOFT_SIGNING_KEY} \
  && gpg --export ${GOBYSOFT_SIGNING_KEY} | sudo tee /etc/apt/keyrings/gobysoft.gpg >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/gobysoft.gpg] http://packages.gobysoft.org/$(. /etc/os-release; echo "$ID")/${COMPONENT}/ $(. /etc/os-release; echo "$VERSION_CODENAME")/" \
  | sudo tee /etc/apt/sources.list.d/gobysoft_${COMPONENT}.list
sudo apt update
sudo apt install python3-dccl4 libdccl4
```

`rosdep -r` will warn that it cannot resolve `python3-dccl4` / `libdccl4`; that
is expected. `beacon.codec:=ascii` (tagged `TEL:`) does not need libdccl.

If apt is not an option, extract the GobySoft debs to `~/.local/opt/dccl4` (or
set `DCCL4_PREFIX`) so `dccl_codec` can find `libdccl.so` and the Python
extension. `setup.py` cannot pip-install libdccl.

### Build

```bash
colcon build --packages-select succorfish_msgs succorfish_driver serial_ping_pkg
source install/setup.bash
```

---

## Quickstart

Start the `succorfish_driver` first (it owns the serial port), then bring up the
node(s) you need in the same namespace. Launch files take `arg:=value` overrides;
the YAML files are the fallback for a bare `ros2 run`. See each sub-package
README for the full command lines, and the
[driver README](../succorfish_driver/README.md) for its profiles. A few entry
points:

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
ros2 launch serial_ping_pkg owtt_follower_node.launch.py
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
