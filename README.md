# serial_ping_pkg

ROS 2 nodes for underwater acoustic ranging and comms with Succorfish Delphis /
NM3 modems (and a Teensy 4.1 front-end for one-way travel time).

Drop this checkout under a colcon workspace `src/` and build. The nodes live in
`serial_ping_pkg/`; the serial driver, vendor manuals, and Teensy sketches ride
along as submodules.

## What's in here

| Path | What it is |
|------|------------|
| [`serial_ping_pkg/`](serial_ping_pkg/README.md) | Acoustic comms and ranging: ping, range-only localization, position relay, TWTT/OWTT leader-follower. |
| [`succorfish_driver/`](succorfish_driver/README.md) | Serial bridge plus `succorfish_msgs`. Owns the modem port and exposes it over ROS. |
| [`vendor/succorfish/`](vendor/README.md) | NM3 firmware and Delphis manuals. |
| [`microcontroller/succor-sketches/`](microcontroller/README.md) | Teensy / Arduino sketches. |

The last three are git submodules (`NinjaTuna007/succorfish_driver`,
`NinjaTuna007/fishsuccor`, `NinjaTuna007/succor-sketches`).

## Clone

```bash
git clone --recurse-submodules https://github.com/NinjaTuna007/serial_ping_pkg.git
# already cloned?
git submodule update --init --recursive
```

## Build

From the workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Just the acoustic stack:

```bash
colcon build --packages-select serial_ping_pkg succorfish_msgs succorfish_driver
```

Each package has its own README for running nodes.

USB Delphis / NM3 only (no Teensy): see
[USB Delphis (no Teensy)](serial_ping_pkg/README.md#usb-delphis-no-teensy)
in the package README, or:

```bash
ros2 launch serial_ping_pkg delphis_ping.launch ping_command:=\$P007
```

## Maintainer

**Shekhar Devm Upadhyay** (sdup@kth.se) — MIT License.
