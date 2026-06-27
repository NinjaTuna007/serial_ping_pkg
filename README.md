# serial_ping (meta-repo)

This repository is a **meta directory** (not itself a ROS 2 package). It groups
several sibling ROS 2 packages and shared vendor resources so they can be cloned
and built together while staying independently buildable by `colcon`.

## Contents

| Path | Type | What it is |
|------|------|------------|
| [`serial_ping_pkg/`](serial_ping_pkg/README.md) | ROS 2 package (`ament_python`) | Acoustic communication & ranging nodes (ping, range-only localization, position relay, TWTT/OWTT leader-follower). |
| [`succorfish_driver/`](succorfish_driver/README.md) | **submodule** (2 packages) | `succorfish_driver` (transparent serial bridge node) + `succorfish_msgs` (interfaces). Owns the serial port and exposes it over ROS topics. |
| [`vendor/succorfish/`](vendor/README.md) | submodule | NM3 firmware + Succorfish Delphis manuals (`NinjaTuna007/fishsuccor`). |
| [`microcontroller/succor-sketches/`](microcontroller/README.md) | submodule | Teensy / Arduino sketches (`NinjaTuna007/succor-sketches`). |

```
serial_ping_pkg/                     # this meta repo (no package.xml at root)
├── serial_ping_pkg/                 # ROS package: acoustic comms/ranging nodes
├── succorfish_driver/               # SUBMODULE: serial bridge
│   ├── succorfish_driver/           #   - ament_python driver node
│   └── succorfish_msgs/             #   - ament_cmake/rosidl interfaces
├── vendor/succorfish/               # SUBMODULE: firmware + manuals
└── microcontroller/succor-sketches/ # SUBMODULE: microcontroller sketches
```

## Clone with submodules

```bash
git clone --recurse-submodules <repo-url>
# or, in an existing checkout:
git submodule update --init --recursive
```

## Build

All packages live under one workspace `src/`. From your workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

To build a subset:

```bash
colcon build --packages-select serial_ping_pkg succorfish_msgs succorfish_driver
```

## Layout note

The repo root intentionally has **no** `package.xml`. Each ROS package is a child
directory, so `colcon` discovers `serial_ping_pkg`, `succorfish_msgs`, and
`succorfish_driver` as independent sibling packages. See each package's README
for usage.

## Maintainer & license

**Shekhar Devm Upadhyay** (sdup@kth.se) — MIT License.
