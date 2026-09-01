# microcontroller/

Teensy / Arduino sketches that pair with the ROS nodes, as a submodule.

## `succor-sketches/`

Tracks [`NinjaTuna007/succor-sketches`](https://github.com/NinjaTuna007/succor-sketches):
OWTT timing and modem protocols the ROS nodes talk to over serial (the
`tuper_owtt` Teensy front-end, PPS/OCXO hybrid micro-ROS sketches, square-wave
generators, LOLO OWTT variants).

## Getting / updating

```bash
# first clone of this repo
git clone --recurse-submodules https://github.com/NinjaTuna007/serial_ping_pkg.git

# already cloned?
git submodule update --init --recursive

# pull newer sketches later
git submodule update --remote microcontroller/succor-sketches
```
