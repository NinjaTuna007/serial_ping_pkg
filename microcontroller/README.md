# microcontroller/

Microcontroller firmware/sketches that pair with this ROS package, pulled in as a
git submodule. Not installed into the ROS `share/` directory.

## `succor-sketches/` (submodule)

Tracks [`NinjaTuna007/succor-sketches`](https://github.com/NinjaTuna007/succor-sketches),
the Teensy/Arduino sketches implementing the OWTT timing and modem protocols the
ROS nodes talk to over serial (e.g. the `tuper_owtt` Teensy front-end). Includes
the PPS/OCXO-disciplined hybrid micro-ROS sketches, square-wave generators, and
the LOLO OWTT modem variants.

## Getting / updating the submodule

```bash
# When first cloning the parent package
git clone --recurse-submodules <serial_ping_pkg url>

# Or, in an existing checkout
git submodule update --init --recursive

# Pull the latest sketches later
git submodule update --remote microcontroller/succor-sketches
```
