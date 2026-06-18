# vendor/

Third-party reference material, kept out of the ROS package itself and pulled in
as a git submodule so the package stays lean. None of this is installed into the
ROS `share/` directory; it is documentation/firmware, not runtime data.

## `succorfish/` (submodule)

Tracks [`NinjaTuna007/fishsuccor`](https://github.com/NinjaTuna007/fishsuccor)
(formerly `suggonit`). It holds the manufacturer material for the Succorfish
Delphis / NM3 acoustic modem:

- `NM3Firmware/` — NM3 firmware, bootloader, and flashing/serial-command docs.
- `User Manual/` — Delphis datasheets, cable pin-out, user guide, and the MRL
  wiki rip (`mrl_wiki_rip.pdf`).
- `scripts/`, `data/` — early vendor/reference scripts and sample data.

## Getting / updating the submodule

```bash
# When first cloning the parent package
git clone --recurse-submodules <serial_ping_pkg url>

# Or, in an existing checkout
git submodule update --init --recursive

# Pull the latest vendor content later
git submodule update --remote vendor/succorfish
```
