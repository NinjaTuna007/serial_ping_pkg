# vendor/

Manufacturer material for the Succorfish Delphis / NM3, pulled in as a submodule.

## `succorfish/`

Tracks [`NinjaTuna007/fishsuccor`](https://github.com/NinjaTuna007/fishsuccor)
(formerly `suggonit`):

- `NM3Firmware/` — firmware, bootloader, flashing docs.
- `User Manual/` — datasheets, cable pin-out, user guide, and the MRL wiki
  rip (`mrl_wiki_rip.pdf`).
- `scripts/`, `data/` — vendor reference scripts and sample data.

## Getting / updating

```bash
# first clone of this repo
git clone --recurse-submodules https://github.com/NinjaTuna007/serial_ping_pkg.git

# already cloned?
git submodule update --init --recursive

# pull newer vendor content later
git submodule update --remote vendor/succorfish
```
