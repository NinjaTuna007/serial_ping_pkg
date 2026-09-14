# On-air codecs

Two contracts, two files. ROS topics stay full-precision (`GeoPoint`, `Float32`);
only the acoustic blob is quantized. The NM3 cap is **64 bytes** either way.

This page is the **digits vs bits** story (how a float is written into that
blob). The driver byte pipe (`succorfish/tx` vs `tx_bytes`) is a different
question: can the UART carry a non-printable payload at all? See
[`succorfish_driver/README.md`](../../succorfish_driver/README.md#byte-pipe-serialframe).

| File | Codec | Who uses it |
|------|--------|-------------|
| [`dccl_acoustic.proto`](dccl_acoustic.proto) | DCCL bit layout (`BeaconTelemetry` id 124, `Pose` id 125) | `beacon.codec:=dccl` (default) |
| [`../serial_ping_pkg/common/ascii_on_air.py`](../serial_ping_pkg/common/ascii_on_air.py) | ASCII decimal strings | `$G`, relay/TWTT `$B`, `beacon.codec:=ascii` |
| [`../serial_ping_pkg/common/dccl_codec.py`](../serial_ping_pkg/common/dccl_codec.py) | `pack` / `unpack` + CRC-8-ATM | only place nodes touch libdccl |
| [`../serial_ping_pkg/common/dccl_acoustic_pb2.py`](../serial_ping_pkg/common/dccl_acoustic_pb2.py) | generated stubs | **do not edit**; regenerate from this proto |

Relay padded depth `DDD.DD` (max 999.99 m) is an ASCII frame width, not the DCCL
depth cap (3000 m). Relay / TWTT / ping stay ASCII. DCCL is the beacon telemetry
path (`TEL:` after the Teensy marker).

There is no ROS `.msg` clone of the proto. After decode, hosts keep a Python dict.

## ASCII digits vs packed bits

Take a number like `14.2334`.

**ASCII** (`ascii_on_air`, tagged `TEL:`, `$G`, relay `$B`) sends the glyphs
`1` `4` `.` `2` `3` `3` `4` — seven bytes for that one field, plus commas and
tags. At the catalog's 7 decimal places the same latitude prints as
`14.2334000` (ten characters). A human can read it on `succorfish/rx`. The cost
scales with how many digits you print, not with the physical range of the
quantity.

**DCCL** (`beacon.codec:=dccl`) does not send those characters. It scales the
value into an integer given `min` / `max` / `precision` in this proto and packs
that integer into bits. Latitude at 7 decimal places (`min: -90`, `max: 90`) is
31 bits (~4 bytes), not ten ASCII digits. Longitude at the same grain is 32
bits. Depth 0–3000 m at centimetres is 19 bits. A CRC-8 trailer is one extra
byte on the packed blob (`dccl_codec`, not inside libdccl).

Same 64-byte modem cap. ASCII burns it on digits and punctuation; DCCL spends
it on more fields and the CRC. A DCCL blob is not UTF-8 text, so it goes out
on `succorfish/tx_bytes` (and back on `rx_bytes`). Printable ASCII payloads
still appear on `succorfish/rx` as well.

There is **no auto-fallback**: a DCCL receiver drops tagged ASCII, and an ASCII
receiver drops a DCCL blob. Beacon and surface unit must set the same
`beacon.codec`.
