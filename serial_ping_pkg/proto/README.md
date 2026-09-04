# On-air codecs

Two contracts, two files. ROS topics stay full-precision (`GeoPoint`, `Float32`);
only the acoustic blob is quantized.

| File | Codec | Who uses it |
|------|--------|-------------|
| [`dccl_acoustic.proto`](dccl_acoustic.proto) | DCCL bit layout (`BeaconTelemetry` id 124, `Pose` id 125) | `beacon.codec:=dccl` (default) |
| [`../serial_ping_pkg/common/ascii_on_air.py`](../serial_ping_pkg/common/ascii_on_air.py) | ASCII decimal strings | `$G`, relay/TWTT `$B`, `beacon.codec:=ascii` |
| [`../serial_ping_pkg/common/dccl_codec.py`](../serial_ping_pkg/common/dccl_codec.py) | `pack` / `unpack` + CRC-8-ATM | only place nodes touch libdccl |
| [`../serial_ping_pkg/common/dccl_acoustic_pb2.py`](../serial_ping_pkg/common/dccl_acoustic_pb2.py) | generated protobuf stubs | **do not edit**; regenerate from this proto |

Relay padded depth `DDD.DD` (max 999.99 m) is an ASCII frame width, not the DCCL
depth cap (3000 m).

There is no ROS `.msg` clone of the proto. After decode, hosts keep a Python dict.
