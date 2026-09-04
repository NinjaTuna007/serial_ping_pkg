"""DCCL encode/decode plus a CRC-8 trailer for NM3 payloads.

Schema lives in ``serial_ping_pkg/proto/dccl_acoustic.proto``. This module is the
only place nodes should touch ``libdccl``. Transmit/receive logic fills a dict
(or protobuf), calls ``pack`` / ``unpack``, and writes the bytes.

Wire layout after the Teensy ``TEL:`` marker::

    <dccl bytes, first byte is the message id> <crc8>

CRC-8 is CRC-8-ATM / ITU-T I.432.1 (poly 0x07, init 0x00). DCCL itself does
not checksum; the extra byte is so a flipped bit in a packed field is dropped
instead of becoming a plausible lat.

Private DCCL ids 124-127 (one-byte). 124 = BeaconTelemetry, 125 = Pose.
"""

from __future__ import annotations

import ctypes
import os
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# libdccl is not on Ubuntu's default PYTHONPATH / ld path. A user-local prefix
# from the GobySoft debs lives at ~/.local/opt/dccl4 (see journal). System
# python3-dccl4 works too if it is already installed.
# ---------------------------------------------------------------------------

_DCCL4_PREFIX = os.environ.get(
    'DCCL4_PREFIX', os.path.expanduser('~/.local/opt/dccl4'))


dccl = None
_PREFIX = Path(_DCCL4_PREFIX)


def _ensure_dccl():
    """Import libdccl on first pack/unpack. ASCII codec never calls this."""
    global dccl
    if dccl is not None:
        return
    prefix = Path(_DCCL4_PREFIX)
    libdir = prefix / 'usr' / 'lib' / 'x86_64-linux-gnu'
    pydir = prefix / 'usr' / 'lib' / 'python3' / 'dist-packages'
    if pydir.is_dir() and str(pydir) not in sys.path:
        sys.path.insert(0, str(pydir))
    if libdir.is_dir():
        # The extension module links libdccl; preload so we don't need
        # LD_LIBRARY_PATH in every launch file.
        for name in ('libcrypto++.so.8', 'libdccl.so.31'):
            so = libdir / name
            if so.is_file():
                ctypes.CDLL(str(so), mode=ctypes.RTLD_GLOBAL)
    import dccl as _dccl  # after path + CDLL
    dccl = _dccl


# CRC-8-ATM / ITU-T I.432.1
_CRC8_POLY = 0x07
_CRC8_INIT = 0x00

ID_BEACON_TELEMETRY = 124
ID_POSE = 125


def crc8_atm(data):
    """CRC-8-ATM (poly 0x07, init 0x00, no reflect, xorout 0)."""
    crc = _CRC8_INIT
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ _CRC8_POLY) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _proto_path():
    here = Path(__file__).resolve()
    # .../serial_ping_pkg/serial_ping_pkg/common/dccl_codec.py -> ament pkg proto/
    src = here.parents[2] / 'proto' / 'dccl_acoustic.proto'
    if src.is_file():
        return src
    try:
        from ament_index_python.packages import get_package_share_directory
        share = (
            Path(get_package_share_directory('serial_ping_pkg'))
            / 'proto' / 'dccl_acoustic.proto')
        if share.is_file():
            return share
    except Exception:
        pass
    raise FileNotFoundError(
        'dccl_acoustic.proto not found in source tree or share/')


_CODEC = None


def _codec():
    global _CODEC
    if _CODEC is not None:
        return _CODEC
    _ensure_dccl()
    for candidate in (_PREFIX / 'usr' / 'include', Path('/usr/include')):
        if (candidate / 'dccl' / 'option_extensions.proto').is_file():
            dccl.addProtoIncludePath(str(candidate))
            break
    proto = _proto_path().resolve()
    dccl.addProtoIncludePath(str(proto.parent))
    dccl.loadProtoFile(str(proto))
    codec = dccl.Codec()
    codec.load('BeaconTelemetry')
    codec.load('Pose')
    _CODEC = codec
    return _CODEC


def pack(msg):
    """Encode a protobuf message and append CRC-8. Returns ``bytes``."""
    encoded = _codec().encode(msg)
    if isinstance(encoded, str):
        encoded = encoded.encode('latin-1')
    else:
        encoded = bytes(encoded)
    return encoded + bytes([crc8_atm(encoded)])


def unpack(blob):
    """Split CRC-8, verify, DCCL-decode. Returns a protobuf message.

    Raises ``ValueError`` on a short blob or CRC mismatch.
    """
    raw = bytes(blob)
    if len(raw) < 2:
        raise ValueError('DCCL blob too short')
    body, got = raw[:-1], raw[-1]
    expect = crc8_atm(body)
    if got != expect:
        raise ValueError(f'CRC-8 mismatch (got 0x{got:02x}, expect 0x{expect:02x})')
    return _codec().decode(body)


def message_id(blob):
    """DCCL id byte, ignoring a trailing CRC if present."""
    raw = bytes(blob)
    if not raw:
        return None
    body = raw[:-1] if len(raw) >= 2 else raw
    ident = getattr(_codec(), 'id', None)
    if ident is None:
        return body[0]
    try:
        return int(ident(body))
    except Exception:
        return body[0]
