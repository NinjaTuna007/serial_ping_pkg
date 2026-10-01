"""Beacon telemetry codec: DCCL ``BeaconTelemetry`` (id 124) plus CRC-8-ATM.

Application bytes sit after the Teensy timestamp envelope. Beacon and surface
unit both use this encoding.
"""

CODEC_DCCL = 'dccl'
DEFAULT_CODEC = CODEC_DCCL

_FORBIDDEN = (';', '\r', '\n')
_TRIM_ORDER = ('bt', 'speed', 'depth', 'svs')
_DCCL_BT_MAX = 16


def normalize_codec(codec):
    """Return ``'dccl'``. ``None`` and ``'dccl'`` are accepted."""
    name = DEFAULT_CODEC if codec is None else str(codec).strip().lower()
    if name != CODEC_DCCL:
        raise ValueError(
            f'beacon codec must be {CODEC_DCCL!r}, got {codec!r}')
    return name


def require_codec_runtime(codec):
    """Validate the codec and load libdccl before the first encode."""
    name = normalize_codec(codec)
    from serial_ping_pkg.common.dccl_codec import _ensure_dccl
    _ensure_dccl()
    return name


def sanitize(text):
    """Strip delimiter characters from a free-text value (e.g. the bt tip)."""
    s = str(text)
    for ch in _FORBIDDEN:
        s = s.replace(ch, ' ')
    return s.strip()


def _encode_dccl(enabled_fields, position, depth, svs, speed, bt,
                 max_bt_len, max_payload_len):
    from serial_ping_pkg.common.dccl_codec import _ensure_dccl, pack
    _ensure_dccl()
    from serial_ping_pkg.common import dccl_acoustic_pb2 as acoustic_pb2

    max_bt_len = min(int(max_bt_len), _DCCL_BT_MAX)
    enabled = set(enabled_fields)
    tokens = {
        'position': position,
        'depth': depth,
        'svs': svs,
        'speed': speed,
        'bt': bt,
    }

    def build(toks):
        msg = acoustic_pb2.BeaconTelemetry()
        if 'position' in enabled and toks.get('position') is not None:
            lat, lon = toks['position']
            msg.lat = float(lat)
            msg.lon = float(lon)
        if 'depth' in enabled and toks.get('depth') is not None:
            msg.depth = float(toks['depth'])
        if 'svs' in enabled and toks.get('svs') is not None:
            msg.svs = float(toks['svs'])
        if 'speed' in enabled and toks.get('speed') is not None:
            msg.speed = float(toks['speed'])
        if 'bt' in enabled and toks.get('bt') is not None:
            msg.bt = sanitize(toks['bt'])[:max_bt_len]
        if not (msg.HasField('lat') or msg.HasField('lon')
                or msg.HasField('depth') or msg.HasField('svs')
                or msg.HasField('speed') or msg.HasField('bt')):
            return b''
        return pack(msg)

    payload = build(tokens)
    if max_payload_len and len(payload) > max_payload_len:
        for field in _TRIM_ORDER:
            if len(payload) <= max_payload_len:
                break
            if tokens.get(field) is not None:
                tokens[field] = None
                payload = build(tokens)
    return payload


def encode_telemetry(enabled_fields, position=None, depth=None, svs=None,
                     speed=None, bt=None, precision=None, max_bt_len=32,
                     max_payload_len=0, codec=DEFAULT_CODEC):
    """Encode enabled telemetry fields as DCCL bytes plus a CRC-8 trailer.

    ``precision`` is unused (DCCL precision lives in the proto). It remains
    so existing call sites keep working.
    """
    del precision
    normalize_codec(codec)
    return _encode_dccl(
        enabled_fields, position, depth, svs, speed, bt,
        max_bt_len, max_payload_len)


def _decode_dccl(payload):
    from serial_ping_pkg.common.dccl_codec import _ensure_dccl, unpack
    _ensure_dccl()
    from serial_ping_pkg.common import dccl_acoustic_pb2 as acoustic_pb2

    raw = (payload if isinstance(payload, (bytes, bytearray))
           else payload.encode('latin-1'))
    try:
        msg = unpack(raw)
    except Exception:
        return None
    if not isinstance(msg, acoustic_pb2.BeaconTelemetry):
        return None
    out = {}
    if msg.HasField('lat') and msg.HasField('lon'):
        out['position'] = (msg.lat, msg.lon)
    if msg.HasField('depth'):
        out['depth'] = msg.depth
    if msg.HasField('svs'):
        out['svs'] = msg.svs
    if msg.HasField('speed'):
        out['speed'] = msg.speed
    if msg.HasField('bt'):
        out['bt'] = msg.bt
    return out


def decode_telemetry(payload, codec=DEFAULT_CODEC):
    """Decode a DCCL telemetry payload to a dict.

    Returns ``None`` if decode fails. Returns ``{}`` for empty input.
    """
    if payload is None or payload == b'' or payload == '':
        return {}
    normalize_codec(codec)
    return _decode_dccl(payload)
