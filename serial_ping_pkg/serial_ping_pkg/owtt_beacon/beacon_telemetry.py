"""Beacon telemetry codec: DCCL by default, tagged ASCII as a mode.

The Teensy marks telemetry frames with a leading ``TEL:`` (see
``teensy_interface.TELEMETRY_MARKER``); everything *after* that marker is what
this module encodes/decodes.

``codec='dccl'`` (default): ``libdccl`` ``BeaconTelemetry`` (id 124) plus a
CRC-8-ATM trailer. ``codec='ascii'``: the older tagged string
``P<lat>,<lon>;D<m>;…``. Beacon and surface unit must use the same mode.
"""

from serial_ping_pkg.common.ascii_on_air import (
    LATLON_DECIMALS,
    format_depth,
    format_latlon,
    format_speed,
    format_svs,
)

CODEC_DCCL = 'dccl'
CODEC_ASCII = 'ascii'
DEFAULT_CODEC = CODEC_DCCL

FIELD_TAGS = {
    'position': 'P',
    'depth': 'D',
    'svs': 'C',
    'speed': 'S',
    'bt': 'B',
}
TAG_FIELDS = {tag: field for field, tag in FIELD_TAGS.items()}
DEFAULT_FIELD_ORDER = ('position', 'depth', 'svs', 'speed', 'bt')
_FORBIDDEN = (';', '\r', '\n')
_TRIM_ORDER = ('bt', 'speed', 'depth', 'svs')
_DCCL_BT_MAX = 16


def normalize_codec(codec):
    """Return ``'dccl'`` or ``'ascii'``. Default is DCCL."""
    name = DEFAULT_CODEC if codec is None else str(codec).strip().lower()
    if name not in (CODEC_DCCL, CODEC_ASCII):
        raise ValueError(
            f'beacon codec must be {CODEC_DCCL!r} or {CODEC_ASCII!r}, '
            f'got {codec!r}')
    return name


def require_codec_runtime(codec):
    """Validate codec and load libdccl when the mode needs it.

    ``ascii`` does not import libdccl. ``dccl`` fails here if the library is
    missing, rather than on the first encode.
    """
    name = normalize_codec(codec)
    if name == CODEC_DCCL:
        from serial_ping_pkg.common.dccl_codec import _ensure_dccl
        _ensure_dccl()
    return name


def sanitize(text):
    """Strip delimiter characters from a free-text value (e.g. the bt tip)."""
    s = str(text)
    for ch in _FORBIDDEN:
        s = s.replace(ch, ' ')
    return s.strip()


def _encode_ascii(enabled_fields, position, depth, svs, speed, bt,
                  precision, max_bt_len, max_payload_len):
    if precision is None:
        precision = LATLON_DECIMALS
    enabled = set(enabled_fields)
    tokens = {}
    for field in DEFAULT_FIELD_ORDER:
        if field not in enabled:
            continue
        if field == 'position' and position is not None:
            lat, lon = position
            tokens['position'] = 'P' + format_latlon(
                lat, lon, decimals=precision)
        elif field == 'depth' and depth is not None:
            tokens['depth'] = 'D' + format_depth(depth)
        elif field == 'svs' and svs is not None:
            tokens['svs'] = 'C' + format_svs(svs)
        elif field == 'speed' and speed is not None:
            tokens['speed'] = 'S' + format_speed(speed)
        elif field == 'bt' and bt is not None:
            tokens['bt'] = 'B' + sanitize(bt)[:max_bt_len]

    def assemble(toks):
        return ';'.join(toks[f] for f in DEFAULT_FIELD_ORDER if f in toks)

    payload = assemble(tokens)
    if max_payload_len and len(payload) > max_payload_len:
        if 'bt' in tokens:
            over = len(payload) - max_payload_len
            bt_text = tokens['bt'][1:]
            kept = bt_text[:max(0, len(bt_text) - over)]
            if kept:
                tokens['bt'] = 'B' + kept
            else:
                tokens.pop('bt')
            payload = assemble(tokens)
        for field in _TRIM_ORDER:
            if len(payload) <= max_payload_len:
                break
            if field in tokens:
                tokens.pop(field)
                payload = assemble(tokens)
    return payload


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
    """Encode enabled telemetry fields.

    ``codec='dccl'`` (default) returns ``bytes`` (DCCL + CRC-8).
    ``codec='ascii'`` returns a tagged ASCII ``str``.
    """
    codec = normalize_codec(codec)
    if codec == CODEC_ASCII:
        return _encode_ascii(
            enabled_fields, position, depth, svs, speed, bt,
            precision, max_bt_len, max_payload_len)
    return _encode_dccl(
        enabled_fields, position, depth, svs, speed, bt,
        max_bt_len, max_payload_len)


def _decode_ascii(payload):
    out = {}
    if isinstance(payload, (bytes, bytearray)):
        try:
            payload = payload.decode('ascii')
        except UnicodeDecodeError:
            return None
    if not payload:
        return out
    for token in payload.split(';'):
        if not token:
            continue
        tag, value = token[0], token[1:]
        field = TAG_FIELDS.get(tag)
        if field == 'position':
            try:
                lat_s, lon_s = value.split(',')
                out['position'] = (float(lat_s), float(lon_s))
            except ValueError:
                pass
        elif field == 'depth':
            try:
                out['depth'] = float(value)
            except ValueError:
                pass
        elif field == 'svs':
            try:
                out['svs'] = float(value)
            except ValueError:
                pass
        elif field == 'speed':
            try:
                out['speed'] = float(value)
            except ValueError:
                pass
        elif field == 'bt':
            out['bt'] = value
        else:
            out.setdefault('unknown', {})[tag] = value
    return out


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
    """Decode post-marker payload to a dict of telemetry values.

    Uses the same ``codec`` as encode (default DCCL). Returns ``None`` if
    decode fails. Returns ``{}`` for empty input.
    """
    if payload is None or payload == b'' or payload == '':
        return {}
    codec = normalize_codec(codec)
    if codec == CODEC_ASCII:
        return _decode_ascii(payload)
    return _decode_dccl(payload)


def strip_marker(data, marker='TEL:'):
    """Return the telemetry payload (drop the leading marker) or ``None``.

    Binary ``TEL:`` + DCCL stays ``bytes``; ASCII ``TEL:…`` stays ``str``.
    """
    if data is None:
        return None
    if isinstance(data, (bytes, bytearray)):
        raw = bytes(data)
        m = marker.encode('ascii')
        if raw.startswith(m) and len(raw) > len(m):
            rest = raw[len(m):]
            try:
                return rest.decode('ascii')
            except UnicodeDecodeError:
                return rest
        return None
    if data and data.startswith(marker):
        return data[len(marker):]
    return None
