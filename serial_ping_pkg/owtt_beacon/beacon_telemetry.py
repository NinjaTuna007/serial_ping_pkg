"""Compact telemetry codec for the OWTT beacon broadcast frame.

The beacon broadcasts a tiny acoustic payload through the Teensy/Succorfish.
The Teensy marks telemetry frames with a leading ``TEL:`` (see
``teensy_interface.TELEMETRY_MARKER``); everything *after* that marker is what
this module encodes/decodes. Acoustic bandwidth is tiny, so the format is a
terse, tagged, ``;``-separated list of fields and the beacon only includes the
fields that are enabled (default: bt only):

    <tag><value>;<tag><value>;...

Tags (host convention, opaque to the Teensy):
    P  position   "P<lat>,<lon>"   (lat/lon at a configurable precision)
    D  depth      "D<m>"           (metres below surface, positive down)
    C  svs        "C<m/s>"         (sound velocity from the beacon's SVS sensor)
    S  speed      "S<m/s>"
    B  bt tip     "B<text>"        (free text, sanitised + length-capped)

The beacon measures the in-situ sound velocity, so it broadcasts it (``svs``)
and the surface units use that for their travel-time -> range conversion instead
of a frozen default.

Depth matters for localisation: the acoustic range the surface units measure is
a *slant* range to a possibly-submerged beacon. Broadcasting depth lets the
inference node convert slant ranges to horizontal ranges before triangulating.

Example (position + depth + svs + speed + bt):
    P58.823229,17.635998;D12.3;C1481.6;S1.20;BA_Chilling (Status.RUNNING)

Keep the total small: the Succorfish broadcast length is a 2-digit byte count
and real payloads are limited (~64 bytes), so prefer few fields / short bt text.
"""

# field name <-> single-char tag
FIELD_TAGS = {
    'position': 'P',
    'depth': 'D',
    'svs': 'C',
    'speed': 'S',
    'bt': 'B',
}
TAG_FIELDS = {tag: field for field, tag in FIELD_TAGS.items()}

# Default order in which fields are laid out in the payload.
DEFAULT_FIELD_ORDER = ('position', 'depth', 'svs', 'speed', 'bt')

# Characters that would corrupt the payload (field/line delimiters).
_FORBIDDEN = (';', '\r', '\n')


def sanitize(text):
    """Strip delimiter characters from a free-text value (e.g. the bt tip)."""
    s = str(text)
    for ch in _FORBIDDEN:
        s = s.replace(ch, ' ')
    return s.strip()


def encode_telemetry(enabled_fields, position=None, depth=None, svs=None, speed=None,
                     bt=None, precision=6, max_bt_len=32):
    """Encode the enabled telemetry fields into the post-marker payload string.

    ``enabled_fields`` is an iterable of field names (subset of
    ``FIELD_TAGS``); only fields that are both enabled AND have a value are
    emitted. ``position`` is ``(lat, lon)``. Returns the payload WITHOUT the
    ``TEL:`` marker (the Teensy prepends it), suitable for
    ``teensy_interface.build_telemetry_command``.
    """
    enabled = set(enabled_fields)
    parts = []
    for field in DEFAULT_FIELD_ORDER:
        if field not in enabled:
            continue
        if field == 'position' and position is not None:
            lat, lon = position
            parts.append(f"P{float(lat):.{precision}f},{float(lon):.{precision}f}")
        elif field == 'depth' and depth is not None:
            parts.append(f"D{float(depth):.1f}")
        elif field == 'svs' and svs is not None:
            parts.append(f"C{float(svs):.1f}")
        elif field == 'speed' and speed is not None:
            parts.append(f"S{float(speed):.2f}")
        elif field == 'bt' and bt is not None:
            parts.append("B" + sanitize(bt)[:max_bt_len])
    return ';'.join(parts)


def decode_telemetry(payload):
    """Decode a post-marker payload string into a dict of telemetry values.

    ``payload`` is the broadcast data with the leading ``TEL:`` marker already
    stripped. Returns a dict that may contain ``position`` ((lat, lon) tuple),
    ``speed`` (float), ``bt`` (str), and ``unknown`` (dict of unparsed tags).
    Malformed fields are skipped rather than raising.
    """
    out = {}
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


def strip_marker(data, marker='TEL:'):
    """Return the telemetry payload (drop the leading marker) or ``None``.

    ``data`` is the raw broadcast data from ``parse_broadcast_payload``. Returns
    the post-marker payload string if ``data`` is a telemetry frame (starts with
    ``marker``), else ``None`` (e.g. it is plain ``lat,lon`` GPS data).
    """
    if data and data.startswith(marker):
        return data[len(marker):]
    return None
