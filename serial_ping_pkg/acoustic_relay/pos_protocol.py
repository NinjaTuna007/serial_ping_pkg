"""Pure wire-format helpers for the acoustic-relay position broadcast.

The relay broadcasts a SMaRC vehicle's GPS (plus optional depth/heading) over
the acoustic modem and the receiver decodes it on the far side. Both the
outbound ``$B`` builder and the inbound ``#B`` parser live here, free of ROS and
serial I/O, so they can be unit-tested directly (mirrors
``tuper_owtt.teensy_interface``).

Frame layout (``<num_chars>`` is the byte count of the payload that follows,
including the field-separating commas)::

    outbound: $B<num_chars><lat>,<lon>,<depth>,<heading>
    inbound:  #B<modem_id(3)><num_chars(2)><lat>,<lon>[,<depth>[,<heading>]]

``lat``/``lon`` use 8 decimal places, ``depth`` is ``DDD.D`` (5 chars, clamped
to [0, 999.9]) and ``heading`` is ``DDD`` (3 chars, clamped to [0, 359]).
"""


def build_pos_broadcast(lat, lon, depth, heading):
    """Build the outbound ``$B`` position broadcast frame (no trailing CRLF).

    ``depth`` is clamped to [0.0, 999.9] and formatted as ``DDD.D``; ``heading``
    is rounded/clamped to the integer range [0, 359] and formatted as ``DDD``.
    The caller appends ``\\r\\n`` before writing to the modem.
    """
    data = f"{lat:.8f},{lon:.8f}"
    depth_val = max(0.0, min(float(depth), 999.9))
    depth_str = f",{depth_val:05.1f}"
    heading_val = max(0, min(int(heading), 359))
    heading_str = f",{heading_val:03d}"
    n_chars = len(data) + len(depth_str) + len(heading_str)
    return f"$B{n_chars}{data}{depth_str}{heading_str}"


def parse_pos_broadcast(line):
    """Parse an inbound ``#B`` position broadcast.

    Returns ``(modem_id, lat, lon, depth, heading)`` where ``modem_id`` is the
    3-char id string, ``lat``/``lon``/``depth`` are floats and ``heading`` is an
    int; ``depth`` and/or ``heading`` are ``None`` when the payload omits them
    (2 fields -> both None, 3 fields -> heading None). Returns ``None`` for a
    malformed frame (wrong prefix, too short, bad length field, length mismatch,
    fewer than two fields, or non-numeric lat/lon).
    """
    if not line.startswith('#B') or len(line) < 7:
        return None
    modem_id = line[2:5]
    try:
        num_chars = int(line[5:7])
    except ValueError:
        return None
    data = line[7:7 + num_chars]
    if len(data) != num_chars:
        return None
    parts = data.split(',')
    if len(parts) < 2:
        return None
    try:
        lat = float(parts[0])
        lon = float(parts[1])
    except ValueError:
        return None
    depth = None
    heading = None
    try:
        if len(parts) == 4:
            depth = float(parts[2])
            heading = int(parts[3])
        elif len(parts) == 3:
            depth = float(parts[2])
    except ValueError:
        return None
    return modem_id, lat, lon, depth, heading
