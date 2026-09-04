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

``lat``/``lon`` use ``ascii_on_air.LATLON_DECIMALS`` (7), ``depth`` is zero-padded
``DDD.DD`` (clamped to [0, 999.99]) and ``heading`` is ``DDD`` (clamped to
[0, 359]). See ``serial_ping_pkg.common.ascii_on_air``.
"""

from serial_ping_pkg.common.ascii_on_air import (
    format_depth_padded,
    format_heading,
    format_latlon,
)


def build_pos_broadcast(lat, lon, depth, heading):
    """Build the outbound ``$B`` position broadcast frame (no trailing CRLF).

    Depth and heading are clamped and width-padded via ``ascii_on_air``. The caller
    appends CRLF before writing to the modem.
    """
    data = format_latlon(lat, lon)
    depth_str = ',' + format_depth_padded(depth)
    heading_str = ',' + format_heading(heading)
    n_chars = len(data) + len(depth_str) + len(heading_str)
    return f'$B{n_chars}{data}{depth_str}{heading_str}'


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
