"""Pure wire-format helpers for the informed leader/follower exchange.

The leader pings a follower, converts the travel-time reply to a range (reusing
``common.ping_protocol``), then broadcasts its own GPS plus that range as a
``$B`` frame; the follower decodes the bare ``lat,lon,dist`` payload. The two
format pieces here are ROS- and serial-free so they can be unit-tested directly
(mirrors ``tuper_owtt.teensy_interface``).

Frame layout (``<num_chars>`` is the length of the payload that follows,
including the two commas)::

    $B<num_chars><lat>,<lon>,<dist>

The length field is computed from the *string* forms of the values exactly as
the leader emits them, so the encoder and the on-wire bytes stay consistent.
"""


def build_position_broadcast(lat, lon, dist):
    """Build the leader's ``$B`` position+distance frame (no trailing CRLF).

    The numeric length prefix counts the payload characters (the stringified
    ``lat``/``lon``/``dist`` plus the two separating commas). The caller appends
    ``\\r\\n`` before writing to the modem.
    """
    num_chars = len(str(lat)) + len(str(lon)) + len(str(dist)) + 2
    return f"$B{num_chars}{lat},{lon},{dist}"


def parse_position_distance(line):
    """Parse a bare ``<lat>,<lon>,<dist>`` follower payload.

    Returns ``(lat, lon, dist)`` as floats, or ``None`` if the line is not
    exactly three comma-separated numbers (e.g. a leftover ``$B`` header or a
    truncated frame).
    """
    parts = line.split(',')
    if len(parts) != 3:
        return None
    try:
        return float(parts[0]), float(parts[1]), float(parts[2])
    except ValueError:
        return None
