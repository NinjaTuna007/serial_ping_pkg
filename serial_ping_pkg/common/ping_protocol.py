"""Pure protocol/math helpers for the general-purpose serial ping nodes.

These are the wire-format and travel-time pieces shared by the ``common`` ping
nodes (and reused by the informed-leader-follower leader). They are deliberately
free of ROS and serial I/O so they can be unit-tested directly, mirroring
``tuper_owtt.teensy_interface``.

Two protocols live here:

* The Succorfish ping reply ``...T<travel_time>`` -> range conversion.
* The leader broadcast frame ``#B00<leader(1)><len(2)><lat,lon,dist>`` parse.

The travel-time -> range constant is ``0.00003125`` (= 1 / 32000), the modem's
two-way tick period; multiplied by the sound velocity and the reported tick
count it yields a distance in metres.
"""

# Modem tick period (s) baked into the Succorfish travel-time reply.
TICK_PERIOD_S = 0.00003125


def travel_time_to_distance(travel_time, sound_velocity):
    """Convert a Succorfish ``T`` travel-time tick count to a range in metres."""
    return TICK_PERIOD_S * sound_velocity * travel_time


def ping_response_complete(buf):
    """True once the accumulated serial buffer holds a full ``T`` reply.

    Matches the node loop's gate: a ``T`` is present and at least five
    characters of travel-time follow it.
    """
    return "T" in buf and len(buf.split("T")[1]) >= 5


def parse_ping_distance(buf, sound_velocity):
    """Range (m) from a complete ping reply buffer.

    Takes the travel-time after the first ``T`` and converts it. Raises
    ``IndexError``/``ValueError`` on a malformed buffer, matching the existing
    node behaviour (callers catch those).
    """
    travel_time = float(buf.split("T")[1])
    return travel_time_to_distance(travel_time, sound_velocity)


def parse_leader_broadcast(line):
    """Parse a ``#B00<leader(1)><len(2)><lat,lon,dist>`` leader broadcast.

    Returns ``(leader_id, lat, lon, dist)`` (leader_id as a 1-char string,
    the rest as floats) or ``None`` if the line is not a well-formed leader
    broadcast (wrong prefix, too short, bad/short length field, or the payload
    is not exactly three comma-separated floats).
    """
    if not line.startswith('#B00') or len(line) < 7:
        return None
    leader_id = line[4]
    try:
        msg_len = int(line[5:7])
    except ValueError:
        return None
    expected_total_length = 7 + msg_len
    if len(line) < expected_total_length:
        return None
    data = line[7:7 + msg_len]
    parts = data.split(',')
    if len(parts) != 3:
        return None
    try:
        lat = float(parts[0])
        lon = float(parts[1])
        dist = float(parts[2])
    except ValueError:
        return None
    return leader_id, lat, lon, dist
