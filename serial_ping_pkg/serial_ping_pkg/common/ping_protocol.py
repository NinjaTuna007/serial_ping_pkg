"""Pure protocol/math helpers for the general-purpose serial ping nodes.

These are the wire-format and travel-time pieces shared by the ``common`` ping
nodes (and reused by the informed-leader-follower leader). They are deliberately
free of ROS and serial I/O so they can be unit-tested directly, mirroring
``tuper_owtt.teensy_interface``.

Two protocols live here:

* The Succorfish ping reply ``#R<id>T<ticks>`` -> range conversion.
* A leader broadcast: a normal ``#B<id(3)><len(2)>`` frame whose payload is
  ``lat,lon,dist``. The id is the modem address (``000``–``255``), so a leader
  at address 111 is ``#B111``.

The travel-time -> range constant is ``0.00003125`` (= 1 / 32000), the modem's
two-way tick period; multiplied by the sound velocity and the reported tick
count it yields a distance in metres.
"""

from succorfish_driver.nm3_frame import parse

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
    """Range (m) from a ping reply.

    A ``#R<id>T<ticks>`` frame uses the ticks from that frame. Any other
    buffer that still contains ``T<number>`` (the reply the driver matched)
    uses the number after the first ``T``. Raises ``IndexError`` /
    ``ValueError`` when there is no numeric travel time.
    """
    msg = parse(buf)
    if msg is not None and msg.kind == 'range':
        return travel_time_to_distance(msg.range_ticks, sound_velocity)
    travel_time = float(str(buf).split("T", 1)[1])
    return travel_time_to_distance(travel_time, sound_velocity)


def parse_leader_broadcast(line):
    """Parse a leader broadcast ``#B<id(3)><len(2)><lat>,<lon>,<dist>``.

    ``id`` is the modem address, zero-padded to three digits (``'007'``,
    ``'111'``). Returns ``(leader_id, lat, lon, dist)`` or ``None`` when the
    line is not a broadcast, the length field overruns, or the payload is not
    exactly three comma-separated floats.
    """
    msg = parse(line)
    if msg is None or msg.kind != 'broadcast' or msg.source_id is None:
        return None
    text = msg.payload_text()
    if text is None:
        return None
    parts = text.split(',')
    if len(parts) != 3:
        return None
    try:
        lat = float(parts[0])
        lon = float(parts[1])
        dist = float(parts[2])
    except ValueError:
        return None
    return msg.source_id, lat, lon, dist
