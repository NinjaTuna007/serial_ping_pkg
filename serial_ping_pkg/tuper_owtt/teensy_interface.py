"""Helpers for the serial protocol spoken with the Teensy 4.1 OWTT front-end.

The Teensy sits between the host (this ROS stack) and the Succorfish modem.
The host configures the Teensy's behaviour with a ``$Y`` command and then either
feeds it GPS positions (transmitter) or consumes time-delta reports (receiver).
In "wire" mode the Teensy is transparent and the host talks to the modem
directly.

Wire format (per the Teensy firmware, OWTT_Modem_LoLo_ver1):

    Config:   ``$Y<own_id(3)><mode><args>``
              ``$Y007W``         set address 007, WIRE mode
              ``$Y101R``         set address 101, RECEIVER mode
              ``$Y042T0001s``    address 042, TRANSMITTER, listen 000 (first), every 1 epoch
              ``$Y042T0074s``    address 042, TRANSMITTER, wait for 007, every 4 epochs
    GPS:      ``$G<lat>,<lon>``  (transmitter mode only; updates stored GPS, not forwarded)
    Broadcast (Teensy -> modem, automatic): ``$Bnn<lat>,<lon>``
    Modem broadcast relayed to host: ``#B<id(3)><nn(2)><data>`` e.g. ``#B00722<lat>,<lon>``
    OWTT delta (after a GPS-bearing modem line): ``#I<delta_us>``
"""

from enum import Enum


class TeensyMode(str, Enum):
    RECEIVER = "receiver"
    TRANSMITTER = "transmitter"
    WIRE = "wire"


# Single-letter codes used inside the $Y config command.
_MODE_CODE = {
    TeensyMode.RECEIVER: "R",
    TeensyMode.TRANSMITTER: "T",
    TeensyMode.WIRE: "W",
}


def build_config_command(mode, own_modem_id, listen_for_modem_id="000",
                         broadcast_interval_s=4, prefix="$Y"):
    """Build the ``$Y<own_id><mode><args>`` config command sent to the Teensy.

    ``own_modem_id`` is THIS node's modem address (set on the modem by the
    Teensy). Examples:
        receiver     -> ``$Y101R``
        transmitter  -> ``$Y042T0001s``   (listen 000 == go first, every 1 epoch)
        transmitter  -> ``$Y042T0074s``   (wait to hear modem 007, every 4 epochs)
        wire         -> ``$Y007W``
    """
    mode = TeensyMode(mode)
    code = _MODE_CODE[mode]
    own = str(own_modem_id).zfill(3)
    if mode == TeensyMode.TRANSMITTER:
        listen = str(listen_for_modem_id).zfill(3)
        return f"{prefix}{own}{code}{listen}{broadcast_interval_s}s"
    # Receiver is forcibly a non-broadcasting follower; wire is a pure passthrough.
    return f"{prefix}{own}{code}"


def build_gps_command(lat, lon, prefix="$G"):
    """Build the ``$G<lat>,<lon>`` command that updates the transmitter's GPS.

    TODO(plan): confirm coordinate precision / formatting expected by the Teensy.
    """
    return f"{prefix}{lat},{lon}"


def parse_owtt_delta(line, prefix="#I"):
    """Return the raw OWTT delta (microseconds) from a Teensy report line.

    Returns ``None`` if the line is not an OWTT-delta line.

    TODO(plan): confirm the numeric format of the delta payload.
    """
    if not line.startswith(prefix):
        return None
    try:
        return float(line[len(prefix):].strip())
    except ValueError:
        return None


def delta_to_range_m(delta_us, offset_us, sound_velocity_mps):
    """Convert a one-way travel-time delta into a range in metres.

    range = (delta_t - offset) * 1e-6 * c
    """
    return (float(delta_us) - float(offset_us)) * 1e-6 * float(sound_velocity_mps)


def parse_broadcast(line, prefix="#B"):
    """Parse a Succorfish broadcast frame relayed by the Teensy.

    Format: ``#B<modem_id(3)><num_chars(2)><data>`` where, in the OWTT scheme,
    ``data`` is ``lat,lon``.

    Returns ``(modem_id, lat, lon)`` or ``None`` if the line is not a parseable
    broadcast.
    """
    if not line.startswith(prefix):
        return None
    if len(line) < 7:
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
        return modem_id, float(parts[0]), float(parts[1])
    except ValueError:
        return None


def import_message_type(type_str):
    """Import a ROS message class from a string like ``svs_interfaces/msg/SVS``.

    Also accepts the two-field form ``pkg/Type``. Raises on failure so callers
    can decide how to degrade.
    """
    import importlib

    parts = [p for p in type_str.strip('/').split('/') if p]
    if len(parts) == 3:
        pkg, _, name = parts
    elif len(parts) == 2:
        pkg, name = parts
    else:
        raise ValueError(f"Unrecognised message type string: {type_str!r}")
    module = importlib.import_module(f"{pkg}.msg")
    return getattr(module, name)
