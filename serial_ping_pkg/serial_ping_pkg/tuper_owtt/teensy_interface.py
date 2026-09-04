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


def normalize_modem_id(value, width=3, default="000"):
    """Coerce a modem id into a canonical zero-padded string (e.g. ``'069'``).

    ``ros2 launch`` will happily turn a numeric-looking override such as ``069``
    into the float ``69.0`` (even when the ``<param>`` declares ``type="str"``),
    and a config YAML may store it as an int. This accepts ``str`` / ``int`` /
    ``float`` and returns the canonical ``width``-digit id, restoring any leading
    zero that the numeric coercion dropped. Non-numeric values pass through
    unchanged.
    """
    if value is None or isinstance(value, bool):
        value = default
    if isinstance(value, float):
        value = int(round(value))
    s = str(value).strip()
    if s.endswith(".0") and s[:-2].isdigit():  # stringified float, e.g. '69.0'
        s = s[:-2]
    return s.zfill(width) if s.isdigit() else s


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
    own = normalize_modem_id(own_modem_id)
    if mode == TeensyMode.TRANSMITTER:
        listen = normalize_modem_id(listen_for_modem_id)
        return f"{prefix}{own}{code}{listen}{broadcast_interval_s}s"
    # Receiver is forcibly a non-broadcasting follower; wire is a pure passthrough.
    return f"{prefix}{own}{code}"


def build_gps_command(lat, lon, prefix="$G"):
    """Build the ``$G<lat>,<lon>`` command that updates the transmitter's GPS.

    Lat/lon are formatted with ``on_air.LATLON_DECIMALS``. The Teensy stores the
    substring after ``$G`` and puts it on air unchanged.
    """
    from serial_ping_pkg.common.on_air import format_latlon
    return f'{prefix}{format_latlon(lat, lon)}'


# Marker that the Teensy prepends to a telemetry broadcast payload. A received
# broadcast whose *application* data starts with this marker is a telemetry
# frame (and the Teensy still emits an #I timing line for it, so it is
# range-able). A multi-char marker (vs a bare 'T') avoids mistaking ordinary
# broadcast data for telemetry — and also avoids colliding with the stick-v1
# absolute-time envelope which likewise starts with ``T``.
TELEMETRY_MARKER = "TEL:"

# stick-v1 / LOLO_ver3 on-air envelope (before the application payload):
#   T<ss>|<seq:hex>|<P|H|W>[|<holdover_age_s:hex>]|<application>
# ss is the TX UTC time-of-minute (seconds, 0-59); broadcasts are PPS-aligned
# so the sub-second fraction is always ~0 and is omitted. Full unix seconds are
# redundant because propagation is only a few seconds; the Teensy resolves the
# minute against its own UTC. holdover is emitted only when non-zero (TX
# free-running); locked transmitters omit the field. Application (``lat,lon``
# or ``TEL:…``) follows. ROS ranging uses Teensy ``#I`` (absolute OWTT), so
# nodes unwrap this and ignore the TX stamp unless a caller asks.


def _as_bytes(data):
    if isinstance(data, (bytes, bytearray, memoryview)):
        return bytes(data)
    if isinstance(data, str):
        return data.encode('latin-1')
    raise TypeError(f'expected bytes or str, got {type(data)!r}')


def application_as_text(data):
    """Return ASCII text, or None when ``data`` is a binary application blob."""
    if isinstance(data, bytes):
        try:
            return data.decode('ascii')
        except UnicodeDecodeError:
            return None
    return data


def unwrap_timestamp_envelope(data):
    """Strip a stick-v1 TX timestamp envelope if present.

    Returns ``(meta, application)``. When ``data`` is not enveloped (legacy
    bare ``lat,lon``, ``TEL:…``, ``START``, …), ``meta`` is ``None`` and
    ``application`` is ``data`` unchanged.

    ``data`` may be ``str`` or ``bytes``. A binary application slot (DCCL)
    is returned as ``bytes``; ASCII application is returned as ``str``.

    ``meta`` keys: ``tx_ss`` (int, 0-59), ``tx_fraction_us`` (int, always 0 for
    the compact form), ``tx_time_us`` (``tx_ss*1e6``), ``sequence``,
    ``timing_mode``, ``holdover_age_s`` (0 when the field is omitted).
    """
    raw = _as_bytes(data)
    if not raw or raw[0] != ord('T') or raw.startswith(TELEMETRY_MARKER.encode('ascii')):
        if isinstance(data, str):
            return None, data
        text = application_as_text(raw)
        return None, text if text is not None else raw

    # T<ss>|<seq:hex>|<P|H|W>[|<holdover:hex>]|<application>
    i = 1
    while i < len(raw) and 48 <= raw[i] <= 57:
        i += 1
    if i == 1 or i >= len(raw) or raw[i] != ord('|'):
        app = application_as_text(raw)
        return None, app if app is not None else data
    try:
        tx_ss = int(raw[1:i])
    except ValueError:
        app = application_as_text(raw)
        return None, app if app is not None else data
    if tx_ss > 59:
        app = application_as_text(raw)
        return None, app if app is not None else data

    rest = raw[i + 1:]
    j = rest.find(b'|')
    if j < 0:
        app = application_as_text(raw)
        return None, app if app is not None else data
    try:
        sequence = int(rest[:j], 16)
    except ValueError:
        app = application_as_text(raw)
        return None, app if app is not None else data
    if sequence > 0xFFFF:
        app = application_as_text(raw)
        return None, app if app is not None else data

    rest = rest[j + 1:]
    if len(rest) < 2 or rest[0] not in (ord('P'), ord('H'), ord('W')) or rest[1] != ord('|'):
        app = application_as_text(raw)
        return None, app if app is not None else data
    timing_mode = chr(rest[0])
    rest = rest[2:]

    holdover_age_s = 0
    if timing_mode == 'H':
        k = rest.find(b'|')
        if k < 0:
            app = application_as_text(raw)
            return None, app if app is not None else data
        try:
            holdover_age_s = int(rest[:k], 16)
        except ValueError:
            app = application_as_text(raw)
            return None, app if app is not None else data
        application = rest[k + 1:]
    else:
        application = rest

    if not application:
        app = application_as_text(raw)
        return None, app if app is not None else data

    meta = {
        'tx_ss': tx_ss,
        'tx_fraction_us': 0,
        'tx_time_us': tx_ss * 1_000_000,
        'sequence': sequence,
        'timing_mode': timing_mode,
        'holdover_age_s': holdover_age_s,
    }
    text = application_as_text(application)
    return meta, text if text is not None else application


def build_telemetry_command(payload, prefix="$K"):
    """Build the ``$K<payload>`` telemetry-update command for the transmitter.

    ``$K`` is used (not ``$T``) because ``$T`` is an existing Succorfish modem
    command; the host->Teensy prefix must not collide with it. The Teensy
    stores ``payload`` with a leading ``TEL:`` marker and broadcasts it as
    ``$Bnn TEL:<payload>`` on its PPS schedule, so receivers see
    ``#B<id><nn>TEL:<payload>``.

    ``payload`` is opaque, host-encoded telemetry (see
    ``owtt_beacon.beacon_telemetry``); it must NOT include the ``TEL:`` marker
    (the firmware adds it) and must be free of CR/LF.
    """
    return f"{prefix}{payload}"


def build_broadcast_command(data, prefix="$B"):
    """Build a one-shot Succorfish broadcast command ``$B<nn><data>``.

    ``nn`` is the 2-digit byte length of ``data``. In transmitter mode the
    Teensy passes such a command straight to the modem (it is not a ``$Y``/``$G``
    /``$K`` command), so it is broadcast immediately rather than on the PPS
    schedule -- handy for short control acknowledgements (e.g. ``OK``).

    ``data`` may be ``str`` or ``bytes``. Returns ``bytes`` when ``data`` is
    binary so the caller can ``write_bytes``; ASCII still returns ``str``.
    """
    raw = _as_bytes(data)
    cmd = prefix.encode('ascii') + f'{len(raw):02d}'.encode('ascii') + raw
    if isinstance(data, str) and application_as_text(raw) is not None:
        return cmd.decode('ascii')
    return cmd


def parse_broadcast_payload(line, prefix="#B"):
    """Parse a Succorfish broadcast frame into ``(modem_id, application_data)``.

    Format: ``#B<modem_id(3)><num_chars(2)><data>``. ``data`` may be a stick-v1
    timestamp envelope; this returns the *application* payload after unwrapping
    (legacy ``lat,lon``, ``TEL:…``, control words, or a binary blob).

    ``line`` may be ``str`` or ``bytes``. Application is ``str`` when ASCII,
    ``bytes`` when not. Returns ``None`` if the frame is not a parseable broadcast.
    """
    raw = _as_bytes(line)
    pfx = prefix.encode('ascii') if isinstance(prefix, str) else prefix
    if not raw.startswith(pfx):
        return None
    if pfx == b'#B':
        if len(raw) < 7:
            return None
        try:
            modem_id = raw[2:5].decode('ascii')
            num_chars = int(raw[5:7])
        except (ValueError, UnicodeDecodeError):
            return None
        data = raw[7:7 + num_chars]
        if len(data) != num_chars:
            return None
        _, application = unwrap_timestamp_envelope(data)
        return modem_id, application
    if pfx == b'#U':
        if len(raw) < 4:
            return None
        try:
            num_chars = int(raw[2:4])
        except ValueError:
            return None
        data = raw[4:4 + num_chars]
        if len(data) != num_chars:
            return None
        _, application = unwrap_timestamp_envelope(data)
        return '000', application
    return None


def parse_owtt_delta(line, prefix="#I"):
    """Return the raw OWTT delta (microseconds) from a Teensy report line.

    Returns ``None`` if the line is not an OWTT-delta line.

    Stick-v1 emits ``#I`` as the absolute TX→RX one-way travel time (can
    exceed 1 s for long ranges). The local PPS residual, when needed, is the
    ``#J`` second field or the ``#OWTT`` ``delta_us`` column.
    """
    if isinstance(line, (bytes, bytearray)):
        try:
            line = bytes(line).decode('ascii')
        except UnicodeDecodeError:
            return None
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
    """Parse a GPS lat/lon Succorfish broadcast relayed by the Teensy.

    Format: ``#B<modem_id(3)><num_chars(2)><data>`` where application ``data``
    is ``lat,lon`` (optionally inside a stick-v1 timestamp envelope).

    Returns ``(modem_id, lat, lon)`` or ``None`` if the line is not a parseable
    GPS broadcast (including telemetry ``TEL:`` frames).
    """
    parsed = parse_broadcast_payload(line, prefix=prefix)
    if parsed is None:
        return None
    modem_id, data = parsed
    text = application_as_text(data)
    if text is None or text.startswith(TELEMETRY_MARKER):
        return None
    parts = text.split(',')
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
