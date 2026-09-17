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
    GPS:      ``$G<lat>,<lon>``  (transmitter mode only; ASCII store, not forwarded)
    Telemetry: ``$K<nn><payload>`` (opaque host bytes; Teensy stores as-is)
    Broadcast (Teensy -> modem, automatic): ``$Bnn…`` (envelope + application)
    Modem broadcast relayed to host: ``#B<id(3)><nn(2)><data>``
    OWTT delta (after a timed modem line): ``#I<delta_us>``

Stick GPS on air is host-packed DCCL ``Pose`` (id 125) sent as ``$K``. The
Teensy does not encode or prefix ``TEL:``; it shunts those bytes and does
timing. ``$G`` remains a valid ASCII store command but the OWTT leader does
not use it. RX may still strip a leftover ``TEL:`` from mixed-flash boxes.
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

    Lat/lon are formatted with ``ascii_on_air.LATLON_DECIMALS``. The Teensy stores the
    substring after ``$G`` and puts it on air unchanged.
    """
    from serial_ping_pkg.common.ascii_on_air import format_latlon
    return f'{prefix}{format_latlon(lat, lon)}'


# Leftover ASCII type tag. Current firmware does not prefix ``TEL:`` on ``$K``.
# RX still strips it when present so one box can be flashed before the other,
# and unwrap treats a bare ``TEL:…`` as application (not a ``T<ss>|`` envelope).
TELEMETRY_MARKER = "TEL:"

# stick-v1 / LOLO_ver3 on-air envelope (before the application payload):
#   T<ss>|<seq:hex>|<P|H|W>[|<holdover_age_s:hex>]|<application>
# ss is the TX UTC time-of-minute (seconds, 0-59); broadcasts are PPS-aligned
# so the sub-second fraction is always ~0 and is omitted. Full unix seconds are
# redundant because propagation is only a few seconds; the Teensy resolves the
# minute against its own UTC. holdover is emitted only when non-zero (TX
# free-running); locked transmitters omit the field. Application (``lat,lon``
# or opaque DCCL) follows. ROS ranging uses Teensy ``#I`` (absolute OWTT), so
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
    bare ``lat,lon``, leftover ``TEL:…``, ``START``, …), ``meta`` is ``None``
    and ``application`` is ``data`` unchanged.

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
    """Build ``$K<nn><payload>`` for the transmitter.

    ``$K`` is used (not ``$T``) because ``$T`` is a Succorfish modem command.
    ``nn`` is the two-digit byte length of ``payload`` (same rule as ``$B``),
    so a DCCL blob may contain CR/LF/NUL. The Teensy stores the bytes as-is
    and broadcasts them on the PPS schedule (envelope + application).

    ``payload`` is the application (DCCL id byte, or ASCII ``lat,lon``). Do
    not prepend ``TEL:``. ``bytes`` return ``bytes`` (caller uses
    ``write_bytes``); an all-ASCII ``str`` still returns ``str`` for the old
    newline path, but prefer ``bytes``.
    """
    raw = _as_bytes(payload)
    cmd = prefix.encode('ascii') + f'{len(raw):02d}'.encode('ascii') + raw
    if isinstance(payload, str) and application_as_text(raw) is not None:
        return cmd.decode('ascii')
    return cmd


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
    (legacy ``lat,lon``, leftover ``TEL:…``, control words, or a binary blob).

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
    ASCII GPS broadcast (including leftover ``TEL:`` frames). DCCL Pose is
    handled by ``parse_broadcast_position``.
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


_POSE_DECODE_ERROR = None


def _pose_from_application(data):
    """Decode a DCCL ``Pose`` from application bytes (strip leftover ``TEL:``)."""
    global _POSE_DECODE_ERROR
    _POSE_DECODE_ERROR = None
    raw = _as_bytes(data)
    marker = TELEMETRY_MARKER.encode('ascii')
    if raw.startswith(marker):
        raw = raw[len(marker):]
    if len(raw) < 2:
        _POSE_DECODE_ERROR = 'short application'
        return None
    try:
        from serial_ping_pkg.common.dccl_codec import unpack
        msg = unpack(raw)
    except Exception as exc:
        _POSE_DECODE_ERROR = f'{type(exc).__name__}: {exc}'
        return None
    # libdccl may return a dynamically loaded class, not dccl_acoustic_pb2.Pose.
    name = getattr(getattr(msg, 'DESCRIPTOR', None), 'name', None)
    if name != 'Pose':
        _POSE_DECODE_ERROR = f'not Pose ({name!r})'
        return None
    return float(msg.lat), float(msg.lon)


def parse_broadcast_position(line, prefix="#B"):
    """Leader position from a relayed ``#B``: ASCII ``lat,lon`` or DCCL Pose.

    Stick GPS on air is packed ``Pose`` after the envelope. Legacy ASCII
    ``lat,lon`` still decodes. Beacon ``BeaconTelemetry`` and leftover
    ``TEL:`` blobs that are not Pose return ``None`` here (surface unit owns
    those).
    """
    ascii_gps = parse_broadcast(line, prefix=prefix)
    if ascii_gps is not None:
        return ascii_gps
    parsed = parse_broadcast_payload(line, prefix=prefix)
    if parsed is None:
        return None
    modem_id, data = parsed
    pose = _pose_from_application(data)
    if pose is None:
        return None
    lat, lon = pose
    return modem_id, lat, lon


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
