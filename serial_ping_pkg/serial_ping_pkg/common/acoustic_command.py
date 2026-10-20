"""DCCL ``Command`` (id 126) plus the accept rules for it.

The on-air frame is ``pack_command`` bytes inside a one-shot ``$B`` broadcast.
CRC-8 lives in ``dccl_codec``; a frame that fails it never becomes a dict.

``code`` values:

* ``1`` ABORT — act only when ``dest`` is this modem and ``confirm`` is 0
* ``2`` ABORT_ALL — panic, only when ``confirm`` is ``0xA11``. The
  sender does not abort; it is the control center.
* ``3`` START_TELEMETRY — beacon begins telemetry; ``dest`` is the beacon
* ``4`` STOP_TELEMETRY — beacon stops telemetry; ``dest`` is the beacon
* ``5`` OK — ack of a targeted abort, enough, or telemetry start/stop.
  Panic does not get an ack. ``dest`` is whoever sent the command.
* ``6`` ENOUGH — act only when ``dest`` is this modem and ``confirm`` is 0.
  Clears the current WARAPS mission. It does not raise the emergency flag.

Modem ``000`` is not a wildcard. A missing or zero target is not a panic.
"""

from serial_ping_pkg.tuper_owtt import teensy_interface as ti

CODE_NONE = 0
CODE_ABORT = 1
CODE_ABORT_ALL = 2
CODE_START_TELEMETRY = 3
CODE_STOP_TELEMETRY = 4
CODE_OK = 5
CODE_ENOUGH = 6

CONFIRM_ALL = 0xA11

CODE_NAMES = {
    CODE_ABORT: 'ABORT',
    CODE_ABORT_ALL: 'ABORT_ALL',
    CODE_START_TELEMETRY: 'START_TELEMETRY',
    CODE_STOP_TELEMETRY: 'STOP_TELEMETRY',
    CODE_OK: 'OK',
    CODE_ENOUGH: 'ENOUGH',
}


def modem_int(value):
    """Modem address as 1–255, or ``None`` when it is missing, 0, or not an id."""
    if value is None or isinstance(value, bool):
        return None
    mid = ti.normalize_modem_id(value)
    if len(mid) != 3 or not mid.isdigit():
        return None
    number = int(mid)
    if number < 1 or number > 255:
        return None
    return number


COMMAND_WORDS = {
    'abort': CODE_ABORT,
    'abort_all': CODE_ABORT_ALL,
    'start_telemetry': CODE_START_TELEMETRY,
    'stop_telemetry': CODE_STOP_TELEMETRY,
    'enough': CODE_ENOUGH,
}


def parse_command(text):
    """Service word to a command code, or ``None`` when it is not one."""
    key = '' if text is None else str(text).strip().lower().replace('-', '_')
    if key in ('all', 'panic'):
        key = 'abort_all'
    return COMMAND_WORDS.get(key)


def parse_target(text):
    """``'all'`` or one modem id.

    Returns ``('all', None)`` or ``('one', n)``. ``None`` means "do not
    transmit": empty, ``0``, ``000``, and anything that is not an id.
    """
    s = '' if text is None else str(text).strip()
    if s.lower() == 'all':
        return ('all', None)
    return ('one', modem_int(s)) if modem_int(s) is not None else None


def pack_command(code, dest, sender, seq, confirm=0):
    """Encode a ``Command`` plus CRC-8. Raises ``ValueError`` on a bad field."""
    from serial_ping_pkg.common.dccl_codec import _ensure_dccl, pack
    _ensure_dccl()
    from serial_ping_pkg.common import dccl_acoustic_pb2 as pb

    code = int(code)
    dest = int(dest)
    sender = int(sender)
    seq = int(seq)
    confirm = int(confirm)
    if code not in CODE_NAMES:
        raise ValueError(f'unknown command code {code}')
    for name, value, limit in (
            ('dest', dest, 255), ('sender', sender, 255), ('seq', seq, 65535),
            ('confirm', confirm, 65535)):
        if value < 0 or value > limit:
            raise ValueError(f'{name} {value} out of range')
    msg = pb.Command(
        code=code, dest=dest, sender=sender, seq=seq, confirm=confirm)
    return pack(msg)


def unpack_command(blob):
    """Return a command dict, or ``None`` when this is not a valid ``Command``."""
    if isinstance(blob, str):
        try:
            raw = blob.encode('latin-1')
        except UnicodeEncodeError:
            return None
    elif isinstance(blob, (bytes, bytearray, memoryview)):
        raw = bytes(blob)
    else:
        return None
    try:
        from serial_ping_pkg.common.dccl_codec import _ensure_dccl, unpack
        _ensure_dccl()
        msg = unpack(raw)
    except Exception:
        return None
    name = getattr(getattr(msg, 'DESCRIPTOR', None), 'name', None)
    if name != 'Command':
        return None
    try:
        return {
            'code': int(msg.code),
            'dest': int(msg.dest),
            'sender': int(msg.sender),
            'seq': int(msg.seq),
            'confirm': int(msg.confirm),
        }
    except (AttributeError, TypeError, ValueError):
        return None


def sender_allowed(sender, allow):
    """Empty allow-list accepts anyone. Otherwise ``sender`` must be listed."""
    if not allow:
        return True
    allowed = set()
    for item in allow:
        number = modem_int(item)
        if number is not None:
            allowed.add(number)
    return int(sender) in allowed


def ack_matches(cmd, envelope_id, seq, own_id, expect_sender=None):
    """True when this OK closes a targeted command we sent.

    ``seq`` must match, ``dest`` must be us, and the payload sender must be
    the modem that transmitted the frame. ``expect_sender`` limits whose ack
    counts (the modem we told to abort).
    """
    own = modem_int(own_id)
    if not is_beacon_ok(cmd, envelope_id) or own is None:
        return False
    if cmd['dest'] != own or int(cmd['seq']) != int(seq):
        return False
    if expect_sender is not None and cmd['sender'] != int(expect_sender):
        return False
    return True


def enough_applies(cmd, own_id):
    """True when this vehicle should enough its current WARAPS mission."""
    own = modem_int(own_id)
    if own is None or cmd is None:
        return False
    return (cmd['code'] == CODE_ENOUGH and cmd['confirm'] == 0
            and cmd['dest'] == own)


def enough_tst_body(unit_name):
    """JSON object the MQTT bridge would deliver for a tst ``$enough``.

    Same shape as the QGIS ``signal-unit`` on ``tst/command``. No tst uuid:
    wasp enoughs every task this vehicle is running and leaves the emergency
    flag down.
    """
    name = '' if unit_name is None else str(unit_name).strip().strip('/')
    if not name:
        name = 'vehicle'
    return {
        'receiver': name,
        'signal': '$enough',
        'unit': f'/{name}',
        'command': 'signal-unit',
        'sender': 'acoustic',
    }


def abort_applies(cmd, own_id):
    """True when this vehicle should publish the SMARC abort for ``cmd``.

    ``ABORT_ALL`` stops every other modem. The sender is the control center
    and stays up, even if this process is the one that transmitted the frame.
    """
    own = modem_int(own_id)
    if own is None or cmd is None:
        return False
    if cmd['code'] == CODE_ABORT and cmd['confirm'] == 0 and cmd['dest'] == own:
        return True
    if (cmd['code'] == CODE_ABORT_ALL and cmd['confirm'] == CONFIRM_ALL
            and cmd['sender'] != own):
        return True
    return False


def beacon_verb(cmd, own_id, envelope_id):
    """``'START_TELEMETRY'`` or ``'STOP_TELEMETRY'`` when this beacon should act.

    ``dest`` must be this beacon, ``sender`` must be the modem that actually
    transmitted the frame, and ``confirm`` must be 0. Anything else is ``None``.
    """
    own = modem_int(own_id)
    envelope = modem_int(envelope_id)
    if cmd is None or own is None or envelope is None:
        return None
    if cmd['sender'] != envelope or cmd['dest'] != own or cmd['confirm'] != 0:
        return None
    if cmd['code'] == CODE_START_TELEMETRY:
        return 'START_TELEMETRY'
    if cmd['code'] == CODE_STOP_TELEMETRY:
        return 'STOP_TELEMETRY'
    return None


def is_beacon_ok(cmd, envelope_id):
    """True for an OK whose payload sender is the modem that sent the frame."""
    envelope = modem_int(envelope_id)
    if cmd is None or envelope is None:
        return False
    return (cmd['code'] == CODE_OK and cmd['confirm'] == 0
            and cmd['sender'] == envelope)


class SeqMemory:
    """Remember the last abort sequence acted on for each sender.

    A repeated ``seq`` from the same sender is a modem retry or a replay.
    Frames that do not apply to this vehicle are not recorded, so a command
    aimed at someone else does not burn the sequence.
    """

    def __init__(self):
        self._last = {}

    def already_acted(self, sender, seq):
        return self._last.get(int(sender)) == int(seq)

    def remember(self, sender, seq):
        self._last[int(sender)] = int(seq)
