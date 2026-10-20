"""Acoustic Command (DCCL id 126): accept rules and the abort node on a pty."""

import threading
import time

import pytest

from serial_ping_pkg.common.acoustic_command import (
    CODE_ABORT,
    CODE_ABORT_ALL,
    CODE_OK,
    CODE_START_TELEMETRY,
    CODE_STOP_TELEMETRY,
    CONFIRM_ALL,
    abort_applies,
    ack_matches,
    beacon_verb,
    pack_command,
    parse_command,
    parse_target,
    unpack_command,
)
from serial_ping_pkg.common.dccl_codec import _ensure_dccl, crc8_atm, pack_pose

_ensure_dccl()

from full_stack_harness import (  # noqa: E402
    DRIVER_AVAILABLE,
    Stack,
    exe,
    teensy_responder,
    wait_until,
)


def _frame(modem, payload):
    mid = f'{int(modem):03d}'.encode('ascii')
    return b'#B' + mid + f'{len(payload):02d}'.encode('ascii') + payload


def _dollar_b(payload):
    return b'$B' + f'{len(payload):02d}'.encode('ascii') + payload


def _payloads(raw):
    """Application bytes of every complete ``$Bnn`` in ``raw``."""
    found = []
    index = 0
    while True:
        at = raw.find(b'$B', index)
        if at < 0 or at + 4 > len(raw):
            break
        size_text = raw[at + 2:at + 4]
        if not size_text.isdigit():
            index = at + 2
            continue
        size = int(size_text)
        blob = raw[at + 4:at + 4 + size]
        if len(blob) < size:
            break
        found.append(blob)
        index = at + 4 + size
    return found


def test_round_trip_and_bad_crc():
    blob = pack_command(CODE_ABORT, dest=7, sender=67, seq=4)
    assert unpack_command(blob) == {
        'code': CODE_ABORT, 'dest': 7, 'sender': 67, 'seq': 4, 'confirm': 0}
    assert blob[-1] == crc8_atm(blob[:-1])
    assert unpack_command(blob[:-1] + bytes([blob[-1] ^ 0xFF])) is None
    assert unpack_command(pack_pose(59.0, 18.0)) is None
    assert unpack_command(b'START') is None


def test_parse_target_rejects_empty_and_zero():
    assert parse_target('all') == ('all', None)
    assert parse_target('ALL') == ('all', None)
    assert parse_target('007') == ('one', 7)
    assert parse_target('7') == ('one', 7)
    assert parse_target('') is None
    assert parse_target('0') is None
    assert parse_target('000') is None
    assert parse_target('256') is None


def test_stop_telemetry_round_trip_and_addressing():
    blob = pack_command(CODE_STOP_TELEMETRY, dest=101, sender=7, seq=8)
    cmd = unpack_command(blob)
    assert cmd == {
        'code': CODE_STOP_TELEMETRY, 'dest': 101, 'sender': 7,
        'seq': 8, 'confirm': 0,
    }
    assert beacon_verb(cmd, 101, 7) == 'STOP_TELEMETRY'
    assert beacon_verb(cmd, 5, 7) is None
    assert beacon_verb(dict(cmd, confirm=CONFIRM_ALL), 101, 7) is None
    start = dict(cmd, code=CODE_START_TELEMETRY)
    assert beacon_verb(start, 101, 7) == 'START_TELEMETRY'
    assert parse_command('stop_telemetry') == CODE_STOP_TELEMETRY
    assert parse_command('STOP-TELEMETRY') == CODE_STOP_TELEMETRY
    assert parse_command('start_telemetry') == CODE_START_TELEMETRY
    assert parse_command('abort_all') == CODE_ABORT_ALL
    assert parse_command('nope') is None


def test_abort_rules():
    own = 7
    targeted = {'code': CODE_ABORT, 'dest': 7, 'sender': 67, 'seq': 1, 'confirm': 0}
    other = dict(targeted, dest=5)
    panic = {
        'code': CODE_ABORT_ALL, 'dest': 67, 'sender': 67, 'seq': 2,
        'confirm': CONFIRM_ALL,
    }
    assert abort_applies(targeted, own)
    assert not abort_applies(other, own)
    assert abort_applies(panic, own)
    assert not abort_applies(dict(panic, sender=own), own)
    assert not abort_applies(dict(panic, confirm=0), own)
    assert not abort_applies(dict(targeted, confirm=CONFIRM_ALL), own)


def test_ack_matches_seq_and_sender():
    ok = {'code': CODE_OK, 'dest': 67, 'sender': 7, 'seq': 3, 'confirm': 0}
    assert ack_matches(ok, envelope_id=7, seq=3, own_id=67, expect_sender=7)
    assert not ack_matches(ok, envelope_id=7, seq=4, own_id=67, expect_sender=7)
    assert not ack_matches(ok, envelope_id=7, seq=3, own_id=67, expect_sender=5)
    assert not ack_matches(ok, envelope_id=9, seq=3, own_id=67, expect_sender=7)


_skip = pytest.mark.skipif(
    not DRIVER_AVAILABLE, reason="succorfish_driver not built/sourced")


def _start_cmd(**params):
    args = [
        '-p', 'own_modem_id:=007',
        '-p', 'abort_topic:=smarc/abort',
        '-p', 'ack_timeout_s:=0.6',
        '-p', 'max_attempts:=3',
        '-p', 'panic_shouts:=3',
        '-p', 'panic_gap_s:=0.2',
    ]
    for key, value in params.items():
        args.extend(['-p', f'{key}:={value}'])
    st = Stack(responder=teensy_responder, profile='teensy').start_driver()
    st.start_probe()
    st.start_node(exe('acoustic_cmd_node'), args)
    return st


def _commands(st):
    return [unpack_command(blob) for blob in _payloads(st.fake.raw_bytes())]


@_skip
@pytest.mark.skipif(exe('acoustic_cmd_node') is None, reason="node not built")
def test_targeted_abort_acks_and_publishes():
    """An abort for this modem publishes Empty and answers with OK."""
    from std_msgs.msg import Empty

    st = _start_cmd()
    aborts = st.probe.collect(Empty, '/smarc/abort')
    time.sleep(0.5)
    blob = pack_command(CODE_ABORT, dest=7, sender=67, seq=9)
    ok = pack_command(CODE_OK, dest=67, sender=7, seq=9)
    # The first frames can be published before the rx subscription matches.
    deadline = time.time() + 8
    while time.time() < deadline and not (
            _dollar_b(ok) in st.fake.raw_bytes() and len(aborts) >= 1):
        st.fake.inject(_frame(67, blob))
        time.sleep(0.3)
    saw = _dollar_b(ok) in st.fake.raw_bytes()
    got_abort = len(aborts) >= 1
    # A repeat of the same seq acks again and does not publish a second abort.
    st.fake.inject(_frame(67, blob))
    wait_until(
        lambda: st.fake.raw_bytes().count(_dollar_b(ok)) >= 2, timeout=4)
    # Someone else's abort is ignored.
    other = pack_command(CODE_ABORT, dest=5, sender=67, seq=10)
    st.fake.inject(_frame(67, other))
    time.sleep(0.4)
    node_out, _ = st.stop()
    assert saw and got_abort, node_out
    assert len(aborts) == 1


@_skip
@pytest.mark.skipif(exe('acoustic_cmd_node') is None, reason="node not built")
def test_panic_from_someone_else_aborts_without_ack():
    from std_msgs.msg import Empty

    st = _start_cmd()
    aborts = st.probe.collect(Empty, '/smarc/abort')
    time.sleep(0.5)
    blob = pack_command(
        CODE_ABORT_ALL, dest=67, sender=67, seq=2, confirm=CONFIRM_ALL)
    deadline = time.time() + 8
    while time.time() < deadline and len(aborts) < 1:
        st.fake.inject(_frame(67, blob))
        time.sleep(0.3)
    # Our own shout must not abort us.
    mine = pack_command(
        CODE_ABORT_ALL, dest=7, sender=7, seq=3, confirm=CONFIRM_ALL)
    st.fake.inject(_frame(7, mine))
    bad = pack_command(CODE_ABORT_ALL, dest=7, sender=67, seq=4, confirm=0)
    st.fake.inject(_frame(67, bad))
    got = wait_until(lambda: len(aborts) == 1, timeout=4)
    time.sleep(0.4)
    node_out, _ = st.stop()
    assert got, node_out
    assert len(aborts) == 1
    assert not any(
        cmd and cmd['code'] == CODE_OK for cmd in _commands(st))


@_skip
@pytest.mark.skipif(exe('acoustic_cmd_node') is None, reason="node not built")
def test_service_targeted_succeeds_on_ack_and_retries():
    from serial_ping_interfaces.srv import AcousticAbort

    st = _start_cmd()
    time.sleep(1.0)
    client = st.probe.node.create_client(AcousticAbort, '/acoustic_abort')
    assert wait_until(client.service_is_ready, timeout=8)

    def watch():
        answered = set()
        while not stop_watch.is_set():
            for blob in _payloads(st.fake.raw_bytes()):
                cmd = unpack_command(blob)
                if (cmd is None or cmd['code'] != CODE_ABORT
                        or cmd['seq'] in answered):
                    continue
                answered.add(cmd['seq'])
                if len(answered) == 1:
                    continue
                ok = pack_command(
                    CODE_OK, dest=cmd['sender'], sender=cmd['dest'], seq=cmd['seq'])
                st.fake.inject(_frame(cmd['dest'], ok))
                return
            time.sleep(0.05)

    stop_watch = threading.Event()
    thread = threading.Thread(target=watch, daemon=True)
    thread.start()
    request = AcousticAbort.Request()
    request.target = '005'
    future = client.call_async(request)
    assert wait_until(lambda: future.done(), timeout=8)
    stop_watch.set()
    thread.join(timeout=1)
    result = future.result()
    wire = _commands(st)
    node_out, _ = st.stop()
    assert result is not None and result.success, f'{result}\n{node_out}'
    aborts = [c for c in wire if c and c['code'] == CODE_ABORT]
    assert len(aborts) >= 2, wire
    assert aborts[0]['seq'] != aborts[1]['seq']
    assert 'ack from 005' in result.message


@_skip
@pytest.mark.skipif(exe('acoustic_cmd_node') is None, reason="node not built")
def test_service_gives_up_without_ack_and_panic_does_not_self_abort():
    from std_msgs.msg import Empty
    from serial_ping_interfaces.srv import AcousticAbort

    st = _start_cmd()
    aborts = st.probe.collect(Empty, '/smarc/abort')
    time.sleep(1.0)
    client = st.probe.node.create_client(AcousticAbort, '/acoustic_abort')
    assert wait_until(client.service_is_ready, timeout=8)

    missed = AcousticAbort.Request()
    missed.target = '005'
    fut = client.call_async(missed)
    assert wait_until(lambda: fut.done(), timeout=8)
    missed_result = fut.result()

    empty = AcousticAbort.Request()
    empty.target = ''
    fut = client.call_async(empty)
    assert wait_until(lambda: fut.done(), timeout=4)
    empty_result = fut.result()

    panic = AcousticAbort.Request()
    panic.target = 'all'
    fut = client.call_async(panic)
    assert wait_until(lambda: fut.done(), timeout=8)
    panic_result = fut.result()
    time.sleep(0.3)
    wire = _commands(st)
    node_out, _ = st.stop()

    assert missed_result is not None and not missed_result.success, node_out
    assert empty_result is not None and not empty_result.success
    assert panic_result is not None and panic_result.success, panic_result
    assert 'no ack expected' in panic_result.message
    shouts = [c for c in wire if c and c['code'] == CODE_ABORT_ALL]
    assert len(shouts) == 3, wire
    assert all(c['confirm'] == CONFIRM_ALL and c['sender'] == 7 for c in shouts)
    assert len({c['seq'] for c in shouts}) == 3
    assert len(aborts) == 0


@_skip
@pytest.mark.skipif(exe('acoustic_cmd_node') is None, reason="node not built")
def test_stop_telemetry_acks_and_service_waits():
    """STOP_TELEMETRY addressed here is acked; the service waits for that OK."""
    from std_msgs.msg import String
    from serial_ping_interfaces.srv import AcousticCommand

    st = _start_cmd(handle_telemetry='true')
    verbs = st.probe.collect(String, '/acoustic_telemetry')
    time.sleep(0.5)
    blob = pack_command(CODE_STOP_TELEMETRY, dest=7, sender=67, seq=11)
    ok = pack_command(CODE_OK, dest=67, sender=7, seq=11)
    deadline = time.time() + 8
    while time.time() < deadline and not (
            _dollar_b(ok) in st.fake.raw_bytes() and len(verbs) >= 1):
        st.fake.inject(_frame(67, blob))
        time.sleep(0.3)
    other = pack_command(CODE_STOP_TELEMETRY, dest=5, sender=67, seq=12)
    st.fake.inject(_frame(67, other))
    time.sleep(0.3)

    client = st.probe.node.create_client(AcousticCommand, '/acoustic_command')
    assert wait_until(client.service_is_ready, timeout=8)

    def watch():
        answered = set()
        while not stop_watch.is_set():
            for sent in _payloads(st.fake.raw_bytes()):
                cmd = unpack_command(sent)
                if (cmd is None or cmd['code'] != CODE_STOP_TELEMETRY
                        or cmd['seq'] in answered):
                    continue
                answered.add(cmd['seq'])
                reply = pack_command(
                    CODE_OK, dest=cmd['sender'], sender=cmd['dest'], seq=cmd['seq'])
                st.fake.inject(_frame(cmd['dest'], reply))
                return
            time.sleep(0.05)

    stop_watch = threading.Event()
    thread = threading.Thread(target=watch, daemon=True)
    thread.start()
    request = AcousticCommand.Request()
    request.command = 'stop_telemetry'
    request.target = '005'
    future = client.call_async(request)
    assert wait_until(lambda: future.done(), timeout=8)
    stop_watch.set()
    thread.join(timeout=1)
    result = future.result()
    node_out, _ = st.stop()
    assert _dollar_b(ok) in st.fake.raw_bytes(), node_out
    assert verbs and all(msg.data == 'STOP_TELEMETRY' for msg in verbs), verbs
    stray = pack_command(CODE_OK, dest=67, sender=7, seq=12)
    assert _dollar_b(stray) not in st.fake.raw_bytes()
    assert result is not None and result.success, f'{result}\n{node_out}'
    assert 'ack from 005' in result.message
    assert 'STOP_TELEMETRY' in result.message


@_skip
@pytest.mark.skipif(exe('acoustic_cmd_node') is None, reason="node not built")
def test_stop_telemetry_is_ignored_without_the_telemetry_flag():
    st = _start_cmd()
    time.sleep(0.5)
    blob = pack_command(CODE_STOP_TELEMETRY, dest=7, sender=67, seq=6)
    st.fake.inject(_frame(67, blob))
    time.sleep(0.6)
    node_out, _ = st.stop()
    assert not any(
        cmd and cmd['code'] == CODE_OK for cmd in _commands(st)), node_out
