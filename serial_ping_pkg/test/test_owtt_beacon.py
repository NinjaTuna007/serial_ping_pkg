"""Pure-logic tests for the owtt_beacon telemetry codec and inference geometry.

Two layers:

* ``beacon_telemetry``: default ``codec='dccl'`` (DCCL+CRC-8); ``codec='ascii'``
  keeps the tagged ``P…;D…`` form. DCCL tests need ``python3-dccl4`` (user
  prefix ``~/.local/opt/dccl4`` is fine).
* ``inference_node`` geometry helpers; import-guarded for ROS/MQTT.
"""

import math
import time

import pytest

from serial_ping_pkg.owtt_beacon.beacon_telemetry import (
    CODEC_ASCII,
    CODEC_DCCL,
    decode_telemetry,
    encode_telemetry,
    normalize_codec,
    sanitize,
    strip_marker,
)
from serial_ping_pkg.common.dccl_codec import _ensure_dccl, crc8_atm, pack, unpack

_ensure_dccl()
from serial_ping_pkg.common import dccl_acoustic_pb2 as acoustic_pb2

from full_stack_harness import (
    DRIVER_AVAILABLE,
    Stack,
    exe,
    teensy_responder,
    wait_until,
)


# --------------------------------------------------------------------------- #
# Telemetry codec                                                             #
# --------------------------------------------------------------------------- #

def test_sanitize_strips_delimiters():
    """Field/line delimiters are replaced with spaces and the result trimmed."""
    assert sanitize("a;b\nx") == "a b x"


def test_encode_decode_round_trip():
    """DCCL+CRC-8 round-trip recovers the numeric fields within proto grain."""
    payload = encode_telemetry(
        ['position', 'depth', 'svs', 'speed', 'bt'],
        position=(58.823229, 17.635998), depth=12.3, svs=1481.6, speed=1.2, bt='move_to')
    assert isinstance(payload, bytes)
    assert payload[-1] == crc8_atm(payload[:-1])
    out = decode_telemetry(payload)
    assert out['position'][0] == pytest.approx(58.823229, abs=5e-8)
    assert out['position'][1] == pytest.approx(17.635998, abs=5e-8)
    assert out['depth'] == pytest.approx(12.3, abs=5e-3)
    assert out['svs'] == pytest.approx(1481.6, abs=5e-2)
    assert out['speed'] == pytest.approx(1.2, abs=5e-3)
    assert out['bt'] == 'move_to'


def test_normalize_codec_default_and_aliases():
    """Empty/None is DCCL; ascii is kept; anything else is rejected."""
    assert normalize_codec(None) == CODEC_DCCL
    assert normalize_codec('DCCL') == CODEC_DCCL
    assert normalize_codec('ascii') == CODEC_ASCII
    with pytest.raises(ValueError):
        normalize_codec('both')


def test_dccl_smaller_than_legacy_ascii_pose():
    """Packed pose+svs+bt is denser than the old tagged ASCII string."""
    blob = encode_telemetry(
        ['position', 'depth', 'svs', 'speed', 'bt'],
        position=(58.823229, 17.635998), depth=12.3, svs=1481.6, speed=1.2, bt='move_to')
    ascii_form = 'P58.8232290,17.6359980;D12.30;C1481.6;S1.20;Bmove_to'
    assert len(blob) < len(ascii_form)


def test_crc8_mismatch_returns_none():
    """A flipped trailer byte is dropped, not decoded as a plausible lat."""
    blob = encode_telemetry(['bt'], bt='hi')
    bad = blob[:-1] + bytes([blob[-1] ^ 0xFF])
    assert decode_telemetry(bad) is None


def test_encode_only_enabled_fields():
    """Fields not in the enabled set are omitted even if a value is given."""
    payload = encode_telemetry(['bt'], position=(1.0, 2.0), bt='hi')
    out = decode_telemetry(payload)
    assert out == {'bt': 'hi'}


def test_strip_marker_present_and_absent():
    """strip_marker returns the post-marker payload, or None for plain data."""
    assert strip_marker("TEL:P1,2") == "P1,2"
    assert strip_marker("58.1,18.2") is None
    blob = encode_telemetry(['bt'], bt='hi')
    rest = strip_marker(b'TEL:' + blob)
    assert rest == blob


def test_strip_marker_after_envelope_unwrap():
    """Surface units see TEL: after parse_broadcast_payload unwraps the envelope."""
    from serial_ping_pkg.tuper_owtt import teensy_interface as ti
    app = 'TEL:P58.8,17.6'
    payload = f'T04|0001|P|{app}'
    frame = f'#B101{len(payload):02d}{payload}'
    modem_id, data = ti.parse_broadcast_payload(frame)
    assert modem_id == '101'
    assert strip_marker(data) == 'P58.8,17.6'


def test_payload_budget_drops_bt_keeps_position():
    """When over budget the free-text bt is sacrificed; position is preserved."""
    payload = encode_telemetry(
        ['position', 'bt'], position=(1.0, 2.0), bt='X' * 16, max_payload_len=20)
    out = decode_telemetry(payload)
    assert out is not None
    assert 'position' in out
    assert 'bt' not in out


def test_ascii_codec_round_trip():
    """codec='ascii' is the tagged P…;D… form (not DCCL)."""
    payload = encode_telemetry(
        ['position', 'depth', 'svs', 'speed', 'bt'],
        position=(58.823229, 17.635998), depth=12.3, svs=1481.6, speed=1.2,
        bt='move_to', codec=CODEC_ASCII)
    assert payload == 'P58.8232290,17.6359980;D12.30;C1481.6;S1.20;Bmove_to'
    out = decode_telemetry(payload, codec=CODEC_ASCII)
    assert out['position'][0] == pytest.approx(58.823229)
    assert out['bt'] == 'move_to'


def test_dccl_mode_rejects_ascii():
    """Default DCCL decode does not silently parse tagged ASCII."""
    ascii_bag = 'P58.8232290,17.6359980;D12.30;C1481.6;S1.20;Bmove_to'
    assert decode_telemetry(ascii_bag) is None
    assert decode_telemetry(ascii_bag, codec=CODEC_DCCL) is None


def test_ascii_decode_unknown_tag_captured():
    """Unrecognised ASCII tags land under 'unknown' rather than being dropped."""
    out = decode_telemetry("Z123", codec=CODEC_ASCII)
    assert out['unknown'] == {'Z': '123'}


def test_ascii_decode_skips_malformed_field():
    """A malformed ASCII field is skipped without breaking the rest."""
    out = decode_telemetry("Pbad;D5.0", codec=CODEC_ASCII)
    assert 'position' not in out
    assert out['depth'] == pytest.approx(5.0)


def test_pose_message_round_trip():
    """Pose (id 125) packs independently of beacon telemetry."""
    msg = acoustic_pb2.Pose(lat=59.0, lon=18.0, depth=3.25)
    blob = pack(msg)
    got = unpack(blob)
    assert isinstance(got, acoustic_pb2.Pose)
    assert got.lat == pytest.approx(59.0, abs=5e-8)
    assert got.lon == pytest.approx(18.0, abs=5e-8)
    assert got.depth == pytest.approx(3.25, abs=5e-3)


def test_dccl_depth_to_3km():
    """DCCL depth max is 3000 m; 2.5 km round-trips at 1 cm grain."""
    payload = encode_telemetry(['depth'], depth=2500.0)
    out = decode_telemetry(payload)
    assert out['depth'] == pytest.approx(2500.0, abs=5e-3)
    payload = encode_telemetry(['depth'], depth=3000.0)
    out = decode_telemetry(payload)
    assert out['depth'] == pytest.approx(3000.0, abs=5e-3)


# --------------------------------------------------------------------------- #
# Inference geometry (import-guarded)                                         #
# --------------------------------------------------------------------------- #

# Importing inference_node pulls ROS/MQTT deps; if they are missing, skip just
# these tests (not the pure telemetry tests above).
try:
    from serial_ping_pkg.owtt_beacon import inference_node as inference
    _HAS_INFERENCE = True
except Exception:  # pragma: no cover - depends on optional runtime deps
    inference = None
    _HAS_INFERENCE = False

requires_inference = pytest.mark.skipif(
    not _HAS_INFERENCE, reason="inference_node import failed (optional ROS/MQTT deps)")


@requires_inference
def test_enu_round_trip():
    """Geodetic -> ENU -> geodetic recovers the original lat/lon."""
    lat0, lon0 = 58.8, 17.6
    lat, lon = 58.81, 17.61
    dE, dN = inference.geodetic_to_enu(lat, lon, lat0, lon0)
    back_lat, back_lon = inference.enu_to_geodetic(dE, dN, lat0, lon0)
    assert back_lat == pytest.approx(lat, abs=1e-9)
    assert back_lon == pytest.approx(lon, abs=1e-9)


@requires_inference
def test_circle_intersections_two_points():
    """Two overlapping circles intersect in two mirrored points."""
    pts = inference.circle_intersections((0.0, 0.0), 5.0, (8.0, 0.0), 5.0)
    xs = sorted((round(x, 6), round(y, 6)) for x, y in pts)
    assert xs == [(4.0, -3.0), (4.0, 3.0)]


@requires_inference
def test_circle_intersections_non_intersecting_fallback():
    """Disjoint circles fall back to the radical foot, duplicated."""
    pts = inference.circle_intersections((0.0, 0.0), 1.0, (10.0, 0.0), 1.0)
    assert pts == [(5.0, 0.0), (5.0, 0.0)]


@requires_inference
def test_circle_intersections_concentric_empty():
    """Concentric circles return no intersection points."""
    assert inference.circle_intersections((0.0, 0.0), 5.0, (0.0, 0.0), 3.0) == []


@requires_inference
def test_multilaterate_recovers_known_point():
    """Noise-free ranges to 3 anchors recover the true point."""
    anchors = [(0.0, 0.0), (10.0, 0.0), (0.0, 10.0)]
    true = (3.0, 4.0)
    ranges = [math.hypot(a[0] - true[0], a[1] - true[1]) for a in anchors]
    x, y = inference.multilaterate(anchors, ranges, init=(0.0, 0.0))
    assert x == pytest.approx(3.0, abs=1e-3)
    assert y == pytest.approx(4.0, abs=1e-3)


@requires_inference
def test_safe_token_sanitises_names():
    """Non-token characters become underscores; leading digit is prefixed."""
    assert inference.InferenceNode._safe_token('buoy-1') == 'buoy_1'
    assert inference.InferenceNode._safe_token('1buoy') == '_1buoy'


@requires_inference
def test_dist_m_zero_and_positive():
    """Geodesic distance is zero for identical points, positive otherwise."""
    a = (58.8, 17.6)
    assert inference.InferenceNode._dist_m(a, a) == pytest.approx(0.0)
    assert inference.InferenceNode._dist_m(a, (58.81, 17.6)) > 0.0


@requires_inference
def test_meas_time_fallback_order():
    """meas_time wins; else recv_time; else 0.0."""
    assert inference.InferenceNode._meas_time({'meas_time': 100.0}) == 100.0
    assert inference.InferenceNode._meas_time({'recv_time': 5.0}) == 5.0
    assert inference.InferenceNode._meas_time({}) == 0.0


# --------------------------------------------------------------------------- #
# Full-stack pty integration (Teensy OWTT profile)                           #
#                                                                             #
# fake Teensy <--pty--> succorfish_driver <--ROS--> beacon/surface node.     #
# The fake Teensy ACKs $Y config with #A<id> + #Y,OK; tests then drive START/OK #
# command channel and the telemetry+#I range path.                           #
# --------------------------------------------------------------------------- #

_skip = pytest.mark.skipif(
    not DRIVER_AVAILABLE, reason="succorfish_driver not built/sourced")


@_skip
@pytest.mark.skipif(exe('owtt_beacon_node') is None, reason="node not built")
def test_owtt_beacon_configures_and_acks_start():
    """Beacon: transmitter config on startup, then a START broadcast -> $B OK ack."""
    st = Stack(responder=teensy_responder, profile='teensy').start_driver()
    st.start_node(
        exe('owtt_beacon_node'),
        ['-p', 'teensy.mode:=transmitter',
         '-p', 'teensy.own_modem_id:=101',
         '-p', 'teensy.listen_for_modem_id:=000',
         '-p', 'teensy.broadcast_interval_s:=1'])
    # own_modem_id 101, listen 000, interval 1 -> transmitter config.
    assert st.fake.wait_for_command('$Y101T0001s', timeout=8), \
        f"no transmitter config; got {st.fake.commands()!r}"
    # A commander (modem 007) broadcasts START; the beacon must ack with $B02OK.
    st.fake.inject('#B00705START')
    saw_ack = st.fake.wait_for_command('$B02OK', timeout=8)
    node_out, _ = st.stop()
    assert saw_ack, f"beacon did not ack START with $B02OK; got {st.fake.commands()!r}\n{node_out}"


@_skip
@pytest.mark.skipif(exe('owtt_surface_unit_node') is None, reason="node not built")
def test_owtt_surface_unit_publishes_range():
    """Surface unit: receiver config, then TEL telemetry + #I delta -> a ROS range."""
    from std_msgs.msg import Float32, String

    st = Stack(responder=teensy_responder, profile='teensy').start_driver()
    st.start_probe()
    ranges = st.probe.collect(Float32, '/surface_unit/owtt_beacon/lolo/range')
    telems = st.probe.collect(String, '/surface_unit/owtt_beacon/lolo/telemetry')
    st.start_node(
        exe('owtt_surface_unit_node'),
        ['-p', 'mqtt.enabled:=false',
         '-p', 'owtt.auto_calibrate:=false',
         '-p', 'owtt.offset_us:=771600.0'])
    assert st.fake.wait_for_command('$Y067R', timeout=8), \
        f"no receiver config; got {st.fake.commands()!r}"
    time.sleep(0.5)
    blob = encode_telemetry(['svs'], svs=1500.0)     # default DCCL+CRC-8
    app = b'TEL:' + blob
    st.fake.inject(b'#B101' + f'{len(app):02d}'.encode('ascii') + app)
    got_tel = wait_until(lambda: len(telems) > 0, timeout=8)
    st.fake.inject('#I871600')                       # (871600-771600)*1e-6*1500 = 150 m
    ok = wait_until(lambda: len(ranges) > 0, timeout=8)
    node_out, _ = st.stop()
    assert got_tel, f"surface unit did not decode DCCL telemetry; out:\n{node_out}"
    assert ok, f"surface unit did not publish a range; out:\n{node_out}"
    assert ranges[0].data == pytest.approx(150.0, rel=1e-2)


@_skip
@pytest.mark.skipif(exe('owtt_surface_unit_node') is None, reason="node not built")
def test_owtt_surface_unit_publishes_range_ascii_codec():
    """codec:=ascii still accepts tagged TEL:C1500.0 bags."""
    from std_msgs.msg import Float32

    st = Stack(responder=teensy_responder, profile='teensy').start_driver()
    st.start_probe()
    ranges = st.probe.collect(Float32, '/surface_unit/owtt_beacon/lolo/range')
    st.start_node(
        exe('owtt_surface_unit_node'),
        ['-p', 'mqtt.enabled:=false',
         '-p', 'beacon.codec:=ascii',
         '-p', 'owtt.auto_calibrate:=false',
         '-p', 'owtt.offset_us:=771600.0'])
    assert st.fake.wait_for_command('$Y067R', timeout=8), \
        f"no receiver config; got {st.fake.commands()!r}"
    time.sleep(0.5)
    payload = 'TEL:C1500.0'
    st.fake.inject(f'#B101{len(payload):02d}{payload}')
    st.fake.inject('#I871600')
    ok = wait_until(lambda: len(ranges) > 0, timeout=8)
    node_out, _ = st.stop()
    assert ok, f"ascii-mode surface unit did not publish a range; out:\n{node_out}"
    assert ranges[0].data == pytest.approx(150.0, rel=1e-2)
