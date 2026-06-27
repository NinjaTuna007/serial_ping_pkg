"""Pure-logic tests for the owtt_beacon telemetry codec and inference geometry.

Two layers:

* ``beacon_telemetry`` is fully pure (no ROS/MQTT/hardware) and tested directly:
  sanitisation, encode/decode round-trip, marker stripping, the on-air payload
  budget trimming, and tolerant decoding.
* ``inference_node`` carries pure geometry helpers (flat-earth ENU,
  circle intersections, multilateration) plus a few static sanitisers. Importing
  it pulls ROS/MQTT deps, so it is guarded with ``importorskip`` to keep the
  suite green where those are unavailable.

Mirrors the style of ``test_ping_estimator_action.py`` / ``test_owtt_protocol.py``.
"""

import math
import time

import pytest

from serial_ping_pkg.owtt_beacon.beacon_telemetry import (
    decode_telemetry,
    encode_telemetry,
    sanitize,
    strip_marker,
)

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


def test_encode_matches_documented_format():
    """The encoder lays fields out in tag order with ';' separators."""
    payload = encode_telemetry(
        ['position', 'depth', 'svs', 'speed', 'bt'],
        position=(58.823229, 17.635998), depth=12.3, svs=1481.6, speed=1.20, bt='move_to')
    assert payload == "P58.823229,17.635998;D12.3;C1481.6;S1.20;Bmove_to"


def test_encode_decode_round_trip():
    """Decoding an encoded payload recovers the numeric fields."""
    payload = encode_telemetry(
        ['position', 'depth', 'svs', 'speed', 'bt'],
        position=(58.823229, 17.635998), depth=12.3, svs=1481.6, speed=1.2, bt='move_to')
    out = decode_telemetry(payload)
    assert out['position'][0] == pytest.approx(58.823229)
    assert out['position'][1] == pytest.approx(17.635998)
    assert out['depth'] == pytest.approx(12.3)
    assert out['svs'] == pytest.approx(1481.6)
    assert out['speed'] == pytest.approx(1.2)
    assert out['bt'] == 'move_to'


def test_encode_only_enabled_fields():
    """Fields not in the enabled set are omitted even if a value is given."""
    payload = encode_telemetry(['bt'], position=(1.0, 2.0), bt='hi')
    assert payload == "Bhi"


def test_strip_marker_present_and_absent():
    """strip_marker returns the post-marker payload, or None for plain data."""
    assert strip_marker("TEL:P1,2") == "P1,2"
    assert strip_marker("58.1,18.2") is None


def test_payload_budget_drops_bt_keeps_position():
    """When over budget the free-text bt is sacrificed; position is preserved."""
    payload = encode_telemetry(
        ['position', 'bt'], position=(1.0, 2.0), bt='X' * 40, max_payload_len=18)
    assert payload == "P1.000000,2.000000"
    assert 'B' not in payload


def test_decode_unknown_tag_captured():
    """Unrecognised tags land under 'unknown' rather than being dropped."""
    out = decode_telemetry("Z123")
    assert out['unknown'] == {'Z': '123'}


def test_decode_skips_malformed_field():
    """A malformed field is skipped without breaking the rest of the payload."""
    out = decode_telemetry("Pbad;D5.0")
    assert 'position' not in out
    assert out['depth'] == pytest.approx(5.0)


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
# The fake Teensy ACKs $Y config with #A<id>; tests then drive the START/OK   #
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
    from std_msgs.msg import Float32

    st = Stack(responder=teensy_responder, profile='teensy').start_driver()
    st.start_probe()
    ranges = st.probe.collect(Float32, '/surface_unit/owtt_beacon/lolo/range')
    st.start_node(
        exe('owtt_surface_unit_node'),
        ['-p', 'mqtt.enabled:=false',
         '-p', 'owtt.auto_calibrate:=false',
         '-p', 'owtt.offset_us:=771600.0'])
    assert st.fake.wait_for_command('$Y067R', timeout=8), \
        f"no receiver config; got {st.fake.commands()!r}"
    time.sleep(0.5)
    payload = 'TEL:C1500.0'                          # svs telemetry frame
    st.fake.inject(f'#B101{len(payload):02d}{payload}')
    st.fake.inject('#I871600')                       # (871600-771600)*1e-6*1500 = 150 m
    ok = wait_until(lambda: len(ranges) > 0, timeout=8)
    node_out, _ = st.stop()
    assert ok, f"surface unit did not publish a range; out:\n{node_out}"
    assert ranges[0].data == pytest.approx(150.0, rel=1e-2)
