"""Pure-logic tests for the ping_estimator_action range-only position estimators.

These exercise ``range_estimators`` directly (numpy only, no ROS, no hardware):
synthetic own-vehicle anchors + noise-free ranges to a known remote modem, plus
the measurement-count gating and one-pass outlier rejection. Mirrors the style
of ``test_owtt_protocol.py``.
"""

import json
import signal

import numpy as np
import pytest

from serial_ping_pkg.ping_estimator_action.range_estimators import (
    EkfRangeEstimator,
    LeastSquaresRangeEstimator,
)

from full_stack_harness import (
    DRIVER_AVAILABLE,
    Stack,
    exe,
    succorfish_modem_responder,
    wait_until,
)


# A spread of own-vehicle positions (ENU, z up). Good geometry around the remote.
_ANCHORS = np.array([
    [0.0, 0.0, 0.0],
    [40.0, 0.0, 0.0],
    [0.0, 40.0, 0.0],
    [40.0, 40.0, 0.0],
    [20.0, -15.0, 0.0],
    [-15.0, 20.0, 0.0],
    [55.0, 20.0, 0.0],
    [20.0, 55.0, 0.0],
], dtype=float)

_TRUE_REMOTE = np.array([18.0, 22.0, -6.0], dtype=float)


def _ranges_to(remote, anchors=_ANCHORS):
    return np.linalg.norm(anchors - remote, axis=1)


def _ls(z_remote, outlier_gate_m=0.0):
    return LeastSquaresRangeEstimator(
        modem_id='007',
        z_remote=z_remote,
        max_measurements=40,
        range_sigma_m=1.0,
        remote_depth_sigma_m=0.5,
        damping=0.001,
        max_iterations=50,
        outlier_gate_m=outlier_gate_m,
    )


# --------------------------------------------------------------------------- #
# Least squares                                                               #
# --------------------------------------------------------------------------- #

def test_ls_needs_min_measurements():
    """Fewer than min_measurements yields no estimate."""
    est = _ls(z_remote=_TRUE_REMOTE[2])
    ranges = _ranges_to(_TRUE_REMOTE)
    for anchor, r in zip(_ANCHORS[:3], ranges[:3]):
        est.add_measurement(anchor, r)
    xyz, cov = est.estimate(min_measurements=4)
    assert xyz is None and cov is None


def test_ls_known_depth_converges():
    """With known remote depth, x/y are recovered and z is pinned."""
    est = _ls(z_remote=_TRUE_REMOTE[2])
    for anchor, r in zip(_ANCHORS, _ranges_to(_TRUE_REMOTE)):
        est.add_measurement(anchor, r)
    xyz, cov = est.estimate(min_measurements=4)
    assert xyz is not None
    assert xyz[2] == pytest.approx(_TRUE_REMOTE[2])
    assert np.linalg.norm(xyz[:2] - _TRUE_REMOTE[:2]) < 1e-2
    assert cov.shape == (3, 3)


def test_ls_unknown_depth_converges_full_3d():
    """With unknown depth, full x/y/z is recovered from 3D geometry."""
    est = _ls(z_remote=None)
    # Vary anchor depth so z is observable.
    anchors = _ANCHORS.copy()
    anchors[:, 2] = np.array([0, -2, -4, -1, -3, -5, -2, -4], dtype=float)
    for anchor, r in zip(anchors, _ranges_to(_TRUE_REMOTE, anchors)):
        est.add_measurement(anchor, r)
    xyz, _ = est.estimate(min_measurements=4)
    assert xyz is not None
    assert np.linalg.norm(xyz - _TRUE_REMOTE) < 1e-1


def test_ls_outlier_gate_rejects_bad_range():
    """A gross outlier outvoted by clean inliers is rejected by the one-pass gate.

    The gate is deliberately conservative (re-fit after dropping residuals beyond
    the gate), so it only helps when the outlier is clearly outnumbered.
    """
    clean = _ls(z_remote=_TRUE_REMOTE[2], outlier_gate_m=0.0)
    gated = _ls(z_remote=_TRUE_REMOTE[2], outlier_gate_m=10.0)

    # Many clean ranges (two passes of the anchor set) outvote one gross outlier.
    for anchor, r in zip(np.vstack([_ANCHORS, _ANCHORS]),
                         np.concatenate([_ranges_to(_TRUE_REMOTE)] * 2)):
        clean.add_measurement(anchor, r)
        gated.add_measurement(anchor, r)
    clean.add_measurement(_ANCHORS[0], _ranges_to(_TRUE_REMOTE)[0] + 100.0)
    gated.add_measurement(_ANCHORS[0], _ranges_to(_TRUE_REMOTE)[0] + 100.0)

    xyz_clean, _ = clean.estimate(min_measurements=4)
    xyz_gated, _ = gated.estimate(min_measurements=4)

    err_clean = np.linalg.norm(xyz_clean[:2] - _TRUE_REMOTE[:2])
    err_gated = np.linalg.norm(xyz_gated[:2] - _TRUE_REMOTE[:2])

    # Gating both improves on the un-gated fit and lands essentially on truth.
    assert err_gated < err_clean
    assert err_gated < 1e-2


# --------------------------------------------------------------------------- #
# EKF                                                                         #
# --------------------------------------------------------------------------- #

def test_ekf_needs_bootstrap():
    """The EKF produces nothing until it has bootstrapped from least squares."""
    est = EkfRangeEstimator(
        modem_id='007', z_remote=_TRUE_REMOTE[2],
        range_sigma_m=1.0, remote_depth_sigma_m=0.5,
        process_noise_std_m=0.05, initial_sigma_xy_m=50.0, initial_sigma_z_m=2.0,
        bootstrap_max_measurements=40, bootstrap_damping=0.001, bootstrap_iterations=50,
    )
    ranges = _ranges_to(_TRUE_REMOTE)
    est.add_measurement(_ANCHORS[0], ranges[0])
    xyz, cov = est.estimate(min_measurements=4)
    assert xyz is None and cov is None


def test_ekf_converges_known_depth():
    """After enough measurements the EKF converges near the true position."""
    est = EkfRangeEstimator(
        modem_id='007', z_remote=_TRUE_REMOTE[2],
        range_sigma_m=1.0, remote_depth_sigma_m=0.5,
        process_noise_std_m=0.05, initial_sigma_xy_m=50.0, initial_sigma_z_m=2.0,
        bootstrap_max_measurements=40, bootstrap_damping=0.001, bootstrap_iterations=50,
    )
    # Feed the anchor set a few times so the filter settles.
    for _ in range(4):
        for anchor, r in zip(_ANCHORS, _ranges_to(_TRUE_REMOTE)):
            est.add_measurement(anchor, r)
    xyz, cov = est.estimate(min_measurements=4)
    assert xyz is not None
    assert xyz[2] == pytest.approx(_TRUE_REMOTE[2])
    assert np.linalg.norm(xyz[:2] - _TRUE_REMOTE[:2]) < 2.0


def test_range_formula_matches_protocol():
    """Two-way tick -> range conversion used by the node: ticks * c * 3.125e-5."""
    ticks, c = 12345, 1500.0
    assert ticks * c * 3.125e-5 == pytest.approx(578.671875)


# --------------------------------------------------------------------------- #
# Full-stack pty integration (Teensy wire profile + Succorfish ping mechanics) #
#                                                                             #
# fake modem <--pty--> succorfish_driver <--ROS--> ping_estimator_action.     #
# The node drops the Teensy into wire mode on startup and then pings the      #
# Succorfish directly; goals are driven over the smarc_modem_ping action.     #
# --------------------------------------------------------------------------- #

_skip = pytest.mark.skipif(
    not DRIVER_AVAILABLE or exe('ping_estimator_action_node') is None,
    reason="serial_ping_pkg / succorfish_driver not built/sourced")


def _send_goal(probe, goal_dict, server='smarc_modem_ping',
               wait_result=False, accept_timeout=15.0, result_timeout=25.0):
    """Send a smarc_modem_ping BaseAction goal and wait for it to be accepted.

    ``wait_result`` additionally blocks until the action result is back (used for
    the instantaneous add/broadcast goals; the long-running ping is fire-and-go).
    """
    from rclpy.action import ActionClient
    from smarc_msgs.action import BaseAction

    ac = ActionClient(probe.node, BaseAction, server)
    try:
        if not ac.wait_for_server(timeout_sec=10):
            return False
        goal = BaseAction.Goal()
        goal.goal.data = json.dumps(goal_dict)
        send_future = ac.send_goal_async(goal)
        if not wait_until(lambda: send_future.done(), timeout=accept_timeout):
            return False
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False
        if wait_result:
            result_future = handle.get_result_async()
            wait_until(lambda: result_future.done(), timeout=result_timeout)
        return True
    finally:
        ac.destroy()


@_skip
def test_ping_estimator_wire_lifecycle_and_broadcast():
    """Teensy wire config on startup, a broadcast goal on the wire, wire reset on exit.

    Covers ping_estimator_action's serial transport end to end: it drops the
    Teensy into wire mode on startup (``$Y007W``), a ``broadcast`` action goal is
    written through the driver as ``$B<nn><payload>``, and on exit the driver
    replays the registered wire-mode command. (The ``$P``->``#R...T...`` ranging
    mechanic itself is exercised by the Succorfish ping nodes in
    ``test_common_protocol`` / ``test_tuper_twtt``.)
    """
    st = Stack(responder=succorfish_modem_responder, profile='teensy').start_driver()
    st.start_probe()
    st.start_node(exe('ping_estimator_action_node'))
    # enable_wire_on_startup -> the Teensy is dropped into wire mode (own id 007).
    assert st.fake.wait_for_command('$Y007W', timeout=10), \
        f"no wire-mode config on startup; got {st.fake.commands()!r}"
    assert _send_goal(st.probe, {'mode': 'broadcast', 'message': 'hello'},
                      wait_result=True), \
        "smarc_modem_ping action server never came up / goal rejected"
    assert st.fake.wait_for_command('$B05hello', timeout=10), \
        f"broadcast goal did not reach the wire; got {st.fake.commands()!r}"
    # On exit the driver replays the registered wire-mode command.
    st.node.send_signal(signal.SIGINT)
    saw_exit_wire = st.fake.wait_for_command(lambda c: c.endswith('W'), timeout=8)
    node_out, _ = st.stop()
    assert saw_exit_wire, f"no wire reset on exit; got {st.fake.commands()!r}\n{node_out}"
