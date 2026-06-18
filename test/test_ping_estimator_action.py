"""Pure-logic tests for the ping_estimator_action range-only position estimators.

These exercise ``range_estimators`` directly (numpy only, no ROS, no hardware):
synthetic own-vehicle anchors + noise-free ranges to a known remote modem, plus
the measurement-count gating and one-pass outlier rejection. Mirrors the style
of ``test_owtt_protocol.py``.
"""

import numpy as np
import pytest

from serial_ping_pkg.ping_estimator_action.range_estimators import (
    EkfRangeEstimator,
    LeastSquaresRangeEstimator,
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
