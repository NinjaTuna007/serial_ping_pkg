"""Range-only acoustic modem position estimators.

Used in ping_estimator_action_node.py
"""

import math

import numpy as np


class LeastSquaresRangeEstimator:
    """
    Sliding-window range-only nonlinear least-squares estimator.

    Measurement model:
        range_i = || p_remote - p_own_i ||

    If z_remote is known, only x,y are estimated and z is fixed.
    If z_remote is None, full x,y,z are estimated.
    """

    def __init__(
        self,
        modem_id: str,
        z_remote: float | None,
        max_measurements: int,
        range_sigma_m: float,
        remote_depth_sigma_m: float,
        damping: float,
        max_iterations: int,
        outlier_gate_m: float,
    ):
        self.modem_id = modem_id
        self.z_remote = z_remote
        self.max_measurements = max_measurements
        self.range_sigma_m = range_sigma_m
        self.remote_depth_sigma_m = remote_depth_sigma_m
        self.damping = damping
        self.max_iterations = max_iterations
        self.outlier_gate_m = outlier_gate_m

        self.measurements: list[tuple[np.ndarray, float]] = []
        self.last_xyz: np.ndarray | None = None
        self.last_cov: np.ndarray | None = None

    def add_measurement(self, own_xyz: np.ndarray, range_m: float):
        self.measurements.append((np.asarray(own_xyz, dtype=float), float(range_m)))
        self.measurements = self.measurements[-self.max_measurements:]

    def estimate(self, min_measurements: int):
        if len(self.measurements) < min_measurements:
            return None, None

        anchors = np.asarray([m[0] for m in self.measurements], dtype=float)
        ranges = np.asarray([m[1] for m in self.measurements], dtype=float)

        result = self._solve(anchors, ranges)
        if result is None:
            return None, None

        xyz, cov, residuals = result

        # One-pass outlier rejection. Keep it conservative; this is for obvious
        # bad ranges, not heavy robust optimization.
        if self.outlier_gate_m > 0.0 and len(residuals) >= min_measurements + 2:
            keep = np.abs(residuals) <= self.outlier_gate_m
            if np.count_nonzero(keep) >= min_measurements and not np.all(keep):
                result2 = self._solve(anchors[keep], ranges[keep])
                if result2 is not None:
                    xyz, cov, residuals = result2

        self.last_xyz = xyz
        self.last_cov = cov
        return xyz, cov

    def _solve(self, anchors: np.ndarray, ranges: np.ndarray):
        known_z = self.z_remote is not None

        if self.last_xyz is not None:
            x = self.last_xyz[:2].copy() if known_z else self.last_xyz.copy()
        else:
            x = np.mean(anchors[:, :2], axis=0) if known_z else np.mean(anchors, axis=0)

        J = None
        residuals = None

        for _ in range(self.max_iterations):
            residuals_list = []
            jac_rows = []

            for a, r_meas in zip(anchors, ranges):
                if known_z:
                    dx = x[0] - a[0]
                    dy = x[1] - a[1]
                    dz = float(self.z_remote) - a[2]
                    pred = math.sqrt(dx * dx + dy * dy + dz * dz)
                    pred = max(pred, 1e-6)

                    residuals_list.append(pred - r_meas)
                    jac_rows.append([dx / pred, dy / pred])
                else:
                    d = x - a
                    pred = max(float(np.linalg.norm(d)), 1e-6)

                    residuals_list.append(pred - r_meas)
                    jac_rows.append((d / pred).tolist())

            residuals = np.asarray(residuals_list, dtype=float)
            J = np.asarray(jac_rows, dtype=float)

            H = J.T @ J + self.damping * np.eye(J.shape[1])
            g = J.T @ residuals

            try:
                step = -np.linalg.solve(H, g)
            except np.linalg.LinAlgError:
                return None

            x = x + step

            if np.linalg.norm(step) < 1e-3:
                break

        if J is None or residuals is None:
            return None

        try:
            cov_state = (self.range_sigma_m ** 2) * np.linalg.inv(J.T @ J)
        except np.linalg.LinAlgError:
            return None

        if known_z:
            xyz = np.array([x[0], x[1], float(self.z_remote)], dtype=float)
            cov = np.zeros((3, 3), dtype=float)
            cov[:2, :2] = cov_state
            cov[2, 2] = self.remote_depth_sigma_m ** 2
        else:
            xyz = np.asarray(x, dtype=float)
            cov = cov_state

        return xyz, cov, residuals


class EkfRangeEstimator:
    """
    Static-beacon EKF for range-only measurements.

    State:
        x = [remote_x, remote_y, remote_z]

    If z_remote is known, z is clamped after every update.
    EKF is initialized from a bootstrap least-squares estimator once enough
    measurements exist. This avoids the usual garbage EKF initialization problem.
    """

    def __init__(
        self,
        modem_id: str,
        z_remote: float | None,
        range_sigma_m: float,
        remote_depth_sigma_m: float,
        process_noise_std_m: float,
        initial_sigma_xy_m: float,
        initial_sigma_z_m: float,
        bootstrap_max_measurements: int,
        bootstrap_damping: float,
        bootstrap_iterations: int,
    ):
        self.modem_id = modem_id
        self.z_remote = z_remote
        self.range_sigma_m = range_sigma_m
        self.remote_depth_sigma_m = remote_depth_sigma_m
        self.process_noise_std_m = process_noise_std_m
        self.initial_sigma_xy_m = initial_sigma_xy_m
        self.initial_sigma_z_m = initial_sigma_z_m

        self.x: np.ndarray | None = None
        self.P: np.ndarray | None = None
        self.measurement_count = 0

        self.bootstrap = LeastSquaresRangeEstimator(
            modem_id=modem_id,
            z_remote=z_remote,
            max_measurements=bootstrap_max_measurements,
            range_sigma_m=range_sigma_m,
            remote_depth_sigma_m=remote_depth_sigma_m,
            damping=bootstrap_damping,
            max_iterations=bootstrap_iterations,
            outlier_gate_m=0.0,
        )

    def add_measurement(self, own_xyz: np.ndarray, range_m: float):
        own_xyz = np.asarray(own_xyz, dtype=float)
        range_m = float(range_m)

        self.bootstrap.add_measurement(own_xyz, range_m)
        self.measurement_count += 1

        if self.x is None:
            xyz, cov = self.bootstrap.estimate(min_measurements=4)
            if xyz is None:
                return
            self._initialize(xyz)

        assert self.x is not None
        assert self.P is not None

        # Static random-walk prediction.
        q = self.process_noise_std_m ** 2
        self.P = self.P + q * np.eye(3)

        if self.z_remote is not None:
            self.x[2] = float(self.z_remote)

        d = self.x - own_xyz
        pred = max(float(np.linalg.norm(d)), 1e-6)

        H = (d / pred).reshape(1, 3)
        R = np.array([[self.range_sigma_m ** 2]], dtype=float)

        y = np.array([[range_m - pred]], dtype=float)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + (K @ y).reshape(3)
        self.P = (np.eye(3) - K @ H) @ self.P

        if self.z_remote is not None:
            self.x[2] = float(self.z_remote)
            self.P[2, :] = 0.0
            self.P[:, 2] = 0.0
            self.P[2, 2] = self.remote_depth_sigma_m ** 2

    def _initialize(self, xyz: np.ndarray):
        self.x = np.asarray(xyz, dtype=float)
        self.P = np.diag([
            self.initial_sigma_xy_m ** 2,
            self.initial_sigma_xy_m ** 2,
            self.initial_sigma_z_m ** 2,
        ]).astype(float)

        if self.z_remote is not None:
            self.x[2] = float(self.z_remote)
            self.P[2, 2] = self.remote_depth_sigma_m ** 2

    def estimate(self, min_measurements: int):
        if self.x is None or self.P is None:
            return None, None
        if self.measurement_count < min_measurements:
            return None, None
        return self.x.copy(), self.P.copy()
