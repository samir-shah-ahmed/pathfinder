"""Seven-state EKF with Joseph covariance update and wrapped yaw innovation."""

import numpy as np

from .model import Bicycle, wrap


class EKF:
    def __init__(self, initial, model=None):
        self.x = np.array(initial, dtype=float)
        self.p = np.diag([1, 1, 0.1, 0.1, 0.02, 0.03, 0.01])
        self.model = model or Bicycle()
        self.q = np.diag([0.01, 0.01, 0.005, 0.02, 0.001, 0.05, 0.002])

    def predict(self, command, dt):
        eps = 1e-5
        columns = []
        for j in range(7):
            offset = np.zeros(7)
            offset[j] = eps
            diff = self.model.step(self.x + offset, command, dt) - self.model.step(
                self.x - offset, command, dt
            )
            diff[2] = wrap(diff[2])
            columns.append(diff / (2 * eps))
        jac = np.column_stack(columns)
        self.x = self.model.step(self.x, command, dt)
        self.p = jac @ self.p @ jac.T + self.q * dt

    def update(self, values, indices, variances):
        z = np.asarray(values, dtype=float)
        indices = list(indices)
        variances = np.asarray(variances, dtype=float)
        if len(z) != len(indices) or variances.shape != z.shape:
            raise ValueError("Measurement dimensions differ")
        if not np.isfinite(z).all() or not np.isfinite(variances).all() or (variances <= 0).any():
            raise ValueError("Invalid measurement/covariance")
        h = np.eye(7)[indices]
        innovation = z - h @ self.x
        for j, index in enumerate(indices):
            if index == 2:
                innovation[j] = wrap(innovation[j])
        r = np.diag(variances)
        s = h @ self.p @ h.T + r
        # Reject extreme outliers before they contaminate the state.
        if innovation @ np.linalg.solve(s, innovation) > 36 * len(indices):
            return False
        k = np.linalg.solve(s, h @ self.p).T
        self.x += k @ innovation
        self.x[2] = wrap(self.x[2])
        residual = np.eye(7) - k @ h
        self.p = residual @ self.p @ residual.T + k @ r @ k.T
        self.p = (self.p + self.p.T) / 2
        return True
