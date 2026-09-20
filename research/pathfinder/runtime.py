"""Transport-independent plant, sampled observations and ordered state fusion."""

from dataclasses import dataclass

import numpy as np

from .estimation import EKF
from .model import Bicycle, Parameters


@dataclass(frozen=True)
class Observation:
    sequence: int
    stamp_ns: int
    command: np.ndarray
    fast: np.ndarray
    variance: np.ndarray
    gnss: np.ndarray | None


def validate_config(config):
    dt = config.get("dt", 0.01)
    duration = config.get("duration", 20.0)
    if not np.isfinite([dt, duration]).all() or not 0 < dt <= 0.02 or not 0 < duration <= 600:
        raise ValueError("Require 0 < dt <= 0.02 and 0 < duration <= 600")
    if not np.isclose(dt * 1e9, round(dt * 1e9), atol=1e-6, rtol=0):
        raise ValueError("Timestep must be an integer number of nanoseconds")
    if not np.isclose(duration / dt, round(duration / dt), atol=1e-6, rtol=0):
        raise ValueError("Duration must contain a whole number of steps")
    dropout = config.get("gnss_dropout", [8, 11])
    if len(dropout) != 2 or not np.isfinite(dropout).all() or not 0 <= dropout[0] <= dropout[1]:
        raise ValueError("Invalid GNSS dropout interval")
    return round(dt * 1e9), round(duration / dt)


class Plant:
    def __init__(self, config):
        self.dt_ns, self.total_steps = validate_config(config)
        self.dt = self.dt_ns / 1e9
        self.config = config
        self.model = Bicycle(Parameters(**config.get("model", {})))
        self.truth = np.array([0, 0.2, 0, 3, 0.02, 0, 0.0])
        self.sequence = 0
        self.rng = np.random.default_rng(config.get("seed", 7))
        self.observation = self.sample(np.zeros(2))

    @property
    def time(self):
        return self.sequence * self.dt

    def sample(self, command):
        std = np.array([0.01, 0.03, 0.004, 0.01, 0.003])
        fast = self.truth[[2, 3, 4, 5, 6]] + self.rng.normal(0, std)
        start, end = self.config.get("gnss_dropout", [8, 11])
        # 5 Hz, sampled at the nearest supported step without float modulo.
        period = max(1, round(0.2 / self.dt))
        available = self.sequence % period == 0 and not start <= self.time < end
        gnss = self.truth[:2] + self.rng.normal(0, 0.25, 2) if available else None
        return Observation(
            self.sequence, self.sequence * self.dt_ns, np.array(command), fast, std**2, gnss
        )

    def step(self, command):
        if self.sequence >= self.total_steps:
            raise RuntimeError("Simulation already complete")
        disturbance = 0.35 if self.config.get("disturbance", True) and 5 <= self.time < 5.2 else 0
        self.truth = self.model.step(self.truth, command, self.dt, disturbance)
        if abs(self.truth[4]) > 0.6:
            raise RuntimeError("Ground-truth lean exceeded simulation envelope")
        self.sequence += 1
        self.observation = self.sample(command)
        return self.observation


class Fusion:
    def __init__(self, config):
        self.dt_ns, _ = validate_config(config)
        self.filter = EKF([0, 0, 0, 3, 0, 0, 0], Bicycle(Parameters(**config.get("model", {}))))
        self.last_sequence = -1
        self.last_gnss_ns = 0
        self.gnss_age = 0.0

    def consume(self, observation):
        o = observation
        if o.sequence != self.last_sequence + 1 or o.stamp_ns != o.sequence * self.dt_ns:
            raise ValueError("Missing, duplicate or out-of-order observation")
        if np.shape(o.fast) != (5,) or np.shape(o.variance) != (5,):
            raise ValueError("Attitude/encoder observation must contain five values")
        if (
            np.shape(o.command) != (2,)
            or not np.isfinite(o.command).all()
            or not np.isfinite(o.fast).all()
            or not np.isfinite(o.variance).all()
            or np.any(o.variance <= 0)
        ):
            raise ValueError("Invalid observation")
        if o.gnss is not None and (np.shape(o.gnss) != (2,) or not np.isfinite(o.gnss).all()):
            raise ValueError("Invalid local GNSS observation")
        if o.sequence > 0:
            self.filter.predict(o.command, self.dt_ns / 1e9)
        self.filter.update(o.fast, [2, 3, 4, 5, 6], o.variance)
        if o.gnss is not None:
            if self.filter.update(o.gnss, [0, 1], [0.0625, 0.0625]):
                self.last_gnss_ns = o.stamp_ns
        self.gnss_age = (o.stamp_ns - self.last_gnss_ns) / 1e9
        self.last_sequence = o.sequence
        return self.filter.x.copy()


class CommandGate:
    """Simulation command validation; timeout uses monotonic WALL time."""

    def __init__(self, parameters=None):
        self.p = parameters or Parameters()
        self.accepted_sequence = -1

    def accept(self, sequence, stamp_ns, command, observation, valid_for_ns):
        if sequence != observation.sequence or stamp_ns != observation.stamp_ns:
            raise ValueError("Command does not match current observation")
        if sequence <= self.accepted_sequence:
            raise ValueError("Duplicate/replayed command")
        if not 1 <= valid_for_ns <= 100_000_000:
            raise ValueError("Command validity must be positive and <= 100 ms")
        if np.shape(command) != (2,) or not np.isfinite(command).all():
            raise ValueError("Invalid command values")
        if abs(command[0]) > self.p.max_steer or abs(command[1]) > self.p.max_accel:
            raise ValueError("Command exceeds actuator limits")
        self.accepted_sequence = sequence
        return np.array(command, dtype=float)


def orientation(yaw, lean_left):
    """ROS quaternion x,y,z,w: positive internal left lean is negative ROS roll."""
    roll = -lean_left
    return np.array(
        [
            np.sin(roll / 2) * np.cos(yaw / 2),
            np.sin(roll / 2) * np.sin(yaw / 2),
            np.cos(roll / 2) * np.sin(yaw / 2),
            np.cos(roll / 2) * np.cos(yaw / 2),
        ]
    )


def pose_covariance(covariance):
    """Map state x/y/yaw/left-lean covariance into ROS x/y/z/roll/pitch/yaw."""
    jac = np.zeros((6, 7))
    jac[0, 0] = jac[1, 1] = jac[5, 2] = 1
    jac[3, 4] = -1
    result = jac @ covariance @ jac.T
    result[2, 2] = result[4, 4] = 1e6  # unestimated z and pitch
    return result
