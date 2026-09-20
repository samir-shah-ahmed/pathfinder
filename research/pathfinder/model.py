"""Reduced nonlinear lean model; not a validated physical bicycle model."""

from dataclasses import dataclass

import numpy as np


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


@dataclass(frozen=True)
class Parameters:
    wheelbase: float = 1.1
    height: float = 0.65
    gravity: float = 9.81
    lean_damping: float = 0.5
    steer_tau: float = 0.10
    max_steer: float = 0.55
    max_steer_rate: float = 1.5
    max_accel: float = 2.0

    def __post_init__(self):
        if not all(np.isfinite(v) and v > 0 for v in self.__dict__.values()):
            raise ValueError("Model parameters must be finite and positive")


class Bicycle:
    """State = x, y, yaw, speed, left lean, left lean rate, steer; SI units."""

    def __init__(self, parameters=None):
        self.p = parameters or Parameters()

    def derivative(self, state, command, disturbance=0.0):
        x, y, yaw, speed, lean, rate, steer = state
        target, accel = command
        p = self.p
        lateral = speed**2 * np.tan(steer) / p.wheelbase
        steer_rate = np.clip(
            (np.clip(target, -p.max_steer, p.max_steer) - steer) / p.steer_tau,
            -p.max_steer_rate,
            p.max_steer_rate,
        )
        return np.array(
            [
                speed * np.cos(yaw),
                speed * np.sin(yaw),
                speed * np.tan(steer) / p.wheelbase,
                np.clip(accel, -p.max_accel, p.max_accel),
                rate,
                (p.gravity * np.sin(lean) - lateral * np.cos(lean)) / p.height
                - p.lean_damping * rate
                + disturbance,
                steer_rate,
            ]
        )

    def step(self, state, command, dt, disturbance=0.0):
        state, command = np.asarray(state, dtype=float), np.asarray(command, dtype=float)
        if state.shape != (7,) or command.shape != (2,):
            raise ValueError("Expected seven state values and two command values")
        if not (0 < dt <= 0.02) or not np.isfinite([*state, *command, disturbance]).all():
            raise ValueError("Finite inputs and 0 < dt <= 0.02 required")

        def f(s):
            return self.derivative(s, command, disturbance)

        a = f(state)
        b = f(state + dt * a / 2)
        c = f(state + dt * b / 2)
        d = f(state + dt * c)
        result = state + dt * (a + 2 * b + 2 * c + d) / 6
        result[2] = wrap(result[2])
        result[3] = max(0.0, result[3])
        result[6] = np.clip(result[6], -self.p.max_steer, self.p.max_steer)
        return result
