"""Analytic path guidance plus explicit speed-scheduled LQR lean regulation."""

from functools import lru_cache

import numpy as np
from scipy.linalg import solve_continuous_are

from .model import Parameters, wrap


class Reference:
    def __init__(self, kind="curve"):
        if kind not in ("straight", "curve"):
            raise ValueError("Unknown reference")
        self.kind = kind

    def at(self, x):
        if self.kind == "straight":
            return 0.0, 0.0, 0.0
        amplitude, frequency = 1.0, 0.07
        y = amplitude * np.sin(frequency * x)
        dy = amplitude * frequency * np.cos(frequency * x)
        ddy = -amplitude * frequency**2 * np.sin(frequency * x)
        return y, np.arctan(dy), ddy / (1 + dy * dy) ** 1.5


class Controller:
    def __init__(self, reference, parameters=None, speed=3.0, mode="lqr"):
        if mode not in ("lqr", "pd"):
            raise ValueError("Unknown balance controller")
        self.reference = reference
        self.p = parameters or Parameters()
        self.speed = speed
        self.mode = mode

    @lru_cache(maxsize=100)
    def gain(self, speed):
        p = self.p
        a = np.array(
            [
                [0, 1, 0],
                [p.gravity / p.height, -p.lean_damping, -(speed**2) / (p.wheelbase * p.height)],
                [0, 0, -1 / p.steer_tau],
            ]
        )
        b = np.array([[0], [0], [1 / p.steer_tau]])
        r = np.array([[1.0]])
        solution = solve_continuous_are(a, b, np.diag([30, 3, 0.5]), r)
        return np.linalg.solve(r, b.T @ solution).ravel()

    def command(self, state):
        x, y, yaw, speed, lean, rate, steer = state
        if not np.isfinite(state).all() or speed < 1.0:
            raise ValueError("Steering-only balance requires finite state and speed >= 1 m/s")
        ref_y, heading, curvature = self.reference.at(x)
        cross = (ref_y - y) * np.cos(heading)
        # Slow outer loop leaves the lean regulator time for counter-steering.
        curvature += 0.35 * wrap(heading - yaw) / speed + 0.07 * cross / speed
        desired_lean = np.clip(np.arctan(speed**2 * curvature / self.p.gravity), -0.15, 0.15)
        desired_steer = np.arctan(
            self.p.gravity * self.p.wheelbase * np.tan(desired_lean) / speed**2
        )
        if self.mode == "lqr":
            error = np.array([lean - desired_lean, rate, steer - desired_steer])
            target = desired_steer - self.gain(round(speed, 1)) @ error
        else:
            target = (
                self.p.wheelbase
                / speed**2
                * (self.p.gravity * lean + self.p.height * (18 * (lean - desired_lean) + 7 * rate))
            )
        return np.array(
            [
                np.clip(target, -self.p.max_steer, self.p.max_steer),
                np.clip(1.5 * (self.speed - speed), -self.p.max_accel, self.p.max_accel),
            ]
        )
