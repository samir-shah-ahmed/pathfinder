"""Published Meijaard et al. (2007) four-state linear bicycle benchmark.

Fixed published geometry, not the measured Pathfinder bicycle. Equations 6.1-6.4;
DOI 10.1098/rspa.2007.1857. State order: lean, steer, lean rate, steer rate.
"""

import argparse
import csv
from functools import lru_cache
import json
from pathlib import Path

import numpy as np
from scipy.linalg import expm
from scipy.optimize import brentq


class WhippleBenchmark:
    """Canonical right-positive angles, constant speed, ideal rolling contact.

    Input is [lean torque, steering torque] in N m, NOT commanded steering angle.
    The existing reduced-model controller must not be connected without an actuator model.
    """

    def __init__(self, speed=5.0, gravity=9.81):
        if not np.isfinite([speed, gravity]).all() or speed < 0 or gravity <= 0:
            raise ValueError("Require finite nonnegative speed and positive gravity")
        self.speed = float(speed)
        self.gravity = float(gravity)
        self.mass = np.array([[80.81722, 2.31941332208709], [2.31941332208709, 0.29784188199686]])
        self.c1 = np.array([[0, 33.86641391492494], [-0.85035641456978, 1.68540397397560]])
        self.k0 = np.array([[-80.95, -2.59951685249872], [-2.59951685249872, -0.80329488458618]])
        self.k2 = np.array([[0, 76.59734589573222], [0, 2.65431523794604]])
        self.a = np.block(
            [
                [np.zeros((2, 2)), np.eye(2)],
                [
                    -np.linalg.solve(self.mass, gravity * self.k0 + speed**2 * self.k2),
                    -np.linalg.solve(self.mass, speed * self.c1),
                ],
            ]
        )
        self.b = np.vstack([np.zeros((2, 2)), np.linalg.solve(self.mass, np.eye(2))])
        for matrix in (self.mass, self.c1, self.k0, self.k2, self.a, self.b):
            matrix.flags.writeable = False

    def eigenvalues(self):
        return np.linalg.eigvals(self.a)

    @lru_cache(maxsize=16)
    def discretize(self, dt):
        if not np.isfinite(dt) or not 0 < dt <= 0.05:
            raise ValueError("Require 0 < dt <= 0.05 s")
        # Exact zero-order-hold discretization without requiring an invertible A.
        augmented = np.zeros((6, 6))
        augmented[:4, :4], augmented[:4, 4:] = self.a, self.b
        transition = expm(augmented * dt)
        transition.flags.writeable = False
        return transition[:4, :4], transition[:4, 4:]

    def step(self, state, torque, dt):
        state, torque = np.asarray(state, dtype=float), np.asarray(torque, dtype=float)
        if state.shape != (4,) or torque.shape != (2,) or not np.isfinite([*state, *torque]).all():
            raise ValueError("Expected four finite states and two finite torques")
        ad, bd = self.discretize(dt)
        return ad @ state + bd @ torque


def stability_boundaries():
    def spectral_abscissa(speed):
        return float(np.max(WhippleBenchmark(speed).eigenvalues().real))

    return brentq(spectral_abscissa, 4, 5, xtol=1e-13), brentq(spectral_abscissa, 6, 7, xtol=1e-13)


def report(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    weave, capsize = stability_boundaries()
    reference_weave, reference_capsize = 4.29238253634111, 6.02426201538837
    metrics = {
        "data_source": "LINEAR_PUBLISHED_BENCHMARK_SIMULATION",
        "reference_doi": "10.1098/rspa.2007.1857",
        "gravity_m_s2": 9.81,
        "weave_boundary_m_s": weave,
        "capsize_boundary_m_s": capsize,
        "absolute_boundary_errors_m_s": [
            abs(weave - reference_weave),
            abs(capsize - reference_capsize),
        ],
        "validates": "Published fixed-geometry linear equations only; not Pathfinder hardware or reduced-model controllers",
    }
    with (output / "eigenvalues.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["speed_m_s", "eigenvalue_real_s_inv", "eigenvalue_imag_s_inv"])
        for speed in np.linspace(0, 10, 101):
            for value in np.sort_complex(WhippleBenchmark(speed).eigenvalues()):
                writer.writerow([speed, value.real, value.imag])
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("runs/whipple"))
    args = parser.parse_args()
    print(json.dumps(report(args.output), indent=2))


if __name__ == "__main__":
    main()
