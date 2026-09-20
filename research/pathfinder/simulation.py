"""Deterministic simulation, synthetic measurements, telemetry and metrics."""

import argparse
import csv
import json
from pathlib import Path
import time

import numpy as np

from .control import Controller, Reference
from .estimation import EKF
from .model import Bicycle, Parameters, wrap
from .safety import Mode, Supervisor


class Simulation:
    def __init__(self, config):
        self.config = config
        self.dt = config.get("dt", 0.01)
        self.model = Bicycle(Parameters(**config.get("model", {})))
        self.truth = np.array([0, 0.2, 0, 3, 0.02, 0, 0], dtype=float)
        self.filter = EKF([0, 0, 0, 3, 0, 0, 0], self.model)
        self.reference = Reference(config.get("route", "curve"))
        self.controller = Controller(
            self.reference, self.model.p, mode=config.get("controller", "lqr")
        )
        self.rng = np.random.default_rng(config.get("seed", 7))
        self.supervisor = Supervisor()
        self.supervisor.check(self.filter.x)
        self.supervisor.arm()
        self.t = 0.0
        self.last_gnss = 0.0
        self.steps = 0

    def step(self):
        command = self.controller.command(self.filter.x)
        disturbance = 0.35 if self.config.get("disturbance", True) and 5 <= self.t < 5.2 else 0
        self.truth = self.model.step(self.truth, command, self.dt, disturbance)
        self.filter.predict(command, self.dt)
        # Idealized attitude-solution + encoders, NOT raw IMU fusion.
        ids = [2, 3, 4, 5, 6]
        std = np.array([0.01, 0.03, 0.004, 0.01, 0.003])
        measured = self.truth[ids] + self.rng.normal(0, std)
        self.filter.update(measured, ids, std**2)
        dropout = self.config.get("gnss_dropout", [8, 11])
        if self.steps % 20 == 0 and not dropout[0] <= self.t < dropout[1]:
            self.filter.update(self.truth[:2] + self.rng.normal(0, 0.25, 2), [0, 1], [0.0625] * 2)
            self.last_gnss = self.t
        mode = self.supervisor.check(self.filter.x, gnss_age=self.t - self.last_gnss)
        if mode == Mode.READY:
            self.supervisor.arm()
        if mode in (Mode.FAULT, Mode.ESTOP) or abs(self.truth[4]) > 0.6:
            raise RuntimeError(f"Simulation halted: {self.supervisor.reason}")
        self.t += self.dt
        self.steps += 1
        return self.truth.copy(), self.filter.x.copy(), command, self.supervisor.mode.value


def run(config, output):
    duration = config.get("duration", 20)
    dt = config.get("dt", 0.01)
    if not (np.isfinite(duration) and 0 < duration <= 600 and 0 < dt <= 0.02):
        raise ValueError("Invalid duration or timestep")
    sim = Simulation(config)
    rows, errors, tracking, leans = [], [], [], []
    start = time.perf_counter()
    for _ in range(round(duration / dt)):
        truth, estimate, command, mode = sim.step()
        error = estimate - truth
        error[2] = wrap(error[2])
        errors.append(error)
        tracking.append(truth[1] - sim.reference.at(truth[0])[0])
        leans.append(truth[4])
        rows.append([sim.t, *truth, *estimate, *command, mode])
    names = ["x", "y", "yaw", "speed", "lean", "lean_rate", "steer"]
    metrics = {
        "data_source": "SIMULATED",
        "seed": config.get("seed", 7),
        "duration_s": sim.t,
        "steps": sim.steps,
        "cross_track_rmse_m": float(np.sqrt(np.mean(np.square(tracking)))),
        "max_abs_lean_rad": float(np.max(np.abs(leans))),
        "lean_rms_rad": float(np.sqrt(np.mean(np.square(leans)))),
        "estimation_rmse": dict(zip(names, np.sqrt(np.mean(np.square(errors), axis=0)).tolist())),
        "wall_time_s": time.perf_counter() - start,
        "degraded_steps": sum(row[-1] == "DEGRADED" for row in rows),
    }
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "telemetry.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "sim_time_s",
                *["truth_" + n for n in names],
                *["estimate_" + n for n in names],
                "command_steer",
                "command_accel",
                "mode",
            ]
        )
        writer.writerows(rows)
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    (output / "config.json").write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config/simulation.json"))
    parser.add_argument("--output", type=Path, default=Path("runs/latest"))
    args = parser.parse_args()
    print(json.dumps(run(json.loads(args.config.read_text()), args.output), indent=2))


if __name__ == "__main__":
    main()
