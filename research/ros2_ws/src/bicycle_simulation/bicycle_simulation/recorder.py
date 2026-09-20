import csv
import json
from pathlib import Path
import time

import numpy as np
from bicycle_interfaces.msg import BicycleState
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String

from pathfinder.control import Reference
from pathfinder.model import wrap

from .common import ConfiguredNode, LATCHED, spin, stamp_ns


class RecorderNode(ConfiguredNode):
    def __init__(self):
        super().__init__("bicycle_recorder")
        self.output = Path(self.declare_parameter("output", "runs/ros-latest").value)
        self.output.mkdir(parents=True, exist_ok=True)
        self.stream = (self.output / "telemetry.csv").open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.stream)
        fields = ["x", "y", "yaw", "speed", "lean", "lean_rate", "steer"]
        self.writer.writerow(
            [
                "sequence",
                "sim_time_s",
                *["truth_" + f for f in fields],
                *["estimate_" + f for f in fields],
                "gnss_age",
            ]
        )
        self.fields = fields
        self.truth, self.estimates, self.rows = {}, {}, []
        self.next_sequence = 0
        self.terminal = None
        self.terminal_received = None
        self.finished = False
        self.reference = Reference(self.config.get("route", "curve"))
        (self.output / "config.json").write_text(json.dumps(self.config, indent=2) + "\n")
        self.create_subscription(BicycleState, "truth", self.on_truth, 100)
        self.create_subscription(BicycleState, "state", self.on_state, 100)
        self.create_subscription(String, "status", self.on_status, LATCHED)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.finish_if_ready, clock=self.wall_clock)

    def on_truth(self, msg):
        self.truth[msg.sequence] = msg
        self.join()

    def on_state(self, msg):
        self.estimates[msg.sequence] = msg
        self.join()

    def join(self):
        while self.next_sequence in self.truth and self.next_sequence in self.estimates:
            truth = self.truth.pop(self.next_sequence)
            state = self.estimates.pop(self.next_sequence)
            if stamp_ns(truth.header.stamp) != stamp_ns(state.header.stamp):
                raise ValueError("Truth/state timestamp mismatch")
            row = [
                truth.sequence,
                stamp_ns(truth.header.stamp) / 1e9,
                *truth.state,
                *state.state,
                state.gnss_age,
            ]
            self.writer.writerow(row)
            self.rows.append(row)
            self.next_sequence += 1
            if self.next_sequence % 100 == 0:
                self.stream.flush()

    def on_status(self, msg):
        status = json.loads(msg.data)
        if status["mode"] in ("COMPLETE", "FAULT", "ESTOP"):
            self.terminal = status
            self.terminal_received = time.monotonic()

    def finish_if_ready(self):
        if self.terminal is None or self.finished:
            return
        complete_pairs = self.next_sequence == self.terminal["sequence"] + 1
        if not complete_pairs and time.monotonic() - self.terminal_received < 2:
            return
        self.stream.flush()
        self.stream.close()
        metrics = dict(self.terminal)
        metrics["samples"] = len(self.rows)
        metrics["all_samples_paired"] = complete_pairs
        if self.rows:
            array = np.array(self.rows)
            truth, estimate = array[:, 2:9], array[:, 9:16]
            errors = estimate - truth
            errors[:, 2] = wrap(errors[:, 2])
            tracking = truth[:, 1] - np.array([self.reference.at(x)[0] for x in truth[:, 0]])
            metrics.update(
                {
                    "cross_track_rmse_m": float(np.sqrt(np.mean(tracking**2))),
                    "max_abs_lean_rad": float(np.max(np.abs(truth[:, 4]))),
                    "estimation_rmse": dict(
                        zip(self.fields, np.sqrt(np.mean(errors**2, axis=0)).tolist())
                    ),
                    "degraded_samples": int(np.sum(array[:, 16] > 1.0)),
                }
            )
        (self.output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
        self.get_logger().info(json.dumps(metrics))
        self.finished = True

    def destroy_node(self):
        if not self.stream.closed:
            self.stream.close()
        return super().destroy_node()


def main():
    spin(RecorderNode)
