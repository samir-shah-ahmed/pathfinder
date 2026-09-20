import json
import time

import numpy as np
from bicycle_interfaces.msg import BicycleCommand, BicycleObservation, BicycleState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.clock import Clock, ClockType
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String
from std_srvs.srv import Trigger

from pathfinder.control import Reference
from pathfinder.runtime import CommandGate, Plant, orientation

from .common import ConfiguredNode, LATCHED, set_stamp, spin, stamp_ns


class PlantNode(ConfiguredNode):
    def __init__(self):
        super().__init__("bicycle_plant")
        self.plant = Plant(self.config)
        self.gate = CommandGate(self.plant.model.p)
        self.rate = float(self.declare_parameter("real_time_factor", 1.0).value)
        self.timeout = float(self.declare_parameter("watchdog_seconds", 2.0).value)
        if (
            not np.isfinite([self.rate, self.timeout]).all()
            or not 0 < self.rate <= 20
            or self.timeout <= 0
        ):
            raise ValueError("Invalid real-time factor or watchdog")
        self.observations = self.create_publisher(BicycleObservation, "observations", 10)
        self.truth = self.create_publisher(BicycleState, "truth", 10)
        self.clock_pub = self.create_publisher(ClockMessage, "/clock", 10)
        self.status_pub = self.create_publisher(String, "status", LATCHED)
        self.reference_pub = self.create_publisher(Path, "reference", LATCHED)
        self.create_subscription(BicycleCommand, "command", self.command, 10)
        self.create_service(Trigger, "estop", self.estop)
        self.started = False
        self.terminal = False
        self.pending = None
        self.ready_since = None
        self.created_at = time.monotonic()
        self.last_progress = self.created_at
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(self.plant.dt / self.rate, self.tick, clock=self.wall_clock)
        self.status("INIT", "Waiting for estimator, controller and recorder")

    def status(self, mode, reason):
        msg = String()
        msg.data = json.dumps(
            {
                "mode": mode,
                "reason": reason,
                "sequence": self.plant.sequence,
                "sim_time_s": self.plant.time,
                "data_source": "SIMULATED",
            }
        )
        self.status_pub.publish(msg)
        if mode in ("FAULT", "ESTOP", "COMPLETE"):
            self.terminal = True
            self.get_logger().info(msg.data)

    def estop(self, request, response):
        del request
        if not self.terminal:
            self.status("ESTOP", "Operator emergency stop; simulation frozen")
        response.success = True
        response.message = "Simulation halted; restart required to reset"
        return response

    def command(self, msg):
        if self.terminal or not self.started:
            return
        if not msg.enabled:
            self.status("FAULT", "Controller inhibited: " + msg.reason)
            return
        try:
            if msg.header.frame_id != "map" or msg.valid_for_ns < self.plant.dt_ns:
                raise ValueError("Wrong frame or expired command before next step")
            self.pending = self.gate.accept(
                msg.sequence,
                stamp_ns(msg.header.stamp),
                msg.command,
                self.plant.observation,
                msg.valid_for_ns,
            )
            self.last_progress = time.monotonic()
        except ValueError as exc:
            self.status("FAULT", str(exc))

    def publish_frame(self):
        o = self.plant.observation
        clock = ClockMessage()
        set_stamp(clock.clock, o.stamp_ns)
        self.clock_pub.publish(clock)
        truth = BicycleState()
        set_stamp(truth.header.stamp, o.stamp_ns)
        truth.header.frame_id = "map"
        truth.sequence = o.sequence
        truth.state = self.plant.truth.tolist()
        self.truth.publish(truth)
        msg = BicycleObservation()
        msg.header = truth.header
        msg.sequence = o.sequence
        msg.applied_command = o.command.tolist()
        msg.fast = o.fast.tolist()
        msg.variance = o.variance.tolist()
        msg.gnss_valid = o.gnss is not None
        if msg.gnss_valid:
            msg.gnss_position = o.gnss.tolist()
        self.observations.publish(msg)

    def publish_reference(self):
        reference = Reference(self.config.get("route", "curve"))
        path = Path()
        path.header.frame_id = "map"
        for x in np.linspace(0, self.config.get("duration", 20) * 3.0 + 5, 200):
            y, yaw, _ = reference.at(x)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            q = orientation(yaw, 0).tolist()
            pose.pose.orientation.x, pose.pose.orientation.y = q[:2]
            pose.pose.orientation.z, pose.pose.orientation.w = q[2:]
            path.poses.append(pose)
        self.reference_pub.publish(path)

    def tick(self):
        now = time.monotonic()
        if self.terminal:
            return
        if not self.started:
            ready = (
                self.count_subscribers("observations") >= 1
                and self.count_subscribers("truth") >= 1
                and self.count_subscribers("state") >= 2
                and self.count_publishers("command") >= 1
            )
            if not ready:
                self.ready_since = None
                if now - self.created_at > 30:
                    self.status("FAULT", "ROS graph startup timeout")
                return
            if self.ready_since is None:
                self.ready_since = now
            if now - self.ready_since < 0.5:
                return
            self.started = True
            self.last_progress = now
            self.publish_reference()
            self.publish_frame()
            self.status("ACTIVE", "Lockstep simulation running")
            return
        if now - self.last_progress > self.timeout:
            self.status("FAULT", "Wall-clock command watchdog expired; simulation frozen")
        elif self.pending is not None:
            if self.plant.sequence == self.plant.total_steps:
                self.status("COMPLETE", "All simulated steps acknowledged")
                return
            try:
                self.plant.step(self.pending)
                self.pending = None
                self.last_progress = now
                self.publish_frame()
            except (ValueError, RuntimeError) as exc:
                self.status("FAULT", str(exc))


def main():
    spin(PlantNode)
