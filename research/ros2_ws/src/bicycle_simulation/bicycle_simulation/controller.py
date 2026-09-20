import numpy as np
from bicycle_interfaces.msg import BicycleCommand, BicycleState

from pathfinder.control import Controller, Reference
from pathfinder.model import Parameters
from pathfinder.safety import Mode, Supervisor

from .common import ConfiguredNode, spin, stamp_ns


class ControllerNode(ConfiguredNode):
    def __init__(self):
        super().__init__("bicycle_controller")
        self.controller = Controller(
            Reference(self.config.get("route", "curve")),
            Parameters(**self.config.get("model", {})),
            mode=self.config.get("controller", "lqr"),
        )
        self.supervisor = Supervisor()
        self.last_sequence = -1
        self.dt_ns = round(self.config.get("dt", 0.01) * 1e9)
        self.publisher = self.create_publisher(BicycleCommand, "command", 10)
        self.create_subscription(BicycleState, "state", self.control, 10)

    def control(self, msg):
        command = BicycleCommand()
        command.header = msg.header
        command.sequence = msg.sequence
        command.valid_for_ns = 2 * self.dt_ns
        try:
            if (
                msg.sequence != self.last_sequence + 1
                or msg.header.frame_id != "map"
                or stamp_ns(msg.header.stamp) != msg.sequence * self.dt_ns
            ):
                raise ValueError("Invalid state sequence, frame or timestamp")
            mode = self.supervisor.check(msg.state, gnss_age=msg.gnss_age)
            if mode == Mode.READY:
                self.supervisor.arm()
            if mode in (Mode.FAULT, Mode.ESTOP):
                raise ValueError(self.supervisor.reason)
            command.command = self.controller.command(np.array(msg.state)).tolist()
            command.enabled = True
            command.reason = self.supervisor.mode.value
            self.last_sequence = msg.sequence
        except (ValueError, RuntimeError) as exc:
            command.enabled = False
            command.reason = str(exc)
        self.publisher.publish(command)


def main():
    spin(ControllerNode)
