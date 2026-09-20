"""Simulation supervisor. This does not implement a physical safe stop."""

from enum import Enum

import numpy as np


class Mode(str, Enum):
    INIT = "INIT"
    READY = "READY"
    ACTIVE = "ACTIVE"
    DEGRADED = "DEGRADED"
    FAULT = "FAULT"
    ESTOP = "ESTOP"


class Supervisor:
    def __init__(self):
        self.mode = Mode.INIT
        self.reason = "awaiting checks"
        self.armed = False

    def check(self, state, sensor_age=0.0, heartbeat_age=0.0, gnss_age=0.0, estop=False):
        if self.mode in (Mode.FAULT, Mode.ESTOP):
            return self.mode
        if estop:
            self.mode, self.reason = Mode.ESTOP, "operator emergency stop"
        elif (
            not np.isfinite(state).all()
            or not np.isfinite([sensor_age, heartbeat_age, gnss_age]).all()
            or min(sensor_age, heartbeat_age, gnss_age) < 0
        ):
            self.mode, self.reason = Mode.FAULT, "invalid state or timestamps"
        elif sensor_age > 0.1 or heartbeat_age > 0.1:
            self.mode, self.reason = Mode.FAULT, "critical sensor or MCU heartbeat timeout"
        elif abs(state[4]) > 0.45 or state[3] < 1.0:
            self.mode, self.reason = Mode.FAULT, "outside balance operating envelope"
        else:
            if gnss_age > 1.0:
                self.mode = Mode.DEGRADED
            elif self.armed:
                self.mode = Mode.ACTIVE
            else:
                self.mode = Mode.READY
            self.reason = "GNSS unavailable" if gnss_age > 1.0 else "checks passed"
        return self.mode

    def arm(self):
        if self.mode != Mode.READY:
            raise RuntimeError("Cannot arm before checks pass")
        self.mode = Mode.ACTIVE
        self.armed = True
