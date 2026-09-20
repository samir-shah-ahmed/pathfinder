import numpy as np
import pytest

from pathfinder.control import Controller, Reference
from pathfinder.estimation import EKF
from pathfinder.model import Bicycle
from pathfinder.planning import astar, inflate
from pathfinder.safety import Mode, Supervisor
from pathfinder.simulation import run


def test_straight_motion():
    model = Bicycle()
    state = np.array([0, 0, 0, 3, 0, 0, 0.0])
    for _ in range(100):
        state = model.step(state, [0, 0], 0.01)
    np.testing.assert_allclose(state, [3, 0, 0, 3, 0, 0, 0], atol=1e-10)


def test_steady_turn_equilibrium_and_countersteer():
    model = Bicycle()
    delta, speed = 0.1, 3.0
    lean = np.arctan(speed**2 * np.tan(delta) / (model.p.wheelbase * model.p.gravity))
    derivative = model.derivative([0, 0, 0, speed, lean, 0, delta], [delta, 0])
    assert derivative[2] > 0
    assert abs(derivative[5]) < 1e-10
    assert model.derivative([0, 0, 0, speed, 0, 0, delta], [delta, 0])[5] < 0


def test_actuator_limits_and_invalid_input():
    model = Bicycle()
    state = np.array([0, 0, 0, 3, 0, 0, 0.0])
    result = model.step(state, [100, 100], 0.01)
    assert result[6] <= model.p.max_steer_rate * 0.01
    assert result[3] - 3 <= model.p.max_accel * 0.01 + 1e-12
    with pytest.raises(ValueError):
        model.step(state, [float("nan"), 0], 0.01)


def test_estimator_wrap_covariance_and_outlier():
    ekf = EKF([0, 0, np.pi - 0.01, 3, 0, 0, 0])
    assert ekf.update([-np.pi + 0.01], [2], [0.01])
    assert abs(abs(ekf.x[2]) - np.pi) < 0.02
    before = ekf.x.copy()
    assert not ekf.update([1000, 1000], [0, 1], [0.01, 0.01])
    np.testing.assert_equal(before, ekf.x)
    assert np.linalg.eigvalsh(ekf.p).min() >= 0


@pytest.mark.parametrize(
    "kwargs,expected",
    [
        ({"estop": True}, Mode.ESTOP),
        ({"sensor_age": 0.2}, Mode.FAULT),
        ({"heartbeat_age": 0.2}, Mode.FAULT),
        ({"gnss_age": 2}, Mode.DEGRADED),
    ],
)
def test_safety_faults(kwargs, expected):
    supervisor = Supervisor()
    state = np.array([0, 0, 0, 3, 0, 0, 0])
    assert supervisor.check(state, **kwargs) == expected
    if expected in (Mode.ESTOP, Mode.FAULT):
        assert supervisor.check(state) == expected
        with pytest.raises(RuntimeError):
            supervisor.arm()


def test_planner_inflates_obstacles_and_rejects_unknown():
    grid = np.zeros((15, 15))
    grid[3:12, 7] = 100
    grid[7, 7] = -1
    blocked = inflate(grid, 1)
    path = astar(grid, (1, 7), (13, 7))
    assert path[0] == (1, 7) and path[-1] == (13, 7)
    assert all(not blocked[y, x] for x, y in path)
    grid[:, 7] = -1
    assert not astar(grid, (1, 7), (13, 7))


@pytest.mark.parametrize(
    "route,controller", [("straight", "lqr"), ("curve", "lqr"), ("curve", "pd")]
)
def test_closed_loop_with_disturbance_and_dropout(tmp_path, route, controller):
    metrics = run({"duration": 15, "route": route, "controller": controller}, tmp_path)
    assert metrics["cross_track_rmse_m"] < 0.3
    assert metrics["max_abs_lean_rad"] < 0.1
    assert metrics["estimation_rmse"]["y"] < 0.2
    assert metrics["degraded_steps"] > 0


def test_low_speed_control_is_rejected():
    with pytest.raises(ValueError):
        Controller(Reference()).command(np.zeros(7))
