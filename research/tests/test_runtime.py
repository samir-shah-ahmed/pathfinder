from dataclasses import replace

import numpy as np
import pytest

from pathfinder.control import Controller, Reference
from pathfinder.runtime import CommandGate, Fusion, Plant, orientation, pose_covariance
from pathfinder.safety import Mode, Supervisor
from pathfinder.simulation import Simulation


@pytest.mark.parametrize(
    "config",
    [
        {"dt": 0},
        {"dt": float("nan")},
        {"duration": 0.015},
        {"gnss_dropout": [3, 2]},
        {"duration": 0},
        {"dt": 0.0201},
    ],
)
def test_invalid_configuration(config):
    with pytest.raises(ValueError):
        Plant(config)


def test_runtime_and_standalone_are_identical():
    config = {"duration": 2, "seed": 11}
    plant, fusion = Plant(config), Fusion(config)
    state = fusion.consume(plant.observation)
    controller = Controller(Reference())
    standalone = Simulation(config)
    for sequence in range(1, 201):
        command = controller.command(state)
        observation = plant.step(command)
        state = fusion.consume(observation)
        truth, estimate, applied, mode = standalone.step()
        assert observation.stamp_ns == sequence * 10_000_000
        np.testing.assert_array_equal(truth, plant.truth)
        np.testing.assert_array_equal(estimate, state)
        np.testing.assert_array_equal(applied, command)
        assert mode == "ACTIVE"


def test_fusion_rejects_stale_missing_and_invalid_before_mutation():
    plant, fusion = Plant({}), Fusion({})
    first = plant.observation
    fusion.consume(first)
    prior = fusion.filter.x.copy()
    for invalid in (
        first,
        replace(first, sequence=2),
        replace(first, sequence=1, stamp_ns=1),
        replace(first, sequence=1, stamp_ns=10_000_000, fast=np.full(5, np.nan)),
    ):
        with pytest.raises(ValueError):
            fusion.consume(invalid)
        np.testing.assert_array_equal(fusion.filter.x, prior)
    assert fusion.last_sequence == 0


def test_gnss_outlier_does_not_refresh_freshness():
    plant, fusion = Plant({}), Fusion({})
    fusion.consume(plant.observation)
    observation = plant.step([0, 0])
    fusion.consume(replace(observation, gnss=np.array([10000, 10000])))
    assert fusion.gnss_age == pytest.approx(0.01)


def test_command_gate_rejects_invalid_and_replayed_commands():
    observation = Plant({}).observation
    gate = CommandGate()
    invalid = [
        (1, 0, [0, 0], 20_000_000),
        (0, 1, [0, 0], 20_000_000),
        (0, 0, [float("nan"), 0], 20_000_000),
        (0, 0, [1, 0], 20_000_000),
        (0, 0, [0, 0], 0),
        (0, 0, [0, 0], 1_000_000_000),
    ]
    for seq, stamp, command, validity in invalid:
        with pytest.raises(ValueError):
            gate.accept(seq, stamp, command, observation, validity)
    gate.accept(0, 0, [0, 0], observation, 20_000_000)
    with pytest.raises(ValueError, match="replayed"):
        gate.accept(0, 0, [0, 0], observation, 20_000_000)


def test_frame_and_covariance_signs():
    quaternion = orientation(0, 0.2)
    assert quaternion[0] < 0
    assert np.linalg.norm(quaternion) == pytest.approx(1)
    covariance = np.eye(7)
    covariance[0, 4] = covariance[4, 0] = 0.2
    mapped = pose_covariance(covariance)
    assert mapped[0, 3] == pytest.approx(-0.2)
    assert np.linalg.eigvalsh(mapped).min() > 0
    assert mapped[2, 2] == 1e6


def test_supervisor_never_arms_from_gnss_recovery():
    supervisor = Supervisor()
    state = [0, 0, 0, 3, 0, 0, 0]
    assert supervisor.check(state, gnss_age=2) == Mode.DEGRADED
    assert supervisor.check(state) == Mode.READY
    supervisor.arm()
    assert supervisor.check(state) == Mode.ACTIVE
    assert supervisor.check(state, gnss_age=2) == Mode.DEGRADED
    assert supervisor.check(state) == Mode.ACTIVE
