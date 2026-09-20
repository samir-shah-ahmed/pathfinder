import numpy as np
import pytest
from scipy.integrate import solve_ivp

from pathfinder.whipple import WhippleBenchmark, stability_boundaries


# Independent reference values transcribed from Meijaard et al. (2007), Table 2.
@pytest.mark.parametrize(
    "speed,expected",
    [
        (0, [-5.53094371765393, -3.13164324790656, 3.13164324790656, 5.53094371765393]),
        (
            4,
            [
                -12.15861426576447,
                -1.42944427361326,
                0.41325331521125 - 3.07910818603206j,
                0.41325331521125 + 3.07910818603206j,
            ],
        ),
        (
            5,
            [
                -14.07838969279822,
                -0.32286642900409,
                -0.77534188219585 - 4.46486771378823j,
                -0.77534188219585 + 4.46486771378823j,
            ],
        ),
        (
            10,
            [
                -24.62459635017404,
                0.16105338653172,
                -3.72016840437287 - 10.90681139476287j,
                -3.72016840437287 + 10.90681139476287j,
            ],
        ),
    ],
)
def test_published_eigenvalues(speed, expected):
    np.testing.assert_allclose(
        np.sort_complex(WhippleBenchmark(speed).eigenvalues()),
        np.sort_complex(expected),
        atol=1e-10,
        rtol=0,
    )


def test_published_self_stability_boundaries():
    np.testing.assert_allclose(
        stability_boundaries(), [4.29238253634111, 6.02426201538837], atol=1e-10, rtol=0
    )
    assert np.max(WhippleBenchmark(3).eigenvalues().real) > 0
    assert np.max(WhippleBenchmark(5).eigenvalues().real) < 0
    assert np.max(WhippleBenchmark(7).eigenvalues().real) > 0


def test_forced_response_against_independent_integrator():
    model = WhippleBenchmark(5)
    state = np.array([0.01, -0.005, 0, 0])
    torque = np.array([0.0, 0.1])

    def equation(t, z):
        del t
        acceleration = np.linalg.solve(
            model.mass, torque - 5 * model.c1 @ z[2:] - (9.81 * model.k0 + 25 * model.k2) @ z[:2]
        )
        return np.concatenate([z[2:], acceleration])

    reference = solve_ivp(equation, (0, 1), state, rtol=1e-11, atol=1e-13)
    for _ in range(100):
        state = model.step(state, torque, 0.01)
    np.testing.assert_allclose(state, reference.y[:, -1], atol=1e-10, rtol=0)


def test_invalid_inputs():
    with pytest.raises(ValueError):
        WhippleBenchmark(-1)
    with pytest.raises(ValueError):
        WhippleBenchmark().step(np.zeros(4), [0, float("nan")], 0.01)
    with pytest.raises(ValueError):
        WhippleBenchmark().discretize(0)
