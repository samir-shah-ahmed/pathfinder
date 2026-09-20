# Simulation research baseline

New work created from `../PROJECT_SPECIFICATION.txt`. This directory is independent
of the imported v1 runtime. No physical actuation is connected.

## Run

Python 3.10–3.13, with the pinned NumPy/SciPy wheels:

```sh
cd research
python -m venv .venv
# Linux: source .venv/bin/activate
# PowerShell: .\.venv\Scripts\Activate.ps1
python -m pip install -e '.[dev]'
python -m pathfinder.simulation
python -m pytest
```

The default simulation produces `runs/latest/telemetry.csv`, `metrics.json`, and
the exact configuration. All observations and results are **SIMULATED**.
Edit `config/simulation.json` to select `straight`/`curve`, `lqr`/`pd`, a random
seed, a lean disturbance, or a GNSS dropout interval. Units are SI.

## Implemented

- Nonlinear reduced bicycle model, RK4 integration, actuator saturation/rate limits.
- Speed-scheduled LQR lean regulation, PD comparison, slow path guidance.
- Seven-state EKF, wrapped heading innovation, Joseph covariance update, outlier gate.
- Synthetic GNSS position, attitude solution, wheel and steering measurements.
- Simulation fault supervisor with latched fault/estop, timeout and lean checks.
- Standalone conservative grid A* with obstacle inflation and unknown-space rejection.
- Seeded telemetry, tracking/estimation metrics, unit and closed-loop tests.

## Model and limits

State: `[x, y, yaw, speed, lean_left, lean_rate_left, steering_left]`.
`x_dot=v cos(yaw)`, `y_dot=v sin(yaw)`, `yaw_dot=v tan(delta)/L`.
`lean_ddot=(g sin(lean)-v² tan(delta) cos(lean)/L)/h-damping*lean_rate`.
Steering uses a first-order servo with bounded angle and angular rate.
Lean is positive **left**; ROS right-handed roll would be its negative.

This is a reduced inverted-pendulum model, not the Whipple benchmark or a
validated tire/contact model. No trail, wheel gyroscopic dynamics, slip,
terrain/contact physics, stopping stability, or stationary balancing is modeled.
The simulation starts at 3 m/s; steering control below 1 m/s is rejected.
The filter sees an idealized attitude solution, not raw accelerometer/gyro fusion.
Truth and estimator share a model; reported errors are not hardware predictions.
GNSS coordinates are local metric positions, not latitude/longitude.

The supervisor halts the simulator on a fault. It does not implement a physical
safe stop or certified safety system. A* returns grid paths, not dynamically
feasible trajectories, and is not connected to the tracking demonstration.

ROS publishing, Gazebo contact simulation, camera rendering, real perception
integration, MPC, MCU watchdogs and physical testing remain future work.
Existing v1 ROS/perception code is preserved in the repository root.
