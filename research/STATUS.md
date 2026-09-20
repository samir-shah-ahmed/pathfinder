# Status — 2026-09-20

## Working and verified

- Portable model/control/EKF experiment with timestamped synthetic observations,
  disturbances, GNSS dropout, telemetry and metrics.
- ROS Jazzy custom interfaces and four separate plant/estimator/controller/recorder
  processes. Docker image builds both ROS packages.
- Odometry, correctly signed lean/roll covariance, TF, joint states, visual URDF,
  and RViz configuration. GUI rendering itself was not exercised.
- Lockstep sequence checks, finite/bounded commands, wall-clock command timeout,
  and latched simulated emergency stop.
- Independent fixed-geometry Whipple linear dynamics implementation reproduces
  published eigenvalues and stability boundaries; it is not a hardware model.

## Verification evidence

Windows Python 3.13.5: **32 portable tests passed**, Ruff lint and formatting passed.
The 20 s portable experiment completed 2,000 steps, with **simulated** cross-track
RMSE 0.093989 m and maximum lean 0.020076 rad. The changed initial sensor sample and
aligned acquisition timestamps account for the difference from the first baseline.
See `experiments/ros-milestone-portable-metrics.json`; the original result is retained
in `experiments/initial-metrics.json` as historical evidence.

Ubuntu/ROS CI: Docker image and generated messages built; **3 real-process tests
passed**, covering closed-loop motion/TF, controller loss, and emergency stop.
The 13 s ROS trial recorded all **1,301** truth/estimate pairs, simulated cross-track
RMSE **0.082498 m**, and 219 samples with GNSS age over one second. Controller loss
and estop froze the simulation and produced partial-result reports.
[Verified run](https://github.com/samir-shah-ahmed/pathfinder/actions/runs/35498891488).
Recorded metrics and provenance are under `experiments/ros-ci/`.
The integration suite also checks numerical parity with the standalone core.

Whipple reference: four published speed/eigenvalue cases pass to 1e-10 absolute
tolerance. Stability-boundary errors are below 6e-14 m/s using the published
matrices. See `experiments/whipple-metrics.json` and `docs/DYNAMICS_VALIDATION.md`.
These are numerical implementation checks, not measurements of hardware accuracy.

## Limits and next work

Current failing portable tests: none. Imported v1 GPU, firmware and ROS tests were
not run. The reduced simulation shares its model with the EKF and uses an idealized
attitude solution. It does not test raw IMU biases, sensor latency, tire slip,
real-time deadlines, stopping balance, or autonomous physical operation.

The installed WSL Ubuntu is 20.04/Python 3.8 and has no ROS; local Windows has no
Docker or ROS on PATH. ROS verification therefore runs in GitHub's Ubuntu Docker
environment. GPU detection does not establish CUDA/Jetson compatibility.

Next milestone: measured bicycle/servo parameters and independent nonlinear
contact-model validation, followed by stamped, calibrated v1 perception adapters.
Metric traversability, feasible obstacle-avoidance trajectories, MPC, hardware
watchdogs and physical actuation remain future milestones.
