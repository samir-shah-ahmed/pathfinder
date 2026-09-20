# Status — 2026-09-20

Working: portable simulation with explicit lean dynamics, saturated steering,
PD/LQR control, EKF, synthetic measurements, GNSS dropout, disturbance injection,
CSV telemetry and JSON metrics. Standalone conservative grid A* is tested.

Verification on Windows, Python 3.13.5: **13 tests passed**, Ruff lint and formatting
passed. Default 20 s simulation completed 2,000 steps. SIMULATED cross-track RMSE:
0.073970 m; maximum absolute lean: 0.020283 rad. Results use one seed and a simplified
shared model, not physical performance. Full recorded metrics are in
`experiments/initial-metrics.json`.

Current failing core tests: none. Imported v1 GPU, firmware and ROS tests were not
run. CI is configured for the new core; this document does not assert CI completion.

Environment: Windows; NVIDIA RTX 3000 Ada laptop GPU detected; Docker, ROS 2,
Gazebo and colcon were not on PATH. WSL enumeration was access denied. No CUDA
toolkit or Jetson runtime compatibility is established by GPU detection.

Next milestone: reconcile v1 ROS interfaces and wrap the numerical core, then
validate against an independent bicycle model. High-fidelity contact simulation,
raw IMU fusion, constrained planning, MPC, perception integration and hardware
actuation remain unimplemented in the new baseline.
