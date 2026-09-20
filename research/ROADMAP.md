# Roadmap

Legend: [x] complete, [~] in progress, [ ] not started, [!] blocked here.

- [x] Modular numerical core and preserved project specification.
- [x] Reduced nonlinear model and model tests.
- [x] Seeded simulation, synthetic observations, telemetry and metrics.
- [x] PD/LQR lean control and analytic path guidance.
- [x] Seven-state EKF baseline; GNSS dropout experiment.
- [x] Grid A* baseline with conservative unknown-space handling.
- [x] Simulation fault supervisor and failure tests.
- [x] ROS message interfaces, separate plant/estimator/controller/recorder processes.
- [x] Odometry, TF, visual URDF, RViz configuration, acquisition-stamped recording.
- [x] Portable sequence/freshness/command/frame tests; ROS process tests implemented.
- [x] ROS integration execution: Docker build and 3 real-process tests passed in CI.
- [ ] Gazebo contact dynamics and rendered sensor simulation.
- [x] Fixed-geometry Whipple linear benchmark; independent published-eigenvalue checks.
- [ ] Measured project geometry, servo adapter and nonlinear multibody/contact validation.
- [ ] Raw IMU/bias estimation, delayed measurements, VIO and RTK fusion.
- [ ] Connect and test v1 perception against explicit output contracts.
- [ ] Metric depth, BEV and semantic traversability integration.
- [ ] Curvature/lean/obstacle constrained trajectory generation.
- [ ] MPC benchmark and independent controller validation.
- [ ] MCU protocol, watchdog, hardware safe-state design and HIL.
- [ ] Dataset synchronization, annotations, rosbag pipeline.
- [ ] Reproduce v1 Jetson benchmarks on the actual device.
- [ ] Physical actuation and closed-course evaluation after safety review.
- [ ] Simulation-only RL and domain randomization after deterministic baselines.
