# Engineering decisions — 2026-09-20

| Decision | Alternatives | Reason and tradeoff |
|---|---|---|
| Preserve v1; add isolated numerical core | Replace v1 | Retains existing work; integration remains explicit future work. |
| Reduced nonlinear dynamics first | Immediate full multibody simulation | Runs and tests on this Windows host; cannot establish physical stability. |
| LQR plus PD comparison | MPC/RL first | Deterministic inspectable baseline; not a constraints-aware predictive planner. |
| EKF with Joseph update | UKF/factor graph | Small baseline with consistent covariance update; no sensor bias states yet. |
| Gazebo Harmonic / ROS Jazzy / Ubuntu 24.04 as future integration target | Isaac Sim | Official supported pairing and ROS integration; no local Gazebo verification. Isaac remains an option for visual datasets. |
| Keep Jetson runtime separate | Assume desktop stack works on Jetson | JetPack 6.2 is based on Ubuntu 22.04; do not claim Jazzy native compatibility without a deployment validation. |
| Lockstep ROS numerical simulation before Gazebo | Free-running asynchronous numerical loop | Each command acknowledges one sensor frame, making ordering and transport parity testable. Does not demonstrate real-time deadlines. |
| Isolated research ROS workspace | Build all v1 packages together | Existing v1 dependencies and unstamped topics differ; integration needs explicit calibration and timestamp adapters. |
| Wall-clock command watchdog | Simulation-clock timeout | A stalled /clock must not disable timeout detection. Freezes the numerical experiment, not a physical safe stop. |
| One ROS adapter package with four executable processes | Four nearly empty packages | Preserves process boundaries without duplicating packaging; split packages when independent deployment requires it. |
| Validate fixed-geometry Whipple benchmark separately | Treat the reduced model as a physical digital twin | Independent published eigenvalues catch dynamics implementation errors. Torque input differs from the reduced servo interface; no controller transfer is claimed. |
| System-visible virtualenv inside ROS container | Upgrade apt-owned Python packages with pip | First CI build exposed an uninstall conflict on apt pluggy; venv isolates pinned dependencies while retaining ROS modules. |

No hardware purchases or final communications bus are selected. The imported
ESP32 branch is historical v1 firmware, not validated for the new core.

## Sources

[1] A. Nindra, “Pathfinder: Autonomous Bicycle,” accessed Sep. 20, 2026.
https://amannindra.com/projects/autonomous-bicycle/

[2] J. P. Meijaard et al., “Linearized dynamics equations for the balance and
steer of a bicycle: a benchmark and review,” Proc. R. Soc. A, vol. 463,
pp. 1955–1982, 2007, doi:10.1098/rspa.2007.1857.
https://arendschwab.com/assets/pdf/meijaard2007linearized.pdf

[3] G. M. Hoffmann et al., “Autonomous Automobile Trajectory Tracking for
Off-Road Driving: Controller Design, Experimental Validation and Racing,” 2007.
https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf

[4] Gazebo, “Installing Gazebo with ROS,” accessed Sep. 20, 2026.
https://gazebosim.org/docs/harmonic/ros_installation/

[5] NVIDIA, “Isaac Sim Requirements,” accessed Sep. 20, 2026.
https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html

[6] NVIDIA, “JetPack SDK 6.2,” accessed Sep. 20, 2026.
https://developer.nvidia.com/embedded/jetpack-sdk-62

The reduced model is not a reproduction of [2]; [2] is the intended next
dynamics validation reference. [3] motivates separate path feedback; this core
does not claim to implement the full Stanley controller.
