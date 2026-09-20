# Independent linear dynamics reference

`pathfinder.whipple.WhippleBenchmark` implements the fixed-geometry linear
benchmark in Meijaard et al. [1]. It complements the reduced nonlinear model.
It does **not** validate the physical Pathfinder bicycle or its controller.

The benchmark uses coupled lean and steering:

`M q_ddot + v C1 q_dot + (g K0 + v² K2) q = torque`.

Matrices are from [1], equations 6.1–6.4. State order is lean, steer, lean rate,
steer rate; speed is held constant. Angles use the paper's canonical convention,
not the ROS adapter's internal left-positive state. Inputs are lean and steering
**torques** in N m, not steering angle commands. It must not be inserted into the
current controller loop without an explicit servo and coordinate adapter.

Tests compare eigenvalues at 0, 4, 5 and 10 m/s with Table 2 of [1] and check its
two stability boundaries to an absolute tolerance of 1e-10 m/s. A separate
adaptive ODE solver checks the zero-order-hold forced-response implementation.

```sh
cd research
python -m pathfinder.whipple
```

This writes a speed/eigenvalue CSV and boundary-error JSON under `runs/whipple`.
The model has ideal rolling contact and small-angle linearization. It excludes
tire slip, rough terrain, finite-angle falls, actuator saturation and braking.
It uses the published geometry, not measured project hardware parameters.
Measured mass/inertia/geometry and nonlinear contact validation remain next steps.

[1] J. P. Meijaard, J. M. Papadopoulos, A. Ruina, and A. L. Schwab,
“Linearized dynamics equations for the balance and steer of a bicycle: a benchmark
and review,” *Proc. R. Soc. A*, vol. 463, pp. 1955–1982, 2007.
[doi:10.1098/rspa.2007.1857](https://doi.org/10.1098/rspa.2007.1857).
[Author-hosted paper](https://arendschwab.com/assets/pdf/meijaard2007linearized.pdf).
