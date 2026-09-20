# Pathfinder

Autonomous bicycle research project, based on the v1 codebase from
[Duck1405/Autonomous-Bicycle](https://github.com/Duck1405/Autonomous-Bicycle).

## Existing v1 code

The root retains the imported perception, Jetson inference, ROS workspaces,
notebooks, configuration, experiments, and original third-party license files.
See [the original README](README.v1.md), [skills.md](skills.md), and
[TensorNotes.md](TensorNotes.md) for the upstream implementation and recorded results.
Those historical results were not independently reproduced during this import.

All six source branch tips are imported as **clean snapshots** under their original
branch names, including `S3-code` for the ESP32-S3 firmware. Historical commits are
not copied because they contain exposed credentials. See [IMPORT.md](IMPORT.md)
for source commits and the exact exclusions/redactions.

## New simulation research

The independent [research](research/README.md) directory adds a reduced bicycle
dynamics model, PD/LQR balance control, EKF, seeded synthetic sensors, telemetry,
fault checks, grid-planning baseline, and tests. It is not wired into v1 ROS or
physical hardware. See [research/STATUS.md](research/STATUS.md) for verified results
and [research/ROADMAP.md](research/ROADMAP.md) for remaining work.

```sh
cd research
python -m pip install -e '.[dev]'
python -m pathfinder.simulation
python -m pytest
```

Use a virtual environment; full instructions are in the research README.
The complete requested scope is preserved in [PROJECT_SPECIFICATION.txt](PROJECT_SPECIFICATION.txt).
This repository does not establish safe autonomous operation of a physical bicycle.

## Licensing and attribution

Original notices and component licenses are preserved. There was no repository-wide
LICENSE at the imported main revision; this import does not relicense upstream code
or grant additional rights. Review each component's terms before redistribution or use.
Model weights, datasets, and other files excluded by upstream Git are not supplied.
