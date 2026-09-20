#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$ROOT/ros2_ws"
python3 "$(command -v colcon)" build --symlink-install --event-handlers console_direct+ \
    --cmake-args "-DPython3_EXECUTABLE=$(command -v python3)"
echo "Build complete. Source $ROOT/ros2_ws/install/setup.bash before launching."
