#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$ROOT/ros2_ws"
colcon build --symlink-install --event-handlers console_direct+
echo "Build complete. Source $ROOT/ros2_ws/install/setup.bash before launching."
