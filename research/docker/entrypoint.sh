#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /workspace/research/ros2_ws/install/setup.bash
exec "$@"
