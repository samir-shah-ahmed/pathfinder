FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-venv python3-colcon-common-extensions \
    ros-jazzy-tf2-ros-py ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 ros-jazzy-rosbag2 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
RUN python3 -m venv --system-site-packages /opt/pathfinder-venv
ENV VIRTUAL_ENV=/opt/pathfinder-venv
ENV PATH="/opt/pathfinder-venv/bin:${PATH}"
COPY research /workspace/research
# Apt ROS modules remain visible; pinned Python dependencies live in the venv.
RUN python3 -m pip install -e '/workspace/research[dev]' \
    && bash research/scripts/build_ros.sh
COPY research/docker/entrypoint.sh /pathfinder-entrypoint.sh
ENTRYPOINT ["bash", "/pathfinder-entrypoint.sh"]
CMD ["ros2", "launch", "bicycle_simulation", "simulation.launch.py"]
