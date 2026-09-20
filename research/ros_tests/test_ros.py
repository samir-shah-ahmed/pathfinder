"""Real ROS process tests. Run after colcon build + sourcing install/setup.bash."""

import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest
import rclpy
from bicycle_interfaces.msg import BicycleState
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from pathfinder.runtime import orientation


class Session:
    def __init__(self, folder, external_controller=False):
        self.folder = folder
        folder.mkdir(parents=True, exist_ok=True)
        config = {
            "duration": 13,
            "dt": 0.01,
            "seed": 7,
            "route": "curve",
            "disturbance": True,
            "gnss_dropout": [8, 11],
        }
        self.config = folder / "input.json"
        self.config.write_text(json.dumps(config))
        self.processes, self.files = [], []
        self.output = folder / "recording"
        rclpy.init()
        self.node = rclpy.create_node("integration_probe")
        self.states, self.odometry, self.clocks, self.transforms, self.statuses = [], [], [], [], []
        self.node.create_subscription(BicycleState, "/bicycle/state", self.states.append, 100)
        self.node.create_subscription(Odometry, "/bicycle/odometry", self.odometry.append, 100)
        self.node.create_subscription(Clock, "/clock", self.clocks.append, 100)
        self.node.create_subscription(TFMessage, "/tf", self.transforms.append, 100)
        self.node.create_subscription(
            String,
            "/bicycle/status",
            self.statuses.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.launch = self.start(
            [
                "ros2",
                "launch",
                "bicycle_simulation",
                "simulation.launch.py",
                "config:=" + str(self.config),
                "output:=" + str(self.output),
                "real_time_factor:=3.0",
                "watchdog_seconds:=2.0",
                "controller:=" + str(not external_controller).lower(),
            ],
            "launch",
        )
        self.controller = None
        if external_controller:
            self.controller = self.start(
                [
                    "ros2",
                    "run",
                    "bicycle_simulation",
                    "controller",
                    "--ros-args",
                    "-r",
                    "__ns:=/bicycle",
                    "-p",
                    "config:=" + str(self.config),
                    "-p",
                    "use_sim_time:=true",
                ],
                "controller",
            )

    def start(self, command, name):
        log = (self.folder / (name + ".log")).open("w")
        self.files.append(log)
        process = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
        )
        self.processes.append(process)
        return process

    def wait(self, condition, timeout=60):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if condition():
                return
            if self.launch.poll() is not None:
                pytest.fail(
                    "Launch exited before condition; see " + str(self.folder / "launch.log")
                )
        pytest.fail("Timed out; see " + str(self.folder / "launch.log"))

    def metrics(self):
        self.wait(lambda: (self.output / "metrics.json").exists())
        return json.loads((self.output / "metrics.json").read_text())

    def close(self):
        for process in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        self.node.destroy_node()
        rclpy.shutdown()
        for stream in self.files:
            stream.close()


@pytest.fixture
def session_factory(tmp_path):
    sessions = []

    def create(external_controller=False):
        # Each test receives an isolated DDS domain, including all subprocesses.
        os.environ["ROS_DOMAIN_ID"] = str(70 + len(list(tmp_path.parent.iterdir())) % 100)
        folder = Path(os.environ.get("ROS_TEST_OUTPUT", str(tmp_path.parent))) / tmp_path.name
        session = Session(folder, external_controller)
        sessions.append(session)
        return session

    yield create
    for session in sessions:
        session.close()


def test_ros_closed_loop_and_frames(session_factory):
    session = session_factory()
    metrics = session.metrics()
    assert metrics["mode"] == "COMPLETE"
    assert metrics["samples"] == 1301
    assert metrics["all_samples_paired"]
    assert metrics["cross_track_rmse_m"] < 0.3
    assert metrics["max_abs_lean_rad"] < 0.1
    assert metrics["degraded_samples"] > 0
    assert session.clocks and session.odometry and session.transforms
    assert any(
        t.child_frame_id == "base_link" and t.header.frame_id == "map"
        for msg in session.transforms
        for t in msg.transforms
    )
    states = {s.header.stamp.sec * 10**9 + s.header.stamp.nanosec: s for s in session.states}
    matched = 0
    for odom in session.odometry:
        stamp = odom.header.stamp.sec * 10**9 + odom.header.stamp.nanosec
        if stamp not in states:
            continue
        state = states[stamp]
        q = odom.pose.pose.orientation
        np.testing.assert_allclose(
            [q.x, q.y, q.z, q.w], orientation(state.state[2], state.state[4])
        )
        assert odom.header.frame_id == "map" and odom.child_frame_id == "base_link"
        matched += 1
    assert matched > 100
    session.launch.wait(timeout=15)
    assert session.launch.returncode == 0


def test_controller_loss_triggers_wall_clock_watchdog(session_factory):
    session = session_factory(external_controller=True)
    session.wait(lambda: len(session.states) > 50)
    os.killpg(session.controller.pid, signal.SIGTERM)
    metrics = session.metrics()
    assert metrics["mode"] == "FAULT"
    assert "watchdog" in metrics["reason"]
    assert metrics["sequence"] < 1300


def test_emergency_stop_latches(session_factory):
    session = session_factory()
    session.wait(lambda: len(session.states) > 50)
    client = session.node.create_client(Trigger, "/bicycle/estop")
    assert client.wait_for_service(timeout_sec=5)
    future = client.call_async(Trigger.Request())
    session.wait(future.done)
    assert future.result().success
    metrics = session.metrics()
    assert metrics["mode"] == "ESTOP"
    assert metrics["sequence"] < 1300
