import json
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from pathfinder.runtime import validate_config

LATCHED = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def set_stamp(stamp, ns):
    stamp.sec, stamp.nanosec = divmod(int(ns), 1_000_000_000)


class ConfiguredNode(Node):
    def __init__(self, name):
        super().__init__(name)
        path = self.declare_parameter("config", "").value
        if not path:
            raise ValueError("The config parameter must name a simulation JSON file")
        self.config = json.loads(Path(path).read_text())
        validate_config(self.config)


def spin(node_class):
    rclpy.init()
    node = None
    try:
        node = node_class()
        while rclpy.ok() and not getattr(node, "finished", False):
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
