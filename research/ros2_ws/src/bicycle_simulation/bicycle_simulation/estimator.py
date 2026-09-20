import numpy as np
from bicycle_interfaces.msg import BicycleObservation, BicycleState
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

from pathfinder.runtime import Fusion, Observation, orientation, pose_covariance

from .common import ConfiguredNode, spin, stamp_ns


class EstimatorNode(ConfiguredNode):
    def __init__(self):
        super().__init__("bicycle_estimator")
        self.fusion = Fusion(self.config)
        self.state_pub = self.create_publisher(BicycleState, "state", 10)
        self.odom_pub = self.create_publisher(Odometry, "odometry", 10)
        self.attitude_pub = self.create_publisher(Imu, "sensors/attitude", 10)
        self.joints_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tf = TransformBroadcaster(self)
        self.wheel_angle = 0.0
        self.create_subscription(BicycleObservation, "observations", self.observe, 10)

    def observe(self, msg):
        if msg.header.frame_id != "map":
            self.get_logger().error("Rejected observation outside map frame")
            return
        observation = Observation(
            msg.sequence,
            stamp_ns(msg.header.stamp),
            np.array(msg.applied_command),
            np.array(msg.fast),
            np.array(msg.variance),
            np.array(msg.gnss_position) if msg.gnss_valid else None,
        )
        try:
            state = self.fusion.consume(observation)
        except ValueError as exc:
            # No output: plant watchdog halts rather than advancing from corrupt data.
            self.get_logger().error(str(exc))
            return
        output = BicycleState()
        output.header = msg.header
        output.sequence = msg.sequence
        output.state = state.tolist()
        output.covariance = self.fusion.filter.p.ravel().tolist()
        output.gnss_age = self.fusion.gnss_age
        self.state_pub.publish(output)
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = state[:2].tolist()
        q = orientation(state[2], state[4]).tolist()
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = q[:2]
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q[2:]
        odom.pose.covariance = pose_covariance(self.fusion.filter.p).ravel().tolist()
        odom.twist.twist.linear.x = float(state[3])
        odom.twist.twist.angular.x = float(-state[5])
        # Derived yaw rate. Covariance includes speed/steering Jacobian.
        p = self.fusion.filter.p
        wheelbase = self.fusion.filter.model.p.wheelbase
        yaw_rate = state[3] * np.tan(state[6]) / wheelbase
        odom.twist.twist.angular.y = float(-np.sin(state[4]) * yaw_rate)
        odom.twist.twist.angular.z = float(np.cos(state[4]) * yaw_rate)
        jac = np.zeros((6, 7))
        jac[0, 3], jac[3, 5] = 1, -1
        yaw_jac = np.zeros(7)
        yaw_jac[3] = np.tan(state[6]) / wheelbase
        yaw_jac[6] = state[3] / (wheelbase * np.cos(state[6]) ** 2)
        jac[4] = -np.sin(state[4]) * yaw_jac
        jac[4, 4] = -np.cos(state[4]) * yaw_rate
        jac[5] = np.cos(state[4]) * yaw_jac
        jac[5, 4] = -np.sin(state[4]) * yaw_rate
        covariance = jac @ p @ jac.T
        covariance[1, 1] = covariance[2, 2] = 1e6
        odom.twist.covariance = covariance.ravel().tolist()
        self.odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = odom.header, odom.child_frame_id
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)
        self.publish_sensors(msg)

    def publish_sensors(self, msg):
        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = "imu_link"
        q = orientation(msg.fast[0], msg.fast[2]).tolist()
        imu.orientation.x, imu.orientation.y = q[:2]
        imu.orientation.z, imu.orientation.w = q[2:]
        imu.orientation_covariance = [
            float(msg.variance[2]),
            0.0,
            0.0,
            0.0,
            1e6,
            0.0,
            0.0,
            0.0,
            float(msg.variance[0]),
        ]
        # This is an attitude solution only, NOT raw gyro/accelerometer data.
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        self.attitude_pub.publish(imu)
        joints = JointState()
        joints.header.stamp = msg.header.stamp
        joints.header.frame_id = "base_link"
        if msg.sequence > 0:
            self.wheel_angle += msg.fast[1] * self.fusion.dt_ns / 1e9 / 0.34
        joints.name = ["rear_wheel_joint", "steering_joint", "front_wheel_joint"]
        joints.position = [float(self.wheel_angle), float(msg.fast[4]), float(self.wheel_angle)]
        self.joints_pub.publish(joints)


def main():
    spin(EstimatorNode)
