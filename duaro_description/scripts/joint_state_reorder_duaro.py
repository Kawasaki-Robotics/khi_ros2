#!/usr/bin/env python3
"""
Relay node: subscribes to /joint_states_raw (IsaacSim order),
reorders joints to URDF order for the Duaro WD002N,
then republishes to /joint_states for MoveIt / controllers.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Active joints in URDF order (WD002N: 6 revolute + 2 prismatic)
URDF_ORDER = [
    "lower_joint1", "lower_joint2", "lower_joint3", "lower_joint4",
    "upper_joint1", "upper_joint2", "upper_joint3", "upper_joint4",
]


class JointStateReorderDuaro(Node):
    def __init__(self):
        super().__init__("joint_state_reorder_duaro")
        self.sub = self.create_subscription(
            JointState, "/joint_states_raw", self.cb, 10
        )
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.get_logger().info(
            "Duaro joint state reorder: /joint_states_raw -> /joint_states"
        )

    def cb(self, msg):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        out = JointState()
        out.header = msg.header
        for jn in URDF_ORDER:
            idx = name_to_idx.get(jn)
            if idx is None:
                continue
            out.name.append(jn)
            out.position.append(msg.position[idx] if idx < len(msg.position) else 0.0)
            if msg.velocity:
                out.velocity.append(msg.velocity[idx] if idx < len(msg.velocity) else 0.0)
            if msg.effort:
                out.effort.append(msg.effort[idx] if idx < len(msg.effort) else 0.0)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = JointStateReorderDuaro()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
