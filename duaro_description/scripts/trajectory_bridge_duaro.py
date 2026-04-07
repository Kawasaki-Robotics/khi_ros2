#!/usr/bin/env python3
"""
Trajectory bridge: provides FollowJointTrajectory action servers for the
Duaro WD002N's two arm controllers and publishes position commands to
Isaac Sim's /joint_commands topic.

Action servers:
  /duaro_lower_arm_controller/follow_joint_trajectory
  /duaro_upper_arm_controller/follow_joint_trajectory

Publishes to: /joint_commands (sensor_msgs/JointState)
Subscribes to: /joint_states (sensor_msgs/JointState) for feedback
"""
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def lerp_positions(p0, p1, alpha):
    """Linearly interpolate between two position vectors."""
    return [a + alpha * (b - a) for a, b in zip(p0, p1)]


class TrajectoryBridgeDuaro(Node):
    PUBLISH_RATE = 200.0  # Hz, matches f_duaro controller rate

    def __init__(self):
        super().__init__("trajectory_bridge_duaro")

        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._current_positions = {}

        self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10,
            callback_group=self._cb_group,
        )

        self._cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)

        self._lower_action = ActionServer(
            self,
            FollowJointTrajectory,
            "/duaro_lower_arm_controller/follow_joint_trajectory",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        self._upper_action = ActionServer(
            self,
            FollowJointTrajectory,
            "/duaro_upper_arm_controller/follow_joint_trajectory",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            "Duaro trajectory bridge ready. Action servers:\n"
            "  /duaro_lower_arm_controller/follow_joint_trajectory\n"
            "  /duaro_upper_arm_controller/follow_joint_trajectory\n"
            "Publishing to: /joint_commands"
        )

    def _joint_state_cb(self, msg):
        with self._lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._current_positions[name] = msg.position[i]

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        joint_names = list(trajectory.joint_names)
        points = trajectory.points

        if not points:
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        self.get_logger().info(
            f"Executing trajectory: {len(points)} points, joints: {joint_names}"
        )

        with self._lock:
            start_positions = [
                self._current_positions.get(jn, 0.0) for jn in joint_names
            ]

        dt = 1.0 / self.PUBLISH_RATE
        t0 = time.monotonic()

        all_points = [JointTrajectoryPoint(positions=start_positions)]
        all_points[0].time_from_start.sec = 0
        all_points[0].time_from_start.nanosec = 0
        all_points.extend(points)

        total_duration = (
            all_points[-1].time_from_start.sec
            + all_points[-1].time_from_start.nanosec * 1e-9
        )

        seg_idx = 0
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            elapsed = time.monotonic() - t0
            if elapsed >= total_duration:
                break

            while (
                seg_idx < len(all_points) - 2
                and elapsed >= (
                    all_points[seg_idx + 1].time_from_start.sec
                    + all_points[seg_idx + 1].time_from_start.nanosec * 1e-9
                )
            ):
                seg_idx += 1

            t_seg_start = (
                all_points[seg_idx].time_from_start.sec
                + all_points[seg_idx].time_from_start.nanosec * 1e-9
            )
            t_seg_end = (
                all_points[seg_idx + 1].time_from_start.sec
                + all_points[seg_idx + 1].time_from_start.nanosec * 1e-9
            )
            seg_duration = t_seg_end - t_seg_start
            alpha = min(1.0, max(0.0, (elapsed - t_seg_start) / seg_duration)) if seg_duration > 0 else 1.0

            positions = lerp_positions(
                list(all_points[seg_idx].positions),
                list(all_points[seg_idx + 1].positions),
                alpha,
            )

            cmd = JointState()
            cmd.name = joint_names
            cmd.position = positions
            self._cmd_pub.publish(cmd)

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = joint_names
            feedback.desired.positions = positions
            with self._lock:
                feedback.actual.positions = [
                    self._current_positions.get(jn, 0.0) for jn in joint_names
                ]
            goal_handle.publish_feedback(feedback)

            time.sleep(dt)

        # Send final point
        cmd = JointState()
        cmd.name = joint_names
        cmd.position = list(all_points[-1].positions)
        self._cmd_pub.publish(cmd)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info("Trajectory execution complete.")
        return result


def main():
    rclpy.init()
    node = TrajectoryBridgeDuaro()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
