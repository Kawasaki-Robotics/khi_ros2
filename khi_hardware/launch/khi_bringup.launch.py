# Copyright 2025 Kawasaki Heavy Industries, Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description=(
                "Prefix of the joint names, useful for multi-robot setup. "
                "If changed, the joint names in khi_controllers.yaml also need to be updated."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot",
            choices=[
                "rs007l-b001",
                "rs015x-a001",
                "rs013n-a001",
                "rs025n-a001",
                "rs080n-a001",
                "bx300l-b001",
                "bxp135x-a001",
            ],
            description="robot name",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_no",
            choices=[
                "0",
                # "0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
                # "10", "11", "12", "13", "14", "15"
            ],
            default_value="0",
            description=(
                "If connecting multiple controllers from a single PC, "
                "Please change the robot controller number."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            choices=[
                "f",
            ],
            default_value="f",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "simulation",
            choices=["true", "false"],
            default_value="false",
            description=(
                "If you have no real robot, specify the argument 'simulation' to use loopback mode"
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.0.2",
            description="ip address",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "actual_current",
            choices=["true", "false"],
            default_value="false",
            description="true: Enables periodic acquisition of actual current values.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "command_current",
            choices=["true", "false"],
            default_value="false",
            description="true: Enables periodic acquisition of command current values.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "actual_encorder",
            choices=["true", "false"],
            default_value="false",
            description="true: Enables periodic acquisition of actual encorder values.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "command_encorder",
            choices=["true", "false"],
            default_value="false",
            description="true: Enables periodic acquisition of command encorder values.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tcp_info",
            choices=["true", "false"],
            default_value="false",
            description=(
                "true: Enables periodic acquisition of the actual position, actual velocity, "
                "      command position, command velocity of the tool center point (TCP)."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "external_signal",
            choices=["true", "false"],
            default_value="false",
            description="true: Enables periodic acquisition of external output/input signal.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "internal_signal",
            choices=["true", "false"],
            default_value="false",
            description="true: Enables periodic acquisition of internal signal.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ft_sensor",
            choices=["true", "false"],
            default_value="false",
            description="true: Enables periodic acquisition of data from ATI F/T sensors.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "update_rate_yaml",
            default_value=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("khi_description"),
                        "config",
                        "update_rate",
                    ]
                ),
                "/",
                LaunchConfiguration("robot_controller"),
                "_controller_update_rate.yaml",
            ],
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(contest, *args, **kwargs):
    prefix = LaunchConfiguration("prefix")
    robot = LaunchConfiguration("robot")
    controller_no = LaunchConfiguration("controller_no")
    simulation = LaunchConfiguration("simulation")
    robot_ip = LaunchConfiguration("robot_ip")
    actual_current = LaunchConfiguration("actual_current")
    command_current = LaunchConfiguration("command_current")
    actual_encorder = LaunchConfiguration("actual_encorder")
    command_encorder = LaunchConfiguration("command_encorder")
    tcp_info = LaunchConfiguration("tcp_info")
    external_signal = LaunchConfiguration("external_signal")
    internal_signal = LaunchConfiguration("internal_signal")
    ft_sensor = LaunchConfiguration("ft_sensor")
    robot_controller = LaunchConfiguration("robot_controller")
    update_rate_yaml = LaunchConfiguration("update_rate_yaml")

    robot_series = ""
    if "rs" in str(robot.perform(contest)):
        robot_series = "rs"
    if "bx" in str(robot.perform(contest)):
        robot_series = "bx"
    if "bxp" in str(robot.perform(contest)):
        robot_series = "bxp"

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("khi_description"), "urdf", "khi.urdf.xacro"]),
            " controller_no:=",
            controller_no,
            " robot_ip:=",
            robot_ip,
            " prefix:=",
            prefix,
            " robot_controller:=",
            robot_controller,
            " robot_name:=",
            robot,
            " robot_series:=",
            robot_series,
            " simulation:=",
            simulation,
            " actual_current:=",
            actual_current,
            " command_current:=",
            command_current,
            " actual_encorder:=",
            actual_encorder,
            " command_encorder:=",
            command_encorder,
            " tcp_info:=",
            tcp_info,
            " external_signal:=",
            external_signal,
            " internal_signal:=",
            internal_signal,
            " ft_sensor:=",
            ft_sensor,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ros2_control_node
    khi_controllers = PathJoinSubstitution(
        [
            FindPackageShare("khi_description"),
            "config",
            robot_series,
            "khi_controllers.yaml",
        ]
    )
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, khi_controllers, update_rate_yaml],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    spawn_khi_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["khi_controller", "--controller-manager", "/controller_manager"],
    )

    nodes = [
        controller_manager_node,
        robot_state_publisher_node,
        spawn_joint_state_broadcaster,
        spawn_khi_controller,
    ]

    return nodes
