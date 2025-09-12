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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
            description="Prefix of the joint names, useful for multi-robot setup. "
            + "If changed, the joint names in khi_controllers.yaml also need to be updated.",
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
    robot_controller = LaunchConfiguration("robot_controller")

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
            " prefix:=",
            prefix,
            " robot_controller:=",
            robot_controller,
            " robot_name:=",
            robot,
            " robot_series:=",
            robot_series,
            " use_gazebo:=",
            "true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    use_sim_time = {
        "use_sim_time": True,
    }
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[use_sim_time, robot_description],
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

    # gazebo classic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
    )
    spawn_gazebo = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", robot, "-topic", "/robot_description"],
        output="screen",
    )

    nodes = [
        gazebo,
        robot_state_publisher_node,
        spawn_gazebo,
        spawn_joint_state_broadcaster,
        spawn_khi_controller,
    ]

    return nodes
