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

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    print(absolute_file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description=(
                "Prefix of the joint names, useful for multi-robot setup. "
                "If changed, the joint names in khi_controllers.yaml, joint_limits.yaml"
                " and controllers.yaml also need to be updated."
                "Additionaly, the manipulator names in the ompl_planning.yaml"
                " and kinematics.yaml also need to be updated.",
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
            "use_rviz",
            default_value="true",
            description="GUI flag",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "planner",
            default_value="ompl",
            choices=["ompl", "pilz"],
            description="GUI flag",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true when using Gazebo.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="false",
            description="When using Gazebo Classic, please set use_gazebo to true.",
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration("prefix")
    robot = LaunchConfiguration("robot")
    controller_no = LaunchConfiguration("controller_no")
    planner = LaunchConfiguration("planner")
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_controller = LaunchConfiguration("robot_controller")

    robot_series = ""
    if "rs" in str(robot.perform(context)):
        robot_series = "rs"
    if "bx" in str(robot.perform(context)):
        robot_series = "bx"
    if "bxp" in str(robot.perform(context)):
        robot_series = "bxp"

    # planning_context
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("khi_description"),
                    "urdf",
                    "khi.urdf.xacro",
                ]
            ),
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
            use_gazebo,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_config = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("khi_description"),
                    "srdf",
                    "khi.srdf.xacro",
                ]
            ),
            " prefix:=",
            prefix,
            " robot_series:=",
            robot_series,
            " robot_name:=",
            robot,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_config, value_type=str
        )
    }

    kinematics_yaml = load_yaml("khi_moveit", "config/kinematics.yaml")

    ompl_planning = load_yaml("khi_moveit", "config/ompl_planning.yaml")
    planning_pipelines = load_yaml("khi_moveit", "config/planning_pipelines.yaml")
    planning_pipelines["ompl"].update(ompl_planning)

    trajectory_execution = load_yaml("khi_moveit", "config/trajectory_execution.yaml")

    moveit_simple_controllers_yaml = load_yaml(
        "khi_moveit", "config/" + robot_series + "/controllers.yaml"
    )
    controller_manager_path = "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": controller_manager_path,
    }

    # robot_description_planning
    planning_scene_monitor_parameters = load_yaml(
        "khi_moveit", "config/planning_scene_monitor_parameters.yaml"
    )
    joint_limits_yaml = load_yaml(
        "khi_description",
        "config/" + robot_series + "/" + str(robot.perform(context)) + "/joint_limits.yaml",
    )
    cartesian_limits_yaml = load_yaml("khi_moveit", "config/cartesian_limits.yaml")
    robot_description_planning = {
        "robot_description_planning": {
            **joint_limits_yaml,
            **cartesian_limits_yaml,
        }
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            planning_pipelines,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            robot_description_planning,
            {"default_planning_pipeline": planner},
            {"use_sim_time": use_sim_time},
        ],
    )

    # RViz
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = PathJoinSubstitution([FindPackageShare("khi_moveit"), "rviz", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipelines,
            kinematics_yaml,
            robot_description_planning,
            {"default_planning_pipeline": planner},
        ],
        condition=IfCondition(use_rviz),
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    nodes = [
        static_tf,
        run_move_group_node,
        rviz_node,
    ]

    return nodes
