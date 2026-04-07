"""
ROS2 launch file for the Duaro (WD002N) Isaac Sim integration.

Starts:
  1. robot_state_publisher — publishes TF from the Duaro URDF
  2. joint_state_reorder — relays /joint_states_raw -> /joint_states
  3. trajectory_bridge — FollowJointTrajectory action -> /joint_commands
  4. MoveIt move_group — motion planning for lower_arm / upper_arm
  5. static_transform_publisher — world -> base_link
  6. RViz2 (optional, default off)

Usage:
    ros2 launch duaro_description duaro_isaac_ros.launch.py
    ros2 launch duaro_description duaro_isaac_ros.launch.py use_rviz:=true
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("planner", default_value="ompl", choices=["ompl"]),
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):
    planner = LaunchConfiguration("planner")
    use_rviz = LaunchConfiguration("use_rviz")

    pkg = "duaro_description"

    # --- Robot description from xacro ---
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(pkg), "urdf", "duaro.urdf.xacro"]),
            " simulation:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # --- SRDF ---
    srdf_path = os.path.join(
        get_package_share_directory(pkg), "srdf", "duaro.srdf"
    )
    with open(srdf_path, "r") as f:
        srdf_content = f.read()
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(srdf_content, value_type=str)
    }

    # --- MoveIt configs ---
    kinematics_yaml_raw = load_yaml(pkg, "config/kinematics.yaml")
    kinematics_yaml = {"robot_description_kinematics": kinematics_yaml_raw}

    ompl_planning = load_yaml(pkg, "config/ompl_planning.yaml")
    planning_pipelines = load_yaml(pkg, "config/planning_pipelines.yaml")
    planning_pipelines["ompl"].update(ompl_planning)

    trajectory_execution = load_yaml(pkg, "config/trajectory_execution.yaml")

    moveit_simple_controllers_yaml = load_yaml(pkg, "config/controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    planning_scene_monitor_parameters = load_yaml(pkg, "config/planning_scene_monitor_parameters.yaml")

    joint_limits_yaml = load_yaml(pkg, "config/joint_limits.yaml")
    cartesian_limits_yaml = load_yaml(pkg, "config/cartesian_limits.yaml")
    robot_description_planning = {
        "robot_description_planning": {
            **joint_limits_yaml,
            **cartesian_limits_yaml,
        }
    }

    # Write planning pipelines to temp file (avoids ROS2 YAML serialization issues)
    def dict_to_param_file(params_dict):
        content = {"/**": {"ros__parameters": params_dict}}
        f = tempfile.NamedTemporaryFile(
            mode="w", prefix="moveit_params_", suffix=".yaml", delete=False
        )
        yaml.dump(content, f, default_flow_style=False)
        f.close()
        return f.name

    planning_pipelines_file = dict_to_param_file(planning_pipelines)

    # Find installed script paths
    pkg_share = get_package_share_directory(pkg)
    pkg_lib = os.path.join(os.path.dirname(pkg_share), "..", "lib", pkg)
    # Normalize (install/lib/duaro_description/)
    pkg_lib = os.path.normpath(pkg_lib)

    # --- Nodes ---

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
        parameters=[{"use_sim_time": True}],
    )

    joint_state_reorder_node = ExecuteProcess(
        cmd=["python3", os.path.join(pkg_lib, "joint_state_reorder_duaro.py")],
        output="screen",
    )

    trajectory_bridge_node = ExecuteProcess(
        cmd=["python3", os.path.join(pkg_lib, "trajectory_bridge_duaro.py")],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            planning_pipelines_file,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            robot_description_planning,
            {"default_planning_pipeline": planner},
            {"use_sim_time": True},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipelines_file,
            kinematics_yaml,
            robot_description_planning,
            {"default_planning_pipeline": planner},
            {"use_sim_time": True},
        ],
        condition=IfCondition(use_rviz),
        additional_env={"DBUS_SESSION_BUS_ADDRESS": "/dev/null"},
    )

    return [
        static_tf,
        robot_state_publisher_node,
        joint_state_reorder_node,
        trajectory_bridge_node,
        move_group_node,
        rviz_node,
    ]
