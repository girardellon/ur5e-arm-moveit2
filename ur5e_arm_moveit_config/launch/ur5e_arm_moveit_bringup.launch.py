from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _load_yaml(pkg_name, relative_path):
    pkg_path = get_package_share_directory(pkg_name)
    path = os.path.join(pkg_path, relative_path)
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")

    ur_driver_share = get_package_share_directory("ur_robot_driver")
    ur_launch = PythonLaunchDescriptionSource([ur_driver_share, "/launch/ur_control.launch.py"])

    moveit_share = get_package_share_directory("ur5e_arm_moveit_config")
    urdf_share = get_package_share_directory("ur5e_arm_description")

    urdf_path = os.path.join(urdf_share, "urdf", "ur5e.urdf")
    srdf_path = os.path.join(moveit_share, "srdf", "ur.srdf")

    with open(urdf_path, "r", encoding="utf-8") as f:
        urdf_xml = f.read()
    with open(srdf_path, "r", encoding="utf-8") as f:
        srdf_xml = f.read()

    robot_description = {"robot_description": ParameterValue(urdf_xml, value_type=str)}
    robot_description_semantic = {"robot_description_semantic": ParameterValue(srdf_xml, value_type=str)}

    controllers = _load_yaml("ur5e_arm_moveit_config", "config/controllers.yaml")
    kinematics = _load_yaml("ur5e_arm_moveit_config", "config/kinematics.yaml")
    joint_limits = _load_yaml("ur5e_arm_moveit_config", "config/joint_limits.yaml")
    ompl = _load_yaml("ur5e_arm_moveit_config", "config/ompl_planning.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.101"),

        IncludeLaunchDescription(
            ur_launch,
            launch_arguments={
                "ur_type": "ur5e",
                "robot_ip": robot_ip,
                "headless_mode": "true",
                "launch_rviz": "false",
            }.items(),
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="moveit_ros_move_group",
                    executable="move_group",
                    name="move_group",
                    output="screen",
                    parameters=[
                        robot_description,
                        robot_description_semantic,
                        kinematics,
                        joint_limits,
                        ompl,
                        controllers,
                        {"planning_pipelines": ["ompl"]},
                        {"default_planning_pipeline": "ompl"},
                    ],
                ),
            ],
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    parameters=[
                        robot_description,
                        robot_description_semantic,
                        kinematics,
                        joint_limits,
                    ],
                ),
            ],
        ),
    ])

