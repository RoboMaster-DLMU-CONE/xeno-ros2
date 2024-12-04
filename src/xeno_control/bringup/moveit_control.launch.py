import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("xeno_control"),
                    "description",
                    "urdf",
                    "xeno.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controllers_config = PathJoinSubstitution(
        [
            FindPackageShare("xeno_control"),
            "config",
            "controllers.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controllers_config],
                output="both",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["xeno_group_controller"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["end_group_controller"],
            ),
        ]
    )
