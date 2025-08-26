import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level for the motor guard node'
        ),
        
        Node(
            package='xeno_control_guard',
            executable='xeno_control_guard_node',
            name='xeno_control_guard_node',
            output='both',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            respawn=True,
            respawn_delay=5.0,
        ),
    ])