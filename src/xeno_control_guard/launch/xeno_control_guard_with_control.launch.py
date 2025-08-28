import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level for the motor guard node'
        ),
        
        DeclareLaunchArgument(
            'start_control',
            default_value='false',
            description='Whether to also start the main control system'
        ),
        
        # Launch the main control system if requested
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('xeno_control'), '/launch/moveit_control.launch.py'
            ]),
            condition=IfCondition(LaunchConfiguration('start_control'))
        ),
        
        # Always start the motor guard node
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