from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=['config/controllers.yaml'],
            output='screen'
        ),
        Node(
            package='joint_state_broadcaster',
            executable='joint_state_broadcaster',
            parameters=['config/controllers.yaml'],
            output='screen'
        ),
        Node(
            package='joint_trajectory_controller',
            executable='joint_trajectory_controller',
            parameters=['config/controllers.yaml'],
            output='screen'
        ),
    ])