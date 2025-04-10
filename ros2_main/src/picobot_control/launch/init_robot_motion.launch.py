from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='picobot_control',
            executable='plan_send_cartesian_commands',
            name='plan_send_cartesian_commands',
            output='screen',
        ),
    ])
