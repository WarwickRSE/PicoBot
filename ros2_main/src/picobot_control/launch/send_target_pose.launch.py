from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='picobot_control',
            executable='send_target_pose',
            name='send_target_pose',
            output='screen',
        ),
    ])
