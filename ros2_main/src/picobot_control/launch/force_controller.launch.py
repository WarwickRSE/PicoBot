from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='picobot_control',
            executable='force_controller',
            name='force_controller',
            output='screen'
        ),
    ])
