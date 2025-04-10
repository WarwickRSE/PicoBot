from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='picobot_control',
            executable='read_bag.py',
            name='read_bag',
            output='screen',
            parameters=[{'file': 'test_file'}]
        )
    ])
