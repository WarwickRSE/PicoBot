from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='picobot_control',
            executable='record_bag.py',
            name='record_bag',
            output='screen',
            parameters=[{'file': 'test_file'}]
        )
    ])
