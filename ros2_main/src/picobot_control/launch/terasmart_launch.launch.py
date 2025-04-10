from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='picobot_control',
            executable='thz_ros.py',
            name='thz_ros',
            output='screen',
            parameters=[{
                'file': 'test_file',
                'time': 10
            }]
        )
    ])
