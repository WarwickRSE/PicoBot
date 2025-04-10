from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='picobot_control',
            executable='get_wrench_base_sim',
            name='get_wrench_base_sim',
            output='screen'
        ),
        Node(
            package='picobot_control',
            executable='wrench_transform_publisher',
            name='wrench_transform_publisher',
            output='screen'
        )
    ])
