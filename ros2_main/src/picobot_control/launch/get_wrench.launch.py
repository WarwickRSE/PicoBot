from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='picobot_control',
            executable='sensor_grav_comp.py',
            name='grav_comp',
            output='screen'),
        Node(
            package='picobot_control',
            executable='tf_force_wrt_THz',
            name='tf_force_wrt_THz',
            output='screen'),
        Node(
            package='netft_utils',
            executable='netft_node',
            name='netft_node',
            output='screen'),
      ])
