from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Include the filters_on_camera launch file
    filters_on_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('picobot_control'), '/launch/filters_on_camera.launch.py'
        ])
    )
    return LaunchDescription([
        filters_on_camera_launch,
        Node(
            package='picobot_control',
            executable='eye_in_hand_raster',
            name='eye_in_hand_raster',
            output='screen'
        ),
        Node(
            package='roi_detection',
            executable='grn_mrkr_cropbox.py',
            name='green_marker_detection_with_cropbox',
            output='screen'
        ),
        Node(
            package='picobot_control',
            executable='ComputePoses',
            name='ComputePoses',
            output='screen'
        ),
        Node(
            package='roi_detection',
            executable='weighted_avg_normals.py',
            name='weighted_avg_normals',
            output='screen'
        ),
        Node(
            package='picobot_control',
            executable='target_to_base',
            name='target_to_base',
            output='screen'
        )
    ])
