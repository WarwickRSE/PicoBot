from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    align_depth_arg = DeclareLaunchArgument(
        'align_depth', default_value='true', description='Align depth to color image'
    )
    filters_arg = DeclareLaunchArgument(
        'filters', default_value='pointcloud', description='Filters to apply'
    )
    rtabmap_args_arg = DeclareLaunchArgument(
        'rtabmap_args', default_value='delete_db_on_start', description='RTAB-Map arguments'
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value='/camera/aligned_depth_to_color/image_raw', description='Depth topic'
    )
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic', default_value='/camera/color/image_raw', description='RGB topic'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='/camera/color/camera_info', description='Camera info topic'
    )
    approx_sync_arg = DeclareLaunchArgument(
        'approx_sync', default_value='false', description='Approximate synchronization'
    )

    # Include the realsense2_camera launch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'), '/launch/rs_camera.launch.py'
        ]),
        launch_arguments={
            'align_depth': LaunchConfiguration('align_depth'),
            'filters': LaunchConfiguration('filters'),
        }.items()
    )

    # Include the cartesian_trajectory_generator trajectory_generator launch file
    trajectory_generator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('cartesian_trajectory_generator'), '/launch/trajectory_generator.launch.py'
        ])
    )

    # Include the cartesian_trajectory_generator publisher_demo launch file
    publisher_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('cartesian_trajectory_generator'), '/launch/publisher_demo.launch.py'
        ])
    )

    # Return the launch description
    return LaunchDescription([
        align_depth_arg,
        filters_arg,
        rtabmap_args_arg,
        depth_topic_arg,
        rgb_topic_arg,
        camera_info_topic_arg,
        approx_sync_arg,
        realsense_launch,
        trajectory_generator_launch,
        publisher_demo_launch,
    ])
