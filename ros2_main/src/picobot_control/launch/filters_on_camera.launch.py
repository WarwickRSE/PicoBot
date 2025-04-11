from launch import LaunchDescription
from launch_ros.actions import Node

#TODO: Check on the filters_on_camera_params.yaml filepath resolution at launch

def generate_launch_description():
    return LaunchDescription([
        # Box Filter PCL Manager Node
        Node(
            package='nodelet',
            executable='nodelet',
            name='box_filter_pcl_manager',
            output='screen',
            parameters=['./filters_on_camera_params.yaml'],
        ),

        # CropBox Node
        Node(
            package='nodelet',
            executable='nodelet',
            name='cropbox',
            output='screen',
            parameters=['./filters_on_camera_params.yaml'],
            remappings=[
                ('~input', '/camera/depth/color/points')
            ]
        ),

        # VoxelGrid Node
        Node(
            package='nodelet',
            executable='nodelet',
            name='voxel_grid',
            output='screen',
            parameters=['./filters_on_camera_params.yaml'],
            remappings=[
                ('~input', '/cropbox/output')
            ]
        ),
    ])
