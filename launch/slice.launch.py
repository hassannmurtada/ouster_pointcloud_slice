from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ouster_pointcloud_slice',
            executable='pointcloud_slice_node',
            name='ouster_slice',
            parameters=[PathJoinSubstitution([
                FindPackageShare('ouster_pointcloud_slice'),
                'config', 'slice_params.yaml'
            ])],
            output='screen'
        )
    ])
