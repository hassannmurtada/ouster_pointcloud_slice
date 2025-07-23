from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Launch arguments
    sensor_hostname_arg = DeclareLaunchArgument(
        'sensor_hostname',
        default_value='169.254.223.207',
        description='IP address of the Ouster sensor'
    )
    
    enable_configurator_arg = DeclareLaunchArgument(
        'enable_configurator',
        default_value='true',
        description='Enable the Ouster configurator node'
    )
    
    auto_apply_config_arg = DeclareLaunchArgument(
        'auto_apply_config',
        default_value='true',
        description='Automatically apply configuration on startup'
    )
    
    # Configuration file paths
    slice_config_path = PathJoinSubstitution([
        FindPackageShare('ouster_pointcloud_slice'),
        'config', 'slice_params.yaml'
    ])
    
    ouster_config_path = PathJoinSubstitution([
        FindPackageShare('ouster_pointcloud_slice'),
        'config', 'ouster_config.yaml'
    ])
    
    # Ouster Configurator Node
    ouster_configurator_node = Node(
        package='ouster_pointcloud_slice',
        executable='ouster_configurator.py',
        name='ouster_configurator',
        parameters=[
            ouster_config_path,
            {
                'sensor_hostname': LaunchConfiguration('sensor_hostname'),
                'auto_apply_on_startup': LaunchConfiguration('auto_apply_config')
            }
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_configurator'))
    )
    
    # Point Cloud Slicer Node
    pointcloud_slicer_node = Node(
        package='ouster_pointcloud_slice',
        executable='pointcloud_slice_node',
        name='ouster_slice',
        parameters=[slice_config_path],
        output='screen'
    )
    
    return LaunchDescription([
        sensor_hostname_arg,
        enable_configurator_arg,
        auto_apply_config_arg,
        ouster_configurator_node,
        pointcloud_slicer_node
    ])
