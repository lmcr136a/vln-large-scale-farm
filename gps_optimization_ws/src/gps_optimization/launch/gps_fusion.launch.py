import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('gps_optimization')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'config.yaml')
    
    # GPS Fusion Node
    gps_fusion_node = Node(
        package='gps_optimization',
        executable='gps_fusion_node',
        name='gps_fusion_node',
        output='screen',
        parameters=[config_file],
        emulate_tty=True
    )
    
    return LaunchDescription([
        gps_fusion_node
    ])
