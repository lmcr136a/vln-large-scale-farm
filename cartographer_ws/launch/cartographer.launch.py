#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Get config directory
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    config_dir = os.path.join(launch_file_dir, '..', 'config')
    config_dir = os.path.abspath(config_dir)
    
    # Get mode value at runtime
    mode = LaunchConfiguration('mode').perform(context)
    
    # Select config file based on mode
    if mode == 'sim':
        config_file = 'isaacsim.lua'
        use_sim_time = True
    else:
        config_file = 'livox_3d.lua'
        use_sim_time = False
    
    print(f"[Cartographer] Mode: {mode}")
    print(f"[Cartographer] Config: {config_file}")
    print(f"[Cartographer] Use sim time: {use_sim_time}")
    
    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', config_file
        ],
        remappings=[
            ('points2', '/lidar3d'),
            ('imu', '/imu')
        ]
    )
    
    # Occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': 0.05
        }]
    )
    
    return [cartographer_node, occupancy_grid_node]

def generate_launch_description():
    # Declare arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='real',
        description='Mode: real or sim'
    )
    
    # Use OpaqueFunction to evaluate LaunchConfiguration at runtime
    opaque_function = OpaqueFunction(function=launch_setup)
    
    return LaunchDescription([
        mode_arg,
        opaque_function
    ])