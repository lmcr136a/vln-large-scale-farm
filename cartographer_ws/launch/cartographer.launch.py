#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Get config directory
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    config_dir = os.path.join(launch_file_dir, '..', 'config')
    config_dir = os.path.abspath(config_dir)
    
    # Get mode value at runtime
    mode = LaunchConfiguration('mode').perform(context)
    load_map = LaunchConfiguration('load_map').perform(context)
    
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
    print(f"[Cartographer] Load map: {load_map}")
    
    # GPS Publisher (standalone script)
    gps_publisher_process = ExecuteProcess(
        cmd=['python3', '/home/nahyeon/box/vln-large-scale-farm/cartographer_ws/launch/gps_publisher.py'],
        output='screen'
    )
    
    # Cartographer node arguments
    cartographer_args = [
        '-configuration_directory', config_dir,
        '-configuration_basename', config_file
    ]
    
    # Add load_state_filename if map file is specified
    if load_map and load_map != '' and load_map != 'none':
        cartographer_args.extend([
            '-load_state_filename', load_map,
            '-start_trajectory_with_default_topics', 'true'
        ])
        print(f"[Cartographer] Loading saved map: {load_map}")
    
    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=cartographer_args,
        remappings=[
            ('points2', '/lidar3d'),
            ('imu', '/imu'),
            ('fix', '/gps/fix')
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
    
    return [gps_publisher_process, cartographer_node, occupancy_grid_node]

def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='real',
        description='Mode: real or sim'
    )
    
    load_map_arg = DeclareLaunchArgument(
        'load_map',
        default_value='',
        description='Path to saved pbstream map file (empty or none to start fresh)'
    )
    
    opaque_function = OpaqueFunction(function=launch_setup)
    
    return LaunchDescription([
        mode_arg,
        load_map_arg,
        opaque_function
    ])