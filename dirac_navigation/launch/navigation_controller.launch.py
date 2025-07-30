#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Generate a launch description for the discrete navigation controller with YAML config.
    """
    # Declare launch arguments
    enable_controller_arg = DeclareLaunchArgument(
        'enable_controller',
        default_value='true',
        description='Enable/disable the discrete navigation controller node'
    )
    
    enable_grid_publisher_arg = DeclareLaunchArgument(
        'enable_grid_publisher',
        default_value='true',
        description='Enable/disable the grid pose publisher node'
    )
    
    agent_id_arg = DeclareLaunchArgument(
        'agent_id',
        default_value='1',
        description='Agent ID for this controller instance'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('dirac_navigation'),
            'config',
            'navigation_controller_config.yaml'
        ]),
        description='Path to the navigation configuration file'
    )
    
    # Grid pose publisher arguments
    grid_size_arg = DeclareLaunchArgument(
        'grid_size',
        default_value='5.0',
        description='Number of cells in each dimension of the grid'
    )
    
    world_size_arg = DeclareLaunchArgument(
        'world_size',
        default_value='11.0',
        description='Size of the world in meters (turtlesim default)'
    )
    
    # Get launch configurations
    enable_controller = LaunchConfiguration('enable_controller')
    enable_grid_publisher = LaunchConfiguration('enable_grid_publisher')
    agent_id = LaunchConfiguration('agent_id')
    config_file = LaunchConfiguration('config_file')
    grid_size = LaunchConfiguration('grid_size')
    world_size = LaunchConfiguration('world_size')
    
    # Discrete navigation controller node with configuration
    navigation_controller_node = Node(
        package='dirac_navigation',
        executable='discrete_navigation_controller',
        name=['discrete_navigation_controller_', agent_id],
        parameters=[config_file],
        output='screen',
        condition=IfCondition(enable_controller)
    )
    
    # Grid pose publisher node
    grid_pose_publisher_node = Node(
        package='dirac_navigation',
        executable='grid_pose_publisher',
        name=['grid_pose_publisher_', agent_id],
        parameters=[{
            'grid_size': grid_size,
            'world_size': world_size
        }],
        output='screen',
        condition=IfCondition(enable_grid_publisher)
    )
    
    return LaunchDescription([
        enable_controller_arg,
        enable_grid_publisher_arg,
        agent_id_arg,
        config_file_arg,
        grid_size_arg,
        world_size_arg,
        navigation_controller_node,
        grid_pose_publisher_node
    ])
