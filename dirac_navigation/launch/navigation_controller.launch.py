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
    
    # Get launch configurations
    enable_controller = LaunchConfiguration('enable_controller')
    agent_id = LaunchConfiguration('agent_id')
    config_file = LaunchConfiguration('config_file')
    
    # Discrete navigation controller node with configuration
    navigation_controller_node = Node(
        package='dirac_navigation',
        executable='discrete_navigation_controller',
        name=['discrete_navigation_controller_', agent_id],
        parameters=[config_file],
        output='screen',
        condition=IfCondition(enable_controller)
    )
    
    return LaunchDescription([
        enable_controller_arg,
        agent_id_arg,
        config_file_arg,
        navigation_controller_node
    ])
