#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Generate a launch description for dirac agent bringup with agent ID.
    Can optionally start turtlesim simulation instead.
    """
    # Declare the agent_id launch argument
    agent_id_arg = DeclareLaunchArgument(
        'agent_id',
        default_value='1',
        description='Unique identifier for the dirac agent'
    )
    
    # Declare turtlesim mode argument (can be overridden by environment variable)
    turtlesim_arg = DeclareLaunchArgument(
        'turtlesim',
        default_value=EnvironmentVariable('TURTLESIM', default_value='false'),
        description='Whether to start turtlesim simulation instead of normal agent'
    )
    
    # Declare number of turtles argument (can be overridden by environment variable)
    num_turtles_arg = DeclareLaunchArgument(
        'num_turtles',
        default_value=EnvironmentVariable('NUM_TURTLES', default_value='1'),
        description='Number of turtles to spawn in simulation (only used when turtlesim=true)'
    )
    
    # Get the launch configuration values
    agent_id = LaunchConfiguration('agent_id')
    turtlesim = LaunchConfiguration('turtlesim')
    num_turtles = LaunchConfiguration('num_turtles')
    
    # Normal agent startup (when turtlesim=false)
    normal_agent = LogInfo(
        msg=['Dirac agent ', agent_id, ' started successfully!'],
        condition=UnlessCondition(turtlesim)
    )
    
    # Turtlesim simulation startup (when turtlesim=true)
    turtlesim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('dirac_simulation'), 
            '/launch/turtle_simulation.launch.py'
        ]),
        launch_arguments={'num_turtles': num_turtles}.items(),
        condition=IfCondition(turtlesim)
    )
    
    return LaunchDescription([
        agent_id_arg,
        turtlesim_arg,
        num_turtles_arg,
        normal_agent,
        turtlesim_launch
    ])
