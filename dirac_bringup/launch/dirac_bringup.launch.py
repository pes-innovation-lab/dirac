#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate a launch description for dirac agent bringup with agent ID.
    """
    # Declare the agent_id launch argument
    agent_id_arg = DeclareLaunchArgument(
        'agent_id',
        default_value='1',
        description='Unique identifier for the dirac agent'
    )
    
    # Get the agent_id value
    agent_id = LaunchConfiguration('agent_id')
    
    return LaunchDescription([
        agent_id_arg,
        LogInfo(msg=['Dirac agent ', agent_id, ' started successfully!'])
    ])
