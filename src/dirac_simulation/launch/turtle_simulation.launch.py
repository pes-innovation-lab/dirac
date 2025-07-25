import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn_turtles(context, *args, **kwargs):
    """Function to dynamically spawn turtles based on num_turtles parameter"""
    num_turtles = int(context.launch_configurations['num_turtles'])
    actions = []
    
    # Spawn additional turtles (turtle1 already exists by default)
    for i in range(2, min(num_turtles + 1, 11)):  # Limit to max 10 turtles
        x_pos = 1.0 + (i - 1) * 1.5
        y_pos = 1.0 + (i - 1) * 0.5
        
        spawn_turtle = TimerAction(
            period=1.0 + i * 0.5,  # Stagger the spawning
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/spawn',
                        'turtlesim/srv/Spawn',
                        f'{{x: {x_pos}, y: {y_pos}, theta: 0.0, name: "turtle{i}"}}'
                    ],
                    output='screen'
                )
            ]
        )
        actions.append(spawn_turtle)
    
    return actions


def generate_launch_description():
    # Declare launch arguments
    num_turtles_arg = DeclareLaunchArgument(
        'num_turtles',
        default_value='1',
        description='Number of turtles to spawn (1-10)'
    )
    
    # Get the number of turtles
    num_turtles = LaunchConfiguration('num_turtles')
    
    # Start turtlesim node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(num_turtles_arg)
    
    # Add turtlesim node
    ld.add_action(turtlesim_node)
    
    # Log info about the number of turtles
    ld.add_action(LogInfo(
        msg=['Starting turtle simulation with ', num_turtles, ' turtles']
    ))
    
    # Add dynamic turtle spawning
    ld.add_action(OpaqueFunction(function=spawn_turtles))
    
    return ld
