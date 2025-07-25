import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn_turtles(context, *args, **kwargs):
    """Function to dynamically spawn turtles based on num_turtles parameter"""
    num_turtles = int(context.launch_configurations['num_turtles'])
    grid_size = float(context.launch_configurations['grid_size'])
    world_size = float(context.launch_configurations['world_size'])
    
    # Calculate tile size
    tile_size = world_size / grid_size
    actions = []
    
    # First, wait a bit for turtlesim to be fully ready, then teleport turtle1
    teleport_turtle1 = TimerAction(
        period=2.0,  # Longer initial delay to ensure turtlesim is ready
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call', '/turtle1/teleport_absolute',
                    'turtlesim/srv/TeleportAbsolute',
                    '{x: 0.5, y: 0.5, theta: 0.0}'
                ],
                output='screen'
            )
        ]
    )
    actions.append(teleport_turtle1)
    
    # Clear traces after teleport
    clear_traces = TimerAction(
        period=2.5,  # Half second after teleport
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/clear', 'std_srvs/srv/Empty', '{}'],
                output='screen'
            )
        ]
    )
    actions.append(clear_traces)
    
    # Spawn additional turtles (turtle1 already exists by default)
    for i in range(2, min(num_turtles + 1, 11)):  # Limit to max 10 turtles
        # Place each turtle one tile to the right of the previous turtle
        x_pos = 0.5 + (i - 1) * tile_size
        y_pos = 0.5  # Keep same y position
        
        spawn_turtle = TimerAction(
            period=3.0 + (i - 2) * 0.5,  # Start at 3 seconds, then half second between each spawn
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
    
    # Add grid parameters to match navigation controller
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
    
    # Get the configurations
    num_turtles = LaunchConfiguration('num_turtles')
    grid_size = LaunchConfiguration('grid_size')
    world_size = LaunchConfiguration('world_size')
    
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
    ld.add_action(grid_size_arg)
    ld.add_action(world_size_arg)
    
    # Add turtlesim node
    ld.add_action(turtlesim_node)
    
    # Log info about the configuration
    ld.add_action(LogInfo(
        msg=['Starting turtle simulation with ', num_turtles, ' turtles, grid size: ', grid_size, ', world size: ', world_size]
    ))
    
    # Add dynamic turtle spawning
    ld.add_action(OpaqueFunction(function=spawn_turtles))
    
    return ld
