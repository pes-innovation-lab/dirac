from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dirac_base',
            executable='agent_node',
            name='agent_1',
            output='screen',
            parameters=[{'agent_id': 'agent_1', 'zone_id': '1', 'agent_x': 10.0, 'agent_y': 20.0}]
        ),
        Node(
            package='dirac_base',
            executable='agent_node',
            name='agent_2',
            output='screen',
            parameters=[{'agent_id': 'agent_2', 'zone_id': '1', 'agent_x': 12.0, 'agent_y': 22.0}]
        ),
        Node(
            package='dirac_base',
            executable='agent_node',
            name='agent_3',
            output='screen',
            parameters=[{'agent_id': 'agent_3', 'zone_id': '1', 'agent_x': 14.0, 'agent_y': 24.0}]
        ),
        Node(
            package='dirac_base',
            executable='agent_node',
            name='agent_4',
            output='screen',
            parameters=[{'agent_id': 'agent_4', 'zone_id': '1', 'agent_x': 16.0, 'agent_y': 26.0}]
        ),
        Node(
            package='dirac_base',
            executable='agent_node',
            name='agent_5',
            output='screen',
            parameters=[{'agent_id': 'agent_5', 'zone_id': '1', 'agent_x': 18.0, 'agent_y': 28.0}]
        ),
    ])
