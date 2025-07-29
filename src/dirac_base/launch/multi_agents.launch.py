from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    spawn_nodes = []

    agents_to_spawn = [
        {'id': 1, 'x':15.0,  'y': 20.0},
        {'id': 2, 'x': 15.0, 'y': 2.0},   
        {'id': 3, 'x': 28.0, 'y': 8.0},  
        {'id': 4, 'x': 1.0,  'y': 15.0},  
        {'id': 5, 'x': 19.0, 'y': 19.0},  
        {'id': 6, 'x': 25.0, 'y': 15.0},  
        {'id': 7, 'x': 0.0,  'y': 20.0},  
        {'id': 8, 'x': 12.0, 'y': 29.0},  
        {'id': 9, 'x': 29.0, 'y': 29.0},  
        {'id': 10, 'x': 30.0, 'y': 30.0}
    ]

    # Spawn each agent as a Node
    for agent in agents_to_spawn:
        node = Node(
            package='dirac_base',
            executable='agent_node',
            name=f"agent_{agent['id']}",
            output='screen',
            emulate_tty=True,
            parameters=[
                {'agent_x': agent['x']},
                {'agent_y': agent['y']},
                {'agent_id': agent['id']}
            ]
        )
        spawn_nodes.append(node)

    return LaunchDescription(spawn_nodes)