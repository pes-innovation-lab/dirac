from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    spawn_nodes = []

    agents_to_spawn = [
        {'id': 1, 'x': 5.0,  'y': 5.0},
        {'id': 2, 'x': 15.0, 'y': 2.0},   
        {'id': 3, 'x': 28.0, 'y': 8.0},  
        {'id': 4, 'x': 1.0,  'y': 15.0},  
        {'id': 5, 'x': 19.0, 'y': 19.0},  
        {'id': 6, 'x': 25.0, 'y': 15.0},  
        {'id': 7, 'x': 0.0,  'y': 20.0},  
        {'id': 8, 'x': 12.0, 'y': 29.0},  
        {'id': 9, 'x': 29.0, 'y': 29.0},  
        {'id': 10, 'x': 30.0, 'y': 30.0},
        {'id': 11, 'x': 7.0, 'y': 12.0},
        {'id': 12, 'x': 22.0, 'y': 7.0},
        {'id': 13, 'x': 3.0, 'y': 25.0},
        {'id': 14, 'x': 18.0, 'y': 22.0},
        {'id': 15, 'x': 27.0, 'y': 18.0},
        {'id': 16, 'x': 8.0, 'y': 28.0},
        {'id': 17, 'x': 14.0, 'y': 14.0},
        {'id': 18, 'x': 26.0, 'y': 4.0},
        {'id': 19, 'x': 2.0, 'y': 8.0},
        {'id': 20, 'x': 20.0, 'y': 30.0},
        {'id': 21, 'x': 6.0, 'y': 3.0},
        {'id': 22, 'x': 9.0, 'y': 17.0},
        {'id': 23, 'x': 13.0, 'y': 21.0},
        {'id': 24, 'x': 17.0, 'y': 27.0},
        {'id': 25, 'x': 21.0, 'y': 11.0},
        {'id': 26, 'x': 23.0, 'y': 24.0},
        {'id': 27, 'x': 11.0, 'y': 6.0},
        {'id': 28, 'x': 4.0, 'y': 18.0},
        {'id': 29, 'x': 16.0, 'y': 9.0},
        {'id': 30, 'x': 24.0, 'y': 28.0},
        {'id': 31, 'x': 10.0, 'y': 2.0},
        {'id': 32, 'x': 19.0, 'y': 5.0},
        {'id': 33, 'x': 8.0, 'y': 23.0},
        {'id': 34, 'x': 27.0, 'y': 13.0},
        {'id': 35, 'x': 3.0, 'y': 11.0},
        {'id': 36, 'x': 22.0, 'y': 17.0},
        {'id': 37, 'x': 15.0, 'y': 25.0},
        {'id': 38, 'x': 29.0, 'y': 3.0},
        {'id': 39, 'x': 7.0, 'y': 27.0},
        {'id': 40, 'x': 25.0, 'y': 22.0},
    ]

    for agent in agents_to_spawn:
        node = Node(
            package='agentswarm',
            executable='agent_node',
            name=f"agent_{agent['id']}",
            output='screen',
            emulate_tty=True,
            parameters=[
                {'pos_x': agent['x']},
                {'pos_y': agent['y']},
                {'agent_id': agent['id']}
            ]
        )
        spawn_nodes.append(node)

    return LaunchDescription(spawn_nodes)
