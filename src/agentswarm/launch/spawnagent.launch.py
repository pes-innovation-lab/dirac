from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # A list to hold all the node actions we create
    spawn_nodes = []

    # A list of dictionaries, where each dictionary represents an agent to spawn
    # 'id' must be unique for each agent
    agents_to_spawn = [
        {'id': 1, 'x': 5.0,  'y': 5.0},
        {'id': 2, 'x': 15.0, 'y': 2.0},   
        {'id': 3, 'x': 28.0, 'y': 8.0},  
        {'id': 4, 'x': 1.0,  'y': 15.0},  
        {'id': 5, 'x': 19.9, 'y': 19.9},  
        {'id': 6, 'x': 25.0, 'y': 15.0},  
        {'id': 7, 'x': 0.0,  'y': 20.0},  
        {'id': 8, 'x': 12.0, 'y': 29.0},  
        {'id': 9, 'x': 29.9, 'y': 29.9},  
        {'id': 10, 'x': 30.0, 'y': 30.0},
        {'id': 11, 'x': 7.5, 'y': 12.5},
        {'id': 12, 'x': 22.0, 'y': 7.0},
        {'id': 13, 'x': 3.0, 'y': 25.0},
        {'id': 14, 'x': 18.0, 'y': 22.0},
        {'id': 15, 'x': 27.0, 'y': 18.0},
        {'id': 16, 'x': 8.0, 'y': 28.0},
        {'id': 17, 'x': 14.0, 'y': 14.0},
        {'id': 18, 'x': 26.0, 'y': 4.0},
        {'id': 19, 'x': 2.0, 'y': 8.0},
        {'id': 20, 'x': 20.0, 'y': 30.0},
    ]

    # Loop through the list of agents
    for agent in agents_to_spawn:
        
        # Create a Node action for each agent
        node = Node(
            package='agentswarm',
            executable='agent_node',
            # This is the key part: we give each node a unique name using the ID
            name=f"agent_{agent['id']}",
            output='screen', # Show the node's output in the terminal
            emulate_tty=True, # Helps with immediate flushing of output
            # Pass the x and y coordinates as parameters to this specific node
            parameters=[
                {'pos_x': agent['x']},
                {'pos_y': agent['y']},
                {'agent_id': agent['id']}
            ]
        )
        spawn_nodes.append(node)

    # Return a LaunchDescription object with all the nodes to be launched
    return LaunchDescription(spawn_nodes)