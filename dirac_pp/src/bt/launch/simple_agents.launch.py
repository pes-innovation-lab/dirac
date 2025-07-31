#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import csv

def generate_launch_description():
    # Path to agents.csv (in install share directory)
    from ament_index_python.packages import get_package_share_directory
    package_share_directory = get_package_share_directory('bt')
    agents_csv_path = os.path.join(package_share_directory, 'agents.csv')
    map_csv_path = os.path.join(package_share_directory, 'map.csv')
    
    nodes = []
    
    # Read agents.csv and create nodes
    try:
        with open(agents_csv_path, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                agent_id = int(row['agent_id'])
                node = Node(
                    package='bt',
                    executable='agent',
                    name=f'agent_{agent_id}',
                    output='screen',
                    parameters=[{'agent_id': agent_id}]
                )
                nodes.append(node)
                print(f"Created node for Agent {agent_id}")
    except FileNotFoundError:
        print(f"Warning: agents.csv not found at {agents_csv_path}")
        # Fallback: create 2 agents manually
        for i in [1, 2]:
            node = Node(
                package='bt',
                executable='agent',
                name=f'agent_{i}',
                output='screen',
                parameters=[
                    {'agent_id': i},
                    {'agents_csv_path': agents_csv_path},
                    {'map_csv_path': map_csv_path}
                ]
            )
            nodes.append(node)
            print(f"Created fallback node for Agent {i}")
    
    return LaunchDescription(nodes)
