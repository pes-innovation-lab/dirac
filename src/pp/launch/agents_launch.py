from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    agents = []

    # Define agent configs
    agent_configs = [
        {"id": "agent1", "x": 1.0, "y": 1.0, "priority": 1, "job_id": "job1", "job_x": 5.0, "job_y": 5.0},
        {"id": "agent2", "x": 2.0, "y": 2.0, "priority": 2, "job_id": "job2", "job_x": 7.0, "job_y": 1.0},
        {"id": "agent3", "x": 3.0, "y": 3.0, "priority": 3, "job_id": "job3", "job_x": 1.0, "job_y": 7.0},
        {"id": "agent4", "x": 4.0, "y": 4.0, "priority": 4, "job_id": "job4", "job_x": 9.0, "job_y": 2.0},
        {"id": "agent5", "x": 5.0, "y": 5.0, "priority": 5, "job_id": "job5", "job_x": 0.0, "job_y": 9.0},
    ]

    # Spawn each agent with its parameters
    for config in agent_configs:
        agents.append(
            Node(
                package="pp",
                executable="agent_node",
                name=config["id"],
                output="screen",
                parameters=[{
                    "agent_id": config["id"],
                    "agent_x": config["x"],
                    "agent_y": config["y"],
                    "priority": config["priority"],
                    "job_id": config["job_id"],
                    "job_x": config["job_x"],
                    "job_y": config["job_y"],
                }]
            )
        )

    return LaunchDescription(agents)
