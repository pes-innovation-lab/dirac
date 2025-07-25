#!/bin/bash

# Dirac Run Script
# This script builds and runs the Dirac system using Docker Compose
# Usage: ./run.sh [-u] [-d] [-n <number>]
#   -u: Build image once and start containers in parallel
#   -d: Stop and remove containers (docker compose down)
#   -n <number>: Number of dirac agents to spin up (default: 1)

set -e  # Exit on any error

# Navigate to the deployment directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOYMENT_DIR="$SCRIPT_DIR/../../dirac_deployment"

echo "Navigating to deployment directory: $DEPLOYMENT_DIR"
cd "$DEPLOYMENT_DIR"

# Parse command line arguments
UP_FLAG=false
DOWN_FLAG=false
NUM_AGENTS=1

while [[ $# -gt 0 ]]; do
    case $1 in
        -u)
            UP_FLAG=true
            shift
            ;;
        -d)
            DOWN_FLAG=true
            shift
            ;;
        -n)
            NUM_AGENTS="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: ./run.sh [-u] [-d] [-n <number>]"
            exit 1
            ;;
    esac
done

# Validate that only one action is specified
if [[ "$UP_FLAG" == true && "$DOWN_FLAG" == true ]]; then
    echo "Error: Cannot specify both -u and -d flags"
    exit 1
fi

# validate that the number of agents is a positive integer
if ! [[ "$NUM_AGENTS" =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: Number of agents must be a positive integer"
    exit 1
fi

# If no flags specified, show usage
if [[ "$UP_FLAG" == false && "$DOWN_FLAG" == false ]]; then
    echo "Usage: ./run.sh [-u] [-d] [-n <number>]"
    echo "  -u: Build image once and start containers in parallel"
    echo "  -d: Stop and remove containers (docker compose down)"
    echo "  -n <number>: Number of dirac agents to spin up (default: 1)"
    exit 1
fi

# Validate number of agents
if [[ "$NUM_AGENTS" -lt 1 ]]; then
    echo "Error: Number of agents must be at least 1"
    exit 1
fi

# Execute the appropriate action
if [[ "$DOWN_FLAG" == true ]]; then
    echo "Stopping and removing Dirac containers..."
    
    # Stop all dirac compose projects
    echo "Stopping all dirac compose projects..."
    
    # Stop agent projects (try a reasonable range)
    for (( i=1; i<=NUM_AGENTS; i++ )); do
        docker compose -p "dirac-agent-$i" down 2>/dev/null || true
    done
    
    # Stop turtlesim project
    docker compose -p "dirac-turtlesim" down 2>/dev/null || true
    
    # Fallback: Stop any remaining dirac containers manually
    docker ps -q --filter "name=dirac-" | xargs -r docker stop 2>/dev/null || true
    docker ps -aq --filter "name=dirac-" | xargs -r docker rm 2>/dev/null || true
    
    # Clean up any leftover compose resources from default project
    docker compose down 2>/dev/null || true
    
    # Remove shared network
    echo "Removing shared ROS network..."
    docker network rm dirac_ros_network 2>/dev/null || echo "Network dirac_ros_network already removed or doesn't exist"
    
    echo "Dirac system shutdown complete!"
elif [[ "$UP_FLAG" == true ]]; then
    echo "Starting Dirac system with Docker Compose..."
    echo "Number of agents to start: $NUM_AGENTS"
    
    # Set up X11 forwarding for GUI applications
    xhost +local:docker 2>/dev/null || true
    
    # Create shared network for ROS communication
    echo "Creating shared ROS network..."
    docker network create dirac_ros_network 2>/dev/null || echo "Network dirac_ros_network already exists"
    
    # Build the image once
    echo "Building Dirac Docker image..."
    docker compose build
    
    # Start agents in parallel using separate compose projects
    echo "Starting agents in parallel..."
    for (( i=1; i<=NUM_AGENTS; i++ )); do
        echo "Starting agent $i..."
        AGENT_ID=$i TURTLESIM=false NUM_TURTLES=0 docker compose -p "dirac-agent-$i" up --no-build -d
    done
    
    # Start turtlesim simulation with its own project
    echo "Starting turtlesim simulation..."
    AGENT_ID=turtlesim TURTLESIM=true NUM_TURTLES=$NUM_AGENTS docker compose -p "dirac-turtlesim" up --no-build -d
    
    echo "All containers started successfully!"
    echo "All agents are connected to the shared ROS network 'dirac_ros_network'"
    echo "ROS topics published by one agent will be visible to all other agents"
    echo ""
    echo "Use 'docker ps' to see running containers"
    echo "Use 'docker logs dirac-<agent_id>' to see logs for a specific agent"
    echo "Use 'docker exec -it dirac-<agent_id> bash' to access a container"
    echo "Use './run.sh -d' to remove all containers"
fi
