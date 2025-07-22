#!/bin/bash

# Dirac Run Script
# This script builds and runs the Dirac system using Docker Compose
# Usage: ./run.sh [-u] [-d] [-n <number>]
#   -u: Start containers (docker compose up --build)
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

# If no flags specified, show usage
if [[ "$UP_FLAG" == false && "$DOWN_FLAG" == false ]]; then
    echo "Usage: ./run.sh [-u] [-d] [-n <number>]"
    echo "  -u: Start containers (docker compose up --build)"
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
    # Stop all agent containers by project name pattern
    for (( i=1; i<=20; i++ )); do  # Try reasonable upper bound
        docker compose down 2>/dev/null || true
    done
    echo "Dirac system shutdown complete!"
elif [[ "$UP_FLAG" == true ]]; then
    echo "Starting Dirac system with Docker Compose..."
    echo "Number of agents to start: $NUM_AGENTS"
    
    # Set up X11 forwarding for GUI applications
    xhost +local:docker 2>/dev/null || true
    
    # Start agents using individual docker compose commands
    echo "Starting agents..."
    for (( i=1; i<=NUM_AGENTS; i++ )); do
        echo "Starting agent $i..."
        export AGENT_ID=$i
        export TURTLESIM=false
        export NUM_TURTLES=0
        docker compose up --build 
    done
    
    # Start turtlesim simulation after all agents
    echo "Starting turtlesim simulation..."
    export AGENT_ID=turtlesim
    export TURTLESIM=true
    export NUM_TURTLES=$NUM_AGENTS
    docker compose up --build
fi
