#!/bin/bash

# Build the workspace
echo "Building the workspace..."
cd /home/venator/pil/dirac/bt_ws
colcon build --packages-select bt

# Check if build was successful
if [ $? -ne 0 ]; then
    echo "Build failed! Exiting..."
    exit 1
fi

# Source the workspace
echo "Sourcing the workspace..."
source install/setup.bash

# Launch the multi-agent system
echo "Launching multi-agent system with slowed-down tick processing..."
echo "Each agent will process ticks every 2 seconds for easy observation."
echo "Press Ctrl+C to stop the agents."
echo ""
ros2 launch bt simple_agents.launch.py
