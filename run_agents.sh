#!/bin/bash

# Build the workspace
echo "Building the workspace..."
cd /home/venator/pil/dirac/bt_ws
colcon build --packages-select bt

# Source the workspace
echo "Sourcing the workspace..."
source install/setup.bash

# Launch the multi-agent system
echo "Launching multi-agent system..."
ros2 launch bt agents.launch.py
