FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && \
    apt-get install -y rapidjson-dev build-essential cmake && \
    rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /pibt_ws

# Copy source code
COPY . ./src/pibt2

# Build
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select pibt2

# Source the workspace and set entrypoint
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"] 