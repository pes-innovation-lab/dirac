FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    build-essential \
    ros-humble-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /workspace
COPY . /workspace

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source the workspace and set entrypoint
SHELL ["/bin/bash", "-c"]
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /workspace/install/setup.bash' >> ~/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && exec $@", "bash"]
