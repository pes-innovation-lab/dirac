# Use the official ROS2 Humble base image
FROM ros:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    ros-humble-turtlesim \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace
WORKDIR /ros2_ws

# Initialize rosdep if not already done
RUN rosdep update

# Copy the entire repository into the src directory
COPY . /ros2_ws/src/dirac

# Install dependencies using rosdep
RUN cd /ros2_ws && \
    rosdep install --from-paths src --ignore-src -r -y

# Source ROS2 and build the workspace
RUN cd /ros2_ws && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source the workspace setup in bashrc for interactive use
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set the default command
CMD ["/bin/bash"]
