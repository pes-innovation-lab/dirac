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
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create dirac user with root-level access
RUN useradd -m -s /bin/bash dirac && \
    echo "dirac:dirac" | chpasswd && \
    usermod -aG sudo dirac && \
    echo "dirac ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

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

# Set ownership of the workspace to dirac user
RUN chown -R dirac:dirac /ros2_ws

# Source the workspace setup in bashrc for both root and dirac user
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /home/dirac/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /home/dirac/.bashrc

# Set the default command
CMD ["/bin/bash"]

# Switch to dirac user and set working directory
USER dirac
WORKDIR /home/dirac

