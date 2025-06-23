# === Stage 1: The "Builder" ===
# This stage installs all build dependencies, copies the source code,
# and compiles the ROS 2 workspace.
FROM osrf/ros:jazzy-desktop-full-noble AS builder

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Install all necessary build dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-dev-tools \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace and copy the source code into it.
WORKDIR /ros2_ws
COPY ./src ./src

# Build the workspace using --merge-install for a clean layout.
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --merge-install


# === Stage 2: The Final Runtime Image ===
# This stage creates the clean, final image. It starts from a fresh base
# and copies only the essential compiled artifacts from the builder stage.
FROM osrf/ros:jazzy-desktop-full-noble

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Install ONLY the essential runtime dependencies, including sudo and gosu for the entrypoint.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    black \
    git \
    nano \
    vim \
    terminator \
    sudo \
    gosu \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-tf-transformations \
    # breaking the system package protection since we are in a controlled container.
    && pip3 install --break-system-packages \
        urdf-parser-py \
    && rm -rf /var/lib/apt/lists/*

# Create a default non-root user. The entrypoint will modify this at runtime.
RUN groupadd --gid 1013 rosuser && \
    useradd --uid 1013 --gid 1013 --create-home --shell /bin/bash rosuser && \
    usermod -aG sudo rosuser && \
    echo 'rosuser ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Set up the workspace directory
WORKDIR /home/rosuser/ros2_ws

# --- The Magic of Multi-Stage ---
# Copy ONLY the 'install' and 'src' directories from the builder stage.
# The bulky 'build' and 'log' directories are left behind.
COPY --from=builder --chown=rosuser:rosuser /ros2_ws/install ./install
COPY --from=builder --chown=rosuser:rosuser /ros2_ws/src ./src

# After coping environment from builder stage it is very important to run this command to give permission to container to make any changes on host
RUN sudo chown -R rosuser:rosuser /home/rosuser/ros2_ws

# Add sourcing to the global bash profile so it applies to ANY user
# when they open an interactive terminal.
RUN echo "source /opt/ros/jazzy/setup.bash && \
         if [ -f /home/rosuser/ros2_ws/install/setup.bash ]; then source /home/rosuser/ros2_ws/install/setup.bash; fi" >> /etc/bash.bashrc

# Copy the entrypoint script that will manage permissions at runtime.
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Set the entrypoint for the container. It will run as root.
ENTRYPOINT ["entrypoint.sh"]

# Set default working directory and command.
WORKDIR /home/rosuser/ros2_ws
CMD ["bash"]
