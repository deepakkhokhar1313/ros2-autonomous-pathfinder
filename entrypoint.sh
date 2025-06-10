#!/bin/bash
# Exit immediately if a command exits with a non-zero status.
set -e

# Fix ownership of the entire workspace at runtime.
# This ensures the 'rosuser' can write to the mounted 'src' directory.
sudo chown -R rosuser:rosuser /home/rosuser/ros2_ws

# Setup the ROS 2 environment by sourcing the master setup file.
source /opt/ros/jazzy/setup.bash

# Check if a local workspace has been built and source it if it exists.
if [ -f "/home/rosuser/ros2_ws/install/setup.bash" ]; then
  source /home/rosuser/ros2_ws/install/setup.bash
fi

# Execute the command passed into the docker run/exec command.
exec "$@"