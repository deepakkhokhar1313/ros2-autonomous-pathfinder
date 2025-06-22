# ros2-autonomous-pathfinder

## Project Setup & Troubleshooting Guide
This document chronicles the real-world process of setting up a robust, containerized development environment for a ROS 2 project. It serves as a practical guide, detailing the common challenges encountered—from Docker permissions to CI/CD pipeline errors—and provides the solutions that led to a successful and efficient workflow.

# Part 1: Local Development Environment
The primary challenge in setting up the local environment was a persistent conflict between the user on the host machine and the user inside the Docker container.

# Problem: The Permission denied Error
# Symptom: 
When running ros2 pkg create ... from inside the container, the command would fail with PermissionError: [Errno 13] Permission denied. A similar error occurred when trying to create files from a host-based code editor like VS Code.

# Root Cause: 
This happened because the directory mounted from the host (./src) was owned by the host user (e.g., UID 1000), while the process inside the container was running as a different user (e.g., the rosuser with UID 1013 created in the Dockerfile). The container user did not have permission to write to a folder owned by the host user.

# Solution: 
The definitive solution was, after copying the builder stage we need to give the permission to our container user to read and write on monted folder/drive. we give it using command: 
# RUN sudo chown -R rosuser:rosuser /home/rosuser/ros2_ws


This journey, filled with practical challenges and solutions, resulted in a robust, professional, and efficient development environment ready for any ROS 2 project.