cmake_minimum_required(VERSION 3.8)
project(pathfinder_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(xacro REQUIRED)

# Install the launch directory so 'ros2 launch' can find our files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the urdf directory so the launch file can find our robot model
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# *** ADDED: Install the rviz directory so the launch file can find our config ***
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
