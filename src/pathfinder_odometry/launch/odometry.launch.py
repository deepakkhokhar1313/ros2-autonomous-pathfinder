import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # --- Paths ---
    # Path to the description package
    description_pkg_path = get_package_share_directory('pathfinder_description')
    # Path to the URDF file
    urdf_file_path = os.path.join(description_pkg_path, 'urdf', 'pathfinder.urdf.xacro')
    # Path to the RViz config file
    rviz_config_path = os.path.join(description_pkg_path, 'rviz', 'display.rviz')

    # --- Nodes ---
    # Process the xacro file to generate the URDF XML
    robot_description_config = xacro.process_file(urdf_file_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 1. Robot State Publisher Node
    # This node publishes the transform of the robot's links (TF).
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. Odometry Publisher Node
    # This is our new node that simulates odometry and publishes the odom->base_link
    # transform, as well as the joint states for the wheels.
    odometry_publisher_node = Node(
        package='pathfinder_odometry',
        executable='odometry_pub',
        name='odometry_publisher',
        output='screen'
    )

    # 3. RViz2 Node
    # This node starts RViz2 for visualization.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(odometry_publisher_node)
    ld.add_action(rviz_node)

    return ld
