import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Get the package share directory for YOUR project
    pkg_share_path = get_package_share_directory('pathfinder_description')

    # Get the path to YOUR URDF file
    urdf_file_path = os.path.join(pkg_share_path, 'urdf', 'pathfinder.urdf.xacro')
    
    # Get the path to the RViz config file
    rviz_config_path = os.path.join(pkg_share_path, 'rviz', 'display.rviz')

    # Process the xacro file to generate the URDF XML
    robot_description_config = xacro.process_file(urdf_file_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Create the standard robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Using your custom joint state publisher
    # Pass the robot_description to this node so it can find the joints.
    joint_state_publisher_node = Node(
        package='pathfinder_jnt_pub',
        executable='jnt_pub', # This matches the entry point in your setup.py
        name='joint_state_publisher',
        parameters=[robot_description]
    )

    # UPDATED: RViz2 node now loads the config file
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
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)

    return ld
