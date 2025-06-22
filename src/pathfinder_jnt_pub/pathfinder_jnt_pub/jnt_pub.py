import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF

class JointStatePublisherNode(Node):
    """
    A node to dynamically publish the state of the robot's joints
    by parsing the robot's URDF description.
    """

    def __init__(self):
        """
        Initialize the node, declare parameters, create a publisher,
        and set up a timer to publish joint states.
        """
        super().__init__('pathfinder_joint_state_publisher')

        # Declare the 'robot_description' parameter. We will get the URDF from this.
        self.declare_parameter('robot_description', '')

        # Create a publisher for the /joint_states topic.
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Parse the URDF to get the joint names.
        self.joint_names = self.get_joint_names_from_urdf()

        # Create a timer that will call the publish_joint_states method.
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        self.get_logger().info(f"Custom Joint State Publisher started for joints: {self.joint_names}")

    def get_joint_names_from_urdf(self):
        """
        Parse the 'robot_description' parameter to extract the names of all
        non-fixed joints.
        """
        joint_names = []
        # Wait for the robot_description parameter to be available.
        self.get_logger().info("Waiting for robot_description parameter...")
        while not self.has_parameter('robot_description') or \
              self.get_parameter('robot_description').get_parameter_value().string_value == '':
            rclpy.spin_once(self, timeout_sec=1.0)
            if not rclpy.ok():
                return []
        
        robot_description_string = self.get_parameter(
            'robot_description').get_parameter_value().string_value
        
        # Use the urdf_parser_py library to parse the URDF string.
        try:
            robot_model = URDF.from_xml_string(robot_description_string)
        except Exception as e:
            self.get_logger().error(f"Failed to parse URDF: {e}")
            return []

        # Iterate through all joints in the model.
        for joint in robot_model.joints:
            # We only want to publish the state for joints that can move.
            if joint.type != 'fixed':
                joint_names.append(joint.name)
        
        return joint_names

    def publish_joint_states(self):
        """
        Create and publish the JointState message with current joint positions.
        """
        if not self.joint_names:
            self.get_logger().warn("No non-fixed joints found to publish.", throttle_duration_sec=10)
            return

        # Create a new JointState message object.
        joint_state = JointState()

        # Set the timestamp for the message.
        joint_state.header.stamp = self.get_clock().now().to_msg()

        # Set the names of the joints discovered from the URDF.
        joint_state.name = self.joint_names

        # Set the position for each joint. For now, we'll keep them all at 0.0.
        joint_state.position = [0.0] * len(self.joint_names)

        # Publish the message to the /joint_states topic.
        self.joint_state_publisher.publish(joint_state)


def main(args=None):
    """
    The main entry point for the script.
    """
    rclpy.init(args=args)
    joint_state_publisher_node = JointStatePublisherNode()
    try:
        rclpy.spin(joint_state_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
