import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist # Import the Twist message
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math
from tf_transformations import quaternion_from_euler
import numpy as np # Import numpy for easier array creation

class OdometryPublisherNode(Node):
    """
    This node simulates the robot's movement and publishes odometry information.
    It now subscribes to /cmd_vel to receive velocity commands.
    """

    def __init__(self):
        super().__init__('pathfinder_odometry_publisher')

        # Robot physical parameters (must match your URDF)
        self.wheel_radius = 0.05  # meters
        self.wheel_base = 0.25    # meters (distance between wheels)

        # --- UPDATED: Velocity is now controlled by /cmd_vel ---
        # Initialize speeds to zero. They will be updated by the subscriber.
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.wheel_rotation_left = 0.0
        self.wheel_rotation_right = 0.0

        # Create publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # *** NEW: Create a subscriber to the /cmd_vel topic ***
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # Create a TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a timer to call the update method at a regular interval (e.g., 50 Hz)
        self.timer = self.create_timer(0.02, self.update_and_publish)

        self.get_logger().info("Pathfinder Odometry Publisher has been started. Subscribing to /cmd_vel.")

    # *** NEW: Callback function for the /cmd_vel subscriber ***
    def cmd_vel_callback(self, msg):
        """
        This function is called every time a new Twist message is received on /cmd_vel.
        It updates the robot's target linear and angular speeds.
        """
        self.linear_speed = msg.linear.x
        self.angular_speed = msg.angular.z
        
    def update_and_publish(self):
        """
        Calculates the new odometry and publishes it.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Delta time in seconds

        # --- Kinematics Calculation ---
        v_left = self.linear_speed - (self.angular_speed * self.wheel_base / 2.0)
        v_right = self.linear_speed + (self.angular_speed * self.wheel_base / 2.0)
        
        delta_x = self.linear_speed * math.cos(self.theta) * dt
        delta_y = self.linear_speed * math.sin(self.theta) * dt
        delta_theta = self.angular_speed * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # --- Prepare and Broadcast the TF2 Transform ---
        q = quaternion_from_euler(0, 0, self.theta)
        
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.tf_broadcaster.sendTransform(t)

        # --- Prepare and Publish the Odometry Message ---
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Set the covariance for the pose
        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # X
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,  # Y
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,  # Z
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,  # Roll
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,  # Pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01   # Yaw
        ]

        # Set the velocity
        odom_msg.twist.twist.linear.x = self.linear_speed
        odom_msg.twist.twist.angular.z = self.angular_speed

        # Set the covariance for the twist (velocity)
        odom_msg.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # Lin_X
            0.0, 999.0, 0.0, 0.0, 0.0, 0.0,  # Lin_Y
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,  # Lin_Z
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,  # Ang_X
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,  # Ang_Y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01   # Ang_Z
        ]

        # Publish the Odometry message
        self.odom_pub.publish(odom_msg)

        # --- Prepare and Publish the JointState Message ---
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        
        self.wheel_rotation_left += (v_left / self.wheel_radius) * dt
        self.wheel_rotation_right += (v_right / self.wheel_radius) * dt
        joint_state.position = [self.wheel_rotation_left, self.wheel_rotation_right]
        
        self.joint_pub.publish(joint_state)

        # Update the last time
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher_node = OdometryPublisherNode()
    try:
        rclpy.spin(odometry_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
