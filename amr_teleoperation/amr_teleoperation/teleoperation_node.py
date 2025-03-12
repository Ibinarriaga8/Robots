import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from nav_msgs.msg import Odometry
import math
from transforms3d.euler import quat2euler

class OdometryNode(LifecycleNode):
    
    def __init__(self):
        """
        Initializes the node, setting up variables and placeholders for 
        the subscription and publisher.
        """
        super().__init__('odometry_node')
        
        # Subscription and publisher placeholders
        self.subscription = None
        self.publisher = None
        
        # Last recorded odometry data
        self.prev_time = None
        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None

    def on_configure(self):
        """
        Configures the node by setting up the subscriber and publisher.
        Returns:
            TransitionCallbackReturn.SUCCESS if successful.
        """
        self.get_logger().info("Configuring OdometryNode...")
        
        # Create subscriber to listen to odometry data
        self.subscription = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            10
        )

        # Create publisher to publish processed odometry data
        self.publisher = self.create_publisher(Odometry, '/odometry', 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self):
        """
        Activates the node. No additional setup required.
        Returns:
            TransitionCallbackReturn.SUCCESS if successful.
        """
        self.get_logger().info("Activating OdometryNode...")
        return TransitionCallbackReturn.SUCCESS

    def odom_callback(self, msg):
        """
        Callback function for odometry messages. Computes linear and angular velocities 
        based on the difference between the current and previous odometry readings.
        
        Parameters:
            msg (Odometry): The incoming odometry message.
        """
        # Get current timestamp and pose
        current_time = self.get_clock().now()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation

        # Convert quaternion to Euler angles to extract yaw (theta)
        _, _, theta = quat2euler([orientation.w, orientation.x, orientation.y, orientation.z])

        # Compute velocity if previous values exist
        if self.prev_time is not None:
            dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
            
            if dt > 0:
                vx = (x - self.prev_x) / dt  # Compute velocity in x direction
                vy = (y - self.prev_y) / dt  # Compute velocity in y direction
                linear_velocity = math.sqrt(vx**2 + vy**2)  # Compute total linear velocity
                angular_velocity = (theta - self.prev_theta) / dt  # Compute angular velocity

                # Create new odometry message
                odom_msg = Odometry()
                odom_msg.header.stamp = current_time.to_msg()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"
                
                # Fill pose data
                odom_msg.pose.pose.position.x = x
                odom_msg.pose.pose.position.y = y
                odom_msg.pose.pose.position.z = 0.0
                odom_msg.pose.pose.orientation = orientation
                
                # Fill twist data (velocity)
                odom_msg.twist.twist.linear.x = linear_velocity
                odom_msg.twist.twist.angular.z = angular_velocity
                
                # Publish the processed odometry message
                self.publisher.publish(odom_msg)

        # Store current values for next iteration
        self.prev_time = current_time
        self.prev_x = x
        self.prev_y = y
        self.prev_theta = theta


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()
