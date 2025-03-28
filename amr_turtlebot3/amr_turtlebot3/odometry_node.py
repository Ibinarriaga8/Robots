import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import message_filters
from amr_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time

import traceback

from transforms3d.euler import quat2euler


class OdometryNode(LifecycleNode):
    def __init__(self):
        """Wall follower node initializer."""
        super().__init__("odometry_node")

        # Parameters
        self.last_odom = [None, None, None]

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'inactive' state.")
        self._odom_subscriber = self.create_subscription(Odometry, "/odom", self._compute_commands_callback, 10)
        self._odom_publisher = self.create_publisher(Odometry, "/odometry",10)

        return super().on_configure(state)


    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles an activating transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'active' state.")

        return super().on_activate(state)
    
     ## crear función de publicar odometry
    def _publish_odometry(self, linear_vel, angular_vel):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.angular.z = angular_vel
        self._odom_publisher.publish(odom_msg)


    def _compute_commands_callback(
        self, odom_msg: Odometry
    ):
        """Subscriber callback. Executes a wall-following controller and publishes v and w commands.

        Ceases to operate once the robot is localized.

        Args:
            odom_msg: Message containing odometry measurements.
            scan_msg: Message containing LiDAR readings.
            pose_msg: Message containing the estimated robot pose.

        """
        dt = 0.05
        current_x = odom_msg.pose.pose.position.x
        current_y = odom_msg.pose.pose.position.y

        current_yaw = self._quaternion_to_yaw(odom_msg.pose.pose.orientation)
        current_t = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9
        
        if self.last_odom[0] is None:
            self.get_logger().info("First odom received. Publishing zero velocities.")
            self._publish_odometry(0.0, 0.0)
            # Update last_odom with current values
            self.last_odom = [current_x, current_y, current_yaw, current_t]
            return

        # Calculate velocities
        v_x = (current_x - self.last_odom[0]) / dt
        w = (current_yaw - self.last_odom[2]) / dt  # Angular velocity

        # Publish velocities
        self._publish_odometry(v_x, w)

        self.last_odom = [current_x, current_y, current_yaw, current_t]
        

    def _quaternion_to_yaw(self, q):
        _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
        return yaw


        
    
def main(args=None):
    rclpy.init(args=args)
    wall_follower_node = OdometryNode()

    try:
        rclpy.spin(wall_follower_node)
    except KeyboardInterrupt:
        pass

    wall_follower_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
