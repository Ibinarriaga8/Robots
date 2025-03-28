"""
odometry_node.py
This node subscribes to the /odom topic and calculates the linear and angular velocities of the robot.
It then publishes the calculated velocities to the /odometry topic.
"""


import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
from nav_msgs.msg import Odometry
import math
from transforms3d.euler import quat2euler


class OdometryNode(LifecycleNode):
    def __init__(self):
        super().__init__('odometry_node')

        self._odom_subscriber = None
        self._odom_publisher = None

        self.prev_time = None
        self.prev_position = None
        self.prev_theta = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Transitioning from '{state.label}' to 'inactive' state.")

        self._odom_subscriber = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self._odom_publisher = self.create_publisher(Odometry, "/odometry", 10)

        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Transitioning from '{state.label}' to 'active' state.")

        return super().on_activate(state)

    def odom_callback(self, msg: Odometry):
        current_time = self.get_clock().now()

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, theta = quat2euler([orientation.w, orientation.x, orientation.y, orientation.z])

        if self.prev_time is not None and self.prev_position is not None:
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            if dt > 0:
                dx = position.x - self.prev_position.x
                dy = position.y - self.prev_position.y

                linear_velocity = math.sqrt(dx**2 + dy**2) / dt
                angular_velocity = (theta - self.prev_theta) / dt

                odom_msg = Odometry()
                odom_msg.header.stamp = current_time.to_msg()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"

                odom_msg.pose.pose.position.x = position.x
                odom_msg.pose.pose.position.y = position.y
                odom_msg.pose.pose.position.z = 0.0
                odom_msg.pose.pose.orientation = orientation

                odom_msg.twist.twist.linear.x = linear_velocity
                odom_msg.twist.twist.angular.z = angular_velocity

                self._odom_publisher.publish(odom_msg)

        self.prev_time = current_time
        self.prev_position = position
        self.prev_theta = theta


def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()
