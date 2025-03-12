import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from nav_msgs.msg import Odometry
import math
from transforms3d.euler import quat2euler

class OdometryNode(LifecycleNode):
    def __init__(self):
        super().__init__('odometry_node')
        self.subscription = None
        self.publisher = None
        
        self.last_time = None
        self.last_x = None
        self.last_y = None
        self.last_theta = None

    def on_configure(self):
        self.get_logger().info("Configuring node...")
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(Odometry, '/odometry', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self):
        self.get_logger().info("Activating node...")
        return TransitionCallbackReturn.SUCCESS

    def odom_callback(self, msg):
        now_time = self.get_clock().now()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        
        # Convertir cuaterniones a ángulos de Euler
        _, _, theta = quat2euler([orientation.w, orientation.x, orientation.y, orientation.z])
        
        if self.last_time is not None:
            dt = (now_time - self.last_time).nanoseconds / 1e9
            if dt > 0:
                vx = (x - self.last_x) / dt
                vy = (y - self.last_y) / dt
                v = math.sqrt(vx**2 + vy**2)
                w = (theta - self.last_theta) / dt

                odom_msg = Odometry()
                odom_msg.header.stamp = now_time.to_msg()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"
                
                odom_msg.pose.pose.position.x = x
                odom_msg.pose.pose.position.y = y
                odom_msg.pose.pose.position.z = 0.0
                odom_msg.pose.pose.orientation = orientation
                
                odom_msg.twist.twist.linear.x = v
                odom_msg.twist.twist.angular.z = w
                
                self.publisher.publish(odom_msg)

        self.last_time = now_time
        self.last_x = x
        self.last_y = y
        self.last_theta = theta


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()