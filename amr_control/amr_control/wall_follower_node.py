import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)

import message_filters
from amr_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import traceback

from amr_control.wall_follower import WallFollower


class WallFollowerNode(LifecycleNode):
    def __init__(self):
        """Wall follower node initializer."""
        super().__init__("wall_follower")

        # Parameters
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("enable_localization", False)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(
            f"Transitioning from '{state.label}' to 'inactive' state."
        )

        try:
            # Parameters
            dt = self.get_parameter("dt").get_parameter_value().double_value
            enable_localization = (
                self.get_parameter("enable_localization")
                .get_parameter_value()
                .bool_value
            )

            scan_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT, durability = QoSDurabilityPolicy.VOLATILE)
            #
            # Subscribers
            # TODO: 2.7. Synchronize _compute_commands_callback with /odometry and /scan.

            self._subscribers: list[message_filters.Subscriber] = []

            # Append as many topics as needed
            self._subscribers.append(
                message_filters.Subscriber(self, Odometry, "/odometry")
            )
            self._subscribers.append(
                message_filters.Subscriber(self, LaserScan, "/scan", qos_profile = scan_profile)
            )
            ts = message_filters.ApproximateTimeSynchronizer(
                self._subscribers,
                queue_size=10,  # tamaño de la cola
                slop=9.0,  # tiempo máximo de espera,
            )

            ts.registerCallback(self._compute_commands_callback)

            # TODO: 4.12. Add /pose to the synced subscriptions only if localization is enabled.

            # Publishers
            # TODO: 2.10. Create the /cmd_vel velocity commands publisher (TwistStamped message).

            self._cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

            # Attribute and object initializations
            self._wall_follower = WallFollower(dt)

        except Exception:
            self.get_logger().error(f"{traceback.format_exc()}")
            return TransitionCallbackReturn.ERROR

        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles an activating transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'active' state.")

        return super().on_activate(state)

    def _compute_commands_callback(
        self,
        odom_msg: Odometry,
        scan_msg: LaserScan,
        pose_msg: PoseStamped = PoseStamped(),
    ):
        """Subscriber callback. Executes a wall-following controller and publishes v and w commands.

        Ceases to operate once the robot is localized.

        Args:
            odom_msg: Message containing odometry measurements.
            scan_msg: Message containing LiDAR readings.
            pose_msg: Message containing the estimated robot pose.

        """
        self.get_logger().info("Received messages.")
        if not pose_msg.localized:
            # TODO: 2.8. Parse the odometry from the Odometry message (i.e., read z_v and z_w).
            z_v: float = odom_msg.twist.twist.linear.x
            z_w: float = odom_msg.twist.twist.angular.z

            # TODO: 2.9. Parse LiDAR measurements from the LaserScan message (i.e., read z_scan).
            z_scan: list[float] = scan_msg.ranges

            # Execute wall follower
            v, w = self._wall_follower.compute_commands(z_scan, z_v, z_w)
            self.get_logger().info(f"Commands: v = {v:.3f} m/s, w = {w:+.3f} rad/s")

            # Publish
            self._publish_velocity_commands(v, w)
            


    def _publish_velocity_commands(self, v: float, w: float) -> None:
        """Publishes velocity commands in a geometry_msgs.msg.TwistStamped message.

        Args:
            v: Linear velocity command [m/s].
            w: Angular velocity command [rad/s].

        """
        # 2.11. Complete the function body with your code (i.e., replace the pass statement).
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = 1.0* w
        self._cmd_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    wall_follower_node = WallFollowerNode()

    try:
        rclpy.spin(wall_follower_node)
    except KeyboardInterrupt:
        pass

    wall_follower_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
