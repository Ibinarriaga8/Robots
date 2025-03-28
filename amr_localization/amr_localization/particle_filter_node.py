import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import message_filters
from amr_msgs.msg import PoseStamped, MoveFlag
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import math
import os
import time
import traceback
from transforms3d.euler import euler2quat

from amr_localization.particle_filter import ParticleFilter


class ParticleFilterNode(LifecycleNode):
    def __init__(self):
        """Particle filter node initializer."""
        super().__init__("particle_filter")

        # Parameters
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("enable_plot", False)
        self.declare_parameter("global_localization", True)
        self.declare_parameter("initial_pose", (0.0, 0.0, math.radians(0)))
        self.declare_parameter("initial_pose_sigma", (0.05, 0.05, math.radians(5)))
        self.declare_parameter("particles", 1000)
        self.declare_parameter("sigma_v", 0.1)
        self.declare_parameter("sigma_w", 0.1)
        self.declare_parameter("sigma_z", 0.1)
        self.declare_parameter("steps_btw_sense_updates", 10)
        self.declare_parameter("world", "lab03")
        self._last_scan = None
        self._stop = False
        self._odometry_measures = []
        self._last_x = None
        self._last_y = None
        self._last_theta = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'inactive' state.")

        try:
            # Parameters
            dt = self.get_parameter("dt").get_parameter_value().double_value
            self._enable_plot = self.get_parameter("enable_plot").get_parameter_value().bool_value
            global_localization = (
                self.get_parameter("global_localization").get_parameter_value().bool_value
            )
            initial_pose = tuple(
                self.get_parameter("initial_pose").get_parameter_value().double_array_value.tolist()
            )
            initial_pose_sigma = tuple(
                self.get_parameter("initial_pose_sigma")
                .get_parameter_value()
                .double_array_value.tolist()
            )
            particles = self.get_parameter("particles").get_parameter_value().integer_value
            sigma_v = self.get_parameter("sigma_v").get_parameter_value().double_value
            sigma_w = self.get_parameter("sigma_w").get_parameter_value().double_value
            sigma_z = self.get_parameter("sigma_z").get_parameter_value().double_value
            self._steps_btw_sense_updates = (
                self.get_parameter("steps_btw_sense_updates").get_parameter_value().integer_value
            )
            world = self.get_parameter("world").get_parameter_value().string_value

            # Attribute and object initializations
            self._localized = False
            self._steps = 0
            map_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", "maps", world + ".json")
            )
            self._particle_filter = ParticleFilter(
                dt,
                map_path,
                particle_count=particles,
                sigma_v=sigma_v,
                sigma_w=sigma_w,
                sigma_z=sigma_z,
                global_localization=global_localization,
                initial_pose=initial_pose,
                initial_pose_sigma=initial_pose_sigma,
            )

            if self._enable_plot:
                self._particle_filter.show("Initialization", save_figure=True)

            # Publishers
            # 3.1. Create the /pose publisher (PoseStamped message).
            
            self._pose_publisher = self.create_publisher(PoseStamped, "pose", 10)
            self._move_publihser = self.create_publisher(MoveFlag, "move", 10)

            
            # Subscribers
            scan_qos_profile = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )

            self._odometry_subscriber = self.create_subscription(Odometry, "/odometry", self._compute_pose_callback, 10)
            self._scan_subscriber = self.create_subscription(LaserScan, "/scan", self._scan_callback, scan_qos_profile)

            

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

    def _compute_pose_callback(self, odom_msg: Odometry):
        """Subscriber callback. Executes a particle filter and publishes (x, y, theta) estimates.

        Args:
            odom_msg: Message containing odometry measurements.
            scan_msg: Message containing LiDAR sensor readings.

        """

        # Parse measurements
        z_v: float = odom_msg.twist.twist.linear.x
        z_w: float = odom_msg.twist.twist.angular.z


        # Update and publish last measurements 
        if self._last_scan is not None:
            if self._last_x is None or self._last_y is None or self._last_theta is None :
                self._last_x, self._last_y, self._last_theta = self._execute_measurement_step(self._last_scan)
                
            self.estimate_pose(z_v, z_w)
            self._publish_pose_estimate(self._last_x, self._last_y, self._last_theta)


        self.get_logger().info(f"Longitud del odom_msg: {len(self._odometry_measures)}")
        if len(self._odometry_measures) >= 20:
            self._stop = True
            
            msg = MoveFlag()
            msg.move = not self._stop
            self._move_publihser.publish(msg)            
            time.sleep(2.5)
            
            for z_v, z_w in self._odometry_measures:
                self._execute_motion_step(z_v, z_w)
                self._steps += 1


            self._odometry_measures = []
            self._stop = False
            msg = MoveFlag()
            msg.move = not self._stop
            self._move_publihser.publish(msg)

            self._last_x, self._last_y, self._last_theta = self._execute_measurement_step(self._last_scan)

            


        elif not self._stop:
            if abs(z_v) > 0.005 or abs(z_w) > 0.005:
                self._odometry_measures.append((z_v,z_w))
        

        

    def estimate_pose(self, z_v: float, z_w: float):
        if self._last_x is not None and self._last_y is not None and self._last_theta is not None:
            self._publish_pose_estimate(self._last_x, self._last_y, self._last_theta)

        prev_theta = self._last_theta

        self._last_theta = self._last_theta + z_w * self._particle_filter._dt
        try:
            self._last_x = self._last_x + z_v * math.cos((self._last_theta + prev_theta) / 2) * self._particle_filter._dt
            self._last_y = self._last_y + z_v * math.sin((self._last_theta + prev_theta) / 2) * self._particle_filter._dt
            
        except ValueError as e:
            print(f"{e}: Variables:\n"
                f"_last_x = {self._last_x}, "
                f"_last_y = {self._last_y}, "
                f"z_v = {z_v}, "
                f"_last_theta = {self._last_theta}, "
                f"prev_theta = {prev_theta}, "
                f"_dt = {self._particle_filter._dt}")


    def _scan_callback(self, scan_msg):

        if not self._stop:
            z_scan: list[float] = scan_msg.ranges
            self._last_scan = z_scan


    def _execute_measurement_step(self, z_us: list[float]) -> tuple[float, float, float]:
        """Executes and monitors the measurement step (sense) of the particle filter.

        Args:
            z_us: Distance from every ultrasonic sensor to the closest obstacle [m].

        Returns:
            Pose estimate (x_h, y_h, theta_h) [m, m, rad]; inf if cannot be computed.
        """
        pose = (float("inf"), float("inf"), float("inf"))

        if self._localized or not self._steps % self._steps_btw_sense_updates:
            start_time = time.perf_counter()
            self._particle_filter.resample(z_us)
            sense_time = time.perf_counter() - start_time

            self.get_logger().info(f"Sense step time: {sense_time:6.3f} s")

            if self._enable_plot:
                self._particle_filter.show("Sense", save_figure=True)

            start_time = time.perf_counter()
            self._localized, pose = self._particle_filter.compute_pose()
            clustering_time = time.perf_counter() - start_time

            self.get_logger().info(f"Clustering time: {clustering_time:6.3f} s")

        return pose

    def _execute_motion_step(self, z_v: float, z_w: float):
        """Executes and monitors the motion step (move) of the particle filter.

        Args:
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].
        """
        start_time = time.perf_counter()
        self._particle_filter.move(z_v, z_w)
        move_time = time.perf_counter() - start_time

        self.get_logger().info(f"Move step time: {move_time:7.3f} s")

        if self._enable_plot:
            self._particle_filter.show("Move", save_figure=True)

    def _publish_pose_estimate(self, x_h: float, y_h: float, theta_h: float) -> None:
        """Publishes the robot's pose estimate in a custom amr_msgs.msg.PoseStamped message.

        Args:
            x_h: x coordinate estimate [m].
            y_h: y coordinate estimate [m].
            theta_h: Heading estimate [rad].

        """
        # TODO: 3.2. Complete the function body with your code (i.e., replace the pass statement).
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.localized = self._localized
        
        if self._localized:
            pose_msg.pose.position.x = x_h
            pose_msg.pose.position.y = y_h
            pose_msg.pose.position.z = 0.0
            
            qw, qx, qy, qz = euler2quat(0, 0, theta_h)
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw
            
        self._pose_publisher.publish(pose_msg)
        

def main(args=None):
    rclpy.init(args=args)
    particle_filter_node = ParticleFilterNode()

    try:
        rclpy.spin(particle_filter_node)
    except KeyboardInterrupt:
        pass

    particle_filter_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()