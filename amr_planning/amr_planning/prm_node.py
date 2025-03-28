import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

from amr_msgs.msg import PoseStamped as AmrPoseStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import os
import time
import traceback

from amr_planning.prm import PRM


class PRMNode(LifecycleNode):
    def __init__(self):
        """Probabilistic roadmap (PRM) node initializer."""
        super().__init__("probabilistic_roadmap")

        # Parameters
        self.declare_parameter("connection_distance", 0.3)
        self.declare_parameter("enable_plot", False)
        self.declare_parameter("goal", (0.0, 0.0))
        self.declare_parameter("grid_size", 0.05)
        self.declare_parameter("node_count", 250)
        self.declare_parameter("obstacle_safety_distance", 0.08)
        self.declare_parameter("smoothing_additional_points", 3)
        self.declare_parameter("smoothing_data_weight", 0.1)
        self.declare_parameter("smoothing_smooth_weight", 0.3)
        self.declare_parameter("use_grid", False)
        self.declare_parameter("world", "project")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'inactive' state.")

        try:
            # Parameters
            connection_distance = (
                self.get_parameter("connection_distance").get_parameter_value().double_value
            )
            self._enable_plot = self.get_parameter("enable_plot").get_parameter_value().bool_value
            self._goal = tuple(
                self.get_parameter("goal").get_parameter_value().double_array_value.tolist()
            )
            grid_size = self.get_parameter("grid_size").get_parameter_value().double_value
            node_count = self.get_parameter("node_count").get_parameter_value().integer_value
            obstacle_safety_distance = (
                self.get_parameter("obstacle_safety_distance").get_parameter_value().double_value
            )
            self._smoothing_additional_points = (
                self.get_parameter("smoothing_additional_points")
                .get_parameter_value()
                .integer_value
            )
            self._smoothing_data_weight = (
                self.get_parameter("smoothing_data_weight").get_parameter_value().double_value
            )
            self._smoothing_smooth_weight = (
                self.get_parameter("smoothing_smooth_weight").get_parameter_value().double_value
            )
            use_grid = self.get_parameter("use_grid").get_parameter_value().bool_value
            world = self.get_parameter("world").get_parameter_value().string_value

            # Attribute and object initializations
            map_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", "maps", world + ".json")
            )
            self._localized = False

            start_time = time.perf_counter()
            self._planning = PRM(
                map_path,
                obstacle_safety_distance,
                use_grid,
                node_count,
                grid_size,
                connection_distance,
            )
            roadmap_creation_time = time.perf_counter() - start_time

            self.get_logger().info(f"Roadmap creation time: {roadmap_creation_time:1.3f} s")

            # Publishers
            # 4.6. Create the /path publisher (Path message).
            self._publisher_path = self.create_publisher(Path, "path", 10)
            
            # Subscribers
            self._subscriber_pose = self.create_subscription(
                AmrPoseStamped, "pose", self._path_callback, 10
            )

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

    def _path_callback(self, pose_msg: AmrPoseStamped):
        """Subscriber callback. Finds a path using A* and publishes the smoothed path to the goal.

        Args:
            pose_msg: Message containing the robot pose estimate.

        """
        
        if pose_msg.localized and not self._localized:
            start = (pose_msg.pose.position.x, pose_msg.pose.position.y)

            start_time = time.perf_counter()
            path = self._planning.find_path(start, self._goal)
            pathfinding_time = time.perf_counter() - start_time

            self.get_logger().info(f"Pathfinding time: {pathfinding_time:1.3f} s")

            start_time = time.perf_counter()
            smoothed_path = PRM.smooth_path(
                path,
                data_weight=self._smoothing_data_weight,
                smooth_weight=self._smoothing_smooth_weight,
                additional_smoothing_points=self._smoothing_additional_points,
            )
            smoothing_time = time.perf_counter() - start_time

            self.get_logger().info(f"Smoothing time: {smoothing_time:1.3f} s")

            if self._enable_plot:
                self._planning.show(path=path, smoothed_path=smoothed_path, save_figure=True)

            self._publish_path(smoothed_path)

        self._localized = pose_msg.localized

    def _publish_path(self, path: list[tuple[float, float]]) -> None:
        """Publishes the robot's path to the goal in a nav_msgs.msg.Path message.

        Args:
            path: Smoothed path (initial location first) in (x, y) format.

        """
        # TODO: 4.7. Complete the function body with your code (i.e., replace the pass statement).
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        path_msg.poses = []

        for (x, y) in path:
            node = PoseStamped()
            node.pose.position.x = x
            node.pose.position.y = y
            path_msg.poses.append(node)
        
        self._publisher_path.publish(path_msg)
        

def main(args=None):
    rclpy.init(args=args)
    prm_node = PRMNode()

    try:
        rclpy.spin(prm_node)
    except KeyboardInterrupt:
        pass

    prm_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
