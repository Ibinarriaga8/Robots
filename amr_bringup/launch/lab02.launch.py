from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    wall_follower_node = LifecycleNode(
        package="amr_control",
        executable="wall_follower",
        name="wall_follower",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
    )

    odometry_node = LifecycleNode(
        package="amr_turtlebot3",
        executable="odometry_node",
        name="odometry_node",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    lifecycle_manager_node = Node(
        package="amr_bringup",
        executable="lifecycle_manager",
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
        parameters=[
            {
                "node_startup_order": (
                    "odometry_node",
                    "wall_follower",
                )
            }
        ],
    )

    return LaunchDescription(
        [
            odometry_node,
            wall_follower_node,
            lifecycle_manager_node,  # Must be launched last
        ]
    )
