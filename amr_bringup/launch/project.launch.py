from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

import math


def generate_launch_description():
    world = "project"
    start = (1.0, -1.0, math.radians(90))
    goal = (-0.6, 1.0)

    particles = 100
    global_localization = False
    start_sigma = (0.05, 0.05, math.radians(5))
    sigma_v = 0.05
    sigma_w = 0.1
    sigma_z = 0.2

    particle_filter_node = LifecycleNode(
        package="amr_localization",
        executable="particle_filter",
        name="particle_filter",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
        parameters=[
            {
                "enable_plot": True,
                "global_localization": global_localization,
                "initial_pose": start,
                "initial_pose_sigma": start_sigma,
                "particles": particles,
                "sigma_v": sigma_v,
                "sigma_w": sigma_w,
                "sigma_z": sigma_z,
                "world": world,
            }
        ],
    )

    wall_follower_node = LifecycleNode(
        package="amr_control",
        executable="wall_follower",
        name="wall_follower",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    probabilistic_roadmap_node = LifecycleNode(
        package="amr_planning",
        executable="probabilistic_roadmap",
        name="probabilistic_roadmap",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
        parameters=[
            {
                "connection_distance": 0.15,  # 0.3,
                "enable_plot": True,
                "goal": goal,
                "grid_size": 0.1,
                "node_count": 250,
                "obstacle_safety_distance": 0.12,  # 0.08,
                "smoothing_additional_points": 3,
                "smoothing_data_weight": 0.1,
                "smoothing_smooth_weight": 0.25,
                "use_grid": True,
                "world": world,
            }
        ],
    )

    pure_pursuit_node = LifecycleNode(
        package="amr_control",
        executable="pure_pursuit",
        name="pure_pursuit",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
        parameters=[{"lookahead_distance": 0.3}],
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
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "node_startup_order": (
                    "odometry_node",
                    "particle_filter",
                    "wall_follower",
                    "probabilistic_roadmap",
                    "pure_pursuit",
                )
            }
        ],
    )

    return LaunchDescription(
        [
            odometry_node,
            particle_filter_node,
            wall_follower_node,
            probabilistic_roadmap_node,
            pure_pursuit_node,
            lifecycle_manager_node,  # Must be launched last
        ]
    )
