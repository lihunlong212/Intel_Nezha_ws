from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("x_min_m", default_value="0.60"),
            DeclareLaunchArgument("x_max_m", default_value="1.00"),
            DeclareLaunchArgument("y_min_m", default_value="-0.10"),
            DeclareLaunchArgument("y_max_m", default_value="3.50"),
            Node(
                package="pillar_detector_pkg",
                executable="pillar_detector_node",
                name="pillar_detector",
                output="screen",
                parameters=[
                    {
                        "scan_topic": "/scan",
                        # Scan/takeoff frame: x forward, y left, metres.
                        "map_x_min_m": ParameterValue(
                            LaunchConfiguration("x_min_m"), value_type=float
                        ),
                        "map_x_max_m": ParameterValue(
                            LaunchConfiguration("x_max_m"), value_type=float
                        ),
                        "map_y_min_m": ParameterValue(
                            LaunchConfiguration("y_min_m"), value_type=float
                        ),
                        "map_y_max_m": ParameterValue(
                            LaunchConfiguration("y_max_m"), value_type=float
                        ),
                        "group_dist_m": 0.25,
                        "min_pts_per_group": 1,
                        "min_pillar_separation_m": 0.20,
                        "accumulation_frames": 10,
                        "cluster_merge_dist_m": 0.20,
                        "min_votes": 5,
                        "max_pillars": 2,
                    }
                ],
            ),
        ]
    )
