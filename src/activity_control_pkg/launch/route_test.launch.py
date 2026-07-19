from typing import Any

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from my_launch.config_loader import (
    load_drone_config,
    resolve_config_path,
    route_parameters,
)


def _launch_route(context: LaunchContext) -> list[Any]:
    config_file = LaunchConfiguration("config_file").perform(context)
    destination_key = LaunchConfiguration("destination_key").perform(context)
    config_path, config = load_drone_config(config_file)
    params = route_parameters(config, destination_key)
    return [
        Node(
            package="activity_control_pkg",
            executable="route_test_node",
            output="screen",
            parameters=[params],
            additional_env={"DRONE_CONFIG_FILE": str(config_path)},
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(resolve_config_path()),
                description="Single drone configuration YAML.",
            ),
            DeclareLaunchArgument(
                "destination_key",
                default_value="delivery_point_1",
                description="Selected delivery branch: car1_home or delivery_point_1.",
            ),
            OpaqueFunction(function=_launch_route),
        ]
    )
