import os
from typing import Any

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from my_launch.config_loader import (
    APRILTAG_36H11_MAX_ID,
    APRILTAG_36H11_MIN_ID,
    VALID_ROUTE_DESTINATIONS,
    DroneConfigError,
    load_drone_config,
    resolve_config_path,
)


def _package_launch(package_name: str, filename: str) -> str:
    package_share = FindPackageShare(package=package_name).find(package_name)
    return os.path.join(package_share, "launch", filename)


def _include(package_name: str, filename: str, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_package_launch(package_name, filename)),
        launch_arguments=(launch_arguments or {}).items(),
    )


def _launch_demo(context: LaunchContext) -> list[Any]:
    requested_config = LaunchConfiguration("config_file").perform(context)
    config_path, config = load_drone_config(requested_config)
    hardware = config["hardware"]
    destination_key = LaunchConfiguration("destination_key").perform(context).strip()
    if destination_key not in VALID_ROUTE_DESTINATIONS:
        raise DroneConfigError(
            f"destination_key must be one of {VALID_ROUTE_DESTINATIONS}, "
            f"got {destination_key!r}"
        )

    tag_override = LaunchConfiguration("apriltag_target_id").perform(context).strip()
    try:
        target_tag_id = (
            int(tag_override)
            if tag_override
            else int(config["vision"]["default_pickup_apriltag_id"])
        )
    except ValueError as exc:
        raise DroneConfigError("apriltag_target_id must be an integer") from exc
    if not APRILTAG_36H11_MIN_ID <= target_tag_id <= APRILTAG_36H11_MAX_ID:
        raise DroneConfigError(
            f"apriltag_target_id must be in [{APRILTAG_36H11_MIN_ID}, "
            f"{APRILTAG_36H11_MAX_ID}]"
        )

    config_arg = {"config_file": str(config_path)}
    route_config_arg = {
        "config_file": str(config_path),
        "destination_key": destination_key,
    }
    actions: list[Any] = [
        _include("my_carto_pkg", "fly_carto.launch.py"),
        TimerAction(
            period=2.0,
            actions=[_include("uart_to_stm32", "uart_to_stm32.launch.py")],
        ),
    ]
    if hardware["use_pillar_detection"]:
        actions.append(
            TimerAction(
                period=2.0,
                actions=[
                    _include(
                        "pillar_detector_pkg",
                        "pillar_detector.launch.py",
                        {
                            "x_min_m": str(config["pillar_detection"]["x_min_m"]),
                            "x_max_m": str(config["pillar_detection"]["x_max_m"]),
                            "y_min_m": str(config["pillar_detection"]["y_min_m"]),
                            "y_max_m": str(config["pillar_detection"]["y_max_m"]),
                        },
                    )
                ],
            )
        )
    actions.extend(
        [
            TimerAction(
                period=4.0,
                actions=[
                    _include(
                        "drone_camera_pkg",
                        "apriltag_camera.launch.py",
                        {
                            "apriltag_dictionary": config["vision"]["apriltag_dictionary"],
                            "apriltag_target_id": str(target_tag_id),
                        },
                    )
                ],
            ),
            TimerAction(
                period=6.0,
                actions=[
                    _include(
                        "pid_control_pkg",
                        "position_pid_controller.launch.py",
                        config_arg,
                    )
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    _include(
                        "activity_control_pkg",
                        "route_test.launch.py",
                        route_config_arg,
                    )
                ],
            ),
        ]
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(resolve_config_path()),
                description="Single drone configuration YAML.",
            ),
            DeclareLaunchArgument(
                "apriltag_target_id",
                default_value="",
                description="Optional per-order pickup AprilTag ID override.",
            ),
            DeclareLaunchArgument(
                "destination_key",
                default_value="delivery_point_1",
                description="Delivery route branch selected from the fleet order plan.",
            ),
            OpaqueFunction(function=_launch_demo),
        ]
    )
