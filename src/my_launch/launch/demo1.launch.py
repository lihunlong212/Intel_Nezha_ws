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
    VALID_TARGET_COLORS,
    DroneConfigError,
    load_drone_config,
    resolve_config_path,
    target_color_for_item,
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

    color_override = LaunchConfiguration("target_square_color").perform(context).strip().lower()
    if color_override:
        if color_override not in VALID_TARGET_COLORS:
            raise DroneConfigError(
                f"target_square_color must be one of {VALID_TARGET_COLORS}, "
                f"got {color_override!r}"
            )
        target_color = color_override
    else:
        target_color = target_color_for_item(config, "*")

    config_arg = {"config_file": str(config_path)}
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
                actions=[_include("pillar_detector_pkg", "pillar_detector.launch.py")],
            )
        )
    actions.extend(
        [
            TimerAction(
                period=4.0,
                actions=[
                    _include(
                        "drone_camera_pkg",
                        "black_circle_camera.launch.py",
                        {"target_square_color": target_color},
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
                        config_arg,
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
                "target_square_color",
                default_value="",
                description="Optional per-order color override: red, black, or blue.",
            ),
            OpaqueFunction(function=_launch_demo),
        ]
    )
