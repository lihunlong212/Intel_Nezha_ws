"""Compatibility wrapper for the AprilTag-only camera launch."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    argument_defaults = {
        "camera_device": "/dev/video0",
        "frame_width": "640",
        "frame_height": "480",
        "fps": "15.0",
        "show_preview": "false",
        "window_name": "drone_camera_preview",
        "fine_data_topic": "/fine_data",
        "vision_mode_topic": "/vision_target_mode",
        "apriltag_dictionary": "DICT_APRILTAG_36h11",
        "apriltag_target_id": "1",
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in argument_defaults.items()
    ]
    launch_path = os.path.join(
        FindPackageShare("drone_camera_pkg").find("drone_camera_pkg"),
        "launch",
        "apriltag_camera.launch.py",
    )
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            name: LaunchConfiguration(name) for name in argument_defaults
        }.items(),
    )
    return LaunchDescription(arguments + [include])
