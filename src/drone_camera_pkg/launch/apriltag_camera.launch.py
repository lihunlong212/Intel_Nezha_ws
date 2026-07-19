from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    names_and_defaults = {
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
        for name, value in names_and_defaults.items()
    ]
    parameters = {
        "camera_device": LaunchConfiguration("camera_device"),
        "frame_width": ParameterValue(LaunchConfiguration("frame_width"), value_type=int),
        "frame_height": ParameterValue(LaunchConfiguration("frame_height"), value_type=int),
        "fps": ParameterValue(LaunchConfiguration("fps"), value_type=float),
        "show_preview": ParameterValue(LaunchConfiguration("show_preview"), value_type=bool),
        "window_name": LaunchConfiguration("window_name"),
        "fine_data_topic": LaunchConfiguration("fine_data_topic"),
        "vision_mode_topic": LaunchConfiguration("vision_mode_topic"),
        "apriltag_dictionary": LaunchConfiguration("apriltag_dictionary"),
        "apriltag_target_id": ParameterValue(
            LaunchConfiguration("apriltag_target_id"), value_type=int
        ),
    }
    return LaunchDescription(
        arguments
        + [
            Node(
                package="drone_camera_pkg",
                executable="drone_camera_node",
                name="drone_camera_node",
                output="screen",
                parameters=[parameters],
            )
        ]
    )
