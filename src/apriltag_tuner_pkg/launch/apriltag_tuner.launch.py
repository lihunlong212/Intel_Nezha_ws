from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
        DeclareLaunchArgument("frame_width", default_value="640"),
        DeclareLaunchArgument("frame_height", default_value="480"),
        DeclareLaunchArgument("fps", default_value="15.0"),
        DeclareLaunchArgument("target_id", default_value="-1"),
        DeclareLaunchArgument(
            "output_file", default_value="/tmp/apriltag_tuned.yaml"
        ),
    ]

    return LaunchDescription(
        arguments
        + [
            Node(
                package="apriltag_tuner_pkg",
                executable="apriltag_tuner_node",
                name="apriltag_tuner",
                output="screen",
                parameters=[
                    {
                        "camera_device": LaunchConfiguration("camera_device"),
                        "frame_width": ParameterValue(
                            LaunchConfiguration("frame_width"), value_type=int
                        ),
                        "frame_height": ParameterValue(
                            LaunchConfiguration("frame_height"), value_type=int
                        ),
                        "fps": ParameterValue(
                            LaunchConfiguration("fps"), value_type=float
                        ),
                        "target_id": ParameterValue(
                            LaunchConfiguration("target_id"), value_type=int
                        ),
                        "output_file": LaunchConfiguration("output_file"),
                    }
                ],
            )
        ]
    )
