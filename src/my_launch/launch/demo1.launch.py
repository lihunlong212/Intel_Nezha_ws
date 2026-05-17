import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _package_launch(package_name: str, filename: str) -> str:
    package_share = FindPackageShare(package=package_name).find(package_name)
    return os.path.join(package_share, "launch", filename)


def generate_launch_description() -> LaunchDescription:
    camera_device = LaunchConfiguration("camera_device")
    frame_width = LaunchConfiguration("frame_width")
    frame_height = LaunchConfiguration("frame_height")
    fps = LaunchConfiguration("fps")
    show_preview = LaunchConfiguration("show_preview")
    fine_data_topic = LaunchConfiguration("fine_data_topic")
    vision_mode_topic = LaunchConfiguration("vision_mode_topic")
    black_threshold = LaunchConfiguration("black_threshold")
    min_circle_area = LaunchConfiguration("min_circle_area")
    min_circularity = LaunchConfiguration("min_circularity")
    apriltag_dictionary = LaunchConfiguration("apriltag_dictionary")
    apriltag_target_id = LaunchConfiguration("apriltag_target_id")

    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_package_launch("my_carto_pkg", "fly_carto.launch.py"))
    )
    uart_to_stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_package_launch("uart_to_stm32", "uart_to_stm32.launch.py"))
    )
    position_pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _package_launch("pid_control_pkg", "position_pid_controller.launch.py")
        )
    )
    route_test_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_package_launch("activity_control_pkg", "route_test.launch.py"))
    )
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _package_launch("drone_camera_pkg", "black_circle_camera.launch.py")
        ),
        launch_arguments={
            "camera_device": camera_device,
            "frame_width": frame_width,
            "frame_height": frame_height,
            "fps": fps,
            "show_preview": show_preview,
            "fine_data_topic": fine_data_topic,
            "vision_mode_topic": vision_mode_topic,
            "black_threshold": black_threshold,
            "min_circle_area": min_circle_area,
            "min_circularity": min_circularity,
            "apriltag_dictionary": apriltag_dictionary,
            "apriltag_target_id": apriltag_target_id,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_device",
                default_value="/dev/video0",
                description="Camera device path used by the vision node.",
            ),
            DeclareLaunchArgument(
                "frame_width",
                default_value="640",
                description="Camera frame width.",
            ),
            DeclareLaunchArgument(
                "frame_height",
                default_value="480",
                description="Camera frame height.",
            ),
            DeclareLaunchArgument(
                "fps",
                default_value="15.0",
                description="Camera frames per second.",
            ),
            DeclareLaunchArgument(
                "show_preview",
                default_value="false",
                description="Show OpenCV preview window for camera debugging.",
            ),
            DeclareLaunchArgument(
                "fine_data_topic",
                default_value="/fine_data",
                description="Vision pixel-offset topic consumed by PID and activity control.",
            ),
            DeclareLaunchArgument(
                "vision_mode_topic",
                default_value="/vision_target_mode",
                description="Vision mode topic: 0 idle, 1 black circle, 2 AprilTag.",
            ),
            DeclareLaunchArgument(
                "black_threshold",
                default_value="31",
                description="Gray threshold for black target segmentation.",
            ),
            DeclareLaunchArgument(
                "min_circle_area",
                default_value="800.0",
                description="Minimum contour area for the black circle target.",
            ),
            DeclareLaunchArgument(
                "min_circularity",
                default_value="0.45",
                description="Minimum circularity, where 1.0 is a perfect circle.",
            ),
            DeclareLaunchArgument(
                "apriltag_dictionary",
                default_value="DICT_APRILTAG_36h11",
                description="OpenCV ArUco AprilTag dictionary name.",
            ),
            DeclareLaunchArgument(
                "apriltag_target_id",
                default_value="-1",
                description="-1 accepts any AprilTag id; otherwise only the specified id is used.",
            ),
            fly_carto_launch,
            uart_to_stm32_launch,
            position_pid_controller_launch,
            route_test_launch,
            camera_launch,
        ]
    )
