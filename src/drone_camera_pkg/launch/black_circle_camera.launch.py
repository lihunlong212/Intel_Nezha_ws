from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


CAMERA_DEVICE = "/dev/video0"
FRAME_WIDTH = "640"
FRAME_HEIGHT = "480"
FPS = "15.0"
SHOW_PREVIEW = "false"
WINDOW_NAME = "drone_camera_preview"
FINE_DATA_TOPIC = "/fine_data"
VISION_MODE_TOPIC = "/vision_target_mode"
BLACK_THRESHOLD = "50"
MIN_CIRCLE_AREA = "100.0"
MIN_CIRCULARITY = "0.45"
APRILTAG_DICTIONARY = "DICT_APRILTAG_36h11"
APRILTAG_TARGET_ID = "-1"


def generate_launch_description():
    camera_device = LaunchConfiguration("camera_device")
    frame_width = LaunchConfiguration("frame_width")
    frame_height = LaunchConfiguration("frame_height")
    fps = LaunchConfiguration("fps")
    show_preview = LaunchConfiguration("show_preview")
    window_name = LaunchConfiguration("window_name")
    fine_data_topic = LaunchConfiguration("fine_data_topic")
    vision_mode_topic = LaunchConfiguration("vision_mode_topic")
    black_threshold = LaunchConfiguration("black_threshold")
    min_circle_area = LaunchConfiguration("min_circle_area")
    min_circularity = LaunchConfiguration("min_circularity")
    apriltag_dictionary = LaunchConfiguration("apriltag_dictionary")
    apriltag_target_id = LaunchConfiguration("apriltag_target_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_device",
                default_value=CAMERA_DEVICE,
                description="Camera device path, for example /dev/video0.",
            ),
            DeclareLaunchArgument(
                "frame_width",
                default_value=FRAME_WIDTH,
                description="Camera frame width.",
            ),
            DeclareLaunchArgument(
                "frame_height",
                default_value=FRAME_HEIGHT,
                description="Camera frame height.",
            ),
            DeclareLaunchArgument(
                "fps",
                default_value=FPS,
                description="Camera frames per second.",
            ),
            DeclareLaunchArgument(
                "show_preview",
                default_value=SHOW_PREVIEW,
                description="Show the OpenCV camera preview window.",
            ),
            DeclareLaunchArgument(
                "window_name",
                default_value=WINDOW_NAME,
                description="OpenCV preview window name.",
            ),
            DeclareLaunchArgument(
                "fine_data_topic",
                default_value=FINE_DATA_TOPIC,
                description="Topic for publishing center offsets as Int32MultiArray.",
            ),
            DeclareLaunchArgument(
                "vision_mode_topic",
                default_value=VISION_MODE_TOPIC,
                description="Topic for switching vision target mode: 0 idle, 1 black circle, 2 AprilTag.",
            ),
            DeclareLaunchArgument(
                "black_threshold",
                default_value=BLACK_THRESHOLD,
                description="Gray threshold for black target segmentation. Larger values detect lighter dark objects.",
            ),
            DeclareLaunchArgument(
                "min_circle_area",
                default_value=MIN_CIRCLE_AREA,
                description="Minimum contour area for the black circle target.",
            ),
            DeclareLaunchArgument(
                "min_circularity",
                default_value=MIN_CIRCULARITY,
                description="Minimum circularity, where 1.0 is a perfect circle.",
            ),
            DeclareLaunchArgument(
                "apriltag_dictionary",
                default_value=APRILTAG_DICTIONARY,
                description="OpenCV ArUco AprilTag dictionary name.",
            ),
            DeclareLaunchArgument(
                "apriltag_target_id",
                default_value=APRILTAG_TARGET_ID,
                description="-1 accepts any AprilTag id; otherwise only the specified id is used.",
            ),
            Node(
                package="drone_camera_pkg",
                executable="drone_camera_node",
                name="drone_camera_node",
                output="screen",
                parameters=[
                    {
                        "camera_device": camera_device,
                        "frame_width": ParameterValue(frame_width, value_type=int),
                        "frame_height": ParameterValue(frame_height, value_type=int),
                        "fps": ParameterValue(fps, value_type=float),
                        "show_preview": ParameterValue(show_preview, value_type=bool),
                        "window_name": window_name,
                        "fine_data_topic": fine_data_topic,
                        "vision_mode_topic": vision_mode_topic,
                        "black_threshold": ParameterValue(black_threshold, value_type=int),
                        "min_circle_area": ParameterValue(min_circle_area, value_type=float),
                        "min_circularity": ParameterValue(min_circularity, value_type=float),
                        "apriltag_dictionary": apriltag_dictionary,
                        "apriltag_target_id": ParameterValue(apriltag_target_id, value_type=int),
                    }
                ],
            ),
        ]
    )
