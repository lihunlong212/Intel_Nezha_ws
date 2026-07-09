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
SQUARE_COLOR_TOPIC = "/vision_square_color"
TARGET_SQUARE_COLOR = "red"
RED_HUE_LOW_MIN = "0"
RED_HUE_LOW_MAX = "10"
RED_HUE_HIGH_MIN = "170"
RED_HUE_HIGH_MAX = "179"
RED_SATURATION_MIN = "60"
RED_VALUE_MIN = "50"
BLUE_HUE_MIN = "58"
BLUE_HUE_MAX = "111"
BLUE_SATURATION_MIN = "49"
BLUE_VALUE_MIN = "75"
BLACK_VALUE_MAX = "80"
MIN_SQUARE_AREA = "340.0"
MIN_SQUARE_FILL_RATIO = "0.22"
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
    square_color_topic = LaunchConfiguration("square_color_topic")
    target_square_color = LaunchConfiguration("target_square_color")
    red_hue_low_min = LaunchConfiguration("red_hue_low_min")
    red_hue_low_max = LaunchConfiguration("red_hue_low_max")
    red_hue_high_min = LaunchConfiguration("red_hue_high_min")
    red_hue_high_max = LaunchConfiguration("red_hue_high_max")
    red_saturation_min = LaunchConfiguration("red_saturation_min")
    red_value_min = LaunchConfiguration("red_value_min")
    blue_hue_min = LaunchConfiguration("blue_hue_min")
    blue_hue_max = LaunchConfiguration("blue_hue_max")
    blue_saturation_min = LaunchConfiguration("blue_saturation_min")
    blue_value_min = LaunchConfiguration("blue_value_min")
    black_value_max = LaunchConfiguration("black_value_max")
    min_square_area = LaunchConfiguration("min_square_area")
    min_square_fill_ratio = LaunchConfiguration("min_square_fill_ratio")
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
                description="Topic for switching vision target mode: 0 idle, 1 color square, 2 AprilTag.",
            ),
            DeclareLaunchArgument(
                "square_color_topic",
                default_value=SQUARE_COLOR_TOPIC,
                description="Topic for switching square color: red, black, or blue.",
            ),
            DeclareLaunchArgument(
                "target_square_color",
                default_value=TARGET_SQUARE_COLOR,
                description="Initial square color: red, black, or blue.",
            ),
            DeclareLaunchArgument(
                "red_hue_low_min",
                default_value=RED_HUE_LOW_MIN,
                description="Minimum OpenCV HSV hue for the lower red range.",
            ),
            DeclareLaunchArgument(
                "red_hue_low_max",
                default_value=RED_HUE_LOW_MAX,
                description="Maximum OpenCV HSV hue for the lower red range.",
            ),
            DeclareLaunchArgument(
                "red_hue_high_min",
                default_value=RED_HUE_HIGH_MIN,
                description="Minimum OpenCV HSV hue for the upper red range.",
            ),
            DeclareLaunchArgument(
                "red_hue_high_max",
                default_value=RED_HUE_HIGH_MAX,
                description="Maximum OpenCV HSV hue for the upper red range.",
            ),
            DeclareLaunchArgument(
                "red_saturation_min",
                default_value=RED_SATURATION_MIN,
                description="Minimum HSV saturation for the red square target.",
            ),
            DeclareLaunchArgument(
                "red_value_min",
                default_value=RED_VALUE_MIN,
                description="Minimum HSV value/brightness for the red square target.",
            ),
            DeclareLaunchArgument(
                "blue_hue_min",
                default_value=BLUE_HUE_MIN,
                description="Minimum OpenCV HSV hue for the blue square target.",
            ),
            DeclareLaunchArgument(
                "blue_hue_max",
                default_value=BLUE_HUE_MAX,
                description="Maximum OpenCV HSV hue for the blue square target.",
            ),
            DeclareLaunchArgument(
                "blue_saturation_min",
                default_value=BLUE_SATURATION_MIN,
                description="Minimum HSV saturation for the blue square target.",
            ),
            DeclareLaunchArgument(
                "blue_value_min",
                default_value=BLUE_VALUE_MIN,
                description="Minimum HSV value/brightness for the blue square target.",
            ),
            DeclareLaunchArgument(
                "black_value_max",
                default_value=BLACK_VALUE_MAX,
                description="Maximum HSV value/brightness for the black square target.",
            ),
            DeclareLaunchArgument(
                "min_square_area",
                default_value=MIN_SQUARE_AREA,
                description="Minimum contour area for the color square target.",
            ),
            DeclareLaunchArgument(
                "min_square_fill_ratio",
                default_value=MIN_SQUARE_FILL_RATIO,
                description="Minimum contour area / bounding rectangle area ratio for the color square target.",
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
                        "square_color_topic": square_color_topic,
                        "target_square_color": target_square_color,
                        "red_hue_low_min": ParameterValue(red_hue_low_min, value_type=int),
                        "red_hue_low_max": ParameterValue(red_hue_low_max, value_type=int),
                        "red_hue_high_min": ParameterValue(red_hue_high_min, value_type=int),
                        "red_hue_high_max": ParameterValue(red_hue_high_max, value_type=int),
                        "red_saturation_min": ParameterValue(
                            red_saturation_min, value_type=int
                        ),
                        "red_value_min": ParameterValue(red_value_min, value_type=int),
                        "blue_hue_min": ParameterValue(blue_hue_min, value_type=int),
                        "blue_hue_max": ParameterValue(blue_hue_max, value_type=int),
                        "blue_saturation_min": ParameterValue(
                            blue_saturation_min, value_type=int
                        ),
                        "blue_value_min": ParameterValue(blue_value_min, value_type=int),
                        "black_value_max": ParameterValue(black_value_max, value_type=int),
                        "min_square_area": ParameterValue(min_square_area, value_type=float),
                        "min_square_fill_ratio": ParameterValue(
                            min_square_fill_ratio, value_type=float
                        ),
                        "apriltag_dictionary": apriltag_dictionary,
                        "apriltag_target_id": ParameterValue(apriltag_target_id, value_type=int),
                    }
                ],
            ),
        ]
    )
