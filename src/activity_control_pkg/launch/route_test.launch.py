from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:  # pragma: no cover
    LaunchDescription = Any
    Node = Any


def generate_launch_description():
    launch_module = import_module("launch")
    launch_actions = import_module("launch.actions")
    launch_substitutions = import_module("launch.substitutions")
    launch_ros_actions = import_module("launch_ros.actions")
    LaunchDescription = getattr(launch_module, "LaunchDescription")
    DeclareLaunchArgument = getattr(launch_actions, "DeclareLaunchArgument")
    LaunchConfiguration = getattr(launch_substitutions, "LaunchConfiguration")
    Node = getattr(launch_ros_actions, "Node")
    use_pillar_detection = LaunchConfiguration("use_pillar_detection")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_pillar_detection",
            default_value="true",
            description="Use two detected pillars to calculate the route transit Y coordinate.",
        ),
        Node(
            package="activity_control_pkg",
            executable="route_test_node",
            # 不设置 name=...：本可执行文件内部有 2 个节点
            #   - RouteTargetPublisherNode 名为 "route_target_publisher"
            #   - RouteTestNode 名为 "route_test_node"
            # 设了 name 会把两个都强制改成同名，触发 DDS 同名冲突，
            # 导致 /height 等订阅在节点之间错乱，状态机判断失败。
            output="screen",
            parameters=[
                {
                    "map_frame": "map",
                    "laser_link_frame": "laser_link",
                    "output_topic": "/target_position",
                    "vision_mode_topic": "/vision_target_mode",
                    "position_tolerance_cm": 6.0,
                    "yaw_tolerance_deg": 8.0,
                    "height_tolerance_cm": 8.0,
                    "height_filter_jump_threshold_cm": 30.0,
                    "height_filter_required_frames": 5,
                    "use_pillar_detection": use_pillar_detection,
                    "default_transit_y_cm": 125.0,
                    "emergency_retract_height_threshold_cm": 50.0,
                    "emergency_retract_z_velocity_threshold_cm_s": 20.0,
                    "visual_align_pixel_threshold": 35.0,
                    "visual_align_required_frames": 3,
                    "visual_takeover_timeout_sec": 15.0,
                    "fine_data_stale_timeout_sec": 0.13,
                    # 抓取航点（type=2）参数
                    "pickup_align_altitude_cm": 27.0,
                    "pickup_grab_altitude_cm": 14.0,
                    "pickup_hold_at_grab_sec": 1.0,
                    "pickup_check_altitude_cm": 60.0,
                    "pickup_check_observe_sec": 2.0,
                    "pickup_max_attempts": 3,
                    "circle_lost_window_sec": 1.0,
                    # 投放航点（type=3）参数
                    "drop_altitude_cm": 42.0,
                    "drop_align_altitude_cm": 50.0,
                    "drop_servo_up_settle_sec": 1.0,
                    "drop_servo_down_settle_sec": 1.0,
                    "drop_servo_retract_delay_sec": 0.5,
                }
            ],
        )
    ])
