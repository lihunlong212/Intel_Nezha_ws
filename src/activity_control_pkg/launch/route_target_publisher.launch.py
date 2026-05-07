from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    LaunchDescription = Any
    Node = Any


def generate_launch_description():
    launch_module = import_module("launch")
    launch_ros_actions = import_module("launch_ros.actions")
    LaunchDescription = getattr(launch_module, "LaunchDescription")
    Node = getattr(launch_ros_actions, "Node")

    route_params = {
        # Frames and target output
        "map_frame": "map",
        "laser_link_frame": "laser_link",
        "output_topic": "/target_position",
        # Reach tolerances
        "position_tolerance_cm": 6.0,
        "yaw_tolerance_deg": 5.0,
        "height_tolerance_cm": 6.0,
        # Visual takeover gating
        "visual_align_pixel_threshold": 100.0,
        "visual_align_required_frames": 3,
        "visual_takeover_timeout_sec": 15.0,
        "fine_data_stale_timeout_sec": 0.5,
        # 抓取航点（type=2）参数
        "pickup_align_altitude_cm": 50.0,
        "pickup_grab_altitude_cm": 20.0,
        "pickup_hold_at_grab_sec": 1.0,
        "pickup_observe_sec": 1.0,
        "pickup_max_attempts": 3,
        "circle_lost_window_sec": 1.0,
        # 投放航点（type=3）参数
        "drop_altitude_cm": 50.0,
        "drop_servo_down_duration_sec": 2.0,
        "drop_magnet_off_delay_sec": 1.0,
    }

    return LaunchDescription([
        Node(
            package="activity_control_pkg",
            executable="route_target_publisher_node",
            name="route_target_publisher",
            output="screen",
            parameters=[route_params],
        )
    ])
