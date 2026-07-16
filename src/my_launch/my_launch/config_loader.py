from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


PACKAGE_NAME = "my_launch"
DEFAULT_CONFIG_NAME = "drone_config.yaml"
VALID_DEVICE_IDS = ("drone1", "drone2", "drone3")
VALID_TARGET_COLORS = ("red", "black", "blue")


class DroneConfigError(ValueError):
    pass


def resolve_config_path(config_file: str | Path | None = None) -> Path:
    if config_file:
        path = Path(config_file).expanduser().resolve()
    else:
        try:
            from ament_index_python.packages import (
                PackageNotFoundError,
                get_package_share_directory,
            )
        except ImportError:
            path = Path(__file__).resolve().parents[1] / "config" / DEFAULT_CONFIG_NAME
        else:
            try:
                path = (
                    Path(get_package_share_directory(PACKAGE_NAME))
                    / "config"
                    / DEFAULT_CONFIG_NAME
                )
            except PackageNotFoundError:
                path = Path(__file__).resolve().parents[1] / "config" / DEFAULT_CONFIG_NAME
    if not path.is_file():
        raise DroneConfigError(f"drone config file does not exist: {path}")
    return path


def _mapping(parent: dict[str, Any], key: str, path: str = "") -> dict[str, Any]:
    value = parent.get(key)
    field = f"{path}.{key}" if path else key
    if not isinstance(value, dict):
        raise DroneConfigError(f"{field} must be a mapping")
    return value


def _string(parent: dict[str, Any], key: str, path: str) -> str:
    value = parent.get(key)
    if not isinstance(value, str) or not value.strip():
        raise DroneConfigError(f"{path}.{key} must be a non-empty string")
    return value.strip()


def _boolean(parent: dict[str, Any], key: str, path: str) -> bool:
    value = parent.get(key)
    if not isinstance(value, bool):
        raise DroneConfigError(f"{path}.{key} must be true or false")
    return value


def _number(parent: dict[str, Any], key: str, path: str) -> float:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DroneConfigError(f"{path}.{key} must be a number")
    return float(value)


def _integer(parent: dict[str, Any], key: str, path: str) -> int:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise DroneConfigError(f"{path}.{key} must be an integer")
    return value


def load_drone_config(config_file: str | Path | None = None) -> tuple[Path, dict[str, Any]]:
    path = resolve_config_path(config_file)
    try:
        with path.open("r", encoding="utf-8") as stream:
            loaded = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise DroneConfigError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(loaded, dict):
        raise DroneConfigError(f"drone config root must be a mapping: {path}")

    config = deepcopy(loaded)
    drone = _mapping(config, "drone")
    hardware = _mapping(config, "hardware")
    height_filter = _mapping(config, "height_filter")
    route = _mapping(config, "route")
    pid = _mapping(config, "pid")
    tasks = _mapping(config, "tasks")

    device_id = _string(drone, "device_id", "drone").lower()
    if device_id not in VALID_DEVICE_IDS:
        raise DroneConfigError(
            f"drone.device_id must be one of {VALID_DEVICE_IDS}, got {device_id!r}"
        )
    drone["device_id"] = device_id
    if _number(drone, "coordination_discovery_sec", "drone") < 0.0:
        raise DroneConfigError("drone.coordination_discovery_sec must be >= 0")

    _boolean(hardware, "use_pillar_detection", "hardware")
    if _number(hardware, "pillar_detection_timeout_sec", "hardware") < 0.0:
        raise DroneConfigError("hardware.pillar_detection_timeout_sec must be >= 0")
    fallback = _mapping(hardware, "fallback_transit_y_cm", "hardware")
    for drone_id in VALID_DEVICE_IDS:
        _number(fallback, drone_id, "hardware.fallback_transit_y_cm")

    min_cm = _number(height_filter, "min_cm", "height_filter")
    max_cm = _number(height_filter, "max_cm", "height_filter")
    if min_cm > max_cm:
        raise DroneConfigError("height_filter.min_cm must be <= height_filter.max_cm")
    if _number(height_filter, "jump_threshold_cm", "height_filter") < 0.0:
        raise DroneConfigError("height_filter.jump_threshold_cm must be >= 0")
    if _integer(height_filter, "required_frames", "height_filter") < 1:
        raise DroneConfigError("height_filter.required_frames must be >= 1")

    route_string_keys = (
        "map_frame",
        "laser_link_frame",
        "output_topic",
        "vision_mode_topic",
        "route_stage_command_topic",
    )
    for key in route_string_keys:
        _string(route, key, "route")
    route_integer_keys = ("visual_align_required_frames", "pickup_max_attempts")
    for key in route_integer_keys:
        if _integer(route, key, "route") < 1:
            raise DroneConfigError(f"route.{key} must be >= 1")
    route_number_keys = (
        "position_tolerance_cm",
        "yaw_tolerance_deg",
        "height_tolerance_cm",
        "emergency_retract_height_threshold_cm",
        "emergency_retract_z_velocity_threshold_cm_s",
        "visual_align_pixel_threshold",
        "visual_takeover_timeout_sec",
        "fine_data_stale_timeout_sec",
        "pickup_align_altitude_cm",
        "pickup_grab_altitude_cm",
        "pickup_hold_at_grab_sec",
        "pickup_check_altitude_cm",
        "pickup_check_observe_sec",
        "circle_lost_window_sec",
        "drop_altitude_cm",
        "drop_align_altitude_cm",
        "drop_servo_up_settle_sec",
        "drop_servo_down_settle_sec",
        "drop_servo_retract_delay_sec",
    )
    for key in route_number_keys:
        _number(route, key, "route")

    pid_string_keys = ("map_frame", "laser_link_frame")
    for key in pid_string_keys:
        _string(pid, key, "pid")
    pid_number_keys = (
        "control_frequency",
        "pose_data_timeout_sec",
        "kp_xy",
        "ki_xy",
        "kd_xy",
        "kp_yaw",
        "ki_yaw",
        "kd_yaw",
        "kp_z",
        "ki_z",
        "kd_z",
        "max_linear_velocity",
        "max_angular_velocity",
        "max_vertical_velocity",
        "visual_kp_x",
        "visual_ki_x",
        "visual_kd_x",
        "visual_kp_y",
        "visual_ki_y",
        "visual_kd_y",
        "visual_pixel_deadzone",
        "visual_max_xy_velocity",
        "visual_data_timeout_sec",
    )
    for key in pid_number_keys:
        _number(pid, key, "pid")
    if _number(pid, "control_frequency", "pid") <= 0.0:
        raise DroneConfigError("pid.control_frequency must be > 0")

    if "*" not in tasks:
        raise DroneConfigError("tasks must contain a '*' fallback entry")
    for item, task_value in tasks.items():
        if not isinstance(task_value, dict):
            raise DroneConfigError(f"tasks.{item} must be a mapping")
        color = _string(task_value, "target_square_color", f"tasks.{item}").lower()
        if color not in VALID_TARGET_COLORS:
            raise DroneConfigError(
                f"tasks.{item}.target_square_color must be one of {VALID_TARGET_COLORS}"
            )
        task_value["target_square_color"] = color

    return path, config


def fallback_transit_y_cm(config: dict[str, Any]) -> float:
    device_id = str(config["drone"]["device_id"])
    return float(config["hardware"]["fallback_transit_y_cm"][device_id])


def route_parameters(config: dict[str, Any]) -> dict[str, Any]:
    hardware = config["hardware"]
    height_filter = config["height_filter"]
    params = dict(config["route"])
    params.update(
        {
            "height_min_cm": height_filter["min_cm"],
            "height_max_cm": height_filter["max_cm"],
            "height_filter_jump_threshold_cm": height_filter["jump_threshold_cm"],
            "height_filter_required_frames": height_filter["required_frames"],
            "use_pillar_detection": hardware["use_pillar_detection"],
            "default_transit_y_cm": fallback_transit_y_cm(config),
            "pillar_detection_timeout_sec": hardware["pillar_detection_timeout_sec"],
        }
    )
    return params


def pid_parameters(config: dict[str, Any]) -> dict[str, Any]:
    hardware = config["hardware"]
    height_filter = config["height_filter"]
    params = dict(config["pid"])
    params.update(
        {
            "height_min_cm": height_filter["min_cm"],
            "height_max_cm": height_filter["max_cm"],
            "height_filter_jump_threshold_cm": height_filter["jump_threshold_cm"],
            "height_filter_required_frames": height_filter["required_frames"],
        }
    )
    return params


def target_color_for_item(config: dict[str, Any], item: str) -> str:
    normalized = item.strip().lower()
    task = config["tasks"].get(normalized) or config["tasks"]["*"]
    return str(task["target_square_color"])
