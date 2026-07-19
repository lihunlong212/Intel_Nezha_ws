from __future__ import annotations

from copy import deepcopy
from math import isfinite
from pathlib import Path
from typing import Any

import yaml


PACKAGE_NAME = "my_launch"
DEFAULT_CONFIG_NAME = "drone_config.yaml"
VALID_DEVICE_IDS = ("drone1", "drone2", "drone3")
VALID_APRILTAG_DICTIONARIES = ("DICT_APRILTAG_36h11",)
VALID_ROUTE_DESTINATIONS = ("car1_home", "delivery_point_1")
TARGET_TYPE_VALUES = {"waypoint": 1, "drop": 3, "search": 4}
ROUTE_STAGE_VALUES = {"pickup": 1, "delivery": 2, "return": 3}
APRILTAG_36H11_MIN_ID = 0
APRILTAG_36H11_MAX_ID = 586


class DroneConfigError(ValueError):
    pass


def resolve_config_path(config_file: str | Path | None = None) -> Path:
    if config_file:
        path = Path(config_file).expanduser().resolve()
    else:
        try:
            from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
        except ImportError:
            path = Path(__file__).resolve().parents[1] / "config" / DEFAULT_CONFIG_NAME
        else:
            try:
                path = Path(get_package_share_directory(PACKAGE_NAME)) / "config" / DEFAULT_CONFIG_NAME
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
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
        raise DroneConfigError(f"{path}.{key} must be a finite number")
    return float(value)


def _integer(parent: dict[str, Any], key: str, path: str) -> int:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise DroneConfigError(f"{path}.{key} must be an integer")
    return value


def _validate_tag_id(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise DroneConfigError(f"{path} must be an integer")
    if not APRILTAG_36H11_MIN_ID <= value <= APRILTAG_36H11_MAX_ID:
        raise DroneConfigError(
            f"{path} must be in [{APRILTAG_36H11_MIN_ID}, {APRILTAG_36H11_MAX_ID}]"
        )
    return value


def _validate_route_point(point: Any, path: str, mission: bool) -> dict[str, Any]:
    if not isinstance(point, dict):
        raise DroneConfigError(f"{path} must be a mapping")
    result: dict[str, Any] = {}
    for key in ("x_cm", "z_cm", "yaw_deg"):
        result[key] = _number(point, key, path)
    y_value = point.get("y_cm")
    if mission and y_value == "transit_y_cm":
        result["y_cm"] = "transit_y_cm"
    elif isinstance(y_value, bool) or not isinstance(y_value, (int, float)) or not isfinite(y_value):
        raise DroneConfigError(
            f"{path}.y_cm must be a finite number"
            + (" or the exact string 'transit_y_cm'" if mission else "")
        )
    else:
        result["y_cm"] = float(y_value)
    if mission:
        type_name = _string(point, "type", path).lower()
        stage_name = _string(point, "stage", path).lower()
        if type_name not in TARGET_TYPE_VALUES:
            raise DroneConfigError(f"{path}.type must be one of {tuple(TARGET_TYPE_VALUES)}")
        if stage_name not in ROUTE_STAGE_VALUES:
            raise DroneConfigError(f"{path}.stage must be one of {tuple(ROUTE_STAGE_VALUES)}")
        result["type"] = type_name
        result["stage"] = stage_name
    return result


def _validate_routes(config: dict[str, Any]) -> None:
    routes = _mapping(config, "routes")
    for device_id in VALID_DEVICE_IDS:
        profile = _mapping(routes, device_id, "routes")
        _number(profile, "fallback_transit_y_cm", f"routes.{device_id}")
        climb = _number(profile, "post_pickup_climb_altitude_cm", f"routes.{device_id}")
        if climb < 0.0:
            raise DroneConfigError(f"routes.{device_id}.post_pickup_climb_altitude_cm must be >= 0")
        mission = profile.get("mission_prefix")
        if not isinstance(mission, list) or not mission:
            raise DroneConfigError(f"routes.{device_id}.mission_prefix must be a non-empty list")
        normalized_mission = []
        previous_stage = 0
        types: set[str] = set()
        for index, point in enumerate(mission):
            path = f"routes.{device_id}.mission_prefix[{index}]"
            normalized = _validate_route_point(point, path, True)
            stage_value = ROUTE_STAGE_VALUES[normalized["stage"]]
            if stage_value < previous_stage:
                raise DroneConfigError(f"{path}.stage must not move backwards")
            if normalized["type"] == "drop" or normalized["stage"] == "return":
                raise DroneConfigError(
                    f"{path} must not contain the destination drop or return stage"
                )
            previous_stage = stage_value
            types.add(normalized["type"])
            normalized_mission.append(normalized)
        if "search" not in types:
            raise DroneConfigError(f"routes.{device_id}.mission_prefix must contain a search target")
        profile["mission_prefix"] = normalized_mission

        destinations = _mapping(profile, "destinations", f"routes.{device_id}")
        missing_destinations = set(VALID_ROUTE_DESTINATIONS) - set(destinations)
        if missing_destinations:
            raise DroneConfigError(
                f"routes.{device_id}.destinations is missing {sorted(missing_destinations)}"
            )
        for destination_name in VALID_ROUTE_DESTINATIONS:
            destination = _mapping(
                destinations, destination_name, f"routes.{device_id}.destinations"
            )
            drop_target = destination.get("drop_target")
            destination["drop_target"] = _validate_route_point(
                drop_target,
                f"routes.{device_id}.destinations.{destination_name}.drop_target",
                False,
            )
            return_route = destination.get("return_route")
            if not isinstance(return_route, list) or not return_route:
                raise DroneConfigError(
                    f"routes.{device_id}.destinations.{destination_name}.return_route "
                    "must be a non-empty list"
                )
            destination["return_route"] = [
                _validate_route_point(
                    point,
                    f"routes.{device_id}.destinations.{destination_name}.return_route[{index}]",
                    False,
                )
                for index, point in enumerate(return_route)
            ]

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
    vision = _mapping(config, "vision")
    pillar = _mapping(config, "pillar_detection")
    tasks = _mapping(config, "tasks")

    device_id = _string(drone, "device_id", "drone").lower()
    if device_id not in VALID_DEVICE_IDS:
        raise DroneConfigError(f"drone.device_id must be one of {VALID_DEVICE_IDS}, got {device_id!r}")
    drone["device_id"] = device_id
    if _number(drone, "coordination_discovery_sec", "drone") < 0.0:
        raise DroneConfigError("drone.coordination_discovery_sec must be >= 0")

    _boolean(hardware, "use_pillar_detection", "hardware")
    if _number(hardware, "pillar_detection_timeout_sec", "hardware") < 0.0:
        raise DroneConfigError("hardware.pillar_detection_timeout_sec must be >= 0")

    dictionary = _string(vision, "apriltag_dictionary", "vision")
    if dictionary not in VALID_APRILTAG_DICTIONARIES:
        raise DroneConfigError(f"vision.apriltag_dictionary must be {VALID_APRILTAG_DICTIONARIES[0]}")
    _validate_tag_id(vision.get("default_pickup_apriltag_id"), "vision.default_pickup_apriltag_id")

    for axis in ("x", "y"):
        minimum = _number(pillar, f"{axis}_min_m", "pillar_detection")
        maximum = _number(pillar, f"{axis}_max_m", "pillar_detection")
        if minimum > maximum:
            raise DroneConfigError(f"pillar_detection.{axis}_min_m must be <= {axis}_max_m")

    min_cm = _number(height_filter, "min_cm", "height_filter")
    max_cm = _number(height_filter, "max_cm", "height_filter")
    if min_cm > max_cm:
        raise DroneConfigError("height_filter.min_cm must be <= height_filter.max_cm")
    if _number(height_filter, "jump_threshold_cm", "height_filter") < 0.0:
        raise DroneConfigError("height_filter.jump_threshold_cm must be >= 0")
    if _integer(height_filter, "required_frames", "height_filter") < 1:
        raise DroneConfigError("height_filter.required_frames must be >= 1")

    for key in ("map_frame", "laser_link_frame", "output_topic", "vision_mode_topic", "route_stage_command_topic"):
        _string(route, key, "route")
    for key in ("visual_align_required_frames", "pickup_max_attempts"):
        if _integer(route, key, "route") < 1:
            raise DroneConfigError(f"route.{key} must be >= 1")
    for key in (
        "position_tolerance_cm", "yaw_tolerance_deg", "height_tolerance_cm",
        "emergency_retract_height_threshold_cm", "emergency_retract_z_velocity_threshold_cm_s",
        "visual_align_pixel_threshold", "visual_takeover_timeout_sec", "fine_data_stale_timeout_sec",
        "pickup_align_altitude_cm", "pickup_grab_altitude_cm", "pickup_hold_at_grab_sec",
        "pickup_check_altitude_cm", "pickup_check_observe_sec", "circle_lost_window_sec",
        "drop_altitude_cm", "drop_align_altitude_cm", "drop_servo_up_settle_sec",
        "drop_servo_down_settle_sec", "drop_servo_retract_delay_sec",
    ):
        _number(route, key, "route")

    for key in ("map_frame", "laser_link_frame"):
        _string(pid, key, "pid")
    for key in (
        "control_frequency", "pose_data_timeout_sec", "kp_xy", "ki_xy", "kd_xy",
        "kp_yaw", "ki_yaw", "kd_yaw", "kp_z", "ki_z", "kd_z",
        "max_linear_velocity", "max_angular_velocity", "max_vertical_velocity",
        "visual_kp_x", "visual_ki_x", "visual_kd_x", "visual_kp_y", "visual_ki_y",
        "visual_kd_y", "visual_pixel_deadzone", "visual_max_xy_velocity", "visual_data_timeout_sec",
        "failure_hold_vertical_velocity_cm_s",
    ):
        _number(pid, key, "pid")
    if float(pid["control_frequency"]) <= 0.0:
        raise DroneConfigError("pid.control_frequency must be > 0")
    if float(pid["failure_hold_vertical_velocity_cm_s"]) <= 0.0:
        raise DroneConfigError("pid.failure_hold_vertical_velocity_cm_s must be > 0")

    if not tasks:
        raise DroneConfigError("tasks must contain at least one supported item")
    for item, task_value in tasks.items():
        if not isinstance(task_value, dict):
            raise DroneConfigError(f"tasks.{item} must be a mapping")
        task_value["target_apriltag_id"] = _validate_tag_id(
            task_value.get("target_apriltag_id"), f"tasks.{item}.target_apriltag_id"
        )

    _validate_routes(config)
    return path, config


def selected_route(config: dict[str, Any]) -> dict[str, Any]:
    return config["routes"][str(config["drone"]["device_id"])]


def fallback_transit_y_cm(config: dict[str, Any]) -> float:
    return float(selected_route(config)["fallback_transit_y_cm"])


def _flatten_route(points: list[dict[str, Any]], prefix: str, mission: bool) -> dict[str, Any]:
    values: dict[str, Any] = {
        f"{prefix}_x_cm": [float(point["x_cm"]) for point in points],
        f"{prefix}_y_cm": [0.0 if point["y_cm"] == "transit_y_cm" else float(point["y_cm"]) for point in points],
        f"{prefix}_z_cm": [float(point["z_cm"]) for point in points],
        f"{prefix}_yaw_deg": [float(point["yaw_deg"]) for point in points],
    }
    if mission:
        values[f"{prefix}_use_transit_y"] = [point["y_cm"] == "transit_y_cm" for point in points]
        values[f"{prefix}_type"] = [TARGET_TYPE_VALUES[point["type"]] for point in points]
        values[f"{prefix}_stage"] = [ROUTE_STAGE_VALUES[point["stage"]] for point in points]
    return values


def route_parameters(
    config: dict[str, Any], destination_key: str = "delivery_point_1"
) -> dict[str, Any]:
    hardware = config["hardware"]
    height_filter = config["height_filter"]
    profile = selected_route(config)
    normalized_destination = destination_key.strip()
    if normalized_destination not in VALID_ROUTE_DESTINATIONS:
        raise DroneConfigError(
            f"destination_key must be one of {VALID_ROUTE_DESTINATIONS}, "
            f"got {normalized_destination!r}"
        )
    destination = profile["destinations"][normalized_destination]
    mission = list(profile["mission_prefix"])
    mission.append(
        {
            **destination["drop_target"],
            "type": "drop",
            "stage": "delivery",
        }
    )
    mission.extend(
        {
            **point,
            "type": "waypoint",
            "stage": "return",
        }
        for point in destination["return_route"]
    )
    params = dict(config["route"])
    params.update({
        "device_id": config["drone"]["device_id"],
        "destination_key": normalized_destination,
        "height_min_cm": height_filter["min_cm"],
        "height_max_cm": height_filter["max_cm"],
        "height_filter_jump_threshold_cm": height_filter["jump_threshold_cm"],
        "height_filter_required_frames": height_filter["required_frames"],
        "use_pillar_detection": hardware["use_pillar_detection"],
        "default_transit_y_cm": profile["fallback_transit_y_cm"],
        "pillar_detection_timeout_sec": hardware["pillar_detection_timeout_sec"],
        "post_pickup_climb_altitude_cm": profile["post_pickup_climb_altitude_cm"],
    })
    params.update(_flatten_route(mission, "mission", True))
    return params


def pid_parameters(config: dict[str, Any]) -> dict[str, Any]:
    height_filter = config["height_filter"]
    params = dict(config["pid"])
    params.update({
        "height_min_cm": height_filter["min_cm"],
        "height_max_cm": height_filter["max_cm"],
        "height_filter_jump_threshold_cm": height_filter["jump_threshold_cm"],
        "height_filter_required_frames": height_filter["required_frames"],
    })
    return params


def target_apriltag_id_for_item(config: dict[str, Any], item: str) -> int | None:
    task = config["tasks"].get(item.strip().lower())
    return None if task is None else int(task["target_apriltag_id"])
