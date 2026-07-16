from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from my_launch.config_loader import (
    DroneConfigError,
    fallback_transit_y_cm,
    load_drone_config,
    pid_parameters,
    route_parameters,
    target_color_for_item,
)


SOURCE_CONFIG = Path(__file__).resolve().parents[1] / "config" / "drone_config.yaml"


def _base_config():
    with SOURCE_CONFIG.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _write_config(tmp_path, config):
    path = tmp_path / "drone_config.yaml"
    path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    return path


def test_default_config_shares_height_filter():
    _, config = load_drone_config(SOURCE_CONFIG)
    route = route_parameters(config)
    pid = pid_parameters(config)
    assert route["height_min_cm"] == pid["height_min_cm"] == 0.0
    assert route["height_max_cm"] == pid["height_max_cm"] == 200.0
    assert route["height_filter_jump_threshold_cm"] == 35.0
    assert route["height_filter_required_frames"] == 5


@pytest.mark.parametrize(
    ("device_id", "expected_y"),
    (("drone1", 186.0), ("drone2", -186.0), ("drone3", 186.0)),
)
def test_device_selects_fallback_transit_y(tmp_path, device_id, expected_y):
    config = _base_config()
    config["drone"]["device_id"] = device_id
    _, loaded = load_drone_config(_write_config(tmp_path, config))
    assert fallback_transit_y_cm(loaded) == expected_y


def test_task_color_mapping_and_fallback():
    _, config = load_drone_config(SOURCE_CONFIG)
    assert target_color_for_item(config, "huawei") == "red"
    assert target_color_for_item(config, "apple") == "black"
    assert target_color_for_item(config, "xiaomi") == "blue"
    assert target_color_for_item(config, "unknown") == "red"


@pytest.mark.parametrize(
    "mutate",
    (
        lambda c: c.pop("hardware"),
        lambda c: c["drone"].update(device_id="drone4"),
        lambda c: c["hardware"].update(use_pillar_detection="false"),
        lambda c: c["height_filter"].update(min_cm=201.0, max_cm=200.0),
        lambda c: c["height_filter"].update(required_frames=0),
        lambda c: c["route"].pop("pickup_grab_altitude_cm"),
        lambda c: c["pid"].pop("kp_z"),
    ),
)
def test_invalid_config_fails_fast(tmp_path, mutate):
    config = deepcopy(_base_config())
    mutate(config)
    with pytest.raises(DroneConfigError):
        load_drone_config(_write_config(tmp_path, config))
