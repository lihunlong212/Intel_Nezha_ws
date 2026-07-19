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
    selected_pillar_region,
    target_apriltag_id_for_item,
)


SOURCE_CONFIG = Path(__file__).resolve().parents[1] / "config" / "drone_config.yaml"


def _base_config():
    with SOURCE_CONFIG.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _write_config(tmp_path, config):
    path = tmp_path / "drone_config.yaml"
    path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    return path


def test_default_config_shares_height_filter_and_flattens_route():
    _, config = load_drone_config(SOURCE_CONFIG)
    route = route_parameters(config)
    pid = pid_parameters(config)
    assert route["height_min_cm"] == pid["height_min_cm"] == 0.0
    assert route["height_max_cm"] == pid["height_max_cm"] == 200.0
    assert route["height_filter_jump_threshold_cm"] == 35.0
    assert route["height_filter_required_frames"] == 5
    assert pid["failure_hold_vertical_velocity_cm_s"] == 5.0
    assert len(route["mission_x_cm"]) == 8
    assert route["mission_use_transit_y"] == [False, False, True, True, False, False, False, False]
    assert route["mission_type"] == [1, 4, 1, 1, 3, 1, 1, 1]
    assert route["mission_stage"] == [1, 1, 1, 2, 2, 3, 3, 3]
    assert not any("failure_route" in key for key in route)


@pytest.mark.parametrize(
    ("device_id", "expected_y"),
    (("drone1", 186.0), ("drone2", -186.0), ("drone3", 186.0)),
)
def test_device_selects_route_and_fallback(tmp_path, device_id, expected_y):
    config = _base_config()
    config["drone"]["device_id"] = device_id
    config["routes"][device_id]["mission_prefix"][0]["x_cm"] = expected_y
    _, loaded = load_drone_config(_write_config(tmp_path, config))
    assert fallback_transit_y_cm(loaded) == expected_y
    assert route_parameters(loaded)["mission_x_cm"][0] == expected_y


@pytest.mark.parametrize(
    ("device_id", "expected_region"),
    (
        ("drone1", (0.20, 1.00, 0.40, 3.60)),
        ("drone2", (0.20, 1.00, -3.60, -0.40)),
        ("drone3", (0.20, 1.00, 0.40, 3.60)),
    ),
)
def test_device_selects_independent_pillar_region(tmp_path, device_id, expected_region):
    config = _base_config()
    config["drone"]["device_id"] = device_id
    _, loaded = load_drone_config(_write_config(tmp_path, config))
    region = selected_pillar_region(loaded)
    assert (
        region["x_min_m"],
        region["x_max_m"],
        region["y_min_m"],
        region["y_max_m"],
    ) == expected_region


def test_reversed_pillar_endpoints_are_normalized(tmp_path):
    config = _base_config()
    config["drone"]["device_id"] = "drone2"
    config["pillar_detection"]["drone2"].update(
        x_min_m=1.25,
        x_max_m=-0.25,
        y_min_m=0.40,
        y_max_m=-3.60,
    )
    _, loaded = load_drone_config(_write_config(tmp_path, config))
    assert selected_pillar_region(loaded) == {
        "x_min_m": -0.25,
        "x_max_m": 1.25,
        "y_min_m": -3.60,
        "y_max_m": 0.40,
    }


def test_each_drone_has_two_independent_destination_branches():
    config = _base_config()
    for index, device_id in enumerate(("drone1", "drone2", "drone3"), start=1):
        destinations = config["routes"][device_id]["destinations"]
        assert set(destinations) >= {"car1_home", "delivery_point_1"}
        destinations["car1_home"]["drop_target"]["x_cm"] = 100.0 + index
        destinations["car1_home"]["return_route"][0]["x_cm"] = 200.0 + index
        destinations["delivery_point_1"]["drop_target"]["x_cm"] = 300.0 + index
        destinations["delivery_point_1"]["return_route"][0]["x_cm"] = 400.0 + index

    for index, device_id in enumerate(("drone1", "drone2", "drone3"), start=1):
        selected = deepcopy(config)
        selected["drone"]["device_id"] = device_id
        # Validation normally happens in load_drone_config; this source structure is already valid.
        car_route = route_parameters(selected, "car1_home")
        direct_route = route_parameters(selected, "delivery_point_1")
        assert car_route["mission_type"].count(3) == 1
        assert direct_route["mission_type"].count(3) == 1
        drop_index = car_route["mission_type"].index(3)
        assert car_route["mission_x_cm"][drop_index] == 100.0 + index
        assert direct_route["mission_x_cm"][drop_index] == 300.0 + index
        assert car_route["mission_x_cm"][drop_index + 1] == 200.0 + index
        assert direct_route["mission_x_cm"][drop_index + 1] == 400.0 + index


def test_task_apriltag_mapping_has_no_unknown_fallback():
    _, config = load_drone_config(SOURCE_CONFIG)
    assert target_apriltag_id_for_item(config, "huawei") == 1
    assert target_apriltag_id_for_item(config, "apple") == 2
    assert target_apriltag_id_for_item(config, "xiaomi") == 3
    assert target_apriltag_id_for_item(config, "unknown") is None


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
        lambda c: c["pid"].pop("failure_hold_vertical_velocity_cm_s"),
        lambda c: c["pid"].update(failure_hold_vertical_velocity_cm_s=0.0),
        lambda c: c["vision"].update(apriltag_dictionary="DICT_4X4_50"),
        lambda c: c["tasks"]["huawei"].update(target_apriltag_id=999),
        lambda c: c["pillar_detection"]["drone1"].update(x_min_m="invalid"),
        lambda c: c["pillar_detection"].pop("drone3"),
        lambda c: c["routes"]["drone1"]["mission_prefix"][0].update(y_cm="bad_placeholder"),
        lambda c: c["routes"]["drone1"]["mission_prefix"][0].update(stage="delivery"),
        lambda c: c["routes"]["drone1"]["destinations"].pop("car1_home"),
        lambda c: c["routes"]["drone3"]["destinations"]["car1_home"].pop(
            "drop_target"
        ),
        lambda c: c["routes"]["drone2"]["destinations"]["delivery_point_1"].update(
            return_route=[]
        ),
    ),
)
def test_invalid_config_fails_fast(tmp_path, mutate):
    config = deepcopy(_base_config())
    mutate(config)
    with pytest.raises(DroneConfigError):
        load_drone_config(_write_config(tmp_path, config))
