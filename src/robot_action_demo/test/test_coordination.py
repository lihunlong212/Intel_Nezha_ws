from robot_action_demo.coordination import (
    DELIVERY_RELEASE_STATES,
    PICKUP_RELEASE_STATES,
    default_transit_y_cm,
    device_priority,
    normalize_height_source,
    predecessor_reached,
    select_predecessor,
)


def test_device_priority_maps_to_local_domain_number():
    assert device_priority("drone1") == 1
    assert device_priority("drone2") == 2
    assert device_priority("drone3") == 3


def test_default_transit_y_depends_on_drone_number():
    assert default_transit_y_cm("drone1") == 186.0
    assert default_transit_y_cm("drone2") == -186.0
    assert default_transit_y_cm("drone3") == 186.0


def test_height_source_aliases_are_normalized():
    assert normalize_height_source("laser_array") == "laser_array"
    assert normalize_height_source("uart_to_stm32") == "uart_to_stm32"
    assert normalize_height_source("uart_to_32") == "uart_to_stm32"
    assert normalize_height_source("UART") == "uart_to_stm32"


def test_drone3_uses_nearest_busy_predecessor():
    peers = {
        "drone1": {"available": False, "current_state": "DELIVERING"},
        "drone2": {"available": False, "current_state": "ORDER_ACCEPTED"},
    }
    assert select_predecessor("drone3", peers) == "drone2"


def test_drone3_skips_available_or_unseen_drone2():
    peers = {
        "drone1": {"available": False, "current_state": "DELIVERING"},
        "drone2": {"available": True, "current_state": "IDLE"},
    }
    assert select_predecessor("drone3", peers) == "drone1"
    assert select_predecessor("drone3", {"drone1": peers["drone1"]}) == "drone1"


def test_pickup_waits_for_predecessor_delivery():
    predecessor = "drone1"
    peers = {predecessor: {"available": False, "current_state": "PICKING_UP"}}
    assert not predecessor_reached(predecessor, peers, PICKUP_RELEASE_STATES)
    peers[predecessor]["current_state"] = "DELIVERING"
    assert predecessor_reached(predecessor, peers, PICKUP_RELEASE_STATES)


def test_delivery_waits_for_predecessor_drop_completion():
    predecessor = "drone2"
    peers = {predecessor: {"available": False, "current_state": "DELIVERING"}}
    assert not predecessor_reached(predecessor, peers, DELIVERY_RELEASE_STATES)
    peers[predecessor]["current_state"] = "DELIVERED"
    assert predecessor_reached(predecessor, peers, DELIVERY_RELEASE_STATES)


def test_stale_last_state_does_not_auto_release():
    peers = {"drone1": {"available": False, "current_state": "ORDER_ACCEPTED"}}
    assert not predecessor_reached("drone1", peers, PICKUP_RELEASE_STATES)
