import threading
from types import SimpleNamespace

from robot_action_demo.dispatch_server import DispatchReceiver


class _Logger:
    def debug(self, _message):
        pass


def test_chinese_order_item_names_select_the_expected_task_key():
    assert DispatchReceiver._normalize_item("华为手机") == "huawei"
    assert DispatchReceiver._normalize_item("苹果手机") == "apple"
    assert DispatchReceiver._normalize_item("小米手机") == "xiaomi"


def test_handoff_plan_selects_car1_home():
    destination, error = DispatchReceiver._resolve_order_destination(
        {
            "action": "call_drone",
            "device_type": "drone",
            "plan": "drone_to_car_handoff",
            "handoff_location": "car1_home",
            "dst_location": "car1_home",
        }
    )
    assert error is None
    assert destination == "car1_home"


def test_direct_plan_selects_delivery_point_1():
    destination, error = DispatchReceiver._resolve_order_destination(
        {
            "action": "call_drone",
            "device_type": "drone",
            "plan": "drone_direct_bundle_delivery",
            "delivery_location": "delivery_point_1",
        }
    )
    assert error is None
    assert destination == "delivery_point_1"


def test_legacy_direct_plan_and_empty_plan_remain_compatible():
    assert DispatchReceiver._resolve_order_destination(
        {
            "action": "call_drone",
            "device_type": "drone",
            "plan": "direct_drone_delivery",
            "dst_location": "delivery_point_1",
        }
    ) == ("delivery_point_1", None)
    assert DispatchReceiver._resolve_order_destination(
        {
            "action": "call_drone",
            "device_type": "drone",
            "plan": "",
            "final_dst_location": "delivery_point_1",
        }
    ) == ("delivery_point_1", None)


def test_invalid_plan_location_or_device_is_rejected():
    assert DispatchReceiver._resolve_order_destination(
        {
            "action": "call_drone",
            "device_type": "drone",
            "plan": "return_home",
            "dst_location": "delivery_point_1",
        }
    )[1]
    assert DispatchReceiver._resolve_order_destination(
        {
            "action": "call_drone",
            "device_type": "drone",
            "plan": "drone_to_car_handoff",
            "dst_location": "delivery_point_1",
        }
    )[1]
    assert DispatchReceiver._resolve_order_destination(
        {
            "action": "call_drone",
            "plan": "drone_to_car_handoff",
            "device_type": "car",
            "dst_location": "car1_home",
        }
    )[1]
    assert DispatchReceiver._resolve_order_destination(
        {
            "action": "call_drone",
            "device_type": "drone",
            "plan": "drone_to_car_handoff",
        }
    )[1]
    assert DispatchReceiver._resolve_order_destination(
        {
            "device_type": "drone",
            "plan": "drone_direct_bundle_delivery",
            "dst_location": "delivery_point_1",
        }
    )[1]


def test_terminal_result_is_published_only_once():
    events = []
    receiver = SimpleNamespace(
        _state_lock=threading.RLock(),
        _result_task_ids=set(),
        _device_id="drone1",
        get_logger=lambda: _Logger(),
        _publish_order_event=events.append,
    )

    DispatchReceiver._publish_result(receiver, "task-1", True, "DELIVERED", "drop complete")
    DispatchReceiver._publish_result(receiver, "task-1", True, "SUCCEEDED", "landed")

    assert len(events) == 1
    assert events[0]["final_state"] == "DELIVERED"
