from __future__ import annotations

import json
import threading
import time
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String

from robot_task_interfaces.action import DispatchOrder


ORDERS_TOPIC = "/fleet/orders"
EVENTS_TOPIC = "/fleet/order_events"
AIAGENT_DISPATCH_ACTION = "/aiagent/dispatch_order"


class GoalContext:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.events: list[dict[str, Any]] = []


class DispatchTopicHubServer(Node):
    """Action server that broadcasts orders and relays device events."""

    def __init__(self) -> None:
        super().__init__("dispatch_topic_hub_server")
        self._callback_group = ReentrantCallbackGroup()
        self._contexts: dict[str, GoalContext] = {}
        self._contexts_lock = threading.Lock()
        self._order_pub = self.create_publisher(String, ORDERS_TOPIC, 50)
        self._event_sub = self.create_subscription(
            String,
            EVENTS_TOPIC,
            self._on_device_event,
            50,
            callback_group=self._callback_group,
        )
        self._server = ActionServer(
            self,
            DispatchOrder,
            AIAGENT_DISPATCH_ACTION,
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )
        self._timeout_sec = float(self.declare_parameter("result_timeout_sec", 600.0).value)
        self.get_logger().info(f"dispatch topic hub ready: {AIAGENT_DISPATCH_ACTION}")
        self.get_logger().info(f"orders={ORDERS_TOPIC}, events={EVENTS_TOPIC}")

    def goal_callback(self, goal_request: DispatchOrder.Goal) -> GoalResponse:
        self.get_logger().info(
            (
                f"received goal task_id={goal_request.task_id} "
                f"action={goal_request.action} "
                f"target={goal_request.device_type}/{goal_request.device_id} "
                f"item={goal_request.item} quantity={goal_request.quantity}"
            )
        )
        if not goal_request.task_id:
            self.get_logger().warning("rejecting goal without task_id")
            return GoalResponse.REJECT
        if goal_request.item and goal_request.quantity <= 0:
            self.get_logger().warning("rejecting goal with non-positive quantity")
            return GoalResponse.REJECT
        if goal_request.action in {"call_car", "call_drone"} and not goal_request.device_id:
            self.get_logger().warning("rejecting targeted goal without device_id")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle) -> DispatchOrder.Result:
        goal = goal_handle.request
        task_id = goal.task_id
        ctx = GoalContext()
        with self._contexts_lock:
            self._contexts[task_id] = ctx

        try:
            order = dict(message_to_ordereddict(goal))
            order["event"] = "order"
            order["created_at"] = time.time()
            self._publish_json(self._order_pub, order)

            deadline = time.monotonic() + self._timeout_sec
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._publish_json(
                        self._order_pub,
                        {
                            "event": "cancel",
                            "task_id": task_id,
                            "device_id": goal.device_id,
                            "created_at": time.time(),
                        },
                    )
                    goal_handle.canceled()
                    return self._make_result(task_id, False, "CANCELED", "task canceled")

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    goal_handle.abort()
                    return self._make_result(
                        task_id,
                        False,
                        "DEVICE_RESULT_TIMEOUT",
                        f"no device result within {self._timeout_sec:.1f}s",
                    )

                for event in self._wait_events(ctx, min(0.25, remaining)):
                    maybe_result = self._handle_event(goal_handle, event)
                    if maybe_result is not None:
                        return maybe_result
        finally:
            with self._contexts_lock:
                if self._contexts.get(task_id) is ctx:
                    self._contexts.pop(task_id, None)

        goal_handle.abort()
        return self._make_result(task_id, False, "ROS_SHUTDOWN", "rclpy stopped")

    def _on_device_event(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"invalid device event JSON: {exc}")
            return
        if not isinstance(payload, dict):
            return
        task_id = str(payload.get("task_id") or "")
        if not task_id:
            return
        with self._contexts_lock:
            ctx = self._contexts.get(task_id)
        if ctx is None:
            return
        with ctx.condition:
            ctx.events.append(payload)
            ctx.condition.notify_all()

    def _wait_events(self, ctx: GoalContext, timeout_sec: float) -> list[dict[str, Any]]:
        with ctx.condition:
            if not ctx.events:
                ctx.condition.wait(timeout=max(0.01, timeout_sec))
            events = list(ctx.events)
            ctx.events.clear()
            return events

    def _handle_event(self, goal_handle, event: dict[str, Any]):
        event_type = str(event.get("event") or "feedback")
        task_id = str(event.get("task_id") or goal_handle.request.task_id)
        detail = str(event.get("detail") or "")

        terminal_events = {"result", "done", "succeeded", "failed", "aborted", "canceled"}
        if event_type not in terminal_events:
            return None

        success = bool(event.get("success", event_type in {"done", "succeeded"}))
        if event_type in {"failed", "aborted"}:
            success = False
        final_state = str(event.get("final_state") or ("SUCCEEDED" if success else "FAILED"))
        if success:
            goal_handle.succeed()
        elif event_type == "canceled":
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return self._make_result(task_id, success, final_state, detail)

    def _make_result(
        self,
        task_id: str,
        success: bool,
        final_state: str,
        detail: str,
    ) -> DispatchOrder.Result:
        result = DispatchOrder.Result()
        result.success = success
        result.task_id = task_id
        result.final_state = final_state
        result.detail = detail
        return result

    def _publish_json(self, publisher, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DispatchTopicHubServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
