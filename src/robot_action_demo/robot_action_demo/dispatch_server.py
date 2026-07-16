from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from robot_action_demo.coordination import (
    DELIVERY_RELEASE_STATES,
    PICKUP_RELEASE_STATES,
    VALID_DEVICE_IDS,
    default_transit_y_cm,
    device_priority,
    normalize_height_source,
    predecessor_reached,
    select_predecessor,
)
from robot_action_demo.dispatch_worker import (
    LaunchTask,
    LocalDomainListener,
    LocalTelemetryListener,
)


PACKAGE_NAME = "robot_action_demo"
DEFAULT_CONFIG_NAME = "task_launch_map.yaml"
FLEET_DEVICE_STATUS_TOPIC = "/fleet/device_status"
FLEET_ORDERS_TOPIC = "/fleet/orders"
FLEET_ORDER_EVENTS_TOPIC = "/fleet/order_events"

ROUTE_STAGE_HOLD = 0
ROUTE_STAGE_PICKUP = 1
ROUTE_STAGE_DELIVERY = 2
ROUTE_STAGE_RETURN = 3


class FleetOrderContext:
    """Minimal order view shared with the mission execution thread."""

    def __init__(
        self,
        order: dict[str, object],
        cancel_event: threading.Event,
        accepted_at: float,
    ) -> None:
        self.request = SimpleNamespace(
            task_id=str(order.get("task_id") or ""),
            device_id=str(order.get("device_id") or ""),
            item=str(order.get("item") or ""),
        )
        self._cancel_event = cancel_event
        self.accepted_at = accepted_at

    @property
    def is_cancel_requested(self) -> bool:
        return self._cancel_event.is_set()


class DispatchReceiver(Node):
    """One-shot fleet-order receiver for exactly one configured drone."""

    def __init__(self) -> None:
        super().__init__("dispatch_receiver")
        (
            configured_device_id,
            configured_discovery_sec,
            configured_height_source,
            self._tasks,
        ) = self._load_config()
        parameter_device_id = str(
            self.declare_parameter("device_id", configured_device_id).value
        )
        self._device_id = self._normalize_device_id(
            os.environ.get("DISPATCH_DEVICE_ID", parameter_device_id)
        )
        if self._device_id not in VALID_DEVICE_IDS:
            raise RuntimeError(
                f"device_id must be one of {VALID_DEVICE_IDS}, got {self._device_id!r}"
            )

        self._priority = device_priority(self._device_id)
        self._target_domain_id = str(self._priority)
        self._height_source = normalize_height_source(
            os.environ.get("DISPATCH_HEIGHT_SOURCE", configured_height_source)
        )
        self._coordination_discovery_sec = max(
            0.0,
            float(
                self.declare_parameter(
                    "coordination_discovery_sec",
                    configured_discovery_sec,
                ).value
            ),
        )

        self._callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.RLock()
        self._peer_condition = threading.Condition(self._state_lock)
        self._peer_status: dict[str, dict[str, object]] = {}
        self._mission_consumed = False
        self._active_task_id = ""
        self._current_state = "IDLE"
        self._predecessor_id: str | None = None
        self._active_process: subprocess.Popen | None = None
        self._local_telemetry: LocalTelemetryListener | None = None
        self._cancel_event: threading.Event | None = None

        self._device_status_pub = self.create_publisher(
            String, FLEET_DEVICE_STATUS_TOPIC, 10
        )
        self._fleet_event_pub = self.create_publisher(
            String, FLEET_ORDER_EVENTS_TOPIC, 50
        )
        self._fleet_order_sub = self.create_subscription(
            String,
            FLEET_ORDERS_TOPIC,
            self._on_fleet_order,
            50,
            callback_group=self._callback_group,
        )
        self._fleet_status_sub = self.create_subscription(
            String,
            FLEET_DEVICE_STATUS_TOPIC,
            self._on_fleet_device_status,
            50,
            callback_group=self._callback_group,
        )
        self._device_status_timer = self.create_timer(
            1.0,
            self._publish_device_status,
            callback_group=self._callback_group,
        )

        fleet_domain = os.environ.get("ROS_DOMAIN_ID", "0")
        if fleet_domain != "10":
            self.get_logger().warning(
                "dispatch_server should run in ROS_DOMAIN_ID=10; "
                f"current environment is {fleet_domain}"
            )
        self._publish_feedback("IDLE")
        self._publish_device_status()
        self.get_logger().info(
            f"dispatch receiver ready for {self._device_id}; fleet_domain={fleet_domain}, "
            f"local_domain={self._target_domain_id}, "
            f"height_source={self._height_source}, "
            f"discovery={self._coordination_discovery_sec:.1f}s"
        )

    def _on_fleet_device_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        device_id = self._normalize_device_id(str(payload.get("device_id") or ""))
        if device_id not in VALID_DEVICE_IDS or device_id == self._device_id:
            return
        with self._peer_condition:
            self._peer_status[device_id] = dict(payload)
            self._peer_condition.notify_all()

    def _on_fleet_order(self, msg: String) -> None:
        try:
            order = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"invalid fleet order JSON: {exc}")
            return
        if not isinstance(order, dict):
            return

        device_id = self._normalize_device_id(str(order.get("device_id") or ""))
        if device_id != self._device_id:
            return

        event = str(order.get("event") or "")
        if event in {"cancel", "session_reset"}:
            self._request_cancel(str(order.get("task_id") or ""))
            return
        if event != "order":
            return

        task_id = str(order.get("task_id") or "")
        if not task_id:
            self.get_logger().warning("ignoring fleet order without task_id")
            return

        with self._state_lock:
            if self._mission_consumed:
                self._publish_result(
                    task_id,
                    False,
                    "RESTART_REQUIRED",
                    f"{self._device_id} already consumed its one mission; restart dispatch_server",
                )
                return
            self._mission_consumed = True
            self._active_task_id = task_id
            self._current_state = "ORDER_ACCEPTED"
            self._cancel_event = threading.Event()
            cancel_event = self._cancel_event

        accepted_at = time.monotonic()
        self.get_logger().info(
            f"accepted one-shot order task_id={task_id} item={order.get('item') or ''}"
        )
        self._publish_feedback("ORDER_ACCEPTED")
        self._publish_device_status()
        threading.Thread(
            target=self._execute_order,
            args=(dict(order), cancel_event, accepted_at),
            name=f"fleet_order_{task_id}",
            daemon=True,
        ).start()

    def _execute_order(
        self,
        order: dict[str, object],
        cancel_event: threading.Event,
        accepted_at: float,
    ) -> None:
        goal = FleetOrderContext(order, cancel_event, accepted_at)
        task_id = goal.request.task_id
        result: SimpleNamespace
        try:
            result = self._execute_local_task(goal)
        except Exception as exc:  # pragma: no cover - final protection for the flight worker
            self.get_logger().error(f"mission thread crashed: {exc}")
            self._transition_state("FAILED")
            result = self._make_result(task_id, False, "SERVER_ERROR", str(exc))

        self._publish_result(
            result.task_id,
            result.success,
            result.final_state,
            result.detail,
        )

    def _execute_local_task(self, goal: FleetOrderContext) -> SimpleNamespace:
        task_id = goal.request.task_id
        task = self._select_task(goal.request.item)
        if task is None:
            self._transition_state("FAILED")
            return self._make_result(
                task_id,
                False,
                "NO_LAUNCH_CONFIG",
                f"no launch config for item={goal.request.item}",
            )

        listener: LocalDomainListener | None = None
        telemetry: LocalTelemetryListener | None = None
        proc: subprocess.Popen | None = None
        pickup_failed = False
        drop_completed = False
        try:
            listener = LocalDomainListener(self.get_logger(), int(self._target_domain_id))
            listener.publish_route_stage(ROUTE_STAGE_HOLD)
            telemetry = LocalTelemetryListener(
                self.get_logger(), int(self._target_domain_id), "map", "laser_link"
            )
            with self._state_lock:
                self._local_telemetry = telemetry

            proc = self._start_launch_subprocess(task)
            with self._state_lock:
                self._active_process = proc
            self.get_logger().info(f"launch subprocess pid={proc.pid}: {task.display_command}")

            if self._priority == 1:
                self._predecessor_id = None
            else:
                wait_result = self._wait_for_discovery_window(goal, proc)
                if wait_result is not None:
                    return wait_result
                self._predecessor_id = self._select_predecessor()

            if self._predecessor_id is not None:
                self.get_logger().info(
                    f"fixed coordination predecessor: {self._predecessor_id}"
                )
                wait_result = self._wait_for_predecessor(
                    goal,
                    proc,
                    PICKUP_RELEASE_STATES,
                    "pickup",
                )
                if wait_result is not None:
                    return wait_result
            else:
                self.get_logger().info("no active predecessor; pickup may start immediately")

            interrupted = self._check_interrupted(goal, proc)
            if interrupted is not None:
                return interrupted
            self._release_route_stage(listener, ROUTE_STAGE_PICKUP, "PICKING_UP")

            delivery_released = False
            pickup_failure_handled = False
            while rclpy.ok():
                interrupted = self._check_interrupted(goal, proc)
                if interrupted is not None:
                    return interrupted

                if listener.pickup_failed.is_set() and not pickup_failure_handled:
                    pickup_failure_handled = True
                    pickup_failed = True
                    self._transition_state("FAILED")
                    listener.publish_route_stage(ROUTE_STAGE_RETURN)
                    self.get_logger().error(
                        "pickup failed; released the built-in safe return route"
                    )

                if listener.drop_failed.is_set():
                    self._transition_state("FAILED")
                    return self._make_result(
                        task_id, False, "DROP_FAILED", "drop failed"
                    )

                if (
                    listener.pickup_done.is_set()
                    and not delivery_released
                    and not pickup_failed
                    and self._predecessor_reached(DELIVERY_RELEASE_STATES)
                ):
                    delivery_released = True
                    self._release_route_stage(
                        listener, ROUTE_STAGE_DELIVERY, "DELIVERING"
                    )

                if listener.drop_done.is_set() and not drop_completed:
                    drop_completed = True
                    self._release_route_stage(
                        listener, ROUTE_STAGE_RETURN, "DELIVERED"
                    )

                if listener.mission_complete.is_set():
                    time.sleep(1.0)
                    self._stop_subprocess(proc)
                    if pickup_failed:
                        return self._make_result(
                            task_id,
                            False,
                            "PICKUP_FAILED",
                            "pickup failed; safe return and landing completed",
                        )
                    if not drop_completed:
                        self._transition_state("FAILED")
                        return self._make_result(
                            task_id,
                            False,
                            "MISSION_COMPLETED_WITHOUT_DROP",
                            "mission completed before /drop_done",
                        )
                    self._transition_state("LANDED")
                    return self._make_result(
                        task_id,
                        True,
                        "SUCCEEDED",
                        "drop, return, and landing completed",
                    )

                with self._peer_condition:
                    self._peer_condition.wait(timeout=0.2)
        finally:
            if proc is not None and proc.poll() is None:
                self._stop_subprocess(proc)
            if listener is not None:
                listener.shutdown()
            if telemetry is not None:
                telemetry.shutdown()
            with self._state_lock:
                if self._active_process is proc:
                    self._active_process = None
                if self._local_telemetry is telemetry:
                    self._local_telemetry = None

        self._transition_state("FAILED")
        return self._make_result(task_id, False, "ROS_SHUTDOWN", "rclpy stopped")

    def _wait_for_discovery_window(
        self,
        goal: FleetOrderContext,
        proc: subprocess.Popen,
    ) -> SimpleNamespace | None:
        deadline = goal.accepted_at + self._coordination_discovery_sec
        while rclpy.ok() and time.monotonic() < deadline:
            interrupted = self._check_interrupted(goal, proc)
            if interrupted is not None:
                return interrupted
            remaining = deadline - time.monotonic()
            with self._peer_condition:
                self._peer_condition.wait(timeout=max(0.01, min(0.2, remaining)))
        return None

    def _select_predecessor(self) -> str | None:
        with self._state_lock:
            return select_predecessor(self._device_id, self._peer_status)

    def _wait_for_predecessor(
        self,
        goal: FleetOrderContext,
        proc: subprocess.Popen,
        allowed_states: set[str],
        stage_name: str,
    ) -> SimpleNamespace | None:
        predecessor = self._predecessor_id
        while rclpy.ok() and not self._predecessor_reached(allowed_states):
            interrupted = self._check_interrupted(goal, proc)
            if interrupted is not None:
                return interrupted
            with self._state_lock:
                state = str(
                    self._peer_status.get(predecessor or "", {}).get("current_state")
                    or "UNKNOWN"
                )
            self.get_logger().info(
                f"waiting to release {stage_name}: predecessor={predecessor} state={state}"
            )
            with self._peer_condition:
                self._peer_condition.wait(timeout=1.0)
        return None

    def _predecessor_reached(self, allowed_states: set[str]) -> bool:
        with self._state_lock:
            return predecessor_reached(
                self._predecessor_id,
                self._peer_status,
                allowed_states,
            )

    def _release_route_stage(
        self,
        listener: LocalDomainListener,
        route_stage: int,
        fleet_state: str,
    ) -> None:
        listener.publish_route_stage(route_stage)
        self._transition_state(fleet_state)

    def _check_interrupted(
        self,
        goal: FleetOrderContext,
        proc: subprocess.Popen,
    ) -> SimpleNamespace | None:
        if not rclpy.ok():
            self._transition_state("FAILED")
            return self._make_result(
                goal.request.task_id, False, "ROS_SHUTDOWN", "rclpy stopped"
            )
        if goal.is_cancel_requested:
            self._transition_state("FAILED")
            return self._make_result(
                goal.request.task_id, False, "CANCELED", "task canceled"
            )
        return_code = proc.poll()
        if return_code is not None:
            self._transition_state("FAILED")
            return self._make_result(
                goal.request.task_id,
                False,
                "LAUNCH_EXITED",
                f"launch exited with code {return_code}",
            )
        return None

    def _transition_state(self, new_state: str) -> None:
        with self._state_lock:
            if self._current_state == new_state:
                return
            old_state = self._current_state
            self._current_state = new_state
        self.get_logger().info(f"fleet state: {old_state} -> {new_state}")
        self._publish_feedback(new_state)
        self._publish_device_status()

    def _publish_device_status(self) -> None:
        with self._state_lock:
            telemetry = self._local_telemetry
            payload: dict[str, object] = {
                "device_id": self._device_id,
                "device_type": "drone",
                "available": not self._mission_consumed,
                "current_state": self._current_state,
                "active_task_id": self._active_task_id,
                "coordinate_unit": "cm",
                "x_cm": None,
                "y_cm": None,
                "z_cm": None,
            }
        position = telemetry.get_position_cm() if telemetry is not None else None
        if position is not None:
            payload["x_cm"] = position["x"]
            payload["y_cm"] = position["y"]
            payload["z_cm"] = position["z"]
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._device_status_pub.publish(msg)

    def _publish_feedback(self, current_state: str) -> None:
        progress_by_state = {
            "IDLE": 0.0,
            "ORDER_ACCEPTED": 0.05,
            "PICKING_UP": 0.25,
            "DELIVERING": 0.65,
            "DELIVERED": 0.9,
            "LANDED": 1.0,
            "FAILED": 0.0,
        }
        self._publish_order_event(
            {
                "event": "feedback",
                "task_id": self._active_task_id,
                "device_id": self._device_id,
                "device_type": "drone",
                "current_state": current_state,
                "progress": progress_by_state.get(current_state, 0.0),
                "detail": f"{self._device_id} state={current_state}",
            }
        )

    def _publish_result(
        self,
        task_id: str,
        success: bool,
        final_state: str,
        detail: str,
    ) -> None:
        self._publish_order_event(
            {
                "event": "result",
                "task_id": task_id,
                "device_id": self._device_id,
                "device_type": "drone",
                "success": success,
                "final_state": final_state,
                "detail": detail,
            }
        )

    def _publish_order_event(self, payload: dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._fleet_event_pub.publish(msg)

    def _request_cancel(self, task_id: str) -> None:
        with self._state_lock:
            if self._cancel_event is None:
                return
            if task_id and task_id != self._active_task_id:
                return
            self._cancel_event.set()
        with self._peer_condition:
            self._peer_condition.notify_all()

    def _load_config(self) -> tuple[str, float, str, dict[str, LaunchTask]]:
        config_path = self._get_config_path()
        with config_path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        device_id = str(config.get("device_id", "drone1"))
        discovery_sec = float(config.get("coordination_discovery_sec", 20.0))
        height_source = str(config.get("height_source", "laser_array"))
        tasks: dict[str, LaunchTask] = {}
        for item, task_config in (config.get("tasks") or {}).items():
            tasks[str(item)] = LaunchTask(
                package=str(task_config["package"]),
                launch_file=str(task_config["launch_file"]),
                args=[str(arg) for arg in task_config.get("args", [])],
            )
        if not tasks:
            raise RuntimeError(f"no launch tasks configured in {config_path}")
        return device_id, discovery_sec, height_source, tasks

    def _get_config_path(self) -> Path:
        override_path = os.environ.get("DISPATCH_TASK_CONFIG")
        if override_path:
            return Path(override_path)
        try:
            share_dir = Path(get_package_share_directory(PACKAGE_NAME))
            installed_config = share_dir / "config" / DEFAULT_CONFIG_NAME
            if installed_config.exists():
                return installed_config
        except PackageNotFoundError:
            pass
        return Path(__file__).resolve().parents[1] / "config" / DEFAULT_CONFIG_NAME

    def _select_task(self, item: str) -> LaunchTask | None:
        normalized_item = self._normalize_item(item)
        selected = self._tasks.get(normalized_item) or self._tasks.get("*")
        if selected is not None:
            self.get_logger().info(
                f"selected item={item} normalized={normalized_item}: {selected.display_command}"
            )
        return selected

    @staticmethod
    def _normalize_item(item: str) -> str:
        item_lower = item.strip().lower()
        if "华为" in item_lower or "huawei" in item_lower:
            return "huawei"
        if "苹果" in item_lower or "apple" in item_lower or "iphone" in item_lower:
            return "apple"
        if "小米" in item_lower or "xiaomi" in item_lower or item_lower == "mi":
            return "xiaomi"
        return item_lower

    @staticmethod
    def _normalize_device_id(device_id: str) -> str:
        return device_id.strip().lower()

    def _start_launch_subprocess(self, task: LaunchTask) -> subprocess.Popen:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = self._target_domain_id
        transit_y_cm = default_transit_y_cm(self._device_id)
        command = [
            *task.command,
            f"default_transit_y_cm:={transit_y_cm:.1f}",
            f"height_source:={self._height_source}",
        ]
        self.get_logger().info(
            f"starting local mission in domain {self._target_domain_id}, "
            f"fallback transit_y={transit_y_cm:.1f}cm, "
            f"height_source={self._height_source}: {' '.join(command)}"
        )
        kwargs: dict[str, object] = {"env": env}
        if os.name != "nt":
            kwargs["start_new_session"] = True
        return subprocess.Popen(command, **kwargs)

    def _stop_subprocess(self, proc: subprocess.Popen) -> None:
        if proc.poll() is not None:
            return
        self.get_logger().info(f"stopping launch subprocess pid={proc.pid}")
        if os.name != "nt":
            os.killpg(proc.pid, signal.SIGINT)
        else:
            proc.terminate()
        try:
            proc.wait(timeout=5.0)
            return
        except subprocess.TimeoutExpired:
            self.get_logger().warning("launch did not stop after SIGINT; terminating")
        if os.name != "nt":
            os.killpg(proc.pid, signal.SIGTERM)
        else:
            proc.kill()
        proc.wait(timeout=5.0)

    @staticmethod
    def _make_result(
        task_id: str,
        success: bool,
        final_state: str,
        detail: str,
    ) -> SimpleNamespace:
        return SimpleNamespace(
            success=success,
            task_id=task_id,
            final_state=final_state,
            detail=detail,
        )

    def shutdown(self) -> None:
        with self._state_lock:
            cancel_event = self._cancel_event
            proc = self._active_process
        if cancel_event is not None:
            cancel_event.set()
        if proc is not None and proc.poll() is None:
            self._stop_subprocess(proc)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DispatchReceiver()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
