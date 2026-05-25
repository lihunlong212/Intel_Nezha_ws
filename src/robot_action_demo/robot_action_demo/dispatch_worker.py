from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String


PACKAGE_NAME = "robot_action_demo"
DEFAULT_CONFIG_NAME = "task_launch_map.yaml"
ORDERS_TOPIC = "/fleet/orders"
EVENTS_TOPIC = "/fleet/order_events"
STATUS_TOPIC = "/fleet/device_status"


class LaunchTask:
    def __init__(self, package: str, launch_file: str, args: list[str] | None = None) -> None:
        self.package = package
        self.launch_file = launch_file
        self.args = args or []

    @property
    def command(self) -> list[str]:
        return ["ros2", "launch", self.package, self.launch_file, *self.args]

    @property
    def display_command(self) -> str:
        return " ".join(self.command)


class LocalDomainListener:
    """Listens for local task completion topics in the task ROS_DOMAIN_ID."""

    def __init__(self, parent_logger, domain_id: int) -> None:
        self._logger = parent_logger
        self.pickup_done = threading.Event()
        self.pickup_failed = threading.Event()
        self.drop_done = threading.Event()
        self.drop_failed = threading.Event()
        self._context = Context()
        self._context.init(domain_id=domain_id)
        self._node = Node("dispatch_worker_local_listener", context=self._context)
        self._node.create_subscription(Empty, "/pickup_done", self._on_pickup, 10)
        self._node.create_subscription(Empty, "/pickup_failed", self._on_pickup_failed, 10)
        self._node.create_subscription(Empty, "/drop_done", self._on_drop, 10)
        self._node.create_subscription(Empty, "/drop_failed", self._on_drop_failed, 10)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        self._logger.info(f"local-domain listener started on DOMAIN={domain_id}")

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception as exc:  # pragma: no cover
            self._logger.error(f"local listener spin crashed: {exc}")

    def _on_pickup(self, _msg: Empty) -> None:
        self._logger.info("received /pickup_done from local domain")
        self.pickup_done.set()

    def _on_pickup_failed(self, _msg: Empty) -> None:
        self._logger.warning("received /pickup_failed from local domain")
        self.pickup_failed.set()

    def _on_drop(self, _msg: Empty) -> None:
        self._logger.info("received /drop_done from local domain")
        self.drop_done.set()

    def _on_drop_failed(self, _msg: Empty) -> None:
        self._logger.error("received /drop_failed from local domain")
        self.drop_failed.set()

    def shutdown(self) -> None:
        try:
            self._executor.shutdown()
        except Exception:  # pragma: no cover
            pass
        try:
            self._node.destroy_node()
        except Exception:  # pragma: no cover
            pass
        try:
            self._context.shutdown()
        except Exception:  # pragma: no cover
            pass


def parse_args(args: list[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="Device worker for dispatch orders.")
    parser.add_argument("--device-id", default=os.environ.get("DISPATCH_DEVICE_ID", "drone1"))
    parser.add_argument("--device-type", default=os.environ.get("DISPATCH_DEVICE_TYPE", "drone"))
    parser.add_argument("--cargo-item", default=os.environ.get("DISPATCH_CARGO_ITEM", ""))
    parser.add_argument("--cargo-quantity", type=int, default=int(os.environ.get("DISPATCH_CARGO_QTY", "0")))
    parser.add_argument("--status-period", type=float, default=1.0)
    parsed, ros_args = parser.parse_known_args(args)
    return parsed, ros_args


class DispatchWorker(Node):
    def __init__(self, worker_args: argparse.Namespace) -> None:
        super().__init__("dispatch_worker")
        self._args = worker_args
        self._lock = threading.Lock()
        self._busy = False
        self._active_task_id = ""
        self._active_cancel = threading.Event()
        self._active_process: subprocess.Popen | None = None
        self._target_domain_id, self._tasks = self._load_config()

        self._events_pub = self.create_publisher(String, EVENTS_TOPIC, 50)
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self._orders_sub = self.create_subscription(String, ORDERS_TOPIC, self._on_order, 50)
        self._status_timer = self.create_timer(
            max(0.2, float(worker_args.status_period)),
            self._publish_status,
        )
        self.get_logger().info(
            (
                f"dispatch worker ready: {worker_args.device_type}/{worker_args.device_id}; "
                f"orders={ORDERS_TOPIC}, events={EVENTS_TOPIC}, local_domain={self._target_domain_id}"
            )
        )

    def _on_order(self, msg: String) -> None:
        try:
            order = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"invalid order JSON: {exc}")
            return
        if not isinstance(order, dict):
            return

        event = str(order.get("event") or "")
        if event == "cancel":
            self._handle_cancel(order)
            return
        if event != "order":
            return

        task_id = str(order.get("task_id") or "")
        target_id = str(order.get("device_id") or "")
        target_type = str(order.get("device_type") or "")
        self.get_logger().info(f"saw order task_id={task_id} target={target_type}/{target_id}")

        if target_id and target_id != self._args.device_id:
            return
        if target_type and target_type != self._args.device_type:
            return

        task = self._select_task(str(order.get("item") or ""))
        if task is None:
            self._publish_event(
                {
                    "event": "result",
                    "task_id": task_id,
                    "device_id": self._args.device_id,
                    "device_type": self._args.device_type,
                    "success": False,
                    "final_state": "NO_LAUNCH_CONFIG",
                    "detail": "no launch task configured for this item",
                }
            )
            return

        with self._lock:
            if self._busy:
                self._publish_event(
                    {
                        "event": "result",
                        "task_id": task_id,
                        "device_id": self._args.device_id,
                        "device_type": self._args.device_type,
                        "success": False,
                        "final_state": "DEVICE_BUSY",
                        "detail": f"{self._args.device_id} is already running {self._active_task_id}",
                    }
                )
                return
            self._busy = True
            self._active_task_id = task_id
            self._active_cancel.clear()

        threading.Thread(target=self._execute_order, args=(order, task), daemon=True).start()

    def _handle_cancel(self, order: dict[str, Any]) -> None:
        task_id = str(order.get("task_id") or "")
        target_id = str(order.get("device_id") or "")
        with self._lock:
            matches = self._busy and self._active_task_id == task_id
            matches = matches and (not target_id or target_id == self._args.device_id)
            proc = self._active_process if matches else None
            if matches:
                self._active_cancel.set()
        if proc is not None:
            self.get_logger().info(f"cancel requested for task {task_id}")
            self._stop_subprocess(proc)

    def _execute_order(self, order: dict[str, Any], task: LaunchTask) -> None:
        task_id = str(order.get("task_id") or "")
        proc: subprocess.Popen | None = None
        listener: LocalDomainListener | None = None
        start_time = time.monotonic()
        try:
            self._publish_event(
                {
                    "event": "feedback",
                    "task_id": task_id,
                    "device_id": self._args.device_id,
                    "device_type": self._args.device_type,
                    "current_state": "ACCEPTED",
                    "progress": 0.05,
                    "detail": f"{self._args.device_id} accepted task {task_id}",
                }
            )

            try:
                listener = LocalDomainListener(self.get_logger(), int(self._target_domain_id))
                proc = self._start_launch_subprocess(task)
            except Exception as exc:
                self._publish_result(task_id, False, "LAUNCH_START_FAILED", str(exc))
                return

            with self._lock:
                self._active_process = proc
            self.get_logger().info(f"launch subprocess pid={proc.pid}: {task.display_command}")

            while rclpy.ok():
                elapsed = time.monotonic() - start_time
                if self._active_cancel.is_set():
                    if proc.poll() is None:
                        self._stop_subprocess(proc)
                    self._publish_result(task_id, False, "CANCELED", "task canceled", "canceled")
                    return

                if listener.drop_done.is_set():
                    if proc.poll() is None:
                        self._stop_subprocess(proc)
                    self._publish_result(task_id, True, "SUCCEEDED", "drop completed")
                    return

                if listener.pickup_failed.is_set():
                    if proc.poll() is None:
                        self._stop_subprocess(proc)
                    self._publish_result(task_id, False, "PICKUP_FAILED", "pickup attempts exhausted")
                    return

                if listener.drop_failed.is_set():
                    if proc.poll() is None:
                        self._stop_subprocess(proc)
                    self._publish_result(task_id, False, "DROP_FAILED", "drop alignment timed out")
                    return

                if proc.poll() is not None:
                    self._publish_result(
                        task_id,
                        False,
                        "LAUNCH_EXITED",
                        f"launch exited with code {proc.returncode} before drop completed",
                    )
                    return

                if listener.pickup_done.is_set():
                    progress = min(0.95, 0.50 + elapsed / 240.0)
                    state = "DELIVERING"
                    detail = "cargo picked up, delivering"
                else:
                    progress = min(0.45, 0.05 + elapsed / 240.0)
                    state = "PICKING_UP"
                    detail = "moving to pickup point"
                self._publish_event(
                    {
                        "event": "feedback",
                        "task_id": task_id,
                        "device_id": self._args.device_id,
                        "device_type": self._args.device_type,
                        "current_state": state,
                        "progress": progress,
                        "detail": detail,
                    }
                )
                time.sleep(1.0)
        finally:
            if proc is not None and proc.poll() is None:
                self._stop_subprocess(proc)
            if listener is not None:
                listener.shutdown()
            with self._lock:
                if self._active_process is proc:
                    self._active_process = None
                if self._active_task_id == task_id:
                    self._active_task_id = ""
                self._busy = False

    def _publish_result(
        self,
        task_id: str,
        success: bool,
        final_state: str,
        detail: str,
        event: str = "result",
    ) -> None:
        self._publish_event(
            {
                "event": event,
                "task_id": task_id,
                "device_id": self._args.device_id,
                "device_type": self._args.device_type,
                "success": success,
                "final_state": final_state,
                "detail": detail,
                "progress": 1.0 if success else 0.0,
            }
        )

    def _load_config(self) -> tuple[str, dict[str, LaunchTask]]:
        config_path = self._get_config_path()
        with config_path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        target_domain_id = str(config.get("target_domain_id", "0"))
        tasks_config = config.get("tasks", {})
        tasks: dict[str, LaunchTask] = {}
        for item, task_config in tasks_config.items():
            package = str(task_config["package"])
            launch_file = str(task_config["launch_file"])
            args = [str(arg) for arg in task_config.get("args", [])]
            tasks[str(item)] = LaunchTask(package=package, launch_file=launch_file, args=args)

        if not tasks:
            raise RuntimeError(f"no launch tasks configured in {config_path}")
        return target_domain_id, tasks

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
        return self._tasks.get(item) or self._tasks.get("*")

    def _start_launch_subprocess(self, task: LaunchTask) -> subprocess.Popen:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = self._target_domain_id
        self.get_logger().info(
            f"starting launch subprocess in domain {self._target_domain_id}: {task.display_command}"
        )

        kwargs: dict[str, Any] = {"env": env}
        if os.name != "nt":
            kwargs["start_new_session"] = True
        return subprocess.Popen(task.command, **kwargs)

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
            self.get_logger().warning("launch did not stop after SIGINT/terminate, killing")

        if os.name != "nt":
            os.killpg(proc.pid, signal.SIGTERM)
        else:
            proc.kill()

        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            if os.name != "nt":
                os.killpg(proc.pid, signal.SIGKILL)
            else:
                proc.kill()
            proc.wait(timeout=2.0)

    def _publish_event(self, payload: dict[str, Any]) -> None:
        payload.setdefault("created_at", time.time())
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._events_pub.publish(msg)
        self.get_logger().info(f"event: {msg.data}")

    def _publish_status(self) -> None:
        with self._lock:
            busy = self._busy
            active_task_id = self._active_task_id
        payload = {
            "device_id": self._args.device_id,
            "device_type": self._args.device_type,
            "has_cargo": bool(self._args.cargo_item),
            "cargo_item": self._args.cargo_item,
            "cargo_quantity": int(self._args.cargo_quantity),
            "healthy": True,
            "available": not busy,
            "working": busy,
            "active_task_id": active_task_id,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(msg)


def main(args=None) -> None:
    worker_args, ros_args = parse_args(args)
    rclpy.init(args=ros_args)
    node = DispatchWorker(worker_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
