from __future__ import annotations

import fcntl
import json
import os
import select
import signal
import struct
import subprocess
import termios
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from robot_action_demo.dispatch_worker import LaunchTask, LocalDomainListener, LocalTelemetryListener
from robot_task_interfaces.action import DispatchOrder


PACKAGE_NAME = "robot_action_demo"
DEFAULT_CONFIG_NAME = "task_launch_map.yaml"
LOCAL_DEVICE_ID = "drone1"
DRONE2_DEVICE_ID = "drone2"
DRONE2_SERIAL_PORT = "/dev/ttyS2"
DRONE2_SERIAL_BAUD = 115200
DRONE2_START_FRAME = b"\xAA\x01\x01\xFF"
DRONE2_ONLINE_FRAME = b"\xAA\xFF"
DRONE2_DONE_FRAME = b"\xBB\xFF"
DRONE2_COMMAND_PERIOD_SEC = 5.0
DRONE2_ONLINE_TIMEOUT_SEC = 5.0
DRONE2_FEEDBACK_PERIOD_SEC = 1.0
FLEET_DEVICE_STATUS_TOPIC = "/fleet/device_status"
FLEET_DEVICE_DIAGNOSTICS_TOPIC = "/fleet/device_diagnostics"
FLEET_ORDERS_TOPIC = "/fleet/orders"
FLEET_ORDER_EVENTS_TOPIC = "/fleet/order_events"


class FleetOrderGoalHandle:
    """Minimal Action-like handle used for a fleet order received as JSON."""

    def __init__(self, order: dict[str, object]) -> None:
        self.request = SimpleNamespace(
            task_id=str(order.get("task_id") or ""),
            device_id=str(order.get("device_id") or ""),
            item=str(order.get("item") or ""),
        )

    @property
    def is_cancel_requested(self) -> bool:
        return False

    def succeed(self) -> None:
        pass

    def abort(self) -> None:
        pass

    def canceled(self) -> None:
        pass


class Drone2SerialBridge:
    """Always-on raw serial bridge between drone1 and drone2."""

    def __init__(self, logger, port: str = DRONE2_SERIAL_PORT, baud: int = DRONE2_SERIAL_BAUD) -> None:
        self._logger = logger
        self._port = port
        self._baud = baud
        self._fd: int | None = None
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._delivery_done_event = threading.Event()
        self._rx_buffer = bytearray()
        self._last_online_time: float | None = None
        self._last_error = ""
        self._next_open_warning_time = 0.0
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="drone2_serial_reader",
            daemon=True,
        )
        with self._lock:
            self._open_serial_locked()
        self._reader_thread.start()

    def shutdown(self) -> None:
        self._stop_event.set()
        self._reader_thread.join(timeout=2.0)
        with self._lock:
            self._close_serial_locked()

    def is_open(self) -> bool:
        with self._lock:
            return self._fd is not None

    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    def reset_delivery_done(self) -> None:
        self._delivery_done_event.clear()

    def delivery_done(self) -> bool:
        return self._delivery_done_event.is_set()

    def is_online(self, now: float | None = None) -> bool:
        status = self.status(now)
        return bool(status["drone2_online"])

    def status(self, now: float | None = None) -> dict[str, object]:
        if now is None:
            now = time.monotonic()
        with self._lock:
            last_seen = self._last_online_time
            last_error = self._last_error
            is_open = self._fd is not None
        age = None if last_seen is None else max(0.0, now - last_seen)
        return {
            "device_id": DRONE2_DEVICE_ID,
            "drone2_online": age is not None and age <= DRONE2_ONLINE_TIMEOUT_SEC,
            "last_seen_age_sec": None if age is None else round(age, 3),
            "serial_open": is_open,
            "serial_error": last_error,
        }

    def send_start_command(self) -> bool:
        return self.write_frame(DRONE2_START_FRAME)

    def write_frame(self, frame: bytes) -> bool:
        with self._lock:
            if self._fd is None and not self._open_serial_locked():
                return False
            fd = self._fd
            if fd is None:
                return False
            try:
                os.write(fd, frame)
                self._logger.info(f"drone2 serial TX: {self._format_hex(frame)}")
                return True
            except OSError as exc:
                self._last_error = f"write {self._port} failed: {exc}"
                self._logger.error(self._last_error)
                self._close_serial_locked()
                return False

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                if self._fd is None:
                    self._open_serial_locked()
                fd = self._fd
            if fd is None:
                time.sleep(1.0)
                continue

            try:
                readable, _, _ = select.select([fd], [], [], 0.2)
                if not readable:
                    continue
                data = os.read(fd, 256)
                if data:
                    self._logger.info(f"drone2 serial RX: {self._format_hex(data)}")
                    self._process_bytes(data)
            except OSError as exc:
                with self._lock:
                    self._last_error = f"read {self._port} failed: {exc}"
                    self._logger.error(self._last_error)
                    self._close_serial_locked()

    def _process_bytes(self, data: bytes) -> None:
        with self._lock:
            self._rx_buffer.extend(data)
            while self._rx_buffer:
                online_pos = self._rx_buffer.find(DRONE2_ONLINE_FRAME)
                done_pos = self._rx_buffer.find(DRONE2_DONE_FRAME)
                positions = [pos for pos in (online_pos, done_pos) if pos >= 0]
                if not positions:
                    if len(self._rx_buffer) > 1:
                        del self._rx_buffer[:-1]
                    return

                pos = min(positions)
                if pos > 0:
                    del self._rx_buffer[:pos]
                if self._rx_buffer.startswith(DRONE2_ONLINE_FRAME):
                    self._last_online_time = time.monotonic()
                    self._logger.info("received drone2 online frame: AA FF")
                    del self._rx_buffer[: len(DRONE2_ONLINE_FRAME)]
                    continue
                if self._rx_buffer.startswith(DRONE2_DONE_FRAME):
                    if not self._delivery_done_event.is_set():
                        self._logger.info("received drone2 delivery done frame: BB FF")
                    self._delivery_done_event.set()
                    del self._rx_buffer[: len(DRONE2_DONE_FRAME)]
                    continue

    def _open_serial_locked(self) -> bool:
        if self._fd is not None:
            return True
        try:
            fd = os.open(self._port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            self._configure_serial(fd)
            self._clear_modem_control_lines(fd)
            termios.tcflush(fd, termios.TCIOFLUSH)
            self._fd = fd
            self._last_error = ""
            self._logger.info(f"drone2 serial bridge opened {self._port} at {self._baud} baud")
            return True
        except OSError as exc:
            self._last_error = f"open {self._port} failed: {exc}"
            now = time.monotonic()
            if now >= self._next_open_warning_time:
                self._logger.warning(self._last_error)
                self._next_open_warning_time = now + 10.0
            return False

    def _close_serial_locked(self) -> None:
        if self._fd is None:
            return
        try:
            os.close(self._fd)
        except OSError:
            pass
        self._fd = None

    def _configure_serial(self, fd: int) -> None:
        baud_const = self._baud_to_termios(self._baud)
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        cflag = attrs[2]
        cflag &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
        if hasattr(termios, "HUPCL"):
            cflag &= ~termios.HUPCL
        if hasattr(termios, "CRTSCTS"):
            cflag &= ~termios.CRTSCTS
        cflag |= termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[2] = cflag
        attrs[3] = 0
        attrs[4] = baud_const
        attrs[5] = baud_const
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

    @staticmethod
    def _baud_to_termios(baud: int) -> int:
        attr_name = f"B{baud}"
        if not hasattr(termios, attr_name):
            raise OSError(f"unsupported baud rate: {baud}")
        return getattr(termios, attr_name)

    def _clear_modem_control_lines(self, fd: int) -> None:
        clear_bits = 0
        if hasattr(termios, "TIOCM_DTR"):
            clear_bits |= termios.TIOCM_DTR
        if hasattr(termios, "TIOCM_RTS"):
            clear_bits |= termios.TIOCM_RTS
        if not clear_bits or not hasattr(termios, "TIOCMBIC"):
            return
        try:
            fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack("I", clear_bits))
        except OSError as exc:
            self._logger.warning(f"could not clear DTR/RTS on {self._port}: {exc}")

    @staticmethod
    def _format_hex(data: bytes) -> str:
        return " ".join(f"{byte:02X}" for byte in data)


class DispatchServer(Node):
    """Single-command dispatch server: accept action goals and start the local mission launch."""

    def __init__(self) -> None:
        super().__init__("dispatch_server")
        self._target_domain_id, self._tasks = self._load_config()
        self._callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.RLock()
        self._active_process: subprocess.Popen | None = None
        self._active_local_task_id: str | None = None
        self._local_task_state = "IDLE"
        self._local_telemetry: LocalTelemetryListener | None = None
        self._active_drone2_task_id: str | None = None
        self._drone2_delivery_started_task_id: str | None = None
        self._drone2_bridge = Drone2SerialBridge(self.get_logger())
        self._device_status_pub = self.create_publisher(String, FLEET_DEVICE_STATUS_TOPIC, 10)
        self._device_diagnostics_pub = self.create_publisher(String, FLEET_DEVICE_DIAGNOSTICS_TOPIC, 10)
        self._fleet_event_pub = self.create_publisher(String, FLEET_ORDER_EVENTS_TOPIC, 50)
        self._fleet_order_sub = self.create_subscription(
            String,
            FLEET_ORDERS_TOPIC,
            self._on_fleet_order,
            50,
            callback_group=self._callback_group,
        )
        self._device_status_timer = self.create_timer(
            1.0,
            self._publish_device_status,
            callback_group=self._callback_group,
        )
        self._server = ActionServer(
            self,
            DispatchOrder,
            "/dispatch_order",
            self.execute_callback,
            callback_group=self._callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info("dispatch server ready: /dispatch_order")
        self.get_logger().info(
            f"local launch domain={self._target_domain_id}, configured tasks={sorted(self._tasks.keys())}"
        )
        self.get_logger().info(
            f"publishing drone1/drone2 status on {FLEET_DEVICE_STATUS_TOPIC} and diagnostics on "
            f"{FLEET_DEVICE_DIAGNOSTICS_TOPIC}"
        )
        self.get_logger().info(f"listening for fleet orders on {FLEET_ORDERS_TOPIC}")

    def goal_callback(self, goal_request: DispatchOrder.Goal) -> GoalResponse:
        device_id = self._normalize_device_id(goal_request.device_id)
        self.get_logger().info(
            f"received goal task_id={goal_request.task_id} device_id={device_id} item={goal_request.item}"
        )
        if not goal_request.task_id:
            self.get_logger().warning("rejecting goal without task_id")
            return GoalResponse.REJECT
        if device_id not in {LOCAL_DEVICE_ID, DRONE2_DEVICE_ID}:
            self.get_logger().warning(f"rejecting goal for unsupported device_id={goal_request.device_id!r}")
            return GoalResponse.REJECT
        if device_id == LOCAL_DEVICE_ID and self._local_task_running():
            self.get_logger().warning("rejecting drone1 goal because local mission launch is already running")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _cancel_request) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _on_fleet_order(self, msg: String) -> None:
        try:
            order = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"invalid fleet order JSON: {exc}")
            return
        if not isinstance(order, dict) or order.get("event") != "order":
            return

        device_id = self._normalize_device_id(str(order.get("device_id") or ""))
        if device_id not in {LOCAL_DEVICE_ID, DRONE2_DEVICE_ID}:
            return

        task_id = str(order.get("task_id") or "")
        if not task_id:
            self.get_logger().warning("ignoring fleet order without task_id")
            return
        self.get_logger().info(
            f"received fleet order task_id={task_id} device_id={device_id} item={order.get('item') or ''}"
        )
        threading.Thread(
            target=self._execute_fleet_order,
            args=(order, device_id),
            name=f"fleet_order_{task_id}",
            daemon=True,
        ).start()

    def _execute_fleet_order(self, order: dict[str, object], device_id: str) -> None:
        goal_handle = FleetOrderGoalHandle(order)
        if device_id == DRONE2_DEVICE_ID:
            result = self._execute_drone2_task(goal_handle)
        else:
            result = self._execute_local_task(goal_handle)
        event = {
            "event": "result",
            "task_id": result.task_id,
            "device_id": device_id,
            "device_type": "drone",
            "success": result.success,
            "final_state": result.final_state,
            "detail": result.detail,
        }
        msg = String()
        msg.data = json.dumps(event, ensure_ascii=False, separators=(",", ":"))
        self._fleet_event_pub.publish(msg)

    def execute_callback(self, goal_handle) -> DispatchOrder.Result:
        goal = goal_handle.request
        device_id = self._normalize_device_id(goal.device_id)
        if device_id == DRONE2_DEVICE_ID:
            return self._execute_drone2_task(goal_handle)
        if device_id == LOCAL_DEVICE_ID:
            return self._execute_local_task(goal_handle)
        goal_handle.abort()
        return self._make_result(
            goal.task_id,
            False,
            "UNSUPPORTED_DEVICE",
            f"unsupported device_id={goal.device_id}",
        )

    def _execute_local_task(self, goal_handle) -> DispatchOrder.Result:
        goal = goal_handle.request
        if not self._reserve_local_task(goal.task_id):
            goal_handle.abort()
            return self._make_result(
                goal.task_id,
                False,
                "LOCAL_BUSY",
                "drone1 is already running a mission",
            )

        task = self._select_task(goal.item)
        if task is None:
            goal_handle.abort()
            self._release_local_task(goal.task_id)
            return self._make_result(
                goal.task_id,
                False,
                "NO_LAUNCH_CONFIG",
                f"no launch config for item={goal.item}",
            )

        listener: LocalDomainListener | None = None
        telemetry: LocalTelemetryListener | None = None
        proc: subprocess.Popen | None = None
        try:
            telemetry = LocalTelemetryListener(
                self.get_logger(), int(self._target_domain_id), "map", "laser_link"
            )
            self._set_local_task_status(goal.task_id, "LAUNCHING", telemetry)
            listener = LocalDomainListener(self.get_logger(), int(self._target_domain_id))
            proc = self._start_launch_subprocess(task)
            with self._state_lock:
                self._active_process = proc
            self.get_logger().info(f"launch subprocess pid={proc.pid}: {task.display_command}")

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._stop_subprocess(proc)
                    goal_handle.canceled()
                    return self._make_result(goal.task_id, False, "CANCELED", "task canceled")

                if listener.drop_done.is_set():
                    self._stop_subprocess(proc)
                    goal_handle.succeed()
                    return self._make_result(goal.task_id, True, "SUCCEEDED", "drop completed")

                if listener.pickup_failed.is_set():
                    self._stop_subprocess(proc)
                    goal_handle.abort()
                    return self._make_result(goal.task_id, False, "PICKUP_FAILED", "pickup failed")

                if listener.drop_failed.is_set():
                    self._stop_subprocess(proc)
                    goal_handle.abort()
                    return self._make_result(goal.task_id, False, "DROP_FAILED", "drop failed")

                if proc.poll() is not None:
                    goal_handle.abort()
                    return self._make_result(
                        goal.task_id,
                        False,
                        "LAUNCH_EXITED",
                        f"launch exited with code {proc.returncode}",
                    )

                if listener.pickup_done.is_set():
                    self._set_local_task_state(goal.task_id, "DELIVERING")
                else:
                    self._set_local_task_state(goal.task_id, "PICKING_UP")
                time.sleep(1.0)
        except Exception as exc:
            if proc is not None and proc.poll() is None:
                self._stop_subprocess(proc)
            goal_handle.abort()
            return self._make_result(goal.task_id, False, "SERVER_ERROR", str(exc))
        finally:
            self._release_local_task(goal.task_id)
            if listener is not None:
                listener.shutdown()
            if telemetry is not None:
                telemetry.shutdown()
            with self._state_lock:
                if self._active_process is proc:
                    self._active_process = None

        goal_handle.abort()
        return self._make_result(goal.task_id, False, "ROS_SHUTDOWN", "rclpy stopped")

    def _execute_drone2_task(self, goal_handle) -> DispatchOrder.Result:
        goal = goal_handle.request
        if not self._reserve_drone2_task(goal.task_id):
            goal_handle.abort()
            return self._make_result(
                goal.task_id,
                False,
                "DRONE2_BUSY",
                self._json_detail(goal.task_id, "BUSY", self._drone2_bridge.status()),
            )

        self._drone2_bridge.reset_delivery_done()
        try:
            if not self._drone2_bridge.send_start_command():
                goal_handle.abort()
                return self._make_result(
                    goal.task_id,
                    False,
                    "DRONE2_SERIAL_ERROR",
                    self._json_detail(goal.task_id, "SERIAL_ERROR", self._drone2_bridge.status()),
                )
            self._mark_drone2_delivery_started(goal.task_id)
            self.get_logger().info("sent drone2 start frame: AA 01 01 FF")

            start_time = time.monotonic()
            next_command_time = start_time + DRONE2_COMMAND_PERIOD_SEC
            while rclpy.ok():
                now = time.monotonic()
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._make_result(
                        goal.task_id,
                        False,
                        "CANCELED",
                        self._json_detail(goal.task_id, "CANCELED", self._drone2_bridge.status(now)),
                    )

                if now >= next_command_time:
                    if not self._drone2_bridge.send_start_command():
                        goal_handle.abort()
                        return self._make_result(
                            goal.task_id,
                            False,
                            "DRONE2_SERIAL_ERROR",
                            self._json_detail(goal.task_id, "SERIAL_ERROR", self._drone2_bridge.status(now)),
                        )
                    self.get_logger().info("resent drone2 start frame: AA 01 01 FF")
                    next_command_time = now + DRONE2_COMMAND_PERIOD_SEC

                if self._drone2_bridge.delivery_done():
                    goal_handle.succeed()
                    return self._make_result(
                        goal.task_id,
                        True,
                        "DRONE2_SUCCEEDED",
                        self._json_detail(goal.task_id, "COMPLETED", self._drone2_bridge.status(now)),
                    )

                time.sleep(DRONE2_FEEDBACK_PERIOD_SEC)
        except Exception as exc:
            goal_handle.abort()
            return self._make_result(
                goal.task_id,
                False,
                "SERVER_ERROR",
                self._json_detail(goal.task_id, "SERVER_ERROR", self._drone2_bridge.status(), str(exc)),
            )
        finally:
            self._release_drone2_task(goal.task_id)

        goal_handle.abort()
        return self._make_result(
            goal.task_id,
            False,
            "ROS_SHUTDOWN",
            self._json_detail(goal.task_id, "ROS_SHUTDOWN", self._drone2_bridge.status()),
        )

    def _load_config(self) -> tuple[str, dict[str, LaunchTask]]:
        config_path = self._get_config_path()
        with config_path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        target_domain_id = str(config.get("target_domain_id", "0"))
        tasks: dict[str, LaunchTask] = {}
        for item, task_config in (config.get("tasks") or {}).items():
            tasks[str(item)] = LaunchTask(
                package=str(task_config["package"]),
                launch_file=str(task_config["launch_file"]),
                args=[str(arg) for arg in task_config.get("args", [])],
            )
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
        normalized_item = self._normalize_item(item)
        selected = self._tasks.get(normalized_item) or self._tasks.get("*")
        if selected is not None:
            self.get_logger().info(f"selected item={item} normalized={normalized_item}: {selected.display_command}")
        return selected

    @staticmethod
    def _normalize_item(item: str) -> str:
        item_lower = item.strip().lower()
        if "华为" in item_lower or "huawei" in item_lower:
            return "huawei"
        if "苹果" in item_lower or "apple" in item_lower or "iphone" in item_lower:
            return "apple"
        if "小米" in item_lower or "xiaomi" in item_lower or "mi" == item_lower:
            return "xiaomi"
        return item

    @staticmethod
    def _normalize_device_id(device_id: str) -> str:
        return device_id.strip().lower()

    def _local_task_running(self) -> bool:
        with self._state_lock:
            process_running = self._active_process is not None and self._active_process.poll() is None
            return self._active_local_task_id is not None or process_running

    def _reserve_local_task(self, task_id: str) -> bool:
        with self._state_lock:
            if self._active_local_task_id is not None or (
                self._active_process is not None and self._active_process.poll() is None
            ):
                return False
            self._active_local_task_id = task_id
            return True

    def _release_local_task(self, task_id: str) -> None:
        with self._state_lock:
            if self._active_local_task_id == task_id:
                self._active_local_task_id = None
                self._local_task_state = "IDLE"
                self._local_telemetry = None

    def _set_local_task_status(
        self, task_id: str, current_state: str, telemetry: LocalTelemetryListener
    ) -> None:
        with self._state_lock:
            if self._active_local_task_id == task_id:
                self._local_task_state = current_state
                self._local_telemetry = telemetry

    def _set_local_task_state(self, task_id: str, current_state: str) -> None:
        with self._state_lock:
            if self._active_local_task_id == task_id:
                self._local_task_state = current_state

    def _reserve_drone2_task(self, task_id: str) -> bool:
        with self._state_lock:
            if self._active_drone2_task_id is not None:
                return False
            self._active_drone2_task_id = task_id
            return True

    def _release_drone2_task(self, task_id: str) -> None:
        with self._state_lock:
            if self._active_drone2_task_id == task_id:
                self._active_drone2_task_id = None
            if self._drone2_delivery_started_task_id == task_id:
                self._drone2_delivery_started_task_id = None

    def _mark_drone2_delivery_started(self, task_id: str) -> None:
        with self._state_lock:
            if self._active_drone2_task_id == task_id:
                self._drone2_delivery_started_task_id = task_id

    def _publish_device_status(self) -> None:
        now = time.monotonic()
        status = self._drone2_bridge.status(now)
        with self._state_lock:
            local_task_id = self._active_local_task_id or ""
            local_state = self._local_task_state
            local_telemetry = self._local_telemetry
            drone2_task_id = self._drone2_delivery_started_task_id or ""
        local_position = (
            local_telemetry.get_position_cm() if local_telemetry is not None else None
        )

        drone1_payload = {
            "device_id": LOCAL_DEVICE_ID,
            "device_type": "drone",
            "healthy": True,
            "online": True,
            "available": True,
            "working": bool(local_task_id),
            "active_task_id": local_task_id,
            "current_state": local_state,
            "position_frame": "map",
            "coordinate_unit": "cm",
            "x_cm": None if local_position is None else local_position["x"],
            "y_cm": None if local_position is None else local_position["y"],
        }
        drone1_msg = String()
        drone1_msg.data = json.dumps(drone1_payload, ensure_ascii=False, separators=(",", ":"))
        self._device_status_pub.publish(drone1_msg)

        online = bool(status["drone2_online"])
        working = bool(drone2_task_id)

        status_payload = {
            "device_id": DRONE2_DEVICE_ID,
            "device_type": "drone",
            "healthy": online,
            "online": online,
            "available": online,
            "working": working,
            "active_task_id": drone2_task_id,
            "drone2_online": online,
            "current_state": "DELIVERING" if working else "IDLE",
            "delivery_state": "DELIVERING" if working else "IDLE",
            "last_seen_age_sec": status["last_seen_age_sec"],
            "serial_open": status["serial_open"],
            "serial_error": status["serial_error"],
        }
        status_msg = String()
        status_msg.data = json.dumps(status_payload, ensure_ascii=False, separators=(",", ":"))
        self._device_status_pub.publish(status_msg)

        diagnostics_payload = {
            "device_id": DRONE2_DEVICE_ID,
            "device_type": "drone",
            "online_frame": "AA FF",
            "done_frame": "BB FF",
            "start_frame": "AA 01 01 FF",
            "online_timeout_sec": DRONE2_ONLINE_TIMEOUT_SEC,
            **status,
        }
        diagnostics_msg = String()
        diagnostics_msg.data = json.dumps(diagnostics_payload, ensure_ascii=False, separators=(",", ":"))
        self._device_diagnostics_pub.publish(diagnostics_msg)

    @staticmethod
    def _json_detail(
        task_id: str,
        delivery_state: str,
        status: dict[str, object],
        message: str = "",
    ) -> str:
        payload = {
            "task_id": task_id,
            "device_id": DRONE2_DEVICE_ID,
            "delivery_state": delivery_state,
            **status,
        }
        if message:
            payload["message"] = message
        return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))

    def _start_launch_subprocess(self, task: LaunchTask) -> subprocess.Popen:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = self._target_domain_id
        self.get_logger().info(
            f"starting launch subprocess in domain {self._target_domain_id}: {task.display_command}"
        )
        kwargs = {"env": env}
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
        except subprocess.TimeoutExpired:
            if os.name != "nt":
                os.killpg(proc.pid, signal.SIGTERM)
            else:
                proc.kill()
            proc.wait(timeout=5.0)

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DispatchServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node._drone2_bridge.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
