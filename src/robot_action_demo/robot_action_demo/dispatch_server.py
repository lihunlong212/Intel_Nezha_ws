from __future__ import annotations

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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty

from robot_task_interfaces.action import DispatchOrder


PACKAGE_NAME = "robot_action_demo"
DEFAULT_CONFIG_NAME = "task_launch_map.yaml"


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


class _LocalDomainListener:
    """在指定 DOMAIN（任务子进程的本地 DOMAIN）下监听 /pickup_done 和 /drop_done。

    action server 主体跑在 DOMAIN=10（外部通信），但 demo1.launch 子进程跑在 DOMAIN=0（本地）。
    用第二个独立的 rclpy Context 订阅本地 DOMAIN 的事件话题。
    """

    def __init__(self, parent_logger, domain_id: int) -> None:
        self._logger = parent_logger
        self.pickup_done = threading.Event()
        self.pickup_failed = threading.Event()
        self.drop_done = threading.Event()
        self._context = Context()
        self._context.init(domain_id=domain_id)
        self._node = Node("dispatch_local_listener", context=self._context)
        self._node.create_subscription(Empty, "/pickup_done", self._on_pickup, 10)
        self._node.create_subscription(Empty, "/pickup_failed", self._on_pickup_failed, 10)
        self._node.create_subscription(Empty, "/drop_done", self._on_drop, 10)
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
        self._logger.warning("received /pickup_failed from local domain (3 attempts exhausted)")
        self.pickup_failed.set()

    def _on_drop(self, _msg: Empty) -> None:
        self._logger.info("received /drop_done from local domain")
        self.drop_done.set()

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


class DispatchActionServer(Node):
    def __init__(self) -> None:
        super().__init__("dispatch_action_server")
        self._lock = threading.Lock()
        self._busy = False
        self._active_process: subprocess.Popen | None = None
        self._target_domain_id, self._tasks = self._load_config()
        self._callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            DispatchOrder,
            "dispatch_order",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info("dispatch_order action server ready")
        self.get_logger().info(
            f"launch task domain={self._target_domain_id}, configured items={list(self._tasks.keys())}"
        )

    def goal_callback(self, goal_request: DispatchOrder.Goal) -> GoalResponse:
        self.get_logger().info(
            (
                "received goal: "
                f"task_id={goal_request.task_id} "
                f"item={goal_request.item} "
                f"quantity={goal_request.quantity} "
                f"route={goal_request.src_location}->{goal_request.dst_location}"
            )
        )

        task = self._select_task(goal_request.item)
        if task is None:
            self.get_logger().error(f"no launch task configured for item={goal_request.item!r}")
            return GoalResponse.REJECT

        with self._lock:
            if self._busy:
                self.get_logger().warning("rejecting goal because another launch task is running")
                return GoalResponse.REJECT
            self._busy = True

        self.get_logger().info(f"goal accepted, launch command: {task.display_command}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        task = self._select_task(goal.item)
        if task is None:
            return self._finish_failed(goal_handle, goal.task_id, "NO_LAUNCH_CONFIG", "no launch task configured")

        proc: subprocess.Popen | None = None
        listener: _LocalDomainListener | None = None
        try:
            # 1. 在任务子进程的本地 DOMAIN 起一个独立监听器，订阅 /pickup_done /drop_done
            try:
                listener = _LocalDomainListener(self.get_logger(), int(self._target_domain_id))
            except Exception as exc:
                return self._finish_failed(
                    goal_handle, goal.task_id, "LISTENER_INIT_FAILED",
                    f"failed to init local-domain listener: {exc}",
                )

            # 2. 启动 demo1.launch 子进程（DOMAIN=本地 ID）
            try:
                proc = self._start_launch_subprocess(task)
            except OSError as exc:
                return self._finish_failed(
                    goal_handle, goal.task_id, "LAUNCH_START_FAILED",
                    f"failed to start launch command {task.display_command}: {exc}",
                )

            with self._lock:
                self._active_process = proc
            self.get_logger().info(f"launch subprocess pid={proc.pid}")

            # 3. 反馈循环：根据 pickup_done/drop_done 事件切换状态
            feedback = DispatchOrder.Feedback()
            start_time = time.monotonic()

            while rclpy.ok():
                # 投递完成 → 任务成功结束（用这个判断完成，不等子进程退出）
                if listener.drop_done.is_set():
                    elapsed_total = time.monotonic() - start_time
                    self.get_logger().info(
                        f"drop_done received, finishing action successfully (elapsed={elapsed_total:.1f}s)"
                    )
                    # 任务成功后停掉 launch 子进程
                    if proc.poll() is None:
                        self._stop_subprocess(proc)
                    goal_handle.succeed()
                    result = DispatchOrder.Result()
                    result.success = True
                    result.task_id = goal.task_id
                    result.final_state = "SUCCEEDED"
                    result.detail = "arrived"
                    return result

                # 抓取 3 次失败 → 任务失败结束（只有一个货物，抓不到就直接失败）
                if listener.pickup_failed.is_set():
                    elapsed_total = time.monotonic() - start_time
                    self.get_logger().error(
                        f"pickup_failed received after {elapsed_total:.1f}s, aborting task as FAILED"
                    )
                    if proc.poll() is None:
                        self._stop_subprocess(proc)
                    return self._finish_failed(
                        goal_handle, goal.task_id, "PICKUP_FAILED",
                        "pickup attempts exhausted (3/3), cargo not grabbed",
                    )

                # 子进程异常退出（在 drop_done 之前退出）→ 失败
                if proc.poll() is not None:
                    return self._finish_failed(
                        goal_handle, goal.task_id, "LAUNCH_EXITED",
                        f"launch exited (code={proc.returncode}) before drop completed: {task.display_command}",
                    )

                # 用户取消
                if goal_handle.is_cancel_requested:
                    self._stop_subprocess(proc)
                    goal_handle.canceled()
                    result = DispatchOrder.Result()
                    result.success = False
                    result.task_id = goal.task_id
                    result.final_state = "CANCELED"
                    result.detail = f"canceled: {task.display_command}"
                    self.get_logger().info(f"goal canceled: {goal.task_id}")
                    return result

                elapsed = time.monotonic() - start_time
                feedback.current_state = "RUNNING"
                feedback.progress = float(elapsed)
                if listener.pickup_done.is_set():
                    feedback.detail = "正在送货"
                else:
                    feedback.detail = "正在前往取货点"
                goal_handle.publish_feedback(feedback)

                self.get_logger().info(
                    f"task {goal.task_id} elapsed={elapsed:.1f}s phase={feedback.detail}"
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
                self._busy = False

        return self._finish_failed(goal_handle, goal.task_id, "NODE_STOPPED", "rclpy stopped before drop completed")

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

    def _finish_failed(self, goal_handle, task_id: str, final_state: str, detail: str):
        self.get_logger().error(detail)
        goal_handle.abort()
        result = DispatchOrder.Result()
        result.success = False
        result.task_id = task_id
        result.final_state = final_state
        result.detail = detail
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DispatchActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
