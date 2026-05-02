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
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

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
        elapsed = 0
        try:
            try:
                proc = self._start_launch_subprocess(task)
            except OSError as exc:
                return self._finish_failed(
                    goal_handle,
                    goal.task_id,
                    "LAUNCH_START_FAILED",
                    f"failed to start launch command {task.display_command}: {exc}",
                )

            with self._lock:
                self._active_process = proc
            self.get_logger().info(f"launch subprocess pid={proc.pid}")

            feedback = DispatchOrder.Feedback()
            while rclpy.ok():
                if proc.poll() is not None:
                    if proc.returncode == 0:
                        goal_handle.succeed()
                        result = DispatchOrder.Result()
                        result.success = True
                        result.task_id = goal.task_id
                        result.final_state = "FINISHED"
                        result.detail = f"launch finished successfully: {task.display_command}"
                        return result

                    return self._finish_failed(
                        goal_handle,
                        goal.task_id,
                        "LAUNCH_EXITED",
                        f"launch exited unexpectedly with code={proc.returncode}: {task.display_command}",
                    )

                if goal_handle.is_cancel_requested:
                    self._stop_subprocess(proc)
                    goal_handle.canceled()
                    result = DispatchOrder.Result()
                    result.success = False
                    result.task_id = goal.task_id
                    result.final_state = "CANCELED"
                    result.detail = f"launch canceled after {elapsed}s: {task.display_command}"
                    self.get_logger().info(f"goal canceled: {goal.task_id}")
                    return result

                elapsed += 1
                feedback.current_state = "RUNNING"
                feedback.progress = 0.0
                feedback.detail = (
                    f"domain {self._target_domain_id}, elapsed {elapsed}s, command: {task.display_command}"
                )
                goal_handle.publish_feedback(feedback)
                self.get_logger().info(
                    f"launch running in domain {self._target_domain_id} "
                    f"(elapsed={elapsed}s, task={goal.task_id})"
                )
                time.sleep(1.0)
        finally:
            if proc is not None and proc.poll() is None:
                self._stop_subprocess(proc)
            with self._lock:
                if self._active_process is proc:
                    self._active_process = None
                self._busy = False

        return self._finish_failed(goal_handle, goal.task_id, "NODE_STOPPED", "rclpy stopped before launch finished")

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
