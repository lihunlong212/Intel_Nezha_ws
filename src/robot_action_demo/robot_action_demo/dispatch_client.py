from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robot_task_interfaces.action import DispatchOrder


AIAGENT_DISPATCH_ACTION = "/aiagent/dispatch_order"


class DispatchActionClient(Node):
    def __init__(self) -> None:
        super().__init__("dispatch_action_client")
        self._client = ActionClient(self, DispatchOrder, AIAGENT_DISPATCH_ACTION)
        self._goal_done = False
        self._result = None

    def send_goal(self, args: argparse.Namespace):
        goal_msg = DispatchOrder.Goal()
        goal_msg.task_id = args.task_id
        goal_msg.action = args.action
        goal_msg.device_type = args.device_type
        goal_msg.device_id = args.device_id
        goal_msg.item = args.item
        goal_msg.quantity = args.quantity
        goal_msg.src_location = args.src
        goal_msg.dst_location = args.dst
        goal_msg.transfer_to_device_type = args.transfer_to_device_type
        goal_msg.transfer_to_device_id = args.transfer_to_device_id
        goal_msg.final_dst_location = args.final_dst_location
        goal_msg.plan = args.plan

        self.get_logger().info(f"waiting for {AIAGENT_DISPATCH_ACTION} action server...")
        self._client.wait_for_server()
        self.get_logger().info(
            (
                f"sending goal task_id={goal_msg.task_id} action={goal_msg.action} "
                f"target={goal_msg.device_type}/{goal_msg.device_id}"
            )
        )
        return self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )

    def feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(
            "feedback: state=%s progress=%.2f detail=%s",
            feedback.current_state,
            feedback.progress,
            feedback.detail,
        )

    def handle_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("goal rejected")
            self._goal_done = True
            return

        self.get_logger().info("goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_result)

    def handle_result(self, future) -> None:
        wrapper = future.result()
        self._result = wrapper.result
        self._goal_done = True
        self.get_logger().info(
            "result: status=%s success=%s final_state=%s detail=%s",
            wrapper.status,
            self._result.success,
            self._result.final_state,
            self._result.detail,
        )


def parse_args(args: list[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task-id", default=f"task-{int(time.time())}")
    parser.add_argument("--action", default="call_drone")
    parser.add_argument("--device-type", default="drone")
    parser.add_argument("--device-id", default="drone1")
    parser.add_argument("--item", default="apple")
    parser.add_argument("--quantity", type=int, default=1)
    parser.add_argument("--src", default="shelf")
    parser.add_argument("--dst", default="delivery_point_1")
    parser.add_argument("--transfer-to-device-type", default="")
    parser.add_argument("--transfer-to-device-id", default="")
    parser.add_argument("--final-dst-location", default="")
    parser.add_argument("--plan", default="direct_drone_delivery")
    return parser.parse_known_args(args)


def main(args=None) -> None:
    cli_args, ros_args = parse_args(args)
    rclpy.init(args=ros_args)
    node = DispatchActionClient()
    try:
        future = node.send_goal(cli_args)
        future.add_done_callback(node.handle_goal_response)

        while rclpy.ok() and not node._goal_done:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node._result is None or not node._result.success:
            raise SystemExit(1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
