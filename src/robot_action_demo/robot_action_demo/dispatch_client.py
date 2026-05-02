import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robot_task_interfaces.action import DispatchOrder


class DispatchActionClient(Node):
    def __init__(self) -> None:
        super().__init__("dispatch_action_client")
        self._client = ActionClient(self, DispatchOrder, "dispatch_order")
        self._goal_done = False
        self._result = None

    def send_goal(self, task_id: str, item: str, quantity: int, src: str, dst: str):
        goal_msg = DispatchOrder.Goal()
        goal_msg.task_id = task_id
        goal_msg.item = item
        goal_msg.quantity = quantity
        goal_msg.src_location = src
        goal_msg.dst_location = dst

        self.get_logger().info("waiting for action server...")
        self._client.wait_for_server()
        self.get_logger().info("sending goal...")
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
        self._result = future.result().result
        self._goal_done = True
        self.get_logger().info(
            "result: success=%s final_state=%s detail=%s",
            self._result.success,
            self._result.final_state,
            self._result.detail,
        )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--task-id", default=f"task-{int(time.time())}")
    parser.add_argument("--item", default="华为手机")
    parser.add_argument("--quantity", type=int, default=1)
    parser.add_argument("--src", default="货架")
    parser.add_argument("--dst", default="收银台")
    return parser.parse_args()


def main(args=None) -> None:
    cli_args = parse_args()
    rclpy.init(args=args)
    node = DispatchActionClient()
    try:
        future = node.send_goal(
            cli_args.task_id,
            cli_args.item,
            cli_args.quantity,
            cli_args.src,
            cli_args.dst,
        )
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
    main()
