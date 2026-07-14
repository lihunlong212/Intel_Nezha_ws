机器人侧部署说明

本机只作为任务接收端，不启动也不提供 /aiagent/dispatch_order Action Server。
远端调度系统负责把 Action 请求转换为 JSON 订单并发布到 /fleet/orders。
本机 dispatch_receiver 的接口如下：

订阅：
/fleet/orders (std_msgs/msg/String)
  - event=order：接收并执行指定设备的任务
  - event=cancel：取消指定任务
  - event=session_reset：取消该设备当前会话中的任务

发布：
/fleet/device_status (std_msgs/msg/String)：周期性设备状态
/fleet/order_events (std_msgs/msg/String)：UTF-8 JSON 的任务反馈和最终结果

所有设备都会收到订单；本机仅在 device_id 为 drone1 或 drone2 时执行。订单中的 task_id
会原样写入每条 feedback/result。feedback 使用 event=feedback、current_state、progress
（0.0 到 1.0）和 detail；每个已执行订单只发送一条 event=result。

状态映射：

drone1：空闲 -> 取货中 -> 送货中 -> 送货成功 -> 空闲
drone2：空闲 -> 送货中 -> 送货成功 -> 空闲

部署步骤：

mkdir -p ~/ros2_ws/src
# 把 robot_task_interfaces 复制到 ~/ros2_ws/src/
# 如果要先跑 demo，也把 robot_action_demo 复制到 ~/ros2_ws/src/

source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build
source install/setup.bash
机器人侧环境请保持和我们一致：

export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
启动机器人侧任务接收端：

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 run robot_action_demo dispatch_receiver

验证本机没有发布 Action Server：

ros2 action info /aiagent/dispatch_order

验证订单收发：

ros2 topic echo /fleet/orders
ros2 topic echo /fleet/device_status
ros2 topic echo /fleet/order_events
