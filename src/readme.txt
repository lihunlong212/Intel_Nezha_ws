交接说明
我们这边 AI 已经固定使用 ROS2 Action 下发任务。请你们在机器人侧实现并常驻一个 Action Server，用于接收派单、上报过程反馈、返回最终结果。

协议固定如下：

Action 名称：/dispatch_order
Action 类型：robot_task_interfaces/action/DispatchOrder
接口定义文件：DispatchOrder.action
DispatchOrder.action 字段如下：

Goal:
string task_id
string item
int32 quantity
string src_location
string dst_location

Result:
bool success
string task_id
string final_state
string detail

Feedback:
string current_state
float32 progress
string detail
请至少把这个接口包复制到你们机器的 ~/ros2_ws/src/ 并编译：

robot_task_interfaces
如果要先联调用现成 demo，也可以一起复制这个包：

robot_action_demo
机器人侧部署步骤：

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
如果先做联调验证，可以直接启动 demo server：

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 run robot_action_demo dispatch_server
如果接真实机器人，请你们自己实现一个常驻 Action Server，但必须保持这两点不变：

action 名称仍然是 /dispatch_order
action 类型仍然是 robot_task_interfaces/action/DispatchOrder
服务端行为要求：

收到 goal 后尽快 accept/reject
执行过程中持续返回 feedback
完成后返回 result
如果车坏了、任务不能执行，请尽快返回失败，不要一直卡住
我们这边发过去的 Goal 主要会包含：

task_id：任务 ID
item：商品名，比如“华为手机”
quantity：数量
src_location：取货点
dst_location：送达点
联调验收标准：

你们 server 启动后，我们这边执行
python3 scripts/docker_humble_bridge.py action-info --action-name /dispatch_order
如果看到 Action servers: 1，说明已经发现你们的 server
之后 AI 发单时，我们应该能看到 accepted -> feedback -> result