# Intel_Nezha_ws

李俊龙英特尔邀请赛无人机开发 ROS2 工作空间。

## 工作空间结构

```
Intel_Nezha_ws/
├── src/          # 存放所有 ROS2 功能包
├── build/        # 编译产物（由 colcon 生成，已 gitignore）
├── install/      # 安装目录（由 colcon 生成，已 gitignore）
└── log/          # 日志目录（由 colcon 生成，已 gitignore）
```

## 环境依赖

- ROS2 Humble（或更高版本）
- colcon 构建工具

```bash
sudo apt install python3-colcon-common-extensions
```

## 编译工作空间

```bash
# 在工作空间根目录执行
cd Intel_Nezha_ws
colcon build --symlink-install
```

## 加载环境

```bash
source install/setup.bash
```

## 新建功能包（示例）

```bash
cd src
# C++ 包
ros2 pkg create --build-type ament_cmake <package_name>
# Python 包
ros2 pkg create --build-type ament_python <package_name>
```
