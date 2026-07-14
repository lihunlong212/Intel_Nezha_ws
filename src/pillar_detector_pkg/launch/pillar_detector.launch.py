from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pillar_detector_pkg',
            executable='pillar_detector_node',
            name='pillar_detector',
            output='screen',
            parameters=[{
                # ── 话题 ────────────────────────────────────
                'scan_topic': '/scan',

                # ── 柱子有效区域（单位：米） ──────────────────
                # 题目：柱子中心距场地边界 ≥50cm，场地 3×3m
                # 无人机在右下角(0,0)，向前+x，向左+y
                'map_x_min_m': 0.60,   # 前方最近
                'map_x_max_m': 1.00,    # 前方最远
                'map_y_min_m': -0.10,   # 左侧最近（正数）
                'map_y_max_m': 3.5,    # 左侧最远（正数）

                # ── 单帧分组参数 ──────────────────────────────
                # 同组内相邻点最大距离：柱子点间距≤15mm，不同柱子间距≥80cm，25cm足够区分
                'group_dist_m': 0.25,
                # 单帧内连续跳变激光束数；改这个值可调异常束数量门槛
                'min_pts_per_group': 1,
                # 同帧内两个柱子中心最小距离（柱子间距≥80cm，取一半40cm作安全门槛）
                'min_pillar_separation_m': 0.20,

                # ── 多帧累积参数 ──────────────────────────────
                # 使用连续 10 帧雷达数据
                'accumulation_frames': 10,
                # 两次检测在 20cm 内算同一个柱子
                'cluster_merge_dist_m': 0.20,
                # 10 帧中至少 5 个不同帧出现同一跳变才确认为柱子
                'min_votes': 5,
                # 保留异常的第 3/4 根柱子，供航线安全门判定为无效
                'max_pillars': 2,
            }],
        )
    ])
