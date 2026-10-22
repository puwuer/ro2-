"""巡检 + 视觉跟踪 任务层 launch

启动内容:
  1. mycar_patrol/patrol_node        三状态任务节点 (PATROL / TRACK / SEARCH)
  2. tracker/yolo_tracker_node_unn   视觉检测节点 (仿真参数: 只发检测结果, 不自己导航)

前置条件: 已经通过 sim_nav.launch.py 起好 仿真 + 定位 + nav2。

用法:
  ros2 launch mycar_patrol patrol_track.launch.py
  ros2 launch mycar_patrol patrol_track.launch.py enable_tracker:=false
"""

import os
import signal
import subprocess
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 重复启动会让两个 patrol_node / yolo_tracker_node 同时发布 /system_status 与检测结果,
# 导致状态来回跳, 所以启动前先清掉残留的同类节点
STALE_PATTERNS = [
    'mycar_patrol/lib/mycar_patrol/patrol_node',
    'tracker/lib/tracker/yolo_tracker_node_unn',
]


def clean_stale_nodes(context):
    killed = []
    for pattern in STALE_PATTERNS:
        try:
            out = subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True)
        except FileNotFoundError:
            return []
        for pid_text in out.stdout.split():
            pid = int(pid_text)
            if pid == os.getpid():
                continue
            try:
                os.kill(pid, signal.SIGINT)
                killed.append(pid)
            except ProcessLookupError:
                pass
    if not killed:
        return []
    time.sleep(3.0)
    for pid in killed:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    return [LogInfo(msg='清理残留的巡检/跟踪节点: ' + ', '.join(str(p) for p in killed))]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_tracker = LaunchConfiguration("enable_tracker")
    model_path = LaunchConfiguration("model_path")
    confidence_min = LaunchConfiguration("confidence_min")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="使用 /clock 仿真时间")
    declare_enable_tracker = DeclareLaunchArgument(
        "enable_tracker", default_value="true", description="是否启动视觉跟踪节点")
    declare_model_path = DeclareLaunchArgument(
        "model_path",
        default_value="/home/ros/Robot_ws/src/tracker/models/yolov8n.onnx",
        description="YOLOv8 ONNX 模型路径")
    declare_confidence_min = DeclareLaunchArgument(
        "confidence_min", default_value="0.08",
        description="判定检测到人的最低置信度(仿真低模小人分数偏低, 实车可调回 0.4)")
    declare_detect_threshold = DeclareLaunchArgument(
        "detect_threshold", default_value="0.08",
        description="YOLO 框出人的最低分数(仿真低模小人分数偏低)")
    detect_threshold = LaunchConfiguration("detect_threshold")
    # 地图原点相对世界原点的偏移(巡检点按世界坐标填写, 节点按 map = 世界 - 偏移 换算)
    # 用自带 Factory_map(与世界对齐): 0, 0
    # 用 cartographer 现场建图(map 原点=出生点): 出生点世界坐标, 例如 4.225, -3.975
    # 默认地图是 cartographer 建的 1_map.yaml(原点=出生点), 所以默认偏移就是出生点世界坐标;
    # 若改用与世界对齐的 Factory_map(origin -11,-18) 则传 0 0
    declare_map_offset_x = DeclareLaunchArgument("map_offset_x", default_value="4.225")
    declare_map_offset_y = DeclareLaunchArgument("map_offset_y", default_value="-3.975")
    map_offset_x = LaunchConfiguration("map_offset_x")
    map_offset_y = LaunchConfiguration("map_offset_y")

    # 巡检/跟踪/搜索状态机(唯一的 /navigate_to_pose 调用者)
    patrol_node = Node(
        package="mycar_patrol",
        executable="patrol_node",
        name="patrol_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "confidence_min": confidence_min,
            "goal_timeout": 60.0,
            "max_retries": 1,
            "lost_timeout": 2.0,
            "search_timeout": 20.0,
            "search_angular_vel": 0.6,
            "approach_distance": 1.0,
            "goal_min_delta": 0.3,
            "track_enter_count": 2,
            "track_update_period": 1.0,
            "status_rate": 10.0,
            "map_frame": "map",
            "robot_frame": "base_link",
            "map_offset_x": map_offset_x,
            "map_offset_y": map_offset_y,
        }],
    )

    # YOLO 检测节点(仿真话题; 不自己发导航目标, 交给 patrol_node)
    tracker_node = Node(
        package="tracker",
        executable="yolo_tracker_node_unn",
        name="yolo_tracker_node",
        output="screen",
        condition=IfCondition(enable_tracker),
        parameters=[{
            "use_sim_time": use_sim_time,
            "model_path": model_path,
            "rgb_topic": "/image_raw",
            "depth_topic": "/depth_camera",
            "camera_info_topic": "/camera_info",
            "camera_frame": "camera_optical_frame",
            "goal_frame": "map",
            "detection_interval": 5,
            "mirror_flip": False,
            "enable_direct_control": False,
            "enable_nav_client": False,
            "target_min_depth": 0.3,
            "target_max_depth": 6.0,
            "lost_cancel_timeout": 2.0,
            "confidence_min": confidence_min,
            "detect_threshold": detect_threshold,
            "fallback_fx": 220.0,
            "fallback_fy": 220.0,
            "fallback_cx": 128.0,
            "fallback_cy": 128.0,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_tracker,
        declare_model_path,
        declare_confidence_min,
        declare_detect_threshold,
        declare_map_offset_x,
        declare_map_offset_y,
        OpaqueFunction(function=clean_stale_nodes),
        patrol_node,
        tracker_node,
    ])
